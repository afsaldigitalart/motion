#!/usr/bin/env python3
"""
STM32 MOTION CAMERA
===================

STM32 Blue Pill + HC-SR501 PIR  --UART-->  USB-TTL  -->  this program (laptop)

What this program does
  1. A background thread listens to the USB-TTL serial port.
  2. When the STM32 sends "MOTION", ONE frame is captured from the webcam
     and saved as a JPEG in captures/.
  3. A Flask web server (listening on the LAN) shows the latest image.
     The web page polls /api/status and updates itself - no manual refresh.

Messages understood from the STM32 (anything else is ignored):
  MOTION   motion detected          -> capture an image
  WARMUP   STM32 just booted, PIR is warming up
  READY    PIR warm-up finished
  ALIVE    heartbeat (used to notice that the STM32 was unplugged/reset)

Everything stays on this laptop: no cloud, no external APIs.
"""

import glob
import logging
import os
import socket
import threading
import time
from datetime import datetime

import cv2
import serial
from flask import Flask, Response, abort, jsonify, render_template, send_from_directory

# =============================================================================
# CONFIGURATION  -  edit these values (or set the environment variables)
# =============================================================================
# Serial port of the USB-TTL adapter.
#   Windows: "COM3", "COM4", ...      Linux: "/dev/ttyUSB0" or "/dev/ttyACM0"
#   macOS:   "/dev/tty.usbserial-XXXX"
# You can also run:  MOTION_SERIAL_PORT=COM5 python app.py   (Linux: export first)
SERIAL_PORT = os.environ.get(
    "MOTION_SERIAL_PORT", "COM6" if os.name == "nt" else "/dev/ttyUSB0"
)
BAUD_RATE = 115200                # must match the firmware (115200 8N1)
SERIAL_TIMEOUT = 1                # seconds; readline() gives up after this
SERIAL_RECONNECT_SECONDS = 3      # wait between reconnection attempts
STM32_TIMEOUT_SECONDS = 15        # no message for this long => "STM32 DISCONNECTED"
                                  # (firmware sends ALIVE every 5 s). 0 = disable check.

CAMERA_INDEX = int(os.environ.get("MOTION_CAMERA_INDEX", "0"))   # 0 = built-in webcam
CAMERA_KEEP_OPEN = True           # True: open webcam once at start (faster, camera LED stays on)
                                  # False: open/close it for every motion event
CAMERA_FLUSH_FRAMES = 5           # frames thrown away before the real capture, so you get a
                                  # fresh, properly exposed picture instead of a stale buffered one
MAX_IMAGE_WIDTH = None            # e.g. 1280 to downscale wide images; None = keep original size
IMAGE_QUALITY = 90                # JPEG quality 0-100

SERVER_HOST = "0.0.0.0"           # 0.0.0.0 = reachable from other devices on the LAN
SERVER_PORT = int(os.environ.get("MOTION_SERVER_PORT", "5000"))

COOLDOWN_SECONDS = 3              # ignore MOTION messages arriving within this time of a capture
PIR_WARMUP_SECONDS = 30           # keep equal to PIR_WARMUP_SECONDS in the firmware
MOTION_DISPLAY_SECONDS = 5        # how long the web page shows "MOTION DETECTED"
RECENT_COUNT = 8                  # thumbnails in "Recent captures" on the web page
# =============================================================================

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
CAPTURE_DIR = os.path.join(BASE_DIR, "captures")
LATEST_PATH = os.path.join(CAPTURE_DIR, "latest.jpg")

app = Flask(__name__)

# ----------------------------------------------------------------------------
# Shared state.  Always take state_lock before reading/writing `state`.
# camera_lock guarantees that only one thread uses the webcam at a time.
# Lock order (to avoid deadlocks): camera_lock first, then state_lock.
# ----------------------------------------------------------------------------
state_lock = threading.RLock()
camera_lock = threading.Lock()
print_lock = threading.Lock()

camera = None  # cv2.VideoCapture object (protected by camera_lock)

state = {
    "app_state": "STARTING",
    "serial_open": False,          # USB-TTL port is open
    "stm32_seen": None,            # time.monotonic() of the last message from the STM32
    "camera_ok": False,
    "warmup_until": 0.0,           # time.monotonic() until which the PIR is warming up
    "motion_count": 0,             # captures taken since this program started
    "last_motion": None,           # "YYYY-MM-DD HH:MM:SS"
    "last_image": None,            # file name of the newest timestamped image
    "image_version": 0,            # changes whenever there is a new image (cache-busting)
    "motion_until": 0.0,           # time.time() until which "motion detected" is shown
    "last_capture_mono": float("-inf"),
    "latest_bytes": None,          # newest JPEG, kept in memory so /latest is always fast
    "recent": [],                  # newest-first list of image file names
}


# ----------------------------------------------------------------------------
# Logging helpers
# ----------------------------------------------------------------------------
def log(level, message):
    """[INFO] / [OK] / [WARNING] / [ERROR] / [STATE] lines."""
    with print_lock:
        print(f"[{level}] {message}", flush=True)


def tlog(message):
    """Timestamped line used for motion events: [20:51:32] ..."""
    with print_lock:
        print(f"[{datetime.now():%H:%M:%S}] {message}", flush=True)


def set_state(name):
    """Change the application state (printed only when it really changes)."""
    with state_lock:
        if state["app_state"] == name:
            return
        state["app_state"] = name
    log("STATE", name)


# ----------------------------------------------------------------------------
# Connection / status helpers
# ----------------------------------------------------------------------------
def stm32_connected():
    """True when the serial port is open AND the STM32 recently sent something."""
    with state_lock:
        if not state["serial_open"]:
            return False
        if STM32_TIMEOUT_SECONDS <= 0:
            return True
        seen = state["stm32_seen"]
        return seen is not None and (time.monotonic() - seen) <= STM32_TIMEOUT_SECONDS


def pir_warming_up():
    with state_lock:
        return time.monotonic() < state["warmup_until"]


def resting_state():
    """The state the system should be in when nothing is happening."""
    if not stm32_connected():
        return "STM32 DISCONNECTED"
    if pir_warming_up():
        return "PIR WARMING UP"
    with state_lock:
        if not state["camera_ok"]:
            return "CAMERA ERROR"
    return "MONITORING"


def set_camera_ok(value):
    with state_lock:
        state["camera_ok"] = value


# ----------------------------------------------------------------------------
# Webcam
# ----------------------------------------------------------------------------
def _open_camera_locked():
    """Open the webcam. Caller must hold camera_lock."""
    global camera
    backend = cv2.CAP_DSHOW if os.name == "nt" else cv2.CAP_ANY  # DirectShow is more reliable on Windows
    try:
        cam = cv2.VideoCapture(CAMERA_INDEX, backend)
    except Exception as exc:  # OpenCV can raise on odd drivers
        log("ERROR", f"Webcam could not be opened ({exc}).")
        camera = None
        set_camera_ok(False)
        return False
    if cam is None or not cam.isOpened():
        if cam is not None:
            cam.release()
        camera = None
        set_camera_ok(False)
        log("ERROR", "Webcam could not be opened.")
        return False
    camera = cam
    set_camera_ok(True)
    return True


def _close_camera_locked():
    """Release the webcam. Caller must hold camera_lock."""
    global camera
    if camera is not None:
        try:
            camera.release()
        except Exception:
            pass
    camera = None


def init_camera():
    """Open the webcam once at start-up (also checks that it works)."""
    log("INFO", "Initializing webcam...")
    with camera_lock:
        ok = _open_camera_locked()
        if ok and not CAMERA_KEEP_OPEN:
            _close_camera_locked()
    if ok:
        log("OK", "Webcam ready")
        set_state("WEBCAM READY")
    return ok


def _read_fresh_frame_locked(flush):
    """Throw away `flush` buffered frames, then return the next one (or None)."""
    frame = None
    for _ in range(flush + 1):
        ok, frame = camera.read()
        if not ok or frame is None:
            return None
    return frame


def capture_frame():
    """Capture ONE frame. Returns a numpy image, or None on failure."""
    flush = CAMERA_FLUSH_FRAMES if CAMERA_KEEP_OPEN else max(CAMERA_FLUSH_FRAMES, 15)
    with camera_lock:  # two motion events can never touch the camera at once
        frame = None
        for _attempt in (1, 2):
            if camera is None and not _open_camera_locked():
                return None
            frame = _read_fresh_frame_locked(flush)
            if frame is not None:
                set_camera_ok(True)
                break
            log("ERROR", "Failed to capture webcam frame.")
            set_camera_ok(False)
            _close_camera_locked()  # camera may have been unplugged: reopen on the 2nd attempt
        if not CAMERA_KEEP_OPEN:
            _close_camera_locked()
        return frame


# ----------------------------------------------------------------------------
# Saving images
# ----------------------------------------------------------------------------
def save_image(frame):
    """Write the timestamped JPEG + captures/latest.jpg. Returns (name, jpeg_bytes, datetime)."""
    if MAX_IMAGE_WIDTH and frame.shape[1] > MAX_IMAGE_WIDTH:
        scale = MAX_IMAGE_WIDTH / frame.shape[1]
        frame = cv2.resize(frame, (MAX_IMAGE_WIDTH, int(frame.shape[0] * scale)))

    ok, buffer = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, IMAGE_QUALITY])
    if not ok:
        raise RuntimeError("JPEG encoding failed")
    data = buffer.tobytes()

    now = datetime.now()
    base = now.strftime("motion_%Y%m%d_%H%M%S")
    name = base + ".jpg"
    # Same second as an earlier image? Add milliseconds, then a counter if still needed.
    if os.path.exists(os.path.join(CAPTURE_DIR, name)):
        name = f"{base}_{now.microsecond // 1000:03d}.jpg"
        counter = 1
        while os.path.exists(os.path.join(CAPTURE_DIR, name)):
            name = f"{base}_{now.microsecond // 1000:03d}_{counter}.jpg"
            counter += 1

    with open(os.path.join(CAPTURE_DIR, name), "wb") as f:
        f.write(data)

    # latest.jpg: write to a temp file first, then swap it in (never half-written)
    tmp_path = LATEST_PATH + ".tmp"
    try:
        with open(tmp_path, "wb") as f:
            f.write(data)
        os.replace(tmp_path, LATEST_PATH)
    except OSError as exc:
        log("WARNING", f"Could not update latest.jpg ({exc}); the web page still works.")
    return name, data, now


# ----------------------------------------------------------------------------
# Motion handling
# ----------------------------------------------------------------------------
def handle_motion():
    """Called by the serial thread for every 'MOTION' message."""
    if pir_warming_up():
        log("INFO", "MOTION ignored (PIR is still warming up)")
        return

    now_mono = time.monotonic()
    with state_lock:
        if now_mono - state["last_capture_mono"] < COOLDOWN_SECONDS:
            log("INFO", "MOTION ignored (cooldown)")
            return
        state["last_capture_mono"] = now_mono

    tlog("MOTION DETECTED")
    set_state("MOTION DETECTED")
    tlog("Capturing image...")
    set_state("CAPTURING")

    frame = capture_frame()
    if frame is None:
        log("ERROR", "No image saved (webcam problem).")
        set_state(resting_state())
        return

    try:
        name, data, taken = save_image(frame)
    except Exception as exc:
        log("ERROR", f"Could not save image: {exc}")
        set_state(resting_state())
        return

    tlog("Image saved")
    set_state("IMAGE SAVED")
    tlog("Updating webpage")
    with state_lock:
        state["motion_count"] += 1
        state["last_motion"] = taken.strftime("%Y-%m-%d %H:%M:%S")
        state["last_image"] = name
        state["image_version"] = max(state["image_version"] + 1, int(time.time() * 1000))
        state["latest_bytes"] = data
        state["motion_until"] = time.time() + MOTION_DISPLAY_SECONDS
        state["recent"] = ([name] + [n for n in state["recent"] if n != name])[:RECENT_COUNT]
    set_state(resting_state())
    tlog("Waiting for motion...")


def handle_line(line):
    """Interpret one line received from the STM32. Unknown lines are ignored."""
    if line == "MOTION":
        handle_motion()
    elif line == "WARMUP":
        with state_lock:
            state["warmup_until"] = time.monotonic() + PIR_WARMUP_SECONDS + 2
        log("INFO", "STM32 started - PIR warming up...")
        set_state(resting_state())
    elif line == "READY":
        with state_lock:
            state["warmup_until"] = 0.0
        log("OK", "PIR ready")
        set_state(resting_state())
    # "ALIVE" and anything else: only updates the "last seen" time (done by the caller)


# ----------------------------------------------------------------------------
# Serial listener thread (never runs inside a Flask route)
# ----------------------------------------------------------------------------
def open_serial():
    return serial.Serial(
        port=SERIAL_PORT,
        baudrate=BAUD_RATE,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        xonxoff=False,
        rtscts=False,
        timeout=SERIAL_TIMEOUT,
    )


def serial_worker():
    ser = None
    last_error = None
    last_connected = False

    def refresh_connection():
        """Log + update state when the STM32 connection status changes."""
        nonlocal last_connected
        connected = stm32_connected()
        if connected != last_connected:
            last_connected = connected
            if connected:
                log("OK", f"STM32 connected on {SERIAL_PORT}")
                set_state("STM32 CONNECTED")
            else:
                log("WARNING", "STM32 not responding (no data from the board).")
            set_state(resting_state())

    while True:
        # ---- (re)connect -------------------------------------------------
        if ser is None:
            try:
                ser = open_serial()
                ser.reset_input_buffer()
                with state_lock:
                    state["serial_open"] = True
                    state["stm32_seen"] = None
                last_error = None
                log("INFO", f"Serial port {SERIAL_PORT} opened. Waiting for the STM32...")
                refresh_connection()
            except (serial.SerialException, OSError, ValueError) as exc:
                ser = None
                message = str(exc)
                if message != last_error:  # don't repeat the same error every few seconds
                    last_error = message
                    log("WARNING", f"Cannot open {SERIAL_PORT}: {message}")
                    if "ermission" in message or "Errno 13" in message:
                        log("INFO", "Permission denied. Linux: add yourself to the 'dialout' group "
                                    "(sudo usermod -aG dialout $USER), then log out and in. "
                                    "Windows/macOS: close any other program using the port.")
                    else:
                        log("INFO", "Check SERIAL_PORT in app.py and that the USB-TTL adapter is plugged in.")
                with state_lock:
                    state["serial_open"] = False
                refresh_connection()
                time.sleep(SERIAL_RECONNECT_SECONDS)
                continue

        # ---- read one line -------------------------------------------------
        try:
            raw = ser.readline()  # returns b"" after SERIAL_TIMEOUT seconds
        except (serial.SerialException, OSError, TypeError):
            log("WARNING", "Serial device disconnected.")
            log("INFO", "Attempting to reconnect...")
            try:
                ser.close()
            except Exception:
                pass
            ser = None
            with state_lock:
                state["serial_open"] = False
                state["stm32_seen"] = None
            refresh_connection()
            time.sleep(SERIAL_RECONNECT_SECONDS)
            continue

        if raw:
            with state_lock:
                state["stm32_seen"] = time.monotonic()
            line = raw.decode("ascii", errors="ignore").strip()
            refresh_connection()
            try:
                handle_line(line)
            except Exception as exc:  # keep the listener alive no matter what
                log("ERROR", f"Error while handling '{line}': {exc}")
        else:
            refresh_connection()  # timeout: lets us notice a silent STM32


# ----------------------------------------------------------------------------
# Flask routes
# ----------------------------------------------------------------------------
@app.route("/")
def index():
    return render_template("index.html")


@app.route("/latest")
def latest():
    """The most recent capture (no matter what its file name is)."""
    with state_lock:
        data = state["latest_bytes"]
    if data is None:
        abort(404)
    response = Response(data, mimetype="image/jpeg")
    response.headers["Cache-Control"] = "no-store"
    return response


@app.route("/captures/<path:filename>")
def capture_file(filename):
    """Serve older timestamped captures (for the 'Recent captures' strip)."""
    if not (filename.startswith("motion_") and filename.endswith(".jpg")):
        abort(404)
    return send_from_directory(CAPTURE_DIR, filename, max_age=3600)


@app.route("/api/status")
def api_status():
    connected = stm32_connected()
    warming = pir_warming_up()
    with state_lock:
        motion = time.time() < state["motion_until"]
        camera_ok = state["camera_ok"]
        payload = {
            "online": True,
            "motion_detected": motion,
            "motion_count": state["motion_count"],
            "last_motion": state["last_motion"],
            "last_image": state["last_image"],
            "image_version": state["image_version"],
            "recent": list(state["recent"]),
            "stm32_connected": connected,
            "camera_ok": camera_ok,
            "pir_warming_up": warming,
            "app_state": state["app_state"],
        }
    # status text + colour for the web page (priority: connection > warm-up > camera > motion)
    if not connected:
        payload["status"], payload["level"] = "STM32 DISCONNECTED", "orange"
    elif warming:
        payload["status"], payload["level"] = "PIR WARMING UP", "yellow"
    elif not camera_ok:
        payload["status"], payload["level"] = "CAMERA ERROR", "orange"
    elif motion:
        payload["status"], payload["level"] = "MOTION DETECTED", "red"
    else:
        payload["status"], payload["level"] = "MONITORING", "green"
    response = jsonify(payload)
    response.headers["Cache-Control"] = "no-store"
    return response


# ----------------------------------------------------------------------------
# Start-up helpers
# ----------------------------------------------------------------------------
def get_local_ip():
    """Best guess of this laptop's LAN IPv4 address (no internet needed, nothing is sent)."""
    for target in ("10.255.255.255", "192.168.255.255"):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            sock.settimeout(0.2)
            sock.connect((target, 1))  # UDP connect only picks a route; no packet is sent
            ip = sock.getsockname()[0]
            if ip and not ip.startswith("127.") and ip != "0.0.0.0":
                return ip
        except OSError:
            pass
        finally:
            sock.close()
    try:
        ip = socket.gethostbyname(socket.gethostname())
        if ip and not ip.startswith("127."):
            return ip
    except OSError:
        pass
    return None


def load_existing_captures():
    """Show the previous session's newest image right away after a restart."""
    files = sorted(glob.glob(os.path.join(CAPTURE_DIR, "motion_*.jpg")))
    if not files:
        return
    newest = files[-1]
    try:
        with open(newest, "rb") as f:
            data = f.read()
    except OSError:
        return
    with state_lock:
        state["latest_bytes"] = data
        state["last_image"] = os.path.basename(newest)
        state["last_motion"] = datetime.fromtimestamp(os.path.getmtime(newest)).strftime("%Y-%m-%d %H:%M:%S")
        state["image_version"] = int(os.path.getmtime(newest) * 1000)
        state["recent"] = [os.path.basename(p) for p in reversed(files)][:RECENT_COUNT]


def main():
    print("=" * 40)
    print("STM32 MOTION CAMERA")
    print("=" * 40)
    log("INFO", "Starting application...")
    log("INFO", f"Serial port: {SERIAL_PORT}")
    log("INFO", f"Baud rate: {BAUD_RATE}")
    log("INFO", f"Camera index: {CAMERA_INDEX}")

    os.makedirs(CAPTURE_DIR, exist_ok=True)
    load_existing_captures()

    # Flask logs every request (the page polls once a second) - keep the terminal readable.
    logging.getLogger("werkzeug").setLevel(logging.WARNING)

    log("INFO", "Connecting to STM32...")
    threading.Thread(target=serial_worker, name="serial-listener", daemon=True).start()

    init_camera()  # the web server starts even if this fails

    log("INFO", "Starting Flask server...")
    ip = get_local_ip()
    log("INFO", "Local:")
    print(f"       http://127.0.0.1:{SERVER_PORT}")
    if ip:
        log("INFO", "Network:")
        print(f"       http://{ip}:{SERVER_PORT}")
        print()
        print("  Open the Network address on a phone connected to the same Wi-Fi.")
        print()
    else:
        log("WARNING", "Could not detect the LAN IP address. Find it with ipconfig (Windows) "
                       "or 'hostname -I' (Linux).")
    log("INFO", "Waiting for motion...")

    try:
        app.run(host=SERVER_HOST, port=SERVER_PORT, debug=False, threaded=True)
    finally:
        with camera_lock:
            _close_camera_locked()


if __name__ == "__main__":
    main()