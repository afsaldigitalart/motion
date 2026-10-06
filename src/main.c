/*
 * ============================================================
 * STM32F103C8T6 BLUE PILL
 * PIR Motion Detector + Buzzer + USB-TTL UART
 * ============================================================
 *
 * HC-SR501 PIR:
 * VCC -> 5V
 * GND -> GND
 * OUT -> PA0
 *
 * USB-TTL:
 * STM32 PA9  (TX) -> USB-TTL RX
 * STM32 PA10 (RX) -> USB-TTL TX
 * STM32 GND       -> USB-TTL GND
 *
 * BUZZER:
 * Buzzer + -> PB0
 * Buzzer - -> GND
 *
 * UART:
 * 115200 baud
 * 8 data bits
 * No parity
 * 1 stop bit
 *
 * LED:
 * PC13
 * Active LOW
 *
 * BOOT0:
 * LOW  -> Run application from Flash
 * HIGH -> Serial bootloader
 *
 * ============================================================
 */

#include "stm32f1xx.h"
#include <stdint.h>


/* ============================================================
   CONFIGURATION
   ============================================================ */

/* PIR sensor */
#define PIR_GPIO_PORT       GPIOA
#define PIR_GPIO_PIN        0u

/* PIR warm-up */
#define PIR_WARMUP_SECONDS  30u

/* PIR polling interval */
#define PIR_POLL_MS         20u

/* UART */
#define UART_BAUD           115200u

/* Heartbeat */
#define HEARTBEAT_MS        5000u

/* On-board LED */
#define LED_GPIO_PORT       GPIOC
#define LED_GPIO_PIN        13u

/* Buzzer */
#define BUZZER_GPIO_PORT    GPIOB
#define BUZZER_GPIO_PIN     0u

/* Buzzer beep duration */
#define BUZZER_BEEP_MS      500u


/* ============================================================
   GPIO CONFIGURATION VALUES
   STM32F1:
   MODE[1:0] + CNF[1:0]
   ============================================================ */

#define GPIO_INPUT_FLOATING  0x4u
#define GPIO_INPUT_PULL      0x8u
#define GPIO_OUTPUT_PP_2MHZ  0x2u
#define GPIO_AF_PP_50MHZ     0xBu


/* ============================================================
   GLOBAL CLOCK
   ============================================================ */

static uint32_t g_sysclk = 8000000u;


/* ============================================================
   CLOCK INITIALIZATION
   ============================================================ */

static uint32_t clock_init(void)
{
    uint32_t timeout;


    /* --------------------------------------------------------
       Enable HSE
       -------------------------------------------------------- */

    RCC->CR |= RCC_CR_HSEON;

    timeout = 100000u;

    while (!(RCC->CR & RCC_CR_HSERDY) && timeout--)
    {
    }


    /* HSE failed */
    if (!(RCC->CR & RCC_CR_HSERDY))
    {
        RCC->CR &= ~RCC_CR_HSEON;

        return 8000000u;
    }


    /* --------------------------------------------------------
       Flash configuration for 72 MHz
       -------------------------------------------------------- */

    FLASH->ACR |= FLASH_ACR_PRFTBE;

    FLASH->ACR &= ~FLASH_ACR_LATENCY;

    FLASH->ACR |= FLASH_ACR_LATENCY_2;


    /* --------------------------------------------------------
       Clock configuration
       --------------------------------------------------------

       HSE  = 8 MHz
       PLL  = x9
       SYSCLK = 72 MHz

       AHB  = 72 MHz
       APB1 = 36 MHz
       APB2 = 72 MHz
       -------------------------------------------------------- */

    RCC->CFGR &= ~(RCC_CFGR_HPRE |
                   RCC_CFGR_PPRE1 |
                   RCC_CFGR_PPRE2 |
                   RCC_CFGR_PLLSRC |
                   RCC_CFGR_PLLXTPRE |
                   RCC_CFGR_PLLMULL);

    RCC->CFGR |= RCC_CFGR_HPRE_DIV1 |
                 RCC_CFGR_PPRE1_DIV2 |
                 RCC_CFGR_PPRE2_DIV1 |
                 RCC_CFGR_PLLSRC |
                 RCC_CFGR_PLLMULL9;


    /* --------------------------------------------------------
       Enable PLL
       -------------------------------------------------------- */

    RCC->CR |= RCC_CR_PLLON;

    timeout = 100000u;

    while (!(RCC->CR & RCC_CR_PLLRDY) && timeout--)
    {
    }


    /* PLL failed */
    if (!(RCC->CR & RCC_CR_PLLRDY))
    {
        RCC->CR &= ~RCC_CR_PLLON;

        return 8000000u;
    }


    /* --------------------------------------------------------
       Select PLL as system clock
       -------------------------------------------------------- */

    RCC->CFGR &= ~RCC_CFGR_SW;

    RCC->CFGR |= RCC_CFGR_SW_PLL;


    timeout = 100000u;

    while (((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL)
           && timeout--)
    {
    }


    /* PLL switch failed */
    if ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL)
    {
        return 8000000u;
    }


    return 72000000u;
}


/* ============================================================
   SYSTICK
   ============================================================ */

static void systick_init(uint32_t hclk_hz)
{
    SysTick->LOAD = (hclk_hz / 1000u) - 1u;

    SysTick->VAL = 0u;

    SysTick->CTRL =
        SysTick_CTRL_CLKSOURCE_Msk |
        SysTick_CTRL_ENABLE_Msk;
}


/* ============================================================
   DELAY
   ============================================================ */

static void delay_ms(uint32_t ms)
{
    while (ms--)
    {
        while (!(SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk))
        {
        }
    }
}


/* ============================================================
   GPIO CONFIGURATION HELPER
   ============================================================ */

static void gpio_config(GPIO_TypeDef *port,
                        uint32_t pin,
                        uint32_t config)
{
    volatile uint32_t *reg;

    uint32_t shift;


    if (pin < 8u)
    {
        reg = &port->CRL;
    }
    else
    {
        reg = &port->CRH;
    }


    shift = (pin & 7u) * 4u;


    uint32_t value = *reg;


    value &= ~(0xFu << shift);

    value |= (config << shift);


    *reg = value;
}


/* ============================================================
   GPIO INITIALIZATION
   ============================================================ */

static void gpio_init(void)
{
    /*
     * Enable:
     *
     * GPIOA
     * GPIOB
     * GPIOC
     * AFIO
     * USART1
     */

    RCC->APB2ENR |=
        RCC_APB2ENR_IOPAEN |
        RCC_APB2ENR_IOPBEN |
        RCC_APB2ENR_IOPCEN |
        RCC_APB2ENR_AFIOEN |
        RCC_APB2ENR_USART1EN;


    /* Read-back to ensure clocks are enabled */
    (void)RCC->APB2ENR;


    /* ========================================================
       PIR PA0
       Input with pull-down
       ======================================================== */

    gpio_config(
        PIR_GPIO_PORT,
        PIR_GPIO_PIN,
        GPIO_INPUT_PULL
    );


    /*
     * Pull-down:
     *
     * ODR = 0
     */

    PIR_GPIO_PORT->BRR =
        (1u << PIR_GPIO_PIN);


    /* ========================================================
       USART1 TX - PA9
       Alternate Function Push-Pull
       ======================================================== */

    gpio_config(
        GPIOA,
        9u,
        GPIO_AF_PP_50MHZ
    );


    /* ========================================================
       USART1 RX - PA10
       Floating input
       ======================================================== */

    gpio_config(
        GPIOA,
        10u,
        GPIO_INPUT_FLOATING
    );


    /* ========================================================
       ONBOARD LED - PC13
       Output Push-Pull
       ======================================================== */

    gpio_config(
        LED_GPIO_PORT,
        LED_GPIO_PIN,
        GPIO_OUTPUT_PP_2MHZ
    );


    /*
     * Blue Pill LED is active LOW.
     *
     * HIGH = OFF
     */

    LED_GPIO_PORT->BSRR =
        (1u << LED_GPIO_PIN);


    /* ========================================================
       BUZZER - PB0
       Output Push-Pull
       ======================================================== */

    gpio_config(
        BUZZER_GPIO_PORT,
        BUZZER_GPIO_PIN,
        GPIO_OUTPUT_PP_2MHZ
    );


    /*
     * Buzzer OFF initially
     *
     * LOW = OFF
     */

    BUZZER_GPIO_PORT->BRR =
        (1u << BUZZER_GPIO_PIN);
}


/* ============================================================
   READ PIR
   ============================================================ */

static uint8_t pir_read(void)
{
    return (uint8_t)
    (
        (PIR_GPIO_PORT->IDR >> PIR_GPIO_PIN)
        & 1u
    );
}


/* ============================================================
   LED CONTROL
   ============================================================ */

static void led_set(uint8_t on)
{
    if (on)
    {
        /*
         * Blue Pill LED is active LOW.
         *
         * LOW = ON
         */

        LED_GPIO_PORT->BRR =
            (1u << LED_GPIO_PIN);
    }
    else
    {
        /*
         * HIGH = OFF
         */

        LED_GPIO_PORT->BSRR =
            (1u << LED_GPIO_PIN);
    }
}


/* ============================================================
   BUZZER CONTROL
   ============================================================ */

static void buzzer_on(void)
{
    BUZZER_GPIO_PORT->BSRR =
        (1u << BUZZER_GPIO_PIN);
}


static void buzzer_off(void)
{
    BUZZER_GPIO_PORT->BRR =
        (1u << BUZZER_GPIO_PIN);
}


static void buzzer_beep(uint32_t duration_ms)
{
    buzzer_on();

    delay_ms(duration_ms);

    buzzer_off();
}


/* ============================================================
   UART INITIALIZATION
   ============================================================ */

static void uart_init(uint32_t pclk2_hz)
{
    /*
     * Disable USART
     */

    USART1->CR1 = 0u;


    /*
     * One stop bit
     */

    USART1->CR2 = 0u;


    /*
     * No hardware flow control
     */

    USART1->CR3 = 0u;


    /*
     * Baud calculation
     *
     * PCLK2 = 72 MHz
     * Baud  = 115200
     *
     * BRR ≈ 625
     */

    USART1->BRR =
        (pclk2_hz + (UART_BAUD / 2u))
        / UART_BAUD;


    /*
     * Enable:
     *
     * USART
     * Transmitter
     * Receiver
     */

    USART1->CR1 =
        USART_CR1_UE |
        USART_CR1_TE |
        USART_CR1_RE;
}


/* ============================================================
   UART SEND STRING
   ============================================================ */

static void uart_send(const char *message)
{
    /*
     * IMPORTANT:
     *
     * while (*message)
     *
     * NOT:
     *
     * while (message)
     *
     * The latter would never stop at the null terminator.
     */

    while (*message)
    {
        /*
         * Wait until transmit data register is empty
         */

        while (!(USART1->SR & USART_SR_TXE))
        {
        }


        /*
         * Send character
         */

        USART1->DR =
            (uint8_t)*message++;
    }


    /*
     * Wait until final character has completely
     * left the UART.
     */

    while (!(USART1->SR & USART_SR_TC))
    {
    }
}


/* ============================================================
   MAIN
   ============================================================ */

int main(void)
{
    /* --------------------------------------------------------
       Initialize system clock
       -------------------------------------------------------- */

    g_sysclk = clock_init();

    SystemCoreClock = g_sysclk;


    /* --------------------------------------------------------
       Initialize SysTick
       -------------------------------------------------------- */

    systick_init(g_sysclk);


    /* --------------------------------------------------------
       Initialize GPIO
       -------------------------------------------------------- */

    gpio_init();


    /* --------------------------------------------------------
       Initialize UART
       -------------------------------------------------------- */

    uart_init(g_sysclk);


    /* --------------------------------------------------------
       Variables
       -------------------------------------------------------- */

    uint32_t warmup_ms =
        PIR_WARMUP_SECONDS * 1000u;

    uint32_t elapsed_ms = 0u;

    uint32_t heartbeat_ms = 0u;

    uint8_t ready = 0u;


    /*
     * Previous PIR state.
     *
     * Start LOW so the first HIGH after warm-up
     * can potentially be detected.
     */

    uint8_t previous_state = 0u;


    /* --------------------------------------------------------
       Make sure outputs start OFF
       -------------------------------------------------------- */

    led_set(0u);

    buzzer_off();


    /* --------------------------------------------------------
       Startup message
       -------------------------------------------------------- */

    uart_send("WARMUP\r\n");


    /* ========================================================
       MAIN LOOP
       ======================================================== */

    while (1)
    {
        uint8_t current_state =
            pir_read();


        /* ====================================================
           PIR WARM-UP
           ==================================================== */

        if (!ready)
        {
            elapsed_ms += PIR_POLL_MS;


            /*
             * LED OFF during warm-up.
             */

            led_set(0u);


            /*
             * Buzzer OFF during warm-up.
             */

            buzzer_off();


            /*
             * Warm-up complete
             */

            if (elapsed_ms >= warmup_ms)
            {
                ready = 1u;


                /*
                 * Capture current PIR state.
                 *
                 * If PIR is already HIGH,
                 * don't immediately trigger motion.
                 */

                previous_state =
                    pir_read();


                uart_send("READY\r\n");


                /*
                 * Short LED indication that
                 * the system is ready.
                 */

                led_set(1u);

                delay_ms(200);

                led_set(0u);
            }
        }


        /* ====================================================
           NORMAL OPERATION
           ==================================================== */

        else
        {
            /*
             * Detect LOW -> HIGH transition.
             */

            if ((current_state == 1u) &&
                (previous_state == 0u))
            {
                /*
                 * Motion detected
                 */

                uart_send("MOTION\r\n");


                /*
                 * Turn buzzer ON
                 */

                buzzer_on();


                /*
                 * Keep buzzer ON for 500 ms
                 */

                delay_ms(BUZZER_BEEP_MS);


                /*
                 * Turn buzzer OFF
                 */

                buzzer_off();
            }


            /*
             * Save current PIR state
             */

            previous_state =
                current_state;


            /*
             * LED follows PIR state
             *
             * PIR HIGH -> LED ON
             * PIR LOW  -> LED OFF
             */

            led_set(current_state);
        }


        /* ====================================================
           HEARTBEAT
           ==================================================== */

        heartbeat_ms += PIR_POLL_MS;


        if (heartbeat_ms >= HEARTBEAT_MS)
        {
            heartbeat_ms = 0u;

            uart_send("ALIVE\r\n");
        }


        /* ----------------------------------------------------
           Poll every 20 ms
           ---------------------------------------------------- */

        delay_ms(PIR_POLL_MS);
    }
}