/**
 ******************************************************************************
 * @file    main.c
 * @brief   ACC Smart Vehicle - Adaptive Cruise Control
 *          STM32F072RBT6 bare-metal, no HAL
 *
 * BEHAVIOR:
 *   - Drives forward until front obstacle enters target zone
 *   - Backs away from front obstacle using PID to maintain target distance
 *   - If rear obstacle appears while reversing, stops then drives forward
 *   - Dead-band prevents jitter around the target distance
 *   - Moving-average filter + spike rejection on both sensors
 *   - Emergency stop when either obstacle is dangerously close
 *
 * ============================================================
 * PIN MAP  (adjust to match your custom PCB if needed)
 * ============================================================
 *  PA2  USART2_TX  Debug output to ST-Link virtual COM port
 *  PA3  USART2_RX  (optional)
 *  PA5  TRIG_FRONT HC-SR04 front trigger (10 µs pulse out)
 *  PA6  ECHO_FRONT HC-SR04 front echo   (EXTI6, both edges)
 *  PA7  TRIG_REAR  HC-SR04 rear trigger  (10 µs pulse out)
 *  PB0  ECHO_REAR  HC-SR04 rear echo    (EXTI0, both edges)
 *  PB4  ENA        L298N Motor-A PWM    (TIM3_CH1, AF1)
 *  PB5  ENB        L298N Motor-B PWM    (TIM3_CH2, AF1)
 *  PC0  IN1        L298N Motor-A dir +
 *  PC1  IN2        L298N Motor-A dir -
 *  PC2  IN3        L298N Motor-B dir +
 *  PC3  IN4        L298N Motor-B dir -
 * ============================================================
 *
 * TIMERS:
 *  TIM2  1 MHz free-running counter  pulse-width measurement
 *  TIM3  1 kHz PWM                   motor speed (CCR1=ENA, CCR2=ENB)
 *
 * USART2: 115200-8N1, output format each control tick:
 *  "F:XX R:XX ERR:XX PWM:XX DIR:X\r\n"
 ******************************************************************************
 */

#include <stdint.h>

/* ===========================================================================
 * TUNABLE PARAMETERS  <-- start here when adjusting behavior
 * ===========================================================================*/
#define TARGET_DIST_CM   20u   /* desired gap to each obstacle (cm)           */
#define DEAD_BAND_CM      2u   /* ±cm tolerance, no correction inside window  */
#define EMERG_DIST_CM     6u   /* emergency stop: obstacle this close or less */
#define MAX_VALID_CM    250u   /* ignore sensor readings beyond this distance */
#define FILTER_LEN        6u   /* moving-average window (samples)             */
#define SPIKE_LIMIT_CM   40u   /* reject reading if it jumps more than this   */

/* PID gains — start with KI=KD=0, tune KP first                             */
#define KP   12.0f
#define KI    0.3f
#define KD    4.0f

/* PWM: TIM3 ARR = PWM_PERIOD-1, so duty = CCR / PWM_PERIOD                  */
#define PWM_PERIOD      1000u  /* 1 kHz with 1 µs timer tick                  */
#define PWM_MIN_DRIVE    300u  /* minimum duty to overcome motor dead-band     */
#define PWM_MAX_DRIVE    900u  /* cap to avoid mechanical stress               */

/* Timing (milliseconds)                                                       */
#define TRIGGER_INTERVAL_MS  30u   /* fire one sensor every 30 ms (alt front/rear) */
#define CONTROL_INTERVAL_MS  60u   /* PID loop period (matches one full sensor cycle) */
#define ECHO_TIMEOUT_MS      25u   /* give up waiting for echo after this          */

/* ===========================================================================
 * REGISTER BASE ADDRESSES  (STM32F072)
 * ===========================================================================*/
#define PERIPH_BASE     0x40000000UL
#define APB1_BASE       (PERIPH_BASE)
#define APB2_BASE       (PERIPH_BASE + 0x00010000UL)
#define AHB1_BASE       (PERIPH_BASE + 0x00020000UL)
#define AHB2_BASE       0x48000000UL

/* RCC */
#define RCC_BASE        (AHB1_BASE + 0x1000UL)
#define RCC_CR          (*(volatile uint32_t*)(RCC_BASE + 0x00U))
#define RCC_CFGR        (*(volatile uint32_t*)(RCC_BASE + 0x04U))
#define RCC_AHBENR      (*(volatile uint32_t*)(RCC_BASE + 0x14U))
#define RCC_APB2ENR     (*(volatile uint32_t*)(RCC_BASE + 0x18U))
#define RCC_APB1ENR     (*(volatile uint32_t*)(RCC_BASE + 0x1CU))

/* Flash (for wait-state config) */
#define FLASH_ACR       (*(volatile uint32_t*)0x40022000UL)

/* GPIO — helper macros take base address */
#define GPIOA_BASE      (AHB2_BASE + 0x0000UL)
#define GPIOB_BASE      (AHB2_BASE + 0x0400UL)
#define GPIOC_BASE      (AHB2_BASE + 0x0800UL)
#define GPIO_MODER(b)   (*(volatile uint32_t*)((b) + 0x00U))
#define GPIO_OTYPER(b)  (*(volatile uint32_t*)((b) + 0x04U))
#define GPIO_OSPEEDR(b) (*(volatile uint32_t*)((b) + 0x08U))
#define GPIO_PUPDR(b)   (*(volatile uint32_t*)((b) + 0x0CU))
#define GPIO_IDR(b)     (*(volatile uint32_t*)((b) + 0x10U))
#define GPIO_ODR(b)     (*(volatile uint32_t*)((b) + 0x14U))
#define GPIO_BSRR(b)    (*(volatile uint32_t*)((b) + 0x18U))
#define GPIO_AFR0(b)    (*(volatile uint32_t*)((b) + 0x20U))
#define GPIO_AFR1(b)    (*(volatile uint32_t*)((b) + 0x24U))

/* SYSCFG + EXTI */
#define SYSCFG_BASE     (APB2_BASE + 0x0000UL)
#define SYSCFG_EXTICR1  (*(volatile uint32_t*)(SYSCFG_BASE + 0x08U))
#define SYSCFG_EXTICR2  (*(volatile uint32_t*)(SYSCFG_BASE + 0x0CU))
#define EXTI_BASE       (APB2_BASE + 0x0400UL)
#define EXTI_IMR        (*(volatile uint32_t*)(EXTI_BASE + 0x00U))
#define EXTI_RTSR       (*(volatile uint32_t*)(EXTI_BASE + 0x08U))
#define EXTI_FTSR       (*(volatile uint32_t*)(EXTI_BASE + 0x0CU))
#define EXTI_PR         (*(volatile uint32_t*)(EXTI_BASE + 0x14U))

/* TIM2 — 32-bit free-running µs counter */
#define TIM2_BASE       (APB1_BASE + 0x0000UL)
#define TIM2_CR1        (*(volatile uint32_t*)(TIM2_BASE + 0x00U))
#define TIM2_EGR        (*(volatile uint32_t*)(TIM2_BASE + 0x14U))
#define TIM2_CNT        (*(volatile uint32_t*)(TIM2_BASE + 0x24U))
#define TIM2_PSC        (*(volatile uint32_t*)(TIM2_BASE + 0x28U))
#define TIM2_ARR        (*(volatile uint32_t*)(TIM2_BASE + 0x2CU))

/* TIM3 — PWM output */
#define TIM3_BASE       (APB1_BASE + 0x0400UL)
#define TIM3_CR1        (*(volatile uint32_t*)(TIM3_BASE + 0x00U))
#define TIM3_CCMR1      (*(volatile uint32_t*)(TIM3_BASE + 0x18U))
#define TIM3_CCER       (*(volatile uint32_t*)(TIM3_BASE + 0x20U))
#define TIM3_EGR        (*(volatile uint32_t*)(TIM3_BASE + 0x14U))
#define TIM3_PSC        (*(volatile uint32_t*)(TIM3_BASE + 0x28U))
#define TIM3_ARR        (*(volatile uint32_t*)(TIM3_BASE + 0x2CU))
#define TIM3_CCR1       (*(volatile uint32_t*)(TIM3_BASE + 0x34U))
#define TIM3_CCR2       (*(volatile uint32_t*)(TIM3_BASE + 0x38U))

/* USART2 */
#define USART2_BASE     (APB1_BASE + 0x4400UL)
#define USART2_CR1      (*(volatile uint32_t*)(USART2_BASE + 0x00U))
#define USART2_BRR      (*(volatile uint32_t*)(USART2_BASE + 0x0CU))
#define USART2_ISR      (*(volatile uint32_t*)(USART2_BASE + 0x1CU))
#define USART2_TDR      (*(volatile uint32_t*)(USART2_BASE + 0x28U))

/* SysTick */
#define SYSTICK_CTRL    (*(volatile uint32_t*)0xE000E010UL)
#define SYSTICK_LOAD    (*(volatile uint32_t*)0xE000E014UL)
#define SYSTICK_VAL     (*(volatile uint32_t*)0xE000E018UL)

/* NVIC */
#define NVIC_ISER0      (*(volatile uint32_t*)0xE000E100UL)

/* Clock frequency after PLL configuration */
#define SYSCLK_HZ       48000000UL

/* ===========================================================================
 * GLOBAL VOLATILE STATE (shared between main and ISRs)
 * ===========================================================================*/
static volatile uint32_t g_ms         = 0;   /* millisecond tick              */

/* Front sensor echo state */
static volatile uint32_t g_front_rise = 0;   /* TIM2 capture on rising edge   */
static volatile uint32_t g_front_us   = 0;   /* measured pulse width (µs)     */
static volatile uint8_t  g_front_busy = 0;   /* 1 = waiting for echo          */
static volatile uint8_t  g_front_rdy  = 0;   /* 1 = new measurement available */

/* Rear sensor echo state */
static volatile uint32_t g_rear_rise  = 0;
static volatile uint32_t g_rear_us    = 0;
static volatile uint8_t  g_rear_busy  = 0;
static volatile uint8_t  g_rear_rdy   = 0;

/* ===========================================================================
 * STATIC STATE (used only in main loop)
 * ===========================================================================*/
/* Moving-average filter buffers (store distance in cm) */
static uint32_t s_front_buf[FILTER_LEN];
static uint32_t s_rear_buf[FILTER_LEN];
static uint8_t  s_front_idx = 0;
static uint8_t  s_rear_idx  = 0;

/* Current filtered distances */
static uint32_t s_front_cm = TARGET_DIST_CM;
static uint32_t s_rear_cm  = TARGET_DIST_CM;

/* PID state */
static float s_integral  = 0.0f;
static float s_prev_err  = 0.0f;
static int8_t s_last_dir = 0;   /* +1=fwd, -1=bwd, 0=stop — reset I on dir change */

/* ===========================================================================
 * FORWARD DECLARATIONS
 * ===========================================================================*/
static void     clock_init(void);
static void     systick_init(void);
static void     gpio_init(void);
static void     exti_init(void);
static void     tim2_init(void);
static void     tim3_pwm_init(void);
static void     usart2_init(void);
static void     delay_us(uint32_t us);
static uint32_t get_ms(void);
static void     trigger_front(void);
static void     trigger_rear(void);
static uint32_t filter_insert(uint32_t *buf, uint8_t *idx, uint32_t val_cm,
                               uint32_t prev_cm);
static float    pid_update(float error, float dt_s);
static void     motor_forward(uint32_t pwm);
static void     motor_backward(uint32_t pwm);
static void     motor_stop(void);
static void     usart_putc(char c);
static void     usart_puts(const char *s);
static void     usart_put_u32(uint32_t v);
static void     usart_put_i32(int32_t v);

/* ===========================================================================
 * SYSTEM CLOCK — HSI/2 × PLL×12 = 4 MHz × 12 = 48 MHz
 * ===========================================================================*/
/* Called by startup assembly before main() */
void SystemInit(void)
{
    /* 1. Make sure HSI is on and stable */
    RCC_CR |= (1U << 0);                    /* HSION */
    while (!(RCC_CR & (1U << 1)));          /* wait HSIRDY */

    /* 2. Set 1 flash wait-state before raising clock above 24 MHz */
    FLASH_ACR = (FLASH_ACR & ~0x7U) | 0x1U;

    /* 3. Configure PLL: source = HSI/2 (4 MHz), multiplier = ×12 = 48 MHz   *
     *    PLLSRC [bit 16] = 0  → HSI/2                                        *
     *    PLLMUL [21:18]  = 0b1010 = 0xA → ×12                                */
    RCC_CFGR &= ~((0xFU << 18) | (1U << 16) | 0x3U);
    RCC_CFGR |=  (0xAU << 18);             /* PLLMUL = ×12 */

    /* 4. Enable PLL and wait for lock */
    RCC_CR |= (1U << 24);                   /* PLLON */
    while (!(RCC_CR & (1U << 25)));         /* wait PLLRDY */

    /* 5. Switch system clock to PLL */
    RCC_CFGR = (RCC_CFGR & ~0x3U) | 0x2U; /* SW = PLL */
    while ((RCC_CFGR & (0x3U << 2)) != (0x2U << 2)); /* wait SWS = PLL */
}

/* ===========================================================================
 * MAIN
 * ===========================================================================*/
int main(void)
{
    clock_init();       /* peripheral clocks                    */
    systick_init();     /* 1 ms SysTick interrupt               */
    gpio_init();        /* all GPIO directions / AF             */
    tim2_init();        /* 1 MHz free-running µs counter        */
    tim3_pwm_init();    /* 1 kHz PWM on PB4 (ENA) & PB5 (ENB)  */
    usart2_init();      /* 115200-8N1 debug USART               */
    exti_init();        /* ECHO_FRONT (PA6) & ECHO_REAR (PB0)   */

    /* Pre-fill filter buffers with nominal distance so first readings are sane */
    for (uint8_t i = 0; i < FILTER_LEN; i++) {
        s_front_buf[i] = TARGET_DIST_CM;
        s_rear_buf[i]  = TARGET_DIST_CM;
    }

    motor_stop();
    usart_puts("ACC SmartCar READY\r\n");

    uint32_t last_trig_ms    = 0;
    uint32_t last_ctrl_ms    = 0;
    uint8_t  trig_front_turn = 1;   /* alternate which sensor fires */

    while (1)
    {
        uint32_t now = get_ms();

        /* ------------------------------------------------------------------ *
         * 1. SENSOR TRIGGERING  (alternate front / rear every 30 ms)         *
         * ------------------------------------------------------------------ */
        if ((now - last_trig_ms) >= TRIGGER_INTERVAL_MS) {
            last_trig_ms = now;
            if (trig_front_turn) {
                trigger_front();
            } else {
                trigger_rear();
            }
            trig_front_turn ^= 1U;
        }

        /* ------------------------------------------------------------------ *
         * 2. PROCESS COMPLETED ECHO MEASUREMENTS                             *
         * ------------------------------------------------------------------ */
        if (g_front_rdy) {
            g_front_rdy = 0;
            /* Convert pulse width to distance: d(cm) = width(µs) / 58       */
            uint32_t raw_cm = g_front_us / 58U;
            if (raw_cm >= 2U && raw_cm <= MAX_VALID_CM) {
                s_front_cm = filter_insert(s_front_buf, &s_front_idx,
                                           raw_cm, s_front_cm);
            }
            /* Out-of-range: keep previous value (safe fallback)              */
        }

        if (g_rear_rdy) {
            g_rear_rdy = 0;
            uint32_t raw_cm = g_rear_us / 58U;
            if (raw_cm >= 2U && raw_cm <= MAX_VALID_CM) {
                s_rear_cm = filter_insert(s_rear_buf, &s_rear_idx,
                                          raw_cm, s_rear_cm);
            }
        }

        /* ------------------------------------------------------------------ *
         * 3. PID CONTROL LOOP  (runs every CONTROL_INTERVAL_MS)             *
         *                                                                     *
         * PRIORITY:                                                           *
         *  a) EMERGENCY: either obstacle <= EMERG_DIST_CM → full stop        *
         *  b) FRONT too close → back away (PID on front error)               *
         *     - but if rear is also at/past target → stop (nowhere to go)    *
         *  c) REAR too close → move forward (PID on rear error)              *
         *     - but if front is also at/past target → stop                   *
         *  d) Both within dead-band → stop                                   *
         * ------------------------------------------------------------------ */
        if ((now - last_ctrl_ms) >= CONTROL_INTERVAL_MS) {
            float dt = (float)CONTROL_INTERVAL_MS * 0.001f;
            last_ctrl_ms = now;

            /* Signed errors (positive = obstacle is too close)               */
            int32_t front_err_cm = (int32_t)TARGET_DIST_CM - (int32_t)s_front_cm;
            int32_t rear_err_cm  = (int32_t)TARGET_DIST_CM - (int32_t)s_rear_cm;

            uint32_t pwm = 0;

            /* (a) Emergency stop */
            if (s_front_cm <= EMERG_DIST_CM || s_rear_cm <= EMERG_DIST_CM) {
                motor_stop();
                s_integral   = 0.0f;
                s_prev_err   = 0.0f;
                s_last_dir   = 0;
            }
            /* (b) Front too close → move backward */
            else if (front_err_cm > (int32_t)DEAD_BAND_CM) {
                /* Nowhere to go if rear obstacle is already at target or closer */
                if (rear_err_cm > 0) {
                    /* Squeezed between two obstacles — stop */
                    motor_stop();
                    s_integral = 0.0f;
                    s_prev_err = 0.0f;
                    s_last_dir = 0;
                } else {
                    if (s_last_dir != -1) {
                        /* Direction changed — reset integrator to avoid windup */
                        s_integral = 0.0f;
                        s_prev_err = 0.0f;
                        s_last_dir = -1;
                    }
                    float out = pid_update((float)front_err_cm, dt);
                    pwm = (uint32_t)(out < 0.0f ? -out : out);
                    if (pwm < PWM_MIN_DRIVE) pwm = PWM_MIN_DRIVE;
                    if (pwm > PWM_MAX_DRIVE) pwm = PWM_MAX_DRIVE;
                    motor_backward(pwm);
                }
            }
            /* (c) Rear too close → move forward */
            else if (rear_err_cm > (int32_t)DEAD_BAND_CM) {
                if (front_err_cm > 0) {
                    motor_stop();
                    s_integral = 0.0f;
                    s_prev_err = 0.0f;
                    s_last_dir = 0;
                } else {
                    if (s_last_dir != 1) {
                        s_integral = 0.0f;
                        s_prev_err = 0.0f;
                        s_last_dir = 1;
                    }
                    float out = pid_update((float)rear_err_cm, dt);
                    pwm = (uint32_t)(out < 0.0f ? -out : out);
                    if (pwm < PWM_MIN_DRIVE) pwm = PWM_MIN_DRIVE;
                    if (pwm > PWM_MAX_DRIVE) pwm = PWM_MAX_DRIVE;
                    motor_forward(pwm);
                }
            }
            /* (d) Both within dead-band → stop */
            else {
                motor_stop();
                s_integral = 0.0f;
                s_prev_err = 0.0f;
                s_last_dir = 0;
            }

            /* USART telemetry */
            usart_puts("F:");  usart_put_u32(s_front_cm);
            usart_puts(" R:"); usart_put_u32(s_rear_cm);
            usart_puts(" Fe:");usart_put_i32(front_err_cm);
            usart_puts(" Re:");usart_put_i32(rear_err_cm);
            usart_puts(" P:");  usart_put_u32(pwm);
            usart_puts(" D:");
            if      (s_last_dir ==  1) usart_puts("FWD");
            else if (s_last_dir == -1) usart_puts("BWD");
            else                        usart_puts("STP");
            usart_puts("\r\n");
        }
    }
}

/* ===========================================================================
 * INITIALIZATION FUNCTIONS
 * ===========================================================================*/

static void clock_init(void)
{
    /* Enable clocks for used peripherals                                      *
     * AHB:  GPIOA(bit17), GPIOB(bit18), GPIOC(bit19)                        *
     * APB2: SYSCFG(bit0)                                                     *
     * APB1: TIM2(bit0), TIM3(bit1), USART2(bit17)                           */
    RCC_AHBENR  |= (1U << 17) | (1U << 18) | (1U << 19);
    RCC_APB2ENR |= (1U << 0);
    RCC_APB1ENR |= (1U << 0) | (1U << 1) | (1U << 17);
}

static void systick_init(void)
{
    SYSTICK_LOAD = SYSCLK_HZ / 1000U - 1U;  /* 47999 for 1 ms tick           */
    SYSTICK_VAL  = 0U;
    SYSTICK_CTRL = 0x7U;                     /* enable, tick IRQ, CPU clock   */
}

static void gpio_init(void)
{
    /* --- PA2: USART2_TX (AF1), PA3: USART2_RX (AF1) ---------------------- */
    /* MODER: 10 = alternate function */
    GPIO_MODER(GPIOA_BASE) &= ~((3U << 4) | (3U << 6));
    GPIO_MODER(GPIOA_BASE) |=  ((2U << 4) | (2U << 6));
    /* AFR0: PA2=AF1 [11:8], PA3=AF1 [15:12] */
    GPIO_AFR0(GPIOA_BASE) &= ~((0xFU << 8) | (0xFU << 12));
    GPIO_AFR0(GPIOA_BASE) |=  ((1U   << 8) | (1U   << 12));

    /* --- PA5: TRIG_FRONT (output push-pull) ------------------------------- */
    GPIO_MODER(GPIOA_BASE) &= ~(3U << 10);
    GPIO_MODER(GPIOA_BASE) |=  (1U << 10);  /* output */
    GPIO_BSRR(GPIOA_BASE)   = (1U << 21);   /* default LOW (bit 16+5 = reset) */

    /* --- PA6: ECHO_FRONT (input, no pull) --------------------------------- */
    GPIO_MODER(GPIOA_BASE) &= ~(3U << 12);  /* 00 = input */
    GPIO_PUPDR(GPIOA_BASE) &= ~(3U << 12);  /* no pull */

    /* --- PA7: TRIG_REAR (output push-pull) -------------------------------- */
    GPIO_MODER(GPIOA_BASE) &= ~(3U << 14);
    GPIO_MODER(GPIOA_BASE) |=  (1U << 14);
    GPIO_BSRR(GPIOA_BASE)   = (1U << 23);   /* default LOW */

    /* --- PB0: ECHO_REAR (input, no pull) ---------------------------------- */
    GPIO_MODER(GPIOB_BASE) &= ~(3U << 0);
    GPIO_PUPDR(GPIOB_BASE) &= ~(3U << 0);

    /* --- PB4: TIM3_CH1 (AF1), PB5: TIM3_CH2 (AF1) ----------------------- */
    GPIO_MODER(GPIOB_BASE) &= ~((3U << 8) | (3U << 10));
    GPIO_MODER(GPIOB_BASE) |=  ((2U << 8) | (2U << 10));
    /* AFR0: PB4=AF1 [19:16], PB5=AF1 [23:20] */
    GPIO_AFR0(GPIOB_BASE)  &= ~((0xFU << 16) | (0xFU << 20));
    GPIO_AFR0(GPIOB_BASE)  |=  ((1U   << 16) | (1U   << 20));

    /* --- PC0-PC3: Motor direction outputs (L298N IN1-IN4) ---------------- */
    GPIO_MODER(GPIOC_BASE) &= ~0xFFU;
    GPIO_MODER(GPIOC_BASE) |=  0x55U;  /* 01 01 01 01 = all output */
    GPIO_ODR(GPIOC_BASE)   &= ~0xFU;   /* all LOW (motors off) */
}

static void exti_init(void)
{
    /* EXTI0 ← PB0 (ECHO_REAR):  SYSCFG_EXTICR1 bits [3:0]  = 0001 (GPIOB) */
    SYSCFG_EXTICR1 = (SYSCFG_EXTICR1 & ~0xFU) | 0x1U;

    /* EXTI6 ← PA6 (ECHO_FRONT): SYSCFG_EXTICR2 bits [11:8] = 0000 (GPIOA) */
    SYSCFG_EXTICR2 = (SYSCFG_EXTICR2 & ~(0xFU << 8)) | (0x0U << 8);

    /* Trigger on both rising and falling edges */
    EXTI_RTSR |= (1U << 0) | (1U << 6);
    EXTI_FTSR |= (1U << 0) | (1U << 6);

    /* Unmask lines */
    EXTI_IMR  |= (1U << 0) | (1U << 6);

    /* NVIC enable:
     *   EXTI0_1  = IRQ position 5
     *   EXTI4_15 = IRQ position 7                                            */
    NVIC_ISER0 |= (1U << 5) | (1U << 7);
}

static void tim2_init(void)
{
    /* 1 MHz free-running 32-bit counter for µs pulse measurement             */
    TIM2_CR1 = 0U;
    TIM2_PSC = SYSCLK_HZ / 1000000U - 1U;  /* 47 → 1 MHz */
    TIM2_ARR = 0xFFFFFFFFU;
    TIM2_EGR = 1U;                          /* load PSC/ARR now */
    TIM2_CR1 = 1U;                          /* CEN */
}

static void tim3_pwm_init(void)
{
    /* 1 kHz PWM on CH1 (PB4=ENA) and CH2 (PB5=ENB)                         */
    TIM3_CR1  = 0U;
    TIM3_PSC  = SYSCLK_HZ / 1000000U - 1U; /* 47 → 1 MHz timer tick */
    TIM3_ARR  = PWM_PERIOD - 1U;            /* 999 → 1 kHz */
    TIM3_CCR1 = 0U;
    TIM3_CCR2 = 0U;

    /* CCMR1: OC1M=PWM1(110), OC1PE=1, OC2M=PWM1(110), OC2PE=1             */
    TIM3_CCMR1 = (0x6U << 4) | (1U << 3) | (0x6U << 12) | (1U << 11);

    /* CCER: CC1E=1, CC2E=1 */
    TIM3_CCER  = (1U << 0) | (1U << 4);

    TIM3_EGR   = 1U;
    TIM3_CR1   = 1U;  /* CEN */
}

static void usart2_init(void)
{
    /* 115200 baud, 8N1.  BRR = PCLK / baud = 48 000 000 / 115200 ≈ 417     */
    USART2_BRR = 417U;
    USART2_CR1 = (1U << 3) | (1U << 0);  /* TE=1, UE=1 */
}

/* ===========================================================================
 * ULTRASONIC SENSOR HELPERS
 * ===========================================================================*/

/* Blocking µs delay using TIM2 counter — only used for 10 µs trigger pulse  */
static void delay_us(uint32_t us)
{
    uint32_t start = TIM2_CNT;
    while ((TIM2_CNT - start) < us) { /* spin */ }
}

static uint32_t get_ms(void)
{
    return g_ms;
}

static void trigger_front(void)
{
    if (g_front_busy) return;          /* previous measurement still pending  */
    g_front_rdy  = 0;
    g_front_busy = 1;
    /* 10 µs HIGH pulse on PA5 */
    GPIO_BSRR(GPIOA_BASE) = (1U << 5);   /* set PA5 */
    delay_us(10U);
    GPIO_BSRR(GPIOA_BASE) = (1U << 21);  /* reset PA5 (bit 16+5) */
}

static void trigger_rear(void)
{
    if (g_rear_busy) return;
    g_rear_rdy  = 0;
    g_rear_busy = 1;
    /* 10 µs HIGH pulse on PA7 */
    GPIO_BSRR(GPIOA_BASE) = (1U << 7);   /* set PA7 */
    delay_us(10U);
    GPIO_BSRR(GPIOA_BASE) = (1U << 23);  /* reset PA7 (bit 16+7) */
}

/* ===========================================================================
 * MOVING-AVERAGE FILTER  with spike rejection
 * ===========================================================================*/
static uint32_t filter_insert(uint32_t *buf, uint8_t *idx,
                               uint32_t val_cm, uint32_t prev_cm)
{
    /* Spike filter: if jump is huge, discard this sample entirely             */
    uint32_t diff = (val_cm > prev_cm) ? (val_cm - prev_cm) : (prev_cm - val_cm);
    if (diff > SPIKE_LIMIT_CM) {
        return prev_cm;  /* keep previous filtered value */
    }

    buf[*idx] = val_cm;
    *idx = (uint8_t)((*idx + 1U) % FILTER_LEN);

    uint32_t sum = 0U;
    for (uint8_t i = 0; i < FILTER_LEN; i++) sum += buf[i];
    return sum / FILTER_LEN;
}

/* ===========================================================================
 * PID CONTROLLER
 *
 * Input:  signed error in cm (positive = obstacle too close)
 * Output: control effort magnitude (always positive; caller sets direction)
 * ===========================================================================*/
static float pid_update(float error, float dt_s)
{
    s_integral += error * dt_s;

    /* Anti-windup: clamp integral so it can't produce more than PWM_MAX_DRIVE */
    float i_max = (float)PWM_MAX_DRIVE / (KI > 0.001f ? KI : 0.001f);
    if (s_integral >  i_max) s_integral =  i_max;
    if (s_integral < -i_max) s_integral = -i_max;

    float derivative = (dt_s > 1e-6f) ? (error - s_prev_err) / dt_s : 0.0f;
    s_prev_err = error;

    return KP * error + KI * s_integral + KD * derivative;
}

/* ===========================================================================
 * MOTOR CONTROL
 *
 * Forward:  IN1=H IN2=L IN3=H IN4=L  (both motors spin same way)
 * Backward: IN1=L IN2=H IN3=L IN4=H
 * Stop:     all INx = L, PWM duty = 0
 *
 * PC0=IN1, PC1=IN2, PC2=IN3, PC3=IN4
 * ===========================================================================*/
static void motor_forward(uint32_t pwm)
{
    /* PC[3:0] = 0b0101 */
    GPIO_ODR(GPIOC_BASE) = (GPIO_ODR(GPIOC_BASE) & ~0xFU) | 0x5U;
    TIM3_CCR1 = pwm;
    TIM3_CCR2 = pwm;
}

static void motor_backward(uint32_t pwm)
{
    /* PC[3:0] = 0b1010 */
    GPIO_ODR(GPIOC_BASE) = (GPIO_ODR(GPIOC_BASE) & ~0xFU) | 0xAU;
    TIM3_CCR1 = pwm;
    TIM3_CCR2 = pwm;
}

static void motor_stop(void)
{
    GPIO_ODR(GPIOC_BASE) &= ~0xFU;
    TIM3_CCR1 = 0U;
    TIM3_CCR2 = 0U;
}

/* ===========================================================================
 * USART2 HELPERS  (blocking, fine for low-rate telemetry)
 * ===========================================================================*/
static void usart_putc(char c)
{
    while (!(USART2_ISR & (1U << 7)));  /* wait TXE */
    USART2_TDR = (uint8_t)c;
}

static void usart_puts(const char *s)
{
    while (*s) usart_putc(*s++);
}

static void usart_put_u32(uint32_t v)
{
    char buf[11];
    uint8_t i = 0;
    if (v == 0U) { usart_putc('0'); return; }
    while (v > 0U) { buf[i++] = (char)('0' + (v % 10U)); v /= 10U; }
    while (i > 0U) usart_putc(buf[--i]);
}

static void usart_put_i32(int32_t v)
{
    if (v < 0) { usart_putc('-'); usart_put_u32((uint32_t)(-v)); }
    else        { usart_put_u32((uint32_t)v); }
}

/* ===========================================================================
 * INTERRUPT HANDLERS
 * ===========================================================================*/

/* SysTick — 1 ms tick + sensor echo timeout watchdog                        */
void SysTick_Handler(void)
{
    g_ms++;

    /* Timeout watchdog: if echo never came, release busy flag after 25 ms    *
     * keeping the previous distance value (safe fallback)                    */
    static uint32_t front_wait = 0U;
    static uint32_t rear_wait  = 0U;

    if (g_front_busy) {
        if (++front_wait >= ECHO_TIMEOUT_MS) {
            g_front_busy = 0;
            front_wait   = 0U;
            /* g_front_rdy stays 0: main loop will reuse old s_front_cm      */
        }
    } else {
        front_wait = 0U;
    }

    if (g_rear_busy) {
        if (++rear_wait >= ECHO_TIMEOUT_MS) {
            g_rear_busy = 0;
            rear_wait   = 0U;
        }
    } else {
        rear_wait = 0U;
    }
}

/* EXTI0_1 — PB0: ECHO_REAR                                                  */
void EXTI0_1_IRQHandler(void)
{
    if (EXTI_PR & (1U << 0)) {
        EXTI_PR = (1U << 0);                    /* clear pending (write 1)    */
        if (GPIO_IDR(GPIOB_BASE) & (1U << 0)) {
            g_rear_rise = TIM2_CNT;             /* rising edge: start timing  */
        } else {
            if (g_rear_busy) {
                g_rear_us   = TIM2_CNT - g_rear_rise; /* falling: done       */
                g_rear_busy = 0;
                g_rear_rdy  = 1;
            }
        }
    }
}

/* EXTI4_15 — PA6: ECHO_FRONT                                                */
void EXTI4_15_IRQHandler(void)
{
    if (EXTI_PR & (1U << 6)) {
        EXTI_PR = (1U << 6);
        if (GPIO_IDR(GPIOA_BASE) & (1U << 6)) {
            g_front_rise = TIM2_CNT;
        } else {
            if (g_front_busy) {
                g_front_us   = TIM2_CNT - g_front_rise;
                g_front_busy = 0;
                g_front_rdy  = 1;
            }
        }
    }
}
