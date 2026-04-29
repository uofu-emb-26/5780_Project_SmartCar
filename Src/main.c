/**
 * STM32F072 — Forward until front HC-SR04 detects obstacle within 10 cm
 *
 * PIN MAP:
 *  PB4  ENABLE      TIM3_CH1 PWM (AF1)
 *  PA6  INPUT1      motor direction
 *  PA5  INPUT2      motor direction
 *  PA9  TRIG_FRONT  HC-SR04 trigger (10 µs pulse)
 *  PA1  ECHO_FRONT  HC-SR04 echo   (EXTI1, voltage divider 1kΩ+1kΩ required)
 *  PC0  TRIG_REAR   not used in this front-only test
 *  PB0  ECHO_REAR   not used in this front-only test
 *
 * BEHAVIOR:
 *  - Drives forward continuously
 *  - Stops when front sensor distance <= STOP_DIST_CM
 *  - Resumes forward when obstacle clears
 */

#include <stdint.h>

/* ---- tunable ------------------------------------------------------------ */
#define STOP_DIST_CM     10u
#define MAX_VALID_CM    250u
#define PWM_SPEED       700u   /* 0–999 */
#define TRIG_INTERVAL_MS 30u
#define ECHO_TIMEOUT_MS  25u

/* ---- register addresses ------------------------------------------------- */
#define PERIPH_BASE  0x40000000UL
#define AHB1_BASE    (PERIPH_BASE + 0x00020000UL)
#define AHB2_BASE    0x48000000UL
#define APB1_BASE    PERIPH_BASE
#define APB2_BASE    (PERIPH_BASE + 0x00010000UL)

#define RCC_BASE     (AHB1_BASE + 0x1000UL)
#define RCC_CR       (*(volatile uint32_t*)(RCC_BASE + 0x00U))
#define RCC_CFGR     (*(volatile uint32_t*)(RCC_BASE + 0x04U))
#define RCC_AHBENR   (*(volatile uint32_t*)(RCC_BASE + 0x14U))
#define RCC_APB2ENR  (*(volatile uint32_t*)(RCC_BASE + 0x18U))
#define RCC_APB1ENR  (*(volatile uint32_t*)(RCC_BASE + 0x1CU))

#define FLASH_ACR    (*(volatile uint32_t*)0x40022000UL)

#define GPIOA_BASE   (AHB2_BASE + 0x0000UL)
#define GPIOB_BASE   (AHB2_BASE + 0x0400UL)
#define GPIOC_BASE   (AHB2_BASE + 0x0800UL)
#define MODER(b)     (*(volatile uint32_t*)((b) + 0x00U))
#define PUPDR(b)     (*(volatile uint32_t*)((b) + 0x0CU))
#define IDR(b)       (*(volatile uint32_t*)((b) + 0x10U))
#define BSRR(b)      (*(volatile uint32_t*)((b) + 0x18U))
#define AFR0(b)      (*(volatile uint32_t*)((b) + 0x20U))  /* pins 0–7  */
#define AFR1(b)      (*(volatile uint32_t*)((b) + 0x24U))  /* pins 8–15 */

#define SYSCFG_BASE   (APB2_BASE + 0x0000UL)
#define SYSCFG_EXTICR1 (*(volatile uint32_t*)(SYSCFG_BASE + 0x08U))
#define EXTI_BASE     (APB2_BASE + 0x0400UL)
#define EXTI_IMR      (*(volatile uint32_t*)(EXTI_BASE + 0x00U))
#define EXTI_RTSR     (*(volatile uint32_t*)(EXTI_BASE + 0x08U))
#define EXTI_FTSR     (*(volatile uint32_t*)(EXTI_BASE + 0x0CU))
#define EXTI_PR       (*(volatile uint32_t*)(EXTI_BASE + 0x14U))

/* TIM2 — 32-bit free-running µs counter */
#define TIM2_BASE    (APB1_BASE + 0x0000UL)
#define TIM2_CR1     (*(volatile uint32_t*)(TIM2_BASE + 0x00U))
#define TIM2_EGR     (*(volatile uint32_t*)(TIM2_BASE + 0x14U))
#define TIM2_CNT     (*(volatile uint32_t*)(TIM2_BASE + 0x24U))
#define TIM2_PSC     (*(volatile uint32_t*)(TIM2_BASE + 0x28U))
#define TIM2_ARR     (*(volatile uint32_t*)(TIM2_BASE + 0x2CU))

/* TIM3 — PWM on PB4 (CH1) */
#define TIM3_BASE    (APB1_BASE + 0x0400UL)
#define TIM3_CR1     (*(volatile uint32_t*)(TIM3_BASE + 0x00U))
#define TIM3_CCMR1   (*(volatile uint32_t*)(TIM3_BASE + 0x18U))
#define TIM3_CCER    (*(volatile uint32_t*)(TIM3_BASE + 0x20U))
#define TIM3_EGR     (*(volatile uint32_t*)(TIM3_BASE + 0x14U))
#define TIM3_PSC     (*(volatile uint32_t*)(TIM3_BASE + 0x28U))
#define TIM3_ARR     (*(volatile uint32_t*)(TIM3_BASE + 0x2CU))
#define TIM3_CCR1    (*(volatile uint32_t*)(TIM3_BASE + 0x34U))

#define SYSTICK_CTRL (*(volatile uint32_t*)0xE000E010UL)
#define SYSTICK_LOAD (*(volatile uint32_t*)0xE000E014UL)
#define SYSTICK_VAL  (*(volatile uint32_t*)0xE000E018UL)
#define NVIC_ISER0   (*(volatile uint32_t*)0xE000E100UL)

#define SYSCLK_HZ    48000000UL

/* ---- shared ISR state --------------------------------------------------- */
static volatile uint32_t g_ms         = 0;
static volatile uint32_t g_front_rise = 0;
static volatile uint32_t g_front_us   = 0;
static volatile uint8_t  g_front_busy = 0;
static volatile uint8_t  g_front_rdy  = 0;

/* ---- PLL: HSI/2 × 12 = 48 MHz ------------------------------------------ */
static void pll_init(void)
{
    RCC_CR |= (1U << 0);
    while (!(RCC_CR & (1U << 1)));
    FLASH_ACR = (FLASH_ACR & ~0x7U) | 0x1U;
    RCC_CFGR &= ~((0xFU << 18) | (1U << 16) | 0x3U);
    RCC_CFGR |=  (0xAU << 18);
    RCC_CR |= (1U << 24);
    while (!(RCC_CR & (1U << 25)));
    RCC_CFGR = (RCC_CFGR & ~0x3U) | 0x2U;
    while ((RCC_CFGR & (0x3U << 2)) != (0x2U << 2));
}

static void clock_init(void)
{
    /* GPIOA(17), GPIOB(18), GPIOC(19) */
    RCC_AHBENR  |= (1U << 17) | (1U << 18) | (1U << 19);
    /* SYSCFG(0) */
    RCC_APB2ENR |= (1U << 0);
    /* TIM2(0), TIM3(1) */
    RCC_APB1ENR |= (1U << 0) | (1U << 1);
}

static void gpio_init(void)
{
    /* PA1: ECHO_FRONT — input, no pull */
    MODER(GPIOA_BASE) &= ~(3U << 2);
    PUPDR(GPIOA_BASE) &= ~(3U << 2);

    /* PB4: TIM3_CH1 AF1 — ENABLE PWM output */
    MODER(GPIOB_BASE) &= ~(3U << 8);
    MODER(GPIOB_BASE) |=  (2U << 8);
    AFR0(GPIOB_BASE)  &= ~(0xFU << 16);
    AFR0(GPIOB_BASE)  |=  (1U   << 16);

    /* PA9: TRIG_FRONT — output, default LOW */
    MODER(GPIOA_BASE) &= ~(3U << 18);
    MODER(GPIOA_BASE) |=  (1U << 18);
    BSRR(GPIOA_BASE)   = (1U << 25);          /* reset PA9 */

    /* keep rear sensor pins idle in this front-only test */
    MODER(GPIOB_BASE) &= ~(3U << 0);          /* PB0 input */
    PUPDR(GPIOB_BASE) &= ~(3U << 0);
    MODER(GPIOC_BASE) &= ~(3U << 0);          /* PC0 output low */
    MODER(GPIOC_BASE) |=  (1U << 0);
    BSRR(GPIOC_BASE)   = (1U << 16);

    /* PA6: INPUT1, PA5: INPUT2 — output, both LOW */
    MODER(GPIOA_BASE) &= ~((3U << 12) | (3U << 10));
    MODER(GPIOA_BASE) |=  ((1U << 12) | (1U << 10));
    BSRR(GPIOA_BASE)   = (1U << 22) | (1U << 21);
}

static void exti_init(void)
{
    /* EXTI1 ← PA1: SYSCFG_EXTICR1 bits [7:4] = 0000 (GPIOA) */
    SYSCFG_EXTICR1 = (SYSCFG_EXTICR1 & ~(0xFU << 4)) | (0x0U << 4);
    EXTI_RTSR |= (1U << 1);
    EXTI_FTSR |= (1U << 1);
    EXTI_IMR  |= (1U << 1);
    NVIC_ISER0 |= (1U << 5);   /* EXTI0_1 = IRQ 5 */
}

static void tim2_init(void)
{
    TIM2_CR1 = 0U;
    TIM2_PSC = SYSCLK_HZ / 1000000U - 1U;   /* 1 MHz */
    TIM2_ARR = 0xFFFFFFFFU;
    TIM2_EGR = 1U;
    TIM2_CR1 = 1U;
}

static void tim3_pwm_init(void)
{
    TIM3_CR1   = 0U;
    TIM3_PSC   = SYSCLK_HZ / 1000000U - 1U;
    TIM3_ARR   = 999U;
    TIM3_CCR1  = 0U;
    TIM3_CCMR1 = (0x6U << 4) | (1U << 3);
    TIM3_CCER  = (1U << 0);
    TIM3_EGR   = 1U;
    TIM3_CR1   = 1U;
}

/* ---- motor -------------------------------------------------------------- */
static void motor_forward(uint32_t pwm)
{
    /* INPUT1=L (PA6), INPUT2=H (PA5) */
    BSRR(GPIOA_BASE) =  (1U << 22) | (1U << 5);
    TIM3_CCR1 = pwm;
}

static void motor_stop(void)
{
    BSRR(GPIOA_BASE) = (1U << 22) | (1U << 21);
    TIM3_CCR1 = 0U;
}

/* ---- sensor ------------------------------------------------------------- */
static void delay_us(uint32_t us)
{
    uint32_t start = TIM2_CNT;
    while ((TIM2_CNT - start) < us) {}
}

static void trigger_front(void)
{
    if (g_front_busy) return;
    g_front_rdy  = 0;
    g_front_busy = 1;
    BSRR(GPIOA_BASE) = (1U << 9);    /* PA9 HIGH */
    delay_us(10U);
    BSRR(GPIOA_BASE) = (1U << 25);   /* PA9 LOW */
}

/* ---- main --------------------------------------------------------------- */
int main(void)
{
    pll_init();
    clock_init();

    /* SysTick 1 ms */
    SYSTICK_LOAD = SYSCLK_HZ / 1000U - 1U;
    SYSTICK_VAL  = 0U;
    SYSTICK_CTRL = 0x7U;

    gpio_init();
    tim2_init();
    tim3_pwm_init();
    exti_init();

    motor_stop();

    uint32_t last_trig = 0U;
    uint32_t front_cm  = MAX_VALID_CM + 1U;

    while (1)
    {
        uint32_t now = g_ms;

        /* fire front sensor every 30 ms */
        if ((now - last_trig) >= TRIG_INTERVAL_MS) {
            last_trig = now;
            trigger_front();
        }

        /* collect front echo result */
        if (g_front_rdy) {
            g_front_rdy = 0;
            uint32_t measured = g_front_us / 58U;
            if ((measured > 0U) && (measured <= MAX_VALID_CM)) {
                front_cm = measured;
            }
        }

        uint8_t front_blocked = (front_cm > 0U) && (front_cm <= STOP_DIST_CM);

        if (front_blocked) {
            motor_stop();
        } else {
            motor_forward(PWM_SPEED);
        }
    }
}

/* ---- ISRs --------------------------------------------------------------- */
void SysTick_Handler(void)
{
    g_ms++;

    static uint32_t front_wait  = 0U;

    if (g_front_busy) {
        if (++front_wait >= ECHO_TIMEOUT_MS) {
            g_front_busy = 0;
            front_wait = 0U;
        }
    } else {
        front_wait = 0U;
    }
}

/* EXTI0_1 — PA1 ECHO_FRONT */
void EXTI0_1_IRQHandler(void)
{
    if (EXTI_PR & (1U << 1)) {
        EXTI_PR = (1U << 1);
        if (IDR(GPIOA_BASE) & (1U << 1)) {
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
