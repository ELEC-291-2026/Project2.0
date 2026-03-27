#include "stm32l051xx.h"

#define ADC_CH_LEFT         4   /* PA4, physical pin 10 */
#define ADC_CH_RIGHT        5   /* PA5, physical pin 11 */
#define ADC_CH_INTERSECT    6   /* PA6, physical pin 12 */

#define BASE_SPEED          600
#define TURN_SPEED          500
#define MAX_PWM             1000
#define KP                  2
#define DEADBAND            15

#define ENTRY_SIGNAL        10
#define EXIT_SIGNAL         5
#define INTERSECT_ENTRY     20
#define INTERSECT_EXIT      10

#define F_CPU 32000000UL

static void wait_1ms(void)
{
    SysTick->LOAD = (F_CPU / 1000UL) - 1UL;
    SysTick->VAL  = 0;
    SysTick->CTRL = 0x5;
    while ((SysTick->CTRL & (1 << 16)) == 0);
    SysTick->CTRL = 0;
}

static void delayms(unsigned int ms)
{
    while (ms--) wait_1ms();
}

static void clock_init(void)
{
    /* Enable HSI16 oscillator */
    RCC->CR |= (1 << 0);
    while (!(RCC->CR & (1 << 2)));

    /* PLL: source=HSI16, MUL=×4, DIV=÷2 → 32 MHz */
    RCC->CFGR &= ~((0xFU << 18) | (0x3U << 22) | (1U << 16));
    RCC->CFGR |=  (0x1U << 18) |  /* MUL ×4 */
                  (0x1U << 22);    /* DIV ÷2, SRC=HSI16 */

    /* Enable PLL and wait */
    RCC->CR |= (1 << 24);
    while (!(RCC->CR & (1 << 25)));

    /* 1 flash wait state required at 32 MHz */
    FLASH->ACR |= (1 << 0);

    /* Switch sysclk to PLL */
    RCC->CFGR = (RCC->CFGR & ~0x3U) | 0x3U;
    while ((RCC->CFGR & (0x3U << 2)) != (0x3U << 2));
}

static void uart_init(void)
{
    /* Enable GPIOA and USART1 clocks */
    RCC->IOPENR  |= (1 << 0);
    RCC->APB2ENR |= (1 << 14);

    /* PA9 = USART1_TX, AF4 */
    GPIOA->MODER  &= ~(3U << 18);
    GPIOA->MODER  |=  (2U << 18);
    GPIOA->AFR[1] &= ~(0xFU << 4);
    GPIOA->AFR[1] |=  (4U  << 4);

    /* 115200 baud at 32 MHz */
    USART1->BRR = 278;
    USART1->CR1 = (1 << 3) | (1 << 0);  /* TE | UE */
}

static void uart_putc(char c)
{
    while (!(USART1->ISR & (1 << 7)));
    USART1->TDR = (unsigned char)c;
}

static void uart_puts(const char *s)
{
    while (*s) uart_putc(*s++);
}

/* Print a signed integer right-justified in a field of `width` chars */
static void uart_print_int(int val, int width)
{
    char buf[12];
    int  len = 0;
    unsigned int uval;

    if (val < 0) { uart_putc('-'); uval = (unsigned int)(-val); width--; }
    else         { uval = (unsigned int)val; }

    if (uval == 0) { buf[len++] = '0'; }
    else { while (uval) { buf[len++] = '0' + (int)(uval % 10); uval /= 10; } }

    while (len < width) { uart_putc(' '); width--; }
    while (len > 0) uart_putc(buf[--len]);
}

static void pwm_init(void)
{
    RCC->IOPENR  |= (1 << 0);
    RCC->APB1ENR |= (1 << 0);

    GPIOA->MODER &= ~(0xFF);
    GPIOA->MODER |=  (0xAA);

    GPIOA->AFR[0] &= ~(0xFFFF);
    GPIOA->AFR[0] |=  (0x2222);

    TIM2->PSC  = 31;
    TIM2->ARR  = MAX_PWM - 1;
    TIM2->CCR1 = 0;
    TIM2->CCR2 = 0;
    TIM2->CCR3 = 0;
    TIM2->CCR4 = 0;

    TIM2->CCMR1 = (6 << 4) | (1 << 3) | (6 << 12) | (1 << 11);
    TIM2->CCMR2 = (6 << 4) | (1 << 3) | (6 << 12) | (1 << 11);
    TIM2->CCER  = (1<<0) | (1<<4) | (1<<8) | (1<<12);
    TIM2->EGR   = 1;
    TIM2->CR1   = (1 << 7) | (1 << 0);
}

static void motors_stop(void)
{
    TIM2->CCR1 = 0;
    TIM2->CCR2 = 0;
    TIM2->CCR3 = 0;
    TIM2->CCR4 = 0;
}

static void motors_set(int left, int right)
{
    motors_stop();
    if (left > 0)
        TIM2->CCR1 = (unsigned int)(left  > MAX_PWM ? MAX_PWM : left);
    else if (left < 0)
        TIM2->CCR2 = (unsigned int)((-left) > MAX_PWM ? MAX_PWM : -left);

    if (right > 0)
        TIM2->CCR3 = (unsigned int)(right  > MAX_PWM ? MAX_PWM : right);
    else if (right < 0)
        TIM2->CCR4 = (unsigned int)((-right) > MAX_PWM ? MAX_PWM : -right);
}

static void adc_init(void)
{
    RCC->IOPENR  |= (1 << 0);   /* GPIOA clock (shared with PWM) */
    RCC->APB2ENR |= (1 << 9);

    /* PA4, PA5, PA6 → analog mode */
    GPIOA->MODER |= (3 << 8) | (3 << 10) | (3 << 12);

    ADC1->CFGR2 |= (1 << 30);

    if (ADC1->CR & (1 << 0))
    {
        ADC1->CR |= (1 << 1);
        while (ADC1->CR & (1 << 0));
    }

    ADC1->CR |= (1 << 28);
    delayms(1);

    ADC1->CR |= (1 << 31);
    while (ADC1->CR & (1 << 31));

    ADC1->SMPR  = 0x5;
    ADC1->CFGR1 = 0;
    ADC1->CR   |= (1 << 0);
    while (!(ADC1->ISR & (1 << 0)));
}

static int adc_read(int channel)
{
    int sum;
    int i;

    sum = 0;
    ADC1->CHSELR = (1 << channel);
    for (i = 0; i < 4; i++)
    {
        ADC1->CR |= (1 << 2);
        while (!(ADC1->ISR & (1 << 2)));
        sum += (int)(ADC1->DR & 0xFFF);
    }
    return sum / 4;
}

typedef struct {
    int raw;
    int filtered;
    int baseline;
    int signal;
    int detected;
    unsigned int samples;
} sensor_ch_t;

static int mix(int prev, int sample, int k)
{
    return (k * prev + sample) / (k + 1);
}

static int absdiff(int a, int b)
{
    return a > b ? a - b : b - a;
}

static void update_ch(sensor_ch_t *ch, int sample, int entry, int exit_sig)
{
    int filt;
    int bk;
    int sig;
    int det;

    filt = mix(ch->filtered, sample, 3);
    bk   = (ch->samples < 16) ? 7 : 15;
    sig  = absdiff(filt, ch->baseline);

    if (ch->detected)
        det = (sig >= exit_sig);
    else
        det = (sig >= entry);

    if (!det)
        ch->baseline = mix(ch->baseline, filt, bk);

    sig = absdiff(filt, ch->baseline);
    if (ch->detected)
        det = (sig >= exit_sig);
    else
        det = (sig >= entry);

    ch->raw      = sample;
    ch->filtered = filt;
    ch->signal   = sig;
    ch->detected = det;
    if (ch->samples < 0xFFFFFFFFU) ch->samples++;
}

typedef enum { ST_FOLLOW, ST_INTERSECTION, ST_LOST, ST_STOP } state_t;

static const int k_paths[3][8] = {
    { 0, 1, 1, 0, 2, 1, 2, 3 },
    { 1, 2, 1, 2, 0, 0, 3, 3 },
    { 2, 0, 2, 1, 2, 1, 0, 3 }
};

static state_t g_state;
static int     g_action;

static void run_action(int action)
{
    switch (action)
    {
        case 1: motors_set(-TURN_SPEED,  TURN_SPEED); break;
        case 2: motors_set( TURN_SPEED, -TURN_SPEED); break;
        case 3: motors_stop();                         break;
        default: motors_set(BASE_SPEED, BASE_SPEED);  break;
    }
}

static void run_follow(int left_sig, int right_sig)
{
    int error;
    int correction;

    error = left_sig - right_sig;
    if (error > -DEADBAND && error < DEADBAND) error = 0;
    correction = KP * error;
    motors_set(BASE_SPEED - correction, BASE_SPEED + correction);
}

void main(void)
{
    sensor_ch_t left;
    sensor_ch_t right;
    sensor_ch_t intersect;
    int path;
    int ix_count;
    int ix_active;
    unsigned int i;
    unsigned int loop;

    left.raw = 0; left.filtered = 0; left.baseline = 0;
    left.signal = 0; left.detected = 0; left.samples = 0;

    right.raw = 0; right.filtered = 0; right.baseline = 0;
    right.signal = 0; right.detected = 0; right.samples = 0;

    intersect.raw = 0; intersect.filtered = 0; intersect.baseline = 0;
    intersect.signal = 0; intersect.detected = 0; intersect.samples = 0;

    path      = 0;
    ix_count  = 0;
    ix_active = 0;
    g_state   = ST_FOLLOW;
    g_action  = 0;
    loop      = 0;

    clock_init();
    uart_init();
    pwm_init();
    adc_init();
    motors_stop();

    delayms(500);
    uart_puts("\x1b[2J\x1b[1;1H");
    uart_puts("Robot starting up...\r\n");

    /* --- raw ADC dump: print 20 readings before any filtering --- */
    uart_puts("--- RAW ADC DUMP (no filtering) ---\r\n");
    for (i = 0; i < 20; i++)
    {
        uart_puts("L="); uart_print_int(adc_read(ADC_CH_LEFT),      4);
        uart_puts("  R="); uart_print_int(adc_read(ADC_CH_RIGHT),   4);
        uart_puts("  IX="); uart_print_int(adc_read(ADC_CH_INTERSECT), 4);
        uart_puts("\r\n");
        delayms(100);
    }
    uart_puts("--- END DUMP, starting warmup ---\r\n");

    /* warm up */
    for (i = 0; i < 64; i++)
    {
        update_ch(&left,      adc_read(ADC_CH_LEFT),      ENTRY_SIGNAL,    EXIT_SIGNAL);
        update_ch(&right,     adc_read(ADC_CH_RIGHT),     ENTRY_SIGNAL,    EXIT_SIGNAL);
        update_ch(&intersect, adc_read(ADC_CH_INTERSECT), INTERSECT_ENTRY, INTERSECT_EXIT);
        delayms(2);
    }

    uart_puts("Warmup done. Entering main loop.\r\n");

    while (1)
    {
        update_ch(&left,      adc_read(ADC_CH_LEFT),      ENTRY_SIGNAL,    EXIT_SIGNAL);
        update_ch(&right,     adc_read(ADC_CH_RIGHT),     ENTRY_SIGNAL,    EXIT_SIGNAL);
        update_ch(&intersect, adc_read(ADC_CH_INTERSECT), INTERSECT_ENTRY, INTERSECT_EXIT);

        if (loop % 5 == 0)
        {
            uart_puts("L r="); uart_print_int(left.raw,      4);
            uart_puts(" b=");  uart_print_int(left.baseline, 4);
            uart_puts(" s=");  uart_print_int(left.signal,   4);
            uart_puts(" d=");  uart_print_int(left.detected, 1);
            uart_puts(" | R r="); uart_print_int(right.raw,      4);
            uart_puts(" b=");     uart_print_int(right.baseline, 4);
            uart_puts(" s=");     uart_print_int(right.signal,   4);
            uart_puts(" d=");     uart_print_int(right.detected, 1);
            uart_puts(" | IX r="); uart_print_int(intersect.raw,      4);
            uart_puts(" s=");      uart_print_int(intersect.signal,   4);
            uart_puts(" d=");      uart_print_int(intersect.detected, 1);
            uart_puts(" | st=");   uart_print_int((int)g_state, 1);
            uart_puts("\r\n");
        }
        loop++;

        switch (g_state)
        {
            case ST_FOLLOW:
                if (!left.detected && !right.detected)
                {
                    motors_stop();
                    g_state = ST_LOST;
                    break;
                }
                if (intersect.detected && !ix_active)
                {
                    ix_active = 1;
                    g_action  = (ix_count < 8) ? k_paths[path][ix_count++] : 3;
                    g_state   = ST_INTERSECTION;
                    run_action(g_action);
                    break;
                }
                if (!intersect.detected) ix_active = 0;
                run_follow(left.signal, right.signal);
                break;

            case ST_INTERSECTION:
                if (!intersect.detected)
                {
                    ix_active = 0;
                    if (g_action == 3) { g_state = ST_STOP; motors_stop(); }
                    else               { g_state = ST_FOLLOW; }
                }
                else
                {
                    run_action(g_action);
                }
                break;

            case ST_LOST:
                if (left.detected || right.detected)
                {
                    g_state = ST_FOLLOW;
                    run_follow(left.signal, right.signal);
                }
                else
                {
                    motors_stop();
                }
                break;

            case ST_STOP:
            default:
                motors_stop();
                break;
        }

        delayms(10);
    }
}
