#include "stm32l051xx.h"

#define ADC_CH_LEFT         4
#define ADC_CH_RIGHT        5
#define ADC_CH_INTERSECT    6

#define BASE_SPEED          260
#define TURN_SPEED          260
#define MAX_PWM             1000
#define KP                  1
#define KD                  2
#define DEADBAND            35

#define ENTRY_SIGNAL        200
#define EXIT_SIGNAL         100
#define INTERSECT_ENTRY     9999
#define INTERSECT_EXIT      100
#define TRACK_ACQUIRE_COUNT 3
#define SEARCH_SLOW_SPEED   170
#define SEARCH_FAST_SPEED   250
#define MIN_FORWARD_SPEED   120
#define MAX_FORWARD_SPEED   360
#define MAX_CORRECTION      90

#define F_CPU 32000000UL

#define LEFT_MOTOR_SIGN     1
#define RIGHT_MOTOR_SIGN    1
#define MOTOR_OUTPUT_ACTIVE_LOW 1

#define SOFTWARE_PWM_TICK_HZ 100000UL
#define PWM_PERIOD_COUNTS   1000U

static int g_last_left_command;
static int g_last_right_command;
static int g_prev_error;
static volatile unsigned int g_motor_compare[4];
static volatile unsigned int g_pwm_counter;

static void set_motor_pin(unsigned int bit_mask, int active)
{
#if MOTOR_OUTPUT_ACTIVE_LOW
    if (active)
    {
        GPIOA->ODR &= ~bit_mask;
    }
    else
    {
        GPIOA->ODR |= bit_mask;
    }
#else
    if (active)
    {
        GPIOA->ODR |= bit_mask;
    }
    else
    {
        GPIOA->ODR &= ~bit_mask;
    }
#endif
}

static void wait_1ms(void)
{
    SysTick->LOAD = (F_CPU / 1000UL) - 1UL;
    SysTick->VAL = 0;
    SysTick->CTRL = 0x5;

    while ((SysTick->CTRL & (1U << 16U)) == 0U)
    {
    }

    SysTick->CTRL = 0;
}

static void delayms(unsigned int ms)
{
    while (ms-- > 0U)
    {
        wait_1ms();
    }
}

static void clock_init(void)
{
    RCC->CR |= (1U << 0U);
    while ((RCC->CR & (1U << 2U)) == 0U)
    {
    }

    RCC->CFGR &= ~((0xFUL << 18U) | (0x3U << 22U) | (1U << 16U));
    RCC->CFGR |= (0x1U << 18U) | (0x1U << 22U);

    RCC->CR |= (1U << 24U);
    while ((RCC->CR & (1U << 25U)) == 0U)
    {
    }

    FLASH->ACR |= (1U << 0U);

    RCC->CFGR = (RCC->CFGR & ~0x3U) | 0x3U;
    while ((RCC->CFGR & (0x3U << 2U)) != (0x3U << 2U))
    {
    }
}

static void uart_init(void)
{
    RCC->IOPENR |= (1U << 0U);
    RCC->APB2ENR |= (1U << 14U);

    GPIOA->MODER &= ~(3U << 18U);
    GPIOA->MODER |= (2U << 18U);
    GPIOA->AFR[1] &= ~(0xFU << 4U);
    GPIOA->AFR[1] |= (4U << 4U);

    USART1->BRR = 278U;
    USART1->CR1 = (1U << 3U) | (1U << 0U);
}

static void uart_putc(char c)
{
    while ((USART1->ISR & (1U << 7U)) == 0U)
    {
    }

    USART1->TDR = (unsigned char)c;
}

static void uart_puts(const char *s)
{
    while (*s != '\0')
    {
        uart_putc(*s);
        ++s;
    }
}

static void uart_print_int(int val, int width)
{
    char buf[12];
    int len;
    unsigned int uval;

    len = 0;

    if (val < 0)
    {
        uart_putc('-');
        uval = (unsigned int)(-val);
        --width;
    }
    else
    {
        uval = (unsigned int)val;
    }

    if (uval == 0U)
    {
        buf[len++] = '0';
    }
    else
    {
        while (uval > 0U)
        {
            buf[len++] = (char)('0' + (uval % 10U));
            uval /= 10U;
        }
    }

    while (len < width)
    {
        uart_putc(' ');
        --width;
    }

    while (len > 0)
    {
        uart_putc(buf[--len]);
    }
}

static void pwm_init(void)
{
    RCC->IOPENR |= (1U << 0U);
    RCC->APB1ENR |= (1U << 0U);

    GPIOA->MODER &= ~(GPIO_MODER_MODE0 |
        GPIO_MODER_MODE1 |
        GPIO_MODER_MODE2 |
        GPIO_MODER_MODE3);
    GPIOA->MODER |= GPIO_MODER_MODE0_0 |
        GPIO_MODER_MODE1_0 |
        GPIO_MODER_MODE2_0 |
        GPIO_MODER_MODE3_0;

    GPIOA->OTYPER &= ~(BIT0 | BIT1 | BIT2 | BIT3);
    GPIOA->OSPEEDR |= GPIO_OSPEEDER_OSPEED0 |
        GPIO_OSPEEDER_OSPEED1 |
        GPIO_OSPEEDER_OSPEED2 |
        GPIO_OSPEEDER_OSPEED3;
    GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPD0 |
        GPIO_PUPDR_PUPD1 |
        GPIO_PUPDR_PUPD2 |
        GPIO_PUPDR_PUPD3);
#if MOTOR_OUTPUT_ACTIVE_LOW
    GPIOA->ODR |= BIT0 | BIT1 | BIT2 | BIT3;
#else
    GPIOA->ODR &= ~(BIT0 | BIT1 | BIT2 | BIT3);
#endif

    g_motor_compare[0] = 0U;
    g_motor_compare[1] = 0U;
    g_motor_compare[2] = 0U;
    g_motor_compare[3] = 0U;
    g_pwm_counter = 0U;

    TIM2->PSC = 0U;
    TIM2->ARR = (F_CPU / SOFTWARE_PWM_TICK_HZ) - 1U;
    TIM2->SR = 0U;
    TIM2->DIER = TIM_DIER_UIE;
    TIM2->CR1 = TIM_CR1_ARPE;
    TIM2->EGR = 1U;
    NVIC->ISER[0] |= BIT15;
    TIM2->CR1 |= TIM_CR1_CEN;
    __enable_irq();
}

static unsigned int clamp_command(int command)
{
    unsigned int magnitude;

    if (command < 0)
    {
        magnitude = (unsigned int)(-command);
    }
    else
    {
        magnitude = (unsigned int)command;
    }

    if (magnitude > (unsigned int)MAX_PWM)
    {
        magnitude = (unsigned int)MAX_PWM;
    }

    return magnitude;
}

static unsigned int command_to_compare(int command)
{
    unsigned int compare;
    unsigned int magnitude;

    magnitude = clamp_command(command);
    compare = (magnitude * PWM_PERIOD_COUNTS) / (unsigned int)MAX_PWM;

    if (compare >= PWM_PERIOD_COUNTS)
    {
        compare = PWM_PERIOD_COUNTS - 1U;
    }

    return compare;
}

static void motors_stop(void)
{
    g_motor_compare[0] = 0U;
    g_motor_compare[1] = 0U;
    g_motor_compare[2] = 0U;
    g_motor_compare[3] = 0U;
    set_motor_pin(BIT0, 0);
    set_motor_pin(BIT1, 0);
    set_motor_pin(BIT2, 0);
    set_motor_pin(BIT3, 0);
    g_last_left_command = 0;
    g_last_right_command = 0;
    g_prev_error = 0;
}

static void motors_set(int left, int right)
{
    g_last_left_command = left;
    g_last_right_command = right;

    left *= LEFT_MOTOR_SIGN;
    right *= RIGHT_MOTOR_SIGN;

    g_motor_compare[0] = 0U;
    g_motor_compare[1] = 0U;
    g_motor_compare[2] = 0U;
    g_motor_compare[3] = 0U;

    if (left > 0)
    {
        g_motor_compare[0] = command_to_compare(left);
    }
    else if (left < 0)
    {
        g_motor_compare[1] = command_to_compare(left);
    }

    if (right > 0)
    {
        g_motor_compare[2] = command_to_compare(right);
    }
    else if (right < 0)
    {
        g_motor_compare[3] = command_to_compare(right);
    }
}

void TIM2_Handler(void)
{
    TIM2->SR &= ~TIM_SR_UIF;

    set_motor_pin(BIT0, g_motor_compare[0] > g_pwm_counter);
    set_motor_pin(BIT1, g_motor_compare[1] > g_pwm_counter);
    set_motor_pin(BIT2, g_motor_compare[2] > g_pwm_counter);
    set_motor_pin(BIT3, g_motor_compare[3] > g_pwm_counter);

    ++g_pwm_counter;

    if (g_pwm_counter >= PWM_PERIOD_COUNTS)
    {
        g_pwm_counter = 0U;
    }
}

static void adc_init(void)
{
    RCC->IOPENR |= (1U << 0U);
    RCC->APB2ENR |= (1U << 9U);

    GPIOA->MODER |= (3U << 8U) | (3U << 10U) | (3U << 12U);

    ADC1->CFGR2 |= (1U << 30U);

    if ((ADC1->CR & (1U << 0U)) != 0U)
    {
        ADC1->CR |= (1U << 1U);
        while ((ADC1->CR & (1U << 0U)) != 0U)
        {
        }
    }

    ADC1->CR |= (1U << 28U);
    delayms(1U);

    ADC1->CR |= (1U << 31U);
    while ((ADC1->CR & (1U << 31U)) != 0U)
    {
    }

    ADC1->SMPR = 0x5U;
    ADC1->CFGR1 = 0U;
    ADC1->CR |= (1U << 0U);
    while ((ADC1->ISR & (1U << 0U)) == 0U)
    {
    }
}

static int adc_read(int channel)
{
    int sum;
    int i;

    sum = 0;
    ADC1->CHSELR = (1U << channel);

    for (i = 0; i < 4; ++i)
    {
        ADC1->CR |= (1U << 2U);
        while ((ADC1->ISR & (1U << 2U)) == 0U)
        {
        }
        sum += (int)(ADC1->DR & 0xFFFU);
    }

    return sum / 4;
}

typedef struct
{
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
    return (a > b) ? (a - b) : (b - a);
}

static int clamp_value(int value, int lower, int upper)
{
    if (value < lower)
    {
        return lower;
    }

    if (value > upper)
    {
        return upper;
    }

    return value;
}

static void update_ch(sensor_ch_t *ch, int sample, int entry, int exit_sig)
{
    int filt;
    int bk;
    int sig;
    int det;

    filt = mix(ch->filtered, sample, 3);
    bk = (ch->samples < 16U) ? 7 : 15;
    sig = absdiff(filt, ch->baseline);

    if (ch->detected)
    {
        det = (sig >= exit_sig);
    }
    else
    {
        det = (sig >= entry);
    }

    if (!det)
    {
        ch->baseline = mix(ch->baseline, filt, bk);
    }

    sig = absdiff(filt, ch->baseline);

    if (ch->detected)
    {
        det = (sig >= exit_sig);
    }
    else
    {
        det = (sig >= entry);
    }

    ch->raw = sample;
    ch->filtered = filt;
    ch->signal = sig;
    ch->detected = det;

    if (ch->samples < 0xFFFFFFFFU)
    {
        ++ch->samples;
    }
}

typedef enum
{
    ST_FOLLOW,
    ST_INTERSECTION,
    ST_LOST,
    ST_STOP
} state_t;

static const int k_paths[3][8] =
{
    { 0, 1, 1, 0, 2, 1, 2, 3 },
    { 1, 2, 1, 2, 0, 0, 3, 3 },
    { 2, 0, 2, 1, 2, 1, 0, 3 }
};

static state_t g_state;
static int g_action;

static const char *state_name(state_t state)
{
    switch (state)
    {
        case ST_FOLLOW:
            return "FOLLOW";

        case ST_INTERSECTION:
            return "INTERSECTION";

        case ST_LOST:
            return "LOST";

        case ST_STOP:
        default:
            return "STOP";
    }
}

static const char *movement_name(int left, int right)
{
    if (left == 0 && right == 0)
    {
        return "STOP";
    }

    if (left > 0 && right > 0)
    {
        if (left == right)
        {
            return "FORWARD";
        }

        if (left > right)
        {
            return "VEER_RIGHT";
        }

        return "VEER_LEFT";
    }

    if (left < 0 && right < 0)
    {
        if (left == right)
        {
            return "REVERSE";
        }

        if (left < right)
        {
            return "REV_RIGHT";
        }

        return "REV_LEFT";
    }

    if (left < 0 && right > 0)
    {
        return "TURN_LEFT";
    }

    if (left > 0 && right < 0)
    {
        return "TURN_RIGHT";
    }

    if (left == 0)
    {
        if (right > 0)
        {
            return "PIVOT_LEFT";
        }

        return "PIVOT_RIGHT";
    }

    if (left > 0)
    {
        return "ARC_RIGHT";
    }

    return "ARC_LEFT";
}

static void run_action(int action)
{
    switch (action)
    {
        case 1:
            motors_set(-TURN_SPEED, TURN_SPEED);
            break;

        case 2:
            motors_set(TURN_SPEED, -TURN_SPEED);
            break;

        case 3:
            motors_stop();
            break;

        default:
            motors_set(BASE_SPEED, BASE_SPEED);
            break;
    }
}

static void run_follow(int left_sig, int right_sig)
{
    int error;
    int derivative;
    int correction;
    int left_command;
    int right_command;

    error = left_sig - right_sig;
    if (error > -DEADBAND && error < DEADBAND)
    {
        error = 0;
    }

    derivative = error - g_prev_error;
    g_prev_error = error;

    correction = (KP * error) + (KD * derivative);
    correction = clamp_value(correction, -MAX_CORRECTION, MAX_CORRECTION);

    left_command = clamp_value(BASE_SPEED - correction, MIN_FORWARD_SPEED, MAX_FORWARD_SPEED);
    right_command = clamp_value(BASE_SPEED + correction, MIN_FORWARD_SPEED, MAX_FORWARD_SPEED);

    motors_set(left_command, right_command);
}

static void run_single_sensor_follow(int left_detected, int right_detected)
{
    g_prev_error = 0;

    if (left_detected && !right_detected)
    {
        motors_set(SEARCH_SLOW_SPEED, SEARCH_FAST_SPEED);
    }
    else if (right_detected && !left_detected)
    {
        motors_set(SEARCH_FAST_SPEED, SEARCH_SLOW_SPEED);
    }
    else
    {
        motors_stop();
    }
}

void main(void)
{
    sensor_ch_t left;
    sensor_ch_t right;
    sensor_ch_t intersect;
    int path;
    int ix_count;
    int ix_active;
    int ix_timer;
    int ix_sustain;
    unsigned int line_acquire_count;
    unsigned int i;
    unsigned int loop;

    left.raw = 0;
    left.filtered = 0;
    left.baseline = 0;
    left.signal = 0;
    left.detected = 0;
    left.samples = 0U;

    right.raw = 0;
    right.filtered = 0;
    right.baseline = 0;
    right.signal = 0;
    right.detected = 0;
    right.samples = 0U;

    intersect.raw = 0;
    intersect.filtered = 0;
    intersect.baseline = 0;
    intersect.signal = 0;
    intersect.detected = 0;
    intersect.samples = 0U;

    path = 0;
    ix_count = 0;
    ix_active = 0;
    g_state = ST_LOST;
    g_action = 0;
    g_last_left_command = 0;
    g_last_right_command = 0;
    g_prev_error = 0;
    line_acquire_count = 0U;
    loop = 0U;
    ix_timer = 0;
    ix_sustain = 0;

    clock_init();
    uart_init();
    pwm_init();
    adc_init();
    motors_stop();

    delayms(500U);
    uart_puts("Robot starting up...\r\n");
    uart_puts("--- RAW ADC DUMP (no filtering) ---\r\n");

    for (i = 0U; i < 20U; ++i)
    {
        uart_puts("L=");
        uart_print_int(adc_read(ADC_CH_LEFT), 4);
        uart_puts("  R=");
        uart_print_int(adc_read(ADC_CH_RIGHT), 4);
        uart_puts("  IX=");
        uart_print_int(adc_read(ADC_CH_INTERSECT), 4);
        uart_puts("\r\n");
        delayms(100U);
    }

    uart_puts("--- END DUMP, starting warmup ---\r\n");

    for (i = 0U; i < 64U; ++i)
    {
        update_ch(&left, adc_read(ADC_CH_LEFT), ENTRY_SIGNAL, EXIT_SIGNAL);
        update_ch(&right, adc_read(ADC_CH_RIGHT), ENTRY_SIGNAL, EXIT_SIGNAL);
        update_ch(&intersect, adc_read(ADC_CH_INTERSECT), INTERSECT_ENTRY, INTERSECT_EXIT);
        delayms(2U);
    }

    uart_puts("Warmup done. Entering main loop.\r\n");

    while (1)
    {
        update_ch(&left, adc_read(ADC_CH_LEFT), ENTRY_SIGNAL, EXIT_SIGNAL);
        update_ch(&right, adc_read(ADC_CH_RIGHT), ENTRY_SIGNAL, EXIT_SIGNAL);
        update_ch(&intersect, adc_read(ADC_CH_INTERSECT), INTERSECT_ENTRY, INTERSECT_EXIT);

        if ((loop % 5U) == 0U)
        {
            uart_puts("L r=");
            uart_print_int(left.raw, 4);
            uart_puts(" b=");
            uart_print_int(left.baseline, 4);
            uart_puts(" s=");
            uart_print_int(left.signal, 4);
            uart_puts(" d=");
            uart_print_int(left.detected, 1);
            uart_puts(" | R r=");
            uart_print_int(right.raw, 4);
            uart_puts(" b=");
            uart_print_int(right.baseline, 4);
            uart_puts(" s=");
            uart_print_int(right.signal, 4);
            uart_puts(" d=");
            uart_print_int(right.detected, 1);
            uart_puts(" | IX r=");
            uart_print_int(intersect.raw, 4);
            uart_puts(" s=");
            uart_print_int(intersect.signal, 4);
            uart_puts(" d=");
            uart_print_int(intersect.detected, 1);
            uart_puts(" | st=");
            uart_puts(state_name(g_state));
            uart_puts(" dir=");
            uart_puts(movement_name(g_last_left_command, g_last_right_command));
            uart_puts(" lc=");
            uart_print_int(g_last_left_command, 4);
            uart_puts(" rc=");
            uart_print_int(g_last_right_command, 4);
            uart_puts(" xt=");
            uart_print_int(ix_timer, 3);
            uart_puts(" xs=");
            uart_print_int(ix_sustain, 1);
            uart_puts(" la=");
            uart_print_int((int)line_acquire_count, 2);
            uart_puts("\r\n");
        }

        ++loop;

        switch (g_state)
        {
            case ST_FOLLOW:
                if (!left.detected && !right.detected)
                {
                    motors_stop();
                    ix_sustain = 0;
                    line_acquire_count = 0U;
                    g_state = ST_LOST;
                    break;
                }

                if (!(left.detected && right.detected))
                {
                    ix_sustain = 0;
                    ix_active = 0;
                    run_single_sensor_follow(left.detected, right.detected);
                    break;
                }

                if (intersect.detected)
                {
                    ++ix_sustain;
                }
                else
                {
                    ix_sustain = 0;
                    ix_active = 0;
                }

                if (ix_sustain >= 3 && !ix_active)
                {
                    ix_active = 1;
                    ix_sustain = 0;
                    ix_timer = 0;
                    g_action = (ix_count < 8) ? k_paths[path][ix_count++] : 3;
                    g_state = ST_INTERSECTION;
                    run_action(g_action);
                    break;
                }

                run_follow(left.signal, right.signal);
                break;

            case ST_INTERSECTION:
                ++ix_timer;

                if (!intersect.detected || ix_timer > 200)
                {
                    ix_active = 0;
                    ix_sustain = 0;
                    ix_timer = 0;

                    if (g_action == 3)
                    {
                        g_state = ST_STOP;
                        motors_stop();
                    }
                    else
                    {
                        g_state = ST_FOLLOW;
                    }
                }
                else
                {
                    run_action(g_action);
                }
                break;

            case ST_LOST:
                if (left.detected && right.detected)
                {
                    if (line_acquire_count < TRACK_ACQUIRE_COUNT)
                    {
                        ++line_acquire_count;
                    }
                }
                else
                {
                    line_acquire_count = 0U;
                }

                if (line_acquire_count >= TRACK_ACQUIRE_COUNT)
                {
                    g_state = ST_FOLLOW;
                    run_follow(left.signal, right.signal);
                }
                else
                {
                    run_single_sensor_follow(left.detected, right.detected);
                }
                break;

            case ST_STOP:
            default:
                motors_stop();
                break;
        }

        delayms(10U);
    }
}
