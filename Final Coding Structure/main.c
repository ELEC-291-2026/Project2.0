#include "../Common/Include/stm32l051xx.h"

#include "robot_auto_mode.h"
#include "hbridge_motor.h"
#include "field_sensor_adc_config.h"

/*
 * Consolidated auto-mode target.
 *
 * This file folds the logic from the original auto_mode/main.c and
 * auto_mode/robot_auto_mode.c into one translation unit so the final
 * project structure only needs a single firmware source file.
 */

#define ADC_CH_LEFT         5
#define ADC_CH_RIGHT        4
#define ADC_CH_INTERSECT    6

#define MAX_PWM             1000

#define F_CPU 32000000UL

#define LEFT_MOTOR_SIGN     -1
#define RIGHT_MOTOR_SIGN    -1
#define MOTOR_OUTPUT_ACTIVE_LOW 1

#define SOFTWARE_PWM_TICK_HZ 100000UL
#define PWM_PERIOD_COUNTS   1000U

static int g_last_left_command;
static int g_last_right_command;
static volatile unsigned int g_motor_compare[4];
static volatile unsigned int g_pwm_counter;

static robot_state_t g_state = ROBOT_AUTO_FOLLOW;
static path_action_t g_active_action = PATH_STRAIGHT;

static const path_action_t k_path_table[3][8] =
{
    { PATH_LEFT,     PATH_RIGHT,    PATH_LEFT,     PATH_STOP,
      PATH_STOP,     PATH_STOP,     PATH_STOP,     PATH_STOP },
    { PATH_LEFT,     PATH_RIGHT,    PATH_LEFT,     PATH_RIGHT,
      PATH_STRAIGHT, PATH_STRAIGHT, PATH_STOP,     PATH_STOP },
    { PATH_RIGHT,    PATH_STRAIGHT, PATH_RIGHT,    PATH_LEFT,
      PATH_RIGHT,    PATH_LEFT,     PATH_STRAIGHT, PATH_STOP }
};

enum
{
    BASE_SPEED = 420,
    SLOW_SPEED = 0,
    STEER_DEADBAND = 80,
    TRACK_THRESHOLD = 200,
    INTERSECTION_THRESHOLD = 100,
    FILTER_KEEP_COUNT = 3,
    BASELINE_IDLE_KEEP_COUNT = 15,
    BASELINE_STARTUP_KEEP_COUNT = 7,
    STARTUP_SETTLE_SAMPLES = 16,
    TURN_SPEED = 300
};

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

void wait_1ms(void)
{
    SysTick->LOAD = (F_CPU / 1000UL) - 1UL;
    SysTick->VAL = 0;
    SysTick->CTRL = 0x5;

    while ((SysTick->CTRL & (1U << 16U)) == 0U)
    {
    }

    SysTick->CTRL = 0;
}

void delayms(unsigned int ms)
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

void hbridge_motor_init(void)
{
    motors_stop();
}

void hbridge_motor_stop_all(void)
{
    motors_stop();
}

void hbridge_motor_apply(const motor_command_t *command)
{
    if (command == 0)
    {
        hbridge_motor_stop_all();
        return;
    }

    motors_set(command->left_command, command->right_command);
}

motor_command_t hbridge_motor_get_last_command(void)
{
    motor_command_t command;

    command.left_command = g_last_left_command;
    command.right_command = g_last_right_command;

    return command;
}

static int mix_samples(int previous, int sample, int keep_count)
{
    return (keep_count * previous + sample) / (keep_count + 1);
}

static int absolute_difference(int left_value, int right_value)
{
    if (left_value >= right_value)
    {
        return left_value - right_value;
    }

    return right_value - left_value;
}

static int detection_active(int was_detected,
    int signal,
    int filtered,
    int entry_signal,
    int exit_signal,
    int startup_filtered_threshold)
{
    if (was_detected)
    {
        return signal >= exit_signal;
    }

    return signal >= entry_signal || filtered >= startup_filtered_threshold;
}

static int baseline_keep_count(unsigned int samples_seen)
{
    if (samples_seen < STARTUP_SETTLE_SAMPLES)
    {
        return BASELINE_STARTUP_KEEP_COUNT;
    }

    return BASELINE_IDLE_KEEP_COUNT;
}

static void update_sensor_channel(unsigned int samples_seen,
    int sample,
    int entry_signal,
    int exit_signal,
    int startup_filtered_threshold,
    int *raw_value,
    int *filtered_value,
    int *baseline_value,
    int *signal_value,
    int *detected_value)
{
    int filtered_sample;
    int next_signal;
    int next_detected;

    *raw_value = sample;
    filtered_sample = mix_samples(*filtered_value, sample, FILTER_KEEP_COUNT);
    next_signal = absolute_difference(filtered_sample, *baseline_value);
    next_detected = detection_active(*detected_value,
        next_signal,
        filtered_sample,
        entry_signal,
        exit_signal,
        startup_filtered_threshold);
    if (!next_detected)
    {
        *baseline_value = mix_samples(*baseline_value,
            filtered_sample,
            baseline_keep_count(samples_seen));
    }
    next_signal = absolute_difference(filtered_sample, *baseline_value);
    next_detected = detection_active(*detected_value,
        next_signal,
        filtered_sample,
        entry_signal,
        exit_signal,
        startup_filtered_threshold);

    *filtered_value = filtered_sample;
    *signal_value = next_signal;
    *detected_value = next_detected;
}

static int clamp_pwm(int value)
{
    if (value < -MAX_PWM)
    {
        return -MAX_PWM;
    }

    if (value > MAX_PWM)
    {
        return MAX_PWM;
    }

    return value;
}

static motor_command_t make_motor_command(int left_command, int right_command)
{
    motor_command_t command;

    command.left_command = clamp_pwm(left_command);
    command.right_command = clamp_pwm(right_command);

    return command;
}

static void motor_stop(void)
{
    motor_command_t command = make_motor_command(0, 0);
    hbridge_motor_apply(&command);
}

static void motor_set_signed(int left_command, int right_command)
{
    motor_command_t command = make_motor_command(left_command, right_command);
    hbridge_motor_apply(&command);
}

static int intersection_detected(const field_data_t *sensors)
{
    return sensors->intersection_raw > INTERSECTION_THRESHOLD;
}

static int intersection_started(path_context_t *context, int detected_now)
{
    if (detected_now && !context->intersection_active)
    {
        context->intersection_active = 1;
        return 1;
    }

    if (!detected_now)
    {
        context->intersection_active = 0;
    }

    return 0;
}

static void run_follow_controller(const field_data_t *sensors)
{
    int diff = sensors->left_raw - sensors->right_raw;

    if (diff > STEER_DEADBAND)
    {
        motor_set_signed(SLOW_SPEED, BASE_SPEED);
    }
    else if (diff < -STEER_DEADBAND)
    {
        motor_set_signed(BASE_SPEED, SLOW_SPEED);
    }
    else
    {
        motor_set_signed(BASE_SPEED, BASE_SPEED);
    }
}

static void run_path_action(path_action_t action)
{
    switch (action)
    {
        case PATH_LEFT:
            motor_set_signed(-TURN_SPEED, TURN_SPEED);
            break;

        case PATH_RIGHT:
            motor_set_signed(TURN_SPEED, -TURN_SPEED);
            break;

        case PATH_STOP:
            motor_stop();
            break;

        case PATH_STRAIGHT:
        default:
            motor_set_signed(BASE_SPEED, BASE_SPEED);
            break;
    }
}

void field_sensor_reset(field_data_t *sensors)
{
    sensors->left_raw = 0;
    sensors->right_raw = 0;
    sensors->intersection_raw = 0;
    sensors->left_filtered = 0;
    sensors->right_filtered = 0;
    sensors->intersection_filtered = 0;
    sensors->left_baseline = 0;
    sensors->right_baseline = 0;
    sensors->intersection_baseline = 0;
    sensors->left_signal = 0;
    sensors->right_signal = 0;
    sensors->intersection_signal = 0;
    sensors->left_detected = 0;
    sensors->right_detected = 0;
    sensors->intersection_detected = 0;
    sensors->samples_seen = 0U;
}

void field_sensor_update(field_data_t *sensors,
    int left_sample,
    int right_sample,
    int intersection_sample)
{
    update_sensor_channel(sensors->samples_seen,
        left_sample,
        FIELD_SENSOR_TRACK_ENTRY_SIGNAL,
        FIELD_SENSOR_TRACK_EXIT_SIGNAL,
        FIELD_SENSOR_TRACK_STARTUP_MIN_FILTERED,
        &sensors->left_raw,
        &sensors->left_filtered,
        &sensors->left_baseline,
        &sensors->left_signal,
        &sensors->left_detected);
    update_sensor_channel(sensors->samples_seen,
        right_sample,
        FIELD_SENSOR_TRACK_ENTRY_SIGNAL,
        FIELD_SENSOR_TRACK_EXIT_SIGNAL,
        FIELD_SENSOR_TRACK_STARTUP_MIN_FILTERED,
        &sensors->right_raw,
        &sensors->right_filtered,
        &sensors->right_baseline,
        &sensors->right_signal,
        &sensors->right_detected);
    update_sensor_channel(sensors->samples_seen,
        intersection_sample,
        FIELD_SENSOR_INTERSECTION_ENTRY_SIGNAL,
        FIELD_SENSOR_INTERSECTION_EXIT_SIGNAL,
        FIELD_SENSOR_INTERSECTION_STARTUP_MIN_FILTERED,
        &sensors->intersection_raw,
        &sensors->intersection_filtered,
        &sensors->intersection_baseline,
        &sensors->intersection_signal,
        &sensors->intersection_detected);

    if (sensors->samples_seen < 0xFFFFFFFFU)
    {
        ++sensors->samples_seen;
    }
}

void path_context_reset(path_context_t *context, path_id_t selected_path)
{
    context->selected_path = selected_path;
    context->intersection_count = 0;
    context->intersection_active = 0;
}

path_action_t path_context_on_intersection(path_context_t *context)
{
    if (context->intersection_count >= 8)
    {
        return PATH_STOP;
    }

    return k_path_table[context->selected_path][context->intersection_count++];
}

void robot_auto_mode_init(path_context_t *context, path_id_t selected_path)
{
    g_state = ROBOT_AUTO_FOLLOW;
    g_active_action = PATH_STRAIGHT;
    path_context_reset(context, selected_path);
}

void robot_auto_mode_set_path(path_context_t *context, path_id_t selected_path)
{
    context->selected_path = selected_path;
    context->intersection_count = 0;
}

void robot_auto_mode_step(field_data_t *sensors, path_context_t *context)
{
    int detected_now;

    detected_now = intersection_detected(sensors);

    switch (g_state)
    {
        case ROBOT_AUTO_FOLLOW:
            if (intersection_started(context, detected_now))
            {
                g_active_action = path_context_on_intersection(context);
                motor_stop();
                delayms(400);
                if (g_active_action == PATH_LEFT || g_active_action == PATH_RIGHT)
                {
                    run_path_action(g_active_action);
                    delayms(700);
                    motor_stop();
                    g_state = ROBOT_AUTO_FOLLOW;
                }
                else if (g_active_action == PATH_STOP)
                {
                    g_state = ROBOT_AUTO_STOP;
                }
                else
                {
                    g_state = ROBOT_AUTO_FOLLOW;
                }
                break;
            }

            run_follow_controller(sensors);
            break;

        case ROBOT_AUTO_INTERSECTION:
            if (!detected_now)
            {
                context->intersection_active = 0;

                if (g_active_action == PATH_STOP)
                {
                    g_state = ROBOT_AUTO_STOP;
                    motor_stop();
                    break;
                }

                g_state = ROBOT_AUTO_FOLLOW;
            }
            else
            {
                run_path_action(g_active_action);
            }
            break;

        case ROBOT_AUTO_LOST:
            run_follow_controller(sensors);
            if (sensors->left_raw > TRACK_THRESHOLD ||
                sensors->right_raw > TRACK_THRESHOLD)
            {
                g_state = ROBOT_AUTO_FOLLOW;
            }
            break;

        case ROBOT_AUTO_STOP:
        default:
            motor_stop();
            break;
    }
}

void main(void)
{
    field_data_t sensors;
    path_context_t context;
    unsigned int i;
    unsigned int loop;

    field_sensor_reset(&sensors);

    g_last_left_command = 0;
    g_last_right_command = 0;
    loop = 0U;

    clock_init();
    uart_init();
    pwm_init();
    adc_init();
    hbridge_motor_init();

    robot_auto_mode_init(&context, PATH_ID_1);

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
        field_sensor_update(&sensors,
            adc_read(ADC_CH_LEFT),
            adc_read(ADC_CH_RIGHT),
            adc_read(ADC_CH_INTERSECT));
        delayms(2U);
    }

    uart_puts("Warmup done. Entering main loop.\r\n");

    while (1)
    {
        field_sensor_update(&sensors,
            adc_read(ADC_CH_LEFT),
            adc_read(ADC_CH_RIGHT),
            adc_read(ADC_CH_INTERSECT));

        if ((loop % 5U) == 0U)
        {
            uart_puts("L r=");
            uart_print_int(sensors.left_raw, 4);
            uart_puts(" s=");
            uart_print_int(sensors.left_signal, 4);
            uart_puts(" d=");
            uart_print_int(sensors.left_detected, 1);
            uart_puts(" | R r=");
            uart_print_int(sensors.right_raw, 4);
            uart_puts(" s=");
            uart_print_int(sensors.right_signal, 4);
            uart_puts(" d=");
            uart_print_int(sensors.right_detected, 1);
            uart_puts(" | IX r=");
            uart_print_int(sensors.intersection_raw, 4);
            uart_puts(" | dir=");
            uart_puts(movement_name(g_last_left_command, g_last_right_command));
            uart_puts(" lc=");
            uart_print_int(g_last_left_command, 4);
            uart_puts(" rc=");
            uart_print_int(g_last_right_command, 4);
            uart_puts(" ix=");
            uart_print_int(context.intersection_count, 1);
            uart_puts("\r\n");
        }

        ++loop;

        robot_auto_mode_step(&sensors, &context);

        delayms(10U);
    }
}
