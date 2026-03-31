#include "stm32l051xx.h"
#include "robot_auto_mode.h"
#include "collision_detector.h"
#include "../vl53l0x.h"

#define ADC_CH_LEFT         5
#define ADC_CH_RIGHT        4
#define ADC_CH_INTERSECT    6

#define BASE_SPEED          325
#define TURN_SPEED          300
#define MAX_PWM             1000
#define KP                  2
#define KD                  3
#define DEADBAND            20

#define ENTRY_SIGNAL        200
#define EXIT_SIGNAL         100
#define INTERSECT_ENTRY     9999
#define INTERSECT_EXIT      100
#define TRACK_ACQUIRE_COUNT 3
#define SEARCH_SLOW_SPEED   200
#define SEARCH_FAST_SPEED   320
#define MIN_FORWARD_SPEED   150
#define MAX_FORWARD_SPEED   480
#define MAX_CORRECTION      160

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


/* Bridge called by robot_auto_mode.c to drive the physical motors. */
void hbridge_motor_apply(const motor_command_t *command)
{
    motors_set(command->left_command, command->right_command);
}

void main(void)
{
    field_data_t sensors;
    path_context_t context;
    collision_detector_t collision;
    int tof_ok;
    unsigned int tof_poll_counter;
    int obstacle_detected;
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
    motors_stop();

    robot_auto_mode_init(&context, PATH_ID_1);
    tof_ok = collision_detector_init(&collision);
    tof_poll_counter = 0U;
    obstacle_detected = 0;

    delayms(500U);
    uart_puts("Robot starting up...\r\n");
    uart_puts(tof_ok ? "ToF init OK\r\n" : "ToF init FAILED\r\n");
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

    int auto_mode = 1;

    while (1)
    {
        //mode selection
        if (auto_mode == 1)
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
        }
        else
        {
        remote_control_update(...);   // read joystick / remote command
        }
        ++loop;

        /* Poll VL53L0X every 500ms — stop motors while obstacle within 200mm */
        ++tof_poll_counter;
        if (tof_ok && tof_poll_counter >= 50U)
        {
            uint16_t dist_mm;
            tof_poll_counter = 0U;
            if (vl53l0x_read_range_single(&dist_mm))
            {
                obstacle_detected = (dist_mm < 200U);
            }
        }

        if (obstacle_detected)
        {
            motors_stop();
        }
        else
        {
            robot_auto_mode_step(&sensors, &context);
        }

        delayms(10U);
    }
}
