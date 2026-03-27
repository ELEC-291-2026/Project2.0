#include "../Common/Include/stm32l051xx.h"

#define SYSCLK_HZ           32000000UL
#define SOFTWARE_PWM_TICK_HZ 100000UL
#define PWM_PERIOD_COUNTS   1000U
#define MOTOR_COMMAND_MAX   1000

/*
 * STM32L051K8 LQFP32 pin map used here:
 * - PA0 (pin 6)  -> left motor forward
 * - PA1 (pin 7)  -> left motor reverse
 * - PA2 (pin 8)  -> right motor forward
 * - PA3 (pin 9)  -> right motor reverse
 *
 * TIM2 provides a periodic interrupt, and the ISR bit-bangs software PWM on
 * the four motor-control pins.
 * Positive command drives the forward pin.
 * Negative command drives the reverse pin.
 */

static volatile unsigned int g_motor_compare[4] = { 0U, 0U, 0U, 0U };
static volatile unsigned int g_pwm_counter = 0U;

static void wait_1ms(void)
{
    SysTick->LOAD = (SYSCLK_HZ / 1000UL) - 1UL;
    SysTick->VAL = 0;
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk;

    while ((SysTick->CTRL & BIT16) == 0U)
    {
    }

    SysTick->CTRL = 0;
}

static void delayms(unsigned int milliseconds)
{
    while (milliseconds-- > 0U)
    {
        wait_1ms();
    }
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

    if (magnitude > (unsigned int)MOTOR_COMMAND_MAX)
    {
        magnitude = (unsigned int)MOTOR_COMMAND_MAX;
    }

    return magnitude;
}

static unsigned int command_to_compare(int command)
{
    unsigned int compare;
    unsigned int magnitude = clamp_command(command);

    compare = (magnitude * PWM_PERIOD_COUNTS) / (unsigned int)MOTOR_COMMAND_MAX;

    if (compare >= PWM_PERIOD_COUNTS)
    {
        compare = PWM_PERIOD_COUNTS - 1U;
    }

    return compare;
}

static void set_motor_outputs(int left_command, int right_command)
{
    if (left_command > 0)
    {
        g_motor_compare[0] = command_to_compare(left_command);
        g_motor_compare[1] = 0U;
    }
    else if (left_command < 0)
    {
        g_motor_compare[0] = 0U;
        g_motor_compare[1] = command_to_compare(left_command);
    }
    else
    {
        g_motor_compare[0] = 0U;
        g_motor_compare[1] = 0U;
    }

    if (right_command > 0)
    {
        g_motor_compare[2] = command_to_compare(right_command);
        g_motor_compare[3] = 0U;
    }
    else if (right_command < 0)
    {
        g_motor_compare[2] = 0U;
        g_motor_compare[3] = command_to_compare(right_command);
    }
    else
    {
        g_motor_compare[2] = 0U;
        g_motor_compare[3] = 0U;
    }
}

static void configure_gpio_for_pwm(void)
{
    RCC->IOPENR |= BIT0;

    GPIOA->MODER &= ~(GPIO_MODER_MODE0 |
        GPIO_MODER_MODE1 |
        GPIO_MODER_MODE2 |
        GPIO_MODER_MODE3);
    GPIOA->MODER |= GPIO_MODER_MODE0_0 |
        GPIO_MODER_MODE1_0 |
        GPIO_MODER_MODE2_0 |
        GPIO_MODER_MODE3_0;

    GPIOA->OTYPER &= ~(BIT0 | BIT1 | BIT2 | BIT3);

    GPIOA->OSPEEDR |= BIT0 | BIT1 | BIT2 | BIT3 | BIT4 | BIT5 | BIT6 | BIT7;

    GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPD0 |
        GPIO_PUPDR_PUPD1 |
        GPIO_PUPDR_PUPD2 |
        GPIO_PUPDR_PUPD3);
    GPIOA->ODR &= ~(BIT0 | BIT1 | BIT2 | BIT3);
}

static void configure_tim2_pwm(void)
{
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

    TIM2->PSC = 0U;
    TIM2->ARR = (SYSCLK_HZ / SOFTWARE_PWM_TICK_HZ) - 1U;
    TIM2->DIER = TIM_DIER_UIE;
    TIM2->CR1 = TIM_CR1_ARPE;

    set_motor_outputs(0, 0);

    TIM2->SR = 0U;
    TIM2->EGR = TIM_EGR_UG;
    NVIC->ISER[0] |= BIT15;
    TIM2->CR1 |= TIM_CR1_CEN;
    __enable_irq();
}

static void hardware_init(void)
{
    configure_gpio_for_pwm();
    configure_tim2_pwm();
}

void TIM2_Handler(void)
{
    TIM2->SR &= ~TIM_SR_UIF;

    if (g_motor_compare[0] > g_pwm_counter)
    {
        GPIOA->ODR |= BIT0;
    }
    else
    {
        GPIOA->ODR &= ~BIT0;
    }

    if (g_motor_compare[1] > g_pwm_counter)
    {
        GPIOA->ODR |= BIT1;
    }
    else
    {
        GPIOA->ODR &= ~BIT1;
    }

    if (g_motor_compare[2] > g_pwm_counter)
    {
        GPIOA->ODR |= BIT2;
    }
    else
    {
        GPIOA->ODR &= ~BIT2;
    }

    if (g_motor_compare[3] > g_pwm_counter)
    {
        GPIOA->ODR |= BIT3;
    }
    else
    {
        GPIOA->ODR &= ~BIT3;
    }

    ++g_pwm_counter;

    if (g_pwm_counter >= PWM_PERIOD_COUNTS)
    {
        g_pwm_counter = 0U;
    }
}

void main(void)
{
    hardware_init();

    while (1)
    {
        set_motor_outputs(600, -600);
        delayms(3000);

        set_motor_outputs(-600, 600);
        delayms(3000);

        set_motor_outputs(0, 0);
        delayms(2000);
    }
}
