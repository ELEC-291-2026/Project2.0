#include "../../Common/Include/stm32l051xx.h"

#define SYSCLK_HZ           32000000UL
#define PWM_PRESCALER       31U
#define PWM_PERIOD_COUNTS   1000U
#define MOTOR_COMMAND_MAX   1000

/*
 * STM32L051K8 LQFP32 pin map used here:
 * - PA0 (pin 6)  -> TIM2_CH1 -> left motor forward
 * - PA1 (pin 7)  -> TIM2_CH2 -> left motor reverse
 * - PA2 (pin 8)  -> TIM2_CH3 -> right motor forward
 * - PA3 (pin 9)  -> TIM2_CH4 -> right motor reverse
 *
 * Each H-bridge input gets its own PWM channel.
 * Positive command drives the forward pin.
 * Negative command drives the reverse pin.
 */
 
 // direction go from -120 to 120
 
 int volatile x;
 int volatile y;

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
        TIM2->CCR1 = command_to_compare(left_command);
        TIM2->CCR2 = 0;
    }
    else if (left_command < 0)
    {
        TIM2->CCR1 = 0;
        TIM2->CCR2 = command_to_compare(left_command);
    }
    else
    {
        TIM2->CCR1 = 0;
        TIM2->CCR2 = 0;
    }

    if (right_command > 0)
    {
        TIM2->CCR3 = command_to_compare(right_command);
        TIM2->CCR4 = 0;
    }
    else if (right_command < 0)
    {
        TIM2->CCR3 = 0;
        TIM2->CCR4 = command_to_compare(right_command);
    }
    else
    {
        TIM2->CCR3 = 0;
        TIM2->CCR4 = 0;
    }
}

static void configure_gpio_for_pwm(void)
{
    RCC->IOPENR |= BIT0;

    GPIOA->MODER &= ~(GPIO_MODER_MODE0 |
        GPIO_MODER_MODE1 |
        GPIO_MODER_MODE2 |
        GPIO_MODER_MODE3);
    GPIOA->MODER |= GPIO_MODER_MODE0_1 |
        GPIO_MODER_MODE1_1 |
        GPIO_MODER_MODE2_1 |
        GPIO_MODER_MODE3_1;

    GPIOA->OTYPER &= ~(BIT0 | BIT1 | BIT2 | BIT3);

    GPIOA->OSPEEDR |= BIT0 | BIT1 | BIT2 | BIT3 | BIT4 | BIT5 | BIT6 | BIT7;

    GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPD0 |
        GPIO_PUPDR_PUPD1 |
        GPIO_PUPDR_PUPD2 |
        GPIO_PUPDR_PUPD3);

    GPIOA->AFR[0] &= ~0x0000ffffUL;
    GPIOA->AFR[0] |= 0x00002222UL;
}

static void configure_tim2_pwm(void)
{
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

    TIM2->PSC = PWM_PRESCALER;
    TIM2->ARR = PWM_PERIOD_COUNTS - 1U;

    TIM2->CCMR1 = 0;
    TIM2->CCMR2 = 0;

    TIM2->CCMR1 |= BIT6 | BIT5 | BIT3;
    TIM2->CCMR1 |= BIT14 | BIT13 | BIT11;
    TIM2->CCMR2 |= BIT6 | BIT5 | BIT3;
    TIM2->CCMR2 |= BIT14 | BIT13 | BIT11;

    TIM2->CCER |= BIT0 | BIT4 | BIT8 | BIT12;
    TIM2->CR1 |= BIT7;

    set_motor_outputs(0, 0);

    TIM2->EGR |= BIT0;
    TIM2->CR1 |= BIT0;
}

static void hardware_init(void)
{
    configure_gpio_for_pwm();
    configure_tim2_pwm();
}


/** function takes in x and y and according to the gradient returns a motor control to power it max 999, 0 is neutral

**/

static void control(int x, int y)
{
	//we convert to new system where left is controlled by x+y, and right by x-y
	int scaled_x = x*999/120;
	int scaled_y = y*999/120;
	
	int left = scaled_x+scaled_y;
	int right = scaled_x-scaled_y;
	
	set_motor_outputs(left, right);
	

}

//decoder temporarely in this main but needs to go in the main main
void main(void)
{
    hardware_init();

    /****
    while (1)
    {
        set_motor_outputs(600, 0);
        delayms(3000);

        set_motor_outputs(0, 0);
        delayms(1000);

        set_motor_outputs(0, 600);
        delayms(3000);

        set_motor_outputs(0, 0);
        delayms(1000);

        set_motor_outputs(600, 600);
        delayms(3000);

        set_motor_outputs(0, 0);
        delayms(2000);
      **/
    while(1){
   		set_motor_outputs(-600,0);
   	}
}
