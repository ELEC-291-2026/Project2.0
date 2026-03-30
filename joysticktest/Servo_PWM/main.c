
#include "../Common/Include/stm32l051xx.h"
#include "../Common/Include/serial.h"
#include <stdio.h>
#include <stdlib.h>

#define SYSCLK_HZ           32000000UL
#define SOFTWARE_PWM_TICK_HZ 100000UL
#define PWM_PERIOD_COUNTS   1000U
#define MOTOR_COMMAND_MAX  1000
#define LED_SENSOR ((GPIOA->IDR >> 7) & 1U)

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

void wait_1us(void)
{
    // For SysTick info check the STM32L0xxx Cortex-M0 programming manual
    // Load for 1 microsecond: (SYSCLK_HZ / 1,000,000) - 1
    // At 32MHz, this puts 31 into the LOAD register (32 ticks total)
    SysTick->LOAD = (SYSCLK_HZ / 1000000UL) - 1;  
    SysTick->VAL = 0; 
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk; 
    
    // Wait for the COUNTFLAG (Bit 16) to be set
    while((SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) == 0); 
    
    SysTick->CTRL = 0; // Disable Systick counter
}

void waitus(int len)
{
    while(len--) 
    {
        wait_1us();
    }
}

static void waitms(unsigned int milliseconds)
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
    
    // 1. Enable clock for Port B (Port B is BIT 1 in the IOPENR register)
    RCC->IOPENR |= (1U << 1); 

    // 2. Set PB6 to Input Mode
    // MODER6 uses bits [13:12]. Writing 00 sets it to Input.
    GPIOB->MODER &= ~(3U << 14); 
    
    // PA7 setup
	GPIOA->MODER &= ~(3U << 14); // Clear bits 14 and 15 to set PA7 to 'Input Mode'
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

static void control(int x, int y)
{
	//we convert to new system where left is controlled by x+y, and right by x-y
	int scaled_x = x*999/120;
	int scaled_y = y*999/120;
	
	int left = scaled_x+scaled_y;
	int right = scaled_x-scaled_y;
	
	set_motor_outputs(left, right);
	

}


void main(void)
{
    hardware_init();
    
    float counterMS = 0;
	float normalized[2] = {0,0};
	float translated_v[2] = {0,0};
	
	// control var
	int counterNormalize = 3;

    while (1)
    {
        if(LED_SENSOR == 0)
	    {
	        counterMS = 0;
	        while(LED_SENSOR == 0)
	        {
	            counterMS++;
	            wait_1us();
	        }
	        /*
	        if(counterMS <= 500) {}
	        else if(counterMS <= 1200)
	        {
	        	translated_v[0] = counterMS - normalized[0];
	        	
	        	if(translated_v[0] >=118)
	        		translated_v[0] = 120;
	        	else if(translated_v[0] <= -118)
	        		translated_v[0] = -120;
	        	else if(translated_v[0] <= 3 && translated_v[0] >= -3)
	        		translated_v[0] = 0;
	        }
	        else if(counterMS <=  2000)
	        {
	        	translated_v[1] = counterMS - normalized[1];
	        	
	        	if(translated_v[1] >=118)
	        		translated_v[1] = 120;
	        	else if(translated_v[1] <= -118)
	        		translated_v[1] = -120;
	        	else if(translated_v[1] <= 3 && translated_v[1] >= -3)
	        		translated_v[1] = 0;
	        }
	        else if(counterMS <=  3400)
	        {
	        	goto end;
	        }
	        
	        else if(counterMS <=  4000)
	    	{
	    		counterNormalize = 3;
	    	}
	        
	        if(counterNormalize > 0)
	    	{
	    		counterNormalize--;
	    		if(counterMS <= 1200)
					normalized[0] = counterMS;
				else if(counterMS <= 2000)
					normalized[1] = counterMS;
	    	}
	    	*/
	    	printf("\r             %d, x: %d y: %d", (int)counterMS, (int)translated_v[0], (int)translated_v[1]);
	    }
	    else{
	        //printf("\rno");
	    }
	    //control(translated_v[0],translated_v[1]);
	    //end:;
   }
}
