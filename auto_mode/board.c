#include "main.h"

enum
{
    SYSCLK_HZ = 32000000UL,
    SOFTWARE_PWM_TICK_HZ = 100000UL,
    PWM_PERIOD_COUNTS = 1000U,
    ADC_POWERUP_DELAY_CYCLES = 8000U
};

ADC_HandleTypeDef hadc1 = { ADC1, 0U };
TIM_HandleTypeDef htim2 = { TIM2 };

static volatile uint32_t g_motor_compare[4] = { 0U, 0U, 0U, 0U };
static volatile uint32_t g_pwm_counter = 0U;

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

static void wait_cycles(volatile unsigned int cycles)
{
    while (cycles-- > 0U)
    {
    }
}

static uint32_t adc_channel_to_select_mask(uint32_t channel)
{
    if (channel > 18U)
    {
        return 0U;
    }

    return 1UL << channel;
}

static void configure_motor_gpio_for_pwm(void)
{
    __HAL_RCC_GPIOA_CLK_ENABLE();

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
    GPIOA->ODR &= ~(BIT0 | BIT1 | BIT2 | BIT3);
}

static void configure_tim2_pwm(void)
{
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

    TIM2->PSC = 0U;
    TIM2->ARR = (SYSCLK_HZ / SOFTWARE_PWM_TICK_HZ) - 1U;
    TIM2->CCR1 = 0U;
    TIM2->CCR2 = 0U;
    TIM2->CCR3 = 0U;
    TIM2->CCR4 = 0U;
    TIM2->SR = 0U;
    TIM2->DIER = TIM_DIER_UIE;
    TIM2->CR1 = TIM_CR1_ARPE;
    TIM2->EGR = TIM_EGR_UG;
    NVIC->ISER[0] |= BIT15;
    TIM2->CR1 |= TIM_CR1_CEN;
    __enable_irq();
}

static void configure_adc(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;

    /* Match the PrintADC reference setup and clock the ADC from PCLK. */
    ADC1->CFGR2 |= ADC_CFGR2_CKMODE;

    if ((ADC1->CR & ADC_CR_ADEN) != 0U)
    {
        ADC1->CR |= ADC_CR_ADDIS;

        while ((ADC1->CR & ADC_CR_ADEN) != 0U)
        {
        }
    }

    ADC1->ISR |= ADC_ISR_EOCAL | ADC_ISR_EOC | ADC_ISR_EOSEQ | ADC_ISR_OVR | ADC_ISR_ADRDY;
    ADC1->CR |= ADC_CR_ADVREGEN;
    wait_cycles(ADC_POWERUP_DELAY_CYCLES);

    ADC1->CR |= ADC_CR_ADCAL;

    while ((ADC1->ISR & ADC_ISR_EOCAL) == 0U)
    {
    }

    ADC1->ISR |= ADC_ISR_EOCAL;
    ADC1->CFGR1 = 0U;
    ADC1->SMPR = ADC_SAMPLETIME_71CYCLES_5;
    ADC1->CHSELR = adc_channel_to_select_mask(ADC_CHANNEL_4);
    ADC1->ISR |= ADC_ISR_ADRDY;
    ADC1->CR |= ADC_CR_ADEN;

    while ((ADC1->ISR & ADC_ISR_ADRDY) == 0U)
    {
    }
}

void __hal_tim_set_compare(TIM_HandleTypeDef *timer_handle,
    uint32_t channel,
    uint32_t compare)
{
    (void)timer_handle;

    if (compare >= PWM_PERIOD_COUNTS)
    {
        compare = PWM_PERIOD_COUNTS - 1U;
    }

    switch (channel)
    {
        case TIM_CHANNEL_1:
            g_motor_compare[0] = compare;
            break;

        case TIM_CHANNEL_2:
            g_motor_compare[1] = compare;
            break;

        case TIM_CHANNEL_3:
            g_motor_compare[2] = compare;
            break;

        case TIM_CHANNEL_4:
            g_motor_compare[3] = compare;
            break;

        default:
            break;
    }
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

void HAL_GPIO_Init(GPIO_TypeDef *port, GPIO_InitTypeDef *init_struct)
{
    uint32_t pin_mask;
    unsigned int pin_index;

    pin_mask = init_struct->Pin;

    for (pin_index = 0U; pin_index < 16U; ++pin_index)
    {
        uint32_t pin_bit = 1UL << pin_index;
        uint32_t field_shift = pin_index * 2U;

        if ((pin_mask & pin_bit) == 0U)
        {
            continue;
        }

        port->MODER &= ~(0x3UL << field_shift);
        port->MODER |= ((uint32_t)init_struct->Mode & 0x3UL) << field_shift;

        port->PUPDR &= ~(0x3UL << field_shift);
        port->PUPDR |= ((uint32_t)init_struct->Pull & 0x3UL) << field_shift;
    }
}

void HAL_ADC_ConfigChannel(ADC_HandleTypeDef *adc_handle,
    ADC_ChannelConfTypeDef *channel_config)
{
    adc_handle->ConfiguredChannel = channel_config->Channel;
    adc_handle->Instance->CFGR1 |= ADC_CFGR1_AUTOFF;
    adc_handle->Instance->CHSELR = adc_channel_to_select_mask(channel_config->Channel);
    adc_handle->Instance->SMPR = channel_config->SamplingTime & ADC_SMPR_SMP;
}

void HAL_ADC_Start(ADC_HandleTypeDef *adc_handle)
{
    adc_handle->Instance->ISR |= ADC_ISR_EOC | ADC_ISR_EOSEQ | ADC_ISR_OVR;
    adc_handle->Instance->CR |= ADC_CR_ADSTART;
}

void HAL_ADC_PollForConversion(ADC_HandleTypeDef *adc_handle,
    unsigned int timeout_ms)
{
    volatile unsigned int timeout_cycles;

    timeout_cycles = (SYSCLK_HZ / 4000UL) * timeout_ms;

    while (((adc_handle->Instance->ISR & ADC_ISR_EOC) == 0U) &&
        (timeout_cycles > 0U))
    {
        --timeout_cycles;
    }
}

unsigned int HAL_ADC_GetValue(ADC_HandleTypeDef *adc_handle)
{
    return (unsigned int)(adc_handle->Instance->DR & ADC_DR_DATA);
}

void HAL_ADC_Stop(ADC_HandleTypeDef *adc_handle)
{
    adc_handle->Instance->ISR |= ADC_ISR_EOC | ADC_ISR_EOSEQ | ADC_ISR_OVR;
}

void HAL_TIM_PWM_Start(TIM_HandleTypeDef *timer_handle, uint32_t channel)
{
    (void)timer_handle;
    (void)channel;
}

void board_init(void)
{
    configure_motor_gpio_for_pwm();
    configure_tim2_pwm();
    configure_adc();
}

void delayms(unsigned int milliseconds)
{
    while (milliseconds-- > 0U)
    {
        wait_1ms();
    }
}
