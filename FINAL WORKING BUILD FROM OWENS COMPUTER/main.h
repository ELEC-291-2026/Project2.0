#ifndef MAIN_H
#define MAIN_H

#include <stdint.h>

#include "../Common/Include/stm32l051xx.h"

typedef struct
{
    uint32_t Pin;
    uint32_t Mode;
    uint32_t Pull;
} GPIO_InitTypeDef;

typedef struct
{
    ADC_TypeDef *Instance;
    uint32_t ConfiguredChannel;
} ADC_HandleTypeDef;

typedef struct
{
    TIM_TypeDef *Instance;
} TIM_HandleTypeDef;

typedef struct
{
    uint32_t Channel;
    uint32_t Rank;
    uint32_t SamplingTime;
    uint32_t SingleDiff;
    uint32_t OffsetNumber;
    uint32_t Offset;
} ADC_ChannelConfTypeDef;

extern ADC_HandleTypeDef hadc1;
extern TIM_HandleTypeDef htim2;

#define GPIO_PIN_0 BIT0
#define GPIO_PIN_1 BIT1
#define GPIO_PIN_2 BIT2
#define GPIO_PIN_3 BIT3
#define GPIO_PIN_4 BIT4
#define GPIO_PIN_5 BIT5
#define GPIO_PIN_6 BIT6
#define GPIO_PIN_7 BIT7
#define GPIO_PIN_8 BIT8
#define GPIO_PIN_9 BIT9
#define GPIO_PIN_10 BIT10
#define GPIO_PIN_11 BIT11
#define GPIO_PIN_12 BIT12
#define GPIO_PIN_13 BIT13
#define GPIO_PIN_14 BIT14
#define GPIO_PIN_15 BIT15

#define GPIO_MODE_ANALOG 3U
#define GPIO_NOPULL 0U

#define ADC_CHANNEL_0 0U
#define ADC_CHANNEL_1 1U
#define ADC_CHANNEL_2 2U
#define ADC_CHANNEL_3 3U
#define ADC_CHANNEL_4 4U
#define ADC_CHANNEL_5 5U
#define ADC_CHANNEL_6 6U
#define ADC_CHANNEL_7 7U
#define ADC_CHANNEL_8 8U
#define ADC_CHANNEL_9 9U
#define ADC_CHANNEL_10 10U
#define ADC_CHANNEL_11 11U
#define ADC_CHANNEL_12 12U

#define ADC_RANK_CHANNEL_NUMBER 0U
#define ADC_SAMPLETIME_71CYCLES_5 (ADC_SMPR_SMP_1 | ADC_SMPR_SMP_2)
#define ADC_SINGLE_ENDED 0U
#define ADC_OFFSET_NONE 0U

#define TIM_CHANNEL_1 1U
#define TIM_CHANNEL_2 2U
#define TIM_CHANNEL_3 3U
#define TIM_CHANNEL_4 4U

#define __HAL_RCC_GPIOA_CLK_ENABLE() do { RCC->IOPENR |= RCC_IOPENR_GPIOAEN; } while (0)
#define __HAL_RCC_GPIOB_CLK_ENABLE() do { RCC->IOPENR |= RCC_IOPENR_GPIOBEN; } while (0)
#define __HAL_RCC_GPIOC_CLK_ENABLE() do { RCC->IOPENR |= RCC_IOPENR_GPIOCEN; } while (0)
#define __HAL_RCC_GPIOH_CLK_ENABLE() do { RCC->IOPENR |= RCC_IOPENR_GPIOHEN; } while (0)

#define __HAL_TIM_GET_AUTORELOAD(timer_handle) ((timer_handle)->Instance->ARR)

void __hal_tim_set_compare(TIM_HandleTypeDef *timer_handle,
    uint32_t channel,
    uint32_t compare);

#define __HAL_TIM_SET_COMPARE(timer_handle, channel, compare) \
    __hal_tim_set_compare((timer_handle), (channel), (compare))

void HAL_GPIO_Init(GPIO_TypeDef *port, GPIO_InitTypeDef *init_struct);
void HAL_ADC_ConfigChannel(ADC_HandleTypeDef *adc_handle,
    ADC_ChannelConfTypeDef *channel_config);
void HAL_ADC_Start(ADC_HandleTypeDef *adc_handle);
void HAL_ADC_PollForConversion(ADC_HandleTypeDef *adc_handle,
    unsigned int timeout_ms);
unsigned int HAL_ADC_GetValue(ADC_HandleTypeDef *adc_handle);
void HAL_ADC_Stop(ADC_HandleTypeDef *adc_handle);
void HAL_TIM_PWM_Start(TIM_HandleTypeDef *timer_handle, uint32_t channel);

void board_init(void);
void delayms(unsigned int milliseconds);

#endif
