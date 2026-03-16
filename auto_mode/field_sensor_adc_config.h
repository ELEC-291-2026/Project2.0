#ifndef FIELD_SENSOR_ADC_CONFIG_H
#define FIELD_SENSOR_ADC_CONFIG_H

/*
 * Update these macros to match your STM32 CubeMX pinout.
 *
 * This header is written in STM32 HAL style, so it expects your actual project
 * to provide symbols like GPIOA, GPIO_PIN_0, ADC_CHANNEL_0, hadc1, etc.
 *
 * The values below are placeholders only.
 */

#define FIELD_SENSOR_ADC_HANDLE hadc1
#define FIELD_SENSOR_ADC_TIMEOUT_MS 5U

/*
 * Replace this with the sample-time macro for your exact STM32 family.
 * Examples include ADC_SAMPLETIME_71CYCLES_5 or ADC_SAMPLETIME_160CYCLES_5.
 */
#define FIELD_SENSOR_ADC_SAMPLE_TIME ADC_SAMPLETIME_71CYCLES_5

#define FIELD_SENSOR_LEFT_GPIO_CLK_ENABLE()         __HAL_RCC_GPIOA_CLK_ENABLE()
#define FIELD_SENSOR_RIGHT_GPIO_CLK_ENABLE()        __HAL_RCC_GPIOA_CLK_ENABLE()
#define FIELD_SENSOR_INTERSECTION_GPIO_CLK_ENABLE() __HAL_RCC_GPIOA_CLK_ENABLE()

#define FIELD_SENSOR_LEFT_GPIO_PORT          GPIOA
#define FIELD_SENSOR_LEFT_GPIO_PIN           GPIO_PIN_0
#define FIELD_SENSOR_LEFT_ADC_CHANNEL        ADC_CHANNEL_0

#define FIELD_SENSOR_RIGHT_GPIO_PORT         GPIOA
#define FIELD_SENSOR_RIGHT_GPIO_PIN          GPIO_PIN_1
#define FIELD_SENSOR_RIGHT_ADC_CHANNEL       ADC_CHANNEL_1

#define FIELD_SENSOR_INTERSECTION_GPIO_PORT    GPIOA
#define FIELD_SENSOR_INTERSECTION_GPIO_PIN     GPIO_PIN_2
#define FIELD_SENSOR_INTERSECTION_ADC_CHANNEL  ADC_CHANNEL_2

#endif
