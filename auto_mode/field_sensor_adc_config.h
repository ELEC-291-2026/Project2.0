#ifndef FIELD_SENSOR_ADC_CONFIG_H
#define FIELD_SENSOR_ADC_CONFIG_H

/*
 * Update these macros to match your STM32 CubeMX pinout.
 *
 * This header is written in STM32 HAL style, so it expects your actual project
 * to provide symbols like GPIOA, GPIO_PIN_0, ADC_CHANNEL_0, hadc1, etc.
 *
 * The values below match the current robot wiring:
 *   left tracker -> PC0 / ADC_IN10
 *   right tracker -> PC1 / ADC_IN11
 *   intersection -> PC2 / ADC_IN12
 */

#define FIELD_SENSOR_ADC_HANDLE hadc1
#define FIELD_SENSOR_ADC_TIMEOUT_MS 5U
#define FIELD_SENSOR_ADC_OVERSAMPLE_COUNT 4U

/*
 * These are firmware-side placeholder tuning values that depend on your
 * analog front-end gain and detector layout.
 *
 * Higher amplifier gain usually means larger ADC swings, so these thresholds
 * should usually be raised as gain increases and lowered as gain decreases.
 */
#define FIELD_SENSOR_TRACK_ENTRY_SIGNAL            60
#define FIELD_SENSOR_TRACK_EXIT_SIGNAL             35
#define FIELD_SENSOR_TRACK_STARTUP_MIN_FILTERED    80
#define FIELD_SENSOR_INTERSECTION_ENTRY_SIGNAL     140
#define FIELD_SENSOR_INTERSECTION_EXIT_SIGNAL      90
#define FIELD_SENSOR_INTERSECTION_STARTUP_MIN_FILTERED 140

/*
 * Replace this with the sample-time macro for your exact STM32 family.
 * Examples include ADC_SAMPLETIME_71CYCLES_5 or ADC_SAMPLETIME_160CYCLES_5.
 */
#define FIELD_SENSOR_ADC_SAMPLE_TIME ADC_SAMPLETIME_71CYCLES_5

#define FIELD_SENSOR_LEFT_GPIO_CLK_ENABLE()         __HAL_RCC_GPIOC_CLK_ENABLE()
#define FIELD_SENSOR_RIGHT_GPIO_CLK_ENABLE()        __HAL_RCC_GPIOC_CLK_ENABLE()
#define FIELD_SENSOR_INTERSECTION_GPIO_CLK_ENABLE() __HAL_RCC_GPIOC_CLK_ENABLE()

#define FIELD_SENSOR_LEFT_GPIO_PORT          GPIOC
#define FIELD_SENSOR_LEFT_GPIO_PIN           GPIO_PIN_0
#define FIELD_SENSOR_LEFT_ADC_CHANNEL        ADC_CHANNEL_10

#define FIELD_SENSOR_RIGHT_GPIO_PORT         GPIOC
#define FIELD_SENSOR_RIGHT_GPIO_PIN          GPIO_PIN_1
#define FIELD_SENSOR_RIGHT_ADC_CHANNEL       ADC_CHANNEL_11

#define FIELD_SENSOR_INTERSECTION_GPIO_PORT    GPIOC
#define FIELD_SENSOR_INTERSECTION_GPIO_PIN     GPIO_PIN_2
#define FIELD_SENSOR_INTERSECTION_ADC_CHANNEL  ADC_CHANNEL_12

#endif
