#ifndef FIELD_SENSOR_ADC_CONFIG_H
#define FIELD_SENSOR_ADC_CONFIG_H

/*
 * Update these macros to match your STM32 CubeMX pinout.
 *
 * This header is written in STM32 HAL style, so it expects your actual project
 * to provide symbols like GPIOA, GPIO_PIN_0, ADC_CHANNEL_0, hadc1, etc.
 *
 * The values below match the current robot wiring:
 *   left tracker -> PA4 / ADC_IN4
 *   right tracker -> PA5 / ADC_IN5
 *   intersection -> PA6 / ADC_IN6
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
#define FIELD_SENSOR_TRACK_ENTRY_SIGNAL            200
#define FIELD_SENSOR_TRACK_EXIT_SIGNAL             100
#define FIELD_SENSOR_TRACK_STARTUP_MIN_FILTERED    4096
#define FIELD_SENSOR_INTERSECTION_ENTRY_SIGNAL     9999
#define FIELD_SENSOR_INTERSECTION_EXIT_SIGNAL      100
#define FIELD_SENSOR_INTERSECTION_STARTUP_MIN_FILTERED 4096

/*
 * Replace this with the sample-time macro for your exact STM32 family.
 * Examples include ADC_SAMPLETIME_71CYCLES_5 or ADC_SAMPLETIME_160CYCLES_5.
 */
#define FIELD_SENSOR_ADC_SAMPLE_TIME ADC_SAMPLETIME_71CYCLES_5

#define FIELD_SENSOR_LEFT_GPIO_CLK_ENABLE()         __HAL_RCC_GPIOA_CLK_ENABLE()
#define FIELD_SENSOR_RIGHT_GPIO_CLK_ENABLE()        __HAL_RCC_GPIOA_CLK_ENABLE()
#define FIELD_SENSOR_INTERSECTION_GPIO_CLK_ENABLE() __HAL_RCC_GPIOA_CLK_ENABLE()

#define FIELD_SENSOR_LEFT_GPIO_PORT          GPIOA
#define FIELD_SENSOR_LEFT_GPIO_PIN           GPIO_PIN_4
#define FIELD_SENSOR_LEFT_ADC_CHANNEL        ADC_CHANNEL_4

#define FIELD_SENSOR_RIGHT_GPIO_PORT         GPIOA
#define FIELD_SENSOR_RIGHT_GPIO_PIN          GPIO_PIN_5
#define FIELD_SENSOR_RIGHT_ADC_CHANNEL       ADC_CHANNEL_5

#define FIELD_SENSOR_INTERSECTION_GPIO_PORT    GPIOA
#define FIELD_SENSOR_INTERSECTION_GPIO_PIN     GPIO_PIN_6
#define FIELD_SENSOR_INTERSECTION_ADC_CHANNEL  ADC_CHANNEL_6

#endif
