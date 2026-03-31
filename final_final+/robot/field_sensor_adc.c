#include "../header_files/field_sensor_adc.h"
#include "../header_files/field_sensor_adc_config.h"
#include "../header_files/board.h"


#if FIELD_SENSOR_ADC_OVERSAMPLE_COUNT == 0U
#error "FIELD_SENSOR_ADC_OVERSAMPLE_COUNT must be greater than zero."
#endif

static void field_sensor_adc_gpio_init(void)
{
    GPIO_InitTypeDef gpio_init;

    FIELD_SENSOR_LEFT_GPIO_CLK_ENABLE();
    FIELD_SENSOR_RIGHT_GPIO_CLK_ENABLE();
    FIELD_SENSOR_INTERSECTION_GPIO_CLK_ENABLE();

    gpio_init.Pin = FIELD_SENSOR_LEFT_GPIO_PIN;
    gpio_init.Mode = GPIO_MODE_ANALOG;
    gpio_init.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(FIELD_SENSOR_LEFT_GPIO_PORT, &gpio_init);

    gpio_init.Pin = FIELD_SENSOR_RIGHT_GPIO_PIN;
    HAL_GPIO_Init(FIELD_SENSOR_RIGHT_GPIO_PORT, &gpio_init);

    gpio_init.Pin = FIELD_SENSOR_INTERSECTION_GPIO_PIN;
    HAL_GPIO_Init(FIELD_SENSOR_INTERSECTION_GPIO_PORT, &gpio_init);
}

static void field_sensor_adc_configure_channel(unsigned int channel)
{
    ADC_ChannelConfTypeDef channel_config;

    channel_config.Channel = channel;
    channel_config.Rank = ADC_RANK_CHANNEL_NUMBER;
    channel_config.SamplingTime = FIELD_SENSOR_ADC_SAMPLE_TIME;

#ifdef ADC_SINGLE_ENDED
    channel_config.SingleDiff = ADC_SINGLE_ENDED;
#endif

#ifdef ADC_OFFSET_NONE
    channel_config.OffsetNumber = ADC_OFFSET_NONE;
    channel_config.Offset = 0;
#endif

    HAL_ADC_ConfigChannel(&FIELD_SENSOR_ADC_HANDLE, &channel_config);
}

static unsigned int field_sensor_adc_read_averaged_channel(unsigned int channel)
{
    unsigned int sample_index;
    unsigned int sample_total;

    sample_total = 0U;

    for (sample_index = 0U;
         sample_index < FIELD_SENSOR_ADC_OVERSAMPLE_COUNT;
         ++sample_index)
    {
        sample_total += field_sensor_adc_read_channel(channel);
    }

    return sample_total / FIELD_SENSOR_ADC_OVERSAMPLE_COUNT;
}

void field_sensor_adc_init(void)
{
    field_sensor_adc_gpio_init();

    /*
     * If your STM32 family needs ADC calibration, add it here.
     * Examples vary across STM32 families, so this is left as a placeholder.
     */
}

unsigned int field_sensor_adc_read_channel(unsigned int channel)
{
    field_sensor_adc_configure_channel(channel);

    HAL_ADC_Start(&FIELD_SENSOR_ADC_HANDLE);
    HAL_ADC_PollForConversion(&FIELD_SENSOR_ADC_HANDLE, FIELD_SENSOR_ADC_TIMEOUT_MS);

    {
        unsigned int value = (unsigned int)HAL_ADC_GetValue(&FIELD_SENSOR_ADC_HANDLE);
        HAL_ADC_Stop(&FIELD_SENSOR_ADC_HANDLE);
        return value;
    }
}

void field_sensor_adc_read_raw(field_data_t *sensors)
{
    sensors->left_raw = (int)field_sensor_adc_read_averaged_channel(FIELD_SENSOR_LEFT_ADC_CHANNEL);
    sensors->right_raw = (int)field_sensor_adc_read_averaged_channel(FIELD_SENSOR_RIGHT_ADC_CHANNEL);
    sensors->intersection_raw = (int)field_sensor_adc_read_averaged_channel(FIELD_SENSOR_INTERSECTION_ADC_CHANNEL);
}

void field_sensor_adc_update(field_data_t *sensors)
{
    int left_sample;
    int right_sample;
    int intersection_sample;

    left_sample = (int)field_sensor_adc_read_averaged_channel(FIELD_SENSOR_LEFT_ADC_CHANNEL);
    right_sample = (int)field_sensor_adc_read_averaged_channel(FIELD_SENSOR_RIGHT_ADC_CHANNEL);
    intersection_sample = (int)field_sensor_adc_read_averaged_channel(FIELD_SENSOR_INTERSECTION_ADC_CHANNEL);

    field_sensor_update(sensors, left_sample, right_sample, intersection_sample);
}