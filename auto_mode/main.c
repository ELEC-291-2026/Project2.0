#include "main.h"
#include "collision_detector.h"
#include "field_sensor_adc.h"
#include "hbridge_motor.h"
#include "robot_auto_mode.h"

enum
{
    AUTO_MODE_DEFAULT_PATH = PATH_ID_1,
    AUTO_MODE_SENSOR_WARMUP_SAMPLES = 64,
    AUTO_MODE_LOOP_DELAY_MS = 10
};

static void warm_up_field_sensors(field_data_t *sensors)
{
    unsigned int sample_index;

    for (sample_index = 0U;
         sample_index < AUTO_MODE_SENSOR_WARMUP_SAMPLES;
         ++sample_index)
    {
        field_sensor_adc_update(sensors);
        delayms(2U);
    }
}

void main(void)
{
    field_data_t sensors;
    path_context_t path_context;
    collision_detector_t collision;

    board_init();
    field_sensor_adc_init();
    hbridge_motor_init();

    field_sensor_reset(&sensors);
    collision_detector_init(&collision);
    warm_up_field_sensors(&sensors);
    robot_auto_mode_init(&path_context, (path_id_t)AUTO_MODE_DEFAULT_PATH);

    while (1)
    {
        field_sensor_adc_update(&sensors);
        collision_detector_update(&collision);

        if (collision.obstacle_detected)
        {
            hbridge_motor_stop_all();
        }
        else
        {
            robot_auto_mode_step(&sensors, &path_context);
        }

        delayms(AUTO_MODE_LOOP_DELAY_MS);
    }
}
