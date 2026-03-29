#include "main.h"
#include "collision_detector.h"
#include "debug_uart.h"
#include "field_sensor_adc.h"
#include "hbridge_motor.h"
#include "robot_auto_mode.h"

enum
{
    AUTO_MODE_DEFAULT_PATH = PATH_ID_1,
    AUTO_MODE_SENSOR_WARMUP_SAMPLES = 64,
    AUTO_MODE_LOOP_DELAY_MS = 10,
    AUTO_MODE_TELEMETRY_PERIOD_LOOPS = 10
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

static void write_sensor_value(const char *label, int value)
{
    debug_uart_write_string(label);
    debug_uart_write_int(value);
}

static void write_detector_flag(const char *label, int value)
{
    debug_uart_write_string(label);
    debug_uart_write_uint((unsigned int)(value != 0));
}

static const char *motor_movement_label(const motor_command_t *command)
{
    if (command->left_command == 0 && command->right_command == 0)
    {
        return "STOP";
    }

    if (command->left_command > 0 && command->right_command > 0)
    {
        if (command->left_command == command->right_command)
        {
            return "FORWARD";
        }

        if (command->left_command > command->right_command)
        {
            return "VEER_RIGHT";
        }

        return "VEER_LEFT";
    }

    if (command->left_command < 0 && command->right_command < 0)
    {
        if (command->left_command == command->right_command)
        {
            return "REVERSE";
        }

        if (command->left_command < command->right_command)
        {
            return "REV_RIGHT";
        }

        return "REV_LEFT";
    }

    if (command->left_command < 0 && command->right_command > 0)
    {
        return "TURN_LEFT";
    }

    if (command->left_command > 0 && command->right_command < 0)
    {
        return "TURN_RIGHT";
    }

    if (command->left_command == 0)
    {
        if (command->right_command > 0)
        {
            return "PIVOT_LEFT";
        }

        return "PIVOT_RIGHT";
    }

    if (command->left_command > 0)
    {
        return "ARC_RIGHT";
    }

    return "ARC_LEFT";
}

static void write_field_telemetry(const field_data_t *sensors,
    const collision_detector_t *collision)
{
    motor_command_t motor_command = hbridge_motor_get_last_command();

    write_sensor_value("Lraw=", sensors->left_raw);
    debug_uart_write_string(" ");
    write_sensor_value("Rraw=", sensors->right_raw);
    debug_uart_write_string(" ");
    write_sensor_value("Xraw=", sensors->intersection_raw);
    debug_uart_write_string(" | ");
    write_sensor_value("Lsig=", sensors->left_signal);
    debug_uart_write_string(" ");
    write_sensor_value("Rsig=", sensors->right_signal);
    debug_uart_write_string(" ");
    write_sensor_value("Xsig=", sensors->intersection_signal);
    debug_uart_write_string(" | ");
    write_detector_flag("Ld=", sensors->left_detected);
    debug_uart_write_string(" ");
    write_detector_flag("Rd=", sensors->right_detected);
    debug_uart_write_string(" ");
    write_detector_flag("Xd=", sensors->intersection_detected);
    debug_uart_write_string(" ");
    write_detector_flag("Obs=", collision->obstacle_detected);
    debug_uart_write_string(" | ");
    debug_uart_write_string("Move=");
    debug_uart_write_string(motor_movement_label(&motor_command));
    debug_uart_write_string(" ");
    write_sensor_value("Lcmd=", motor_command.left_command);
    debug_uart_write_string(" ");
    write_sensor_value("Rcmd=", motor_command.right_command);
    debug_uart_write_string("\r\n");
}

void main(void)
{
    field_data_t sensors;
    path_context_t path_context;
    collision_detector_t collision;
    unsigned int telemetry_loop_count;

    board_init();
    debug_uart_init();
    field_sensor_adc_init();
    hbridge_motor_init();

    debug_uart_write_string("\r\nAuto mode field detector telemetry\r\n");
    debug_uart_write_string("USART1 on PA9/PA10 at 115200 baud\r\n");

    field_sensor_reset(&sensors);
    collision_detector_init(&collision);
    warm_up_field_sensors(&sensors);
    robot_auto_mode_init(&path_context, (path_id_t)AUTO_MODE_DEFAULT_PATH);
    telemetry_loop_count = 0U;

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

        ++telemetry_loop_count;
        if (telemetry_loop_count >= AUTO_MODE_TELEMETRY_PERIOD_LOOPS)
        {
            telemetry_loop_count = 0U;
            write_field_telemetry(&sensors, &collision);
        }

        delayms(AUTO_MODE_LOOP_DELAY_MS);
    }
}
