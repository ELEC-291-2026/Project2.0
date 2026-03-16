#include "robot_auto_mode.h"

/*
 * Minimal autonomous-mode starter for the field-following robot.
 *
 * This version only solves the first milestone:
 * follow the guide wire with two sensors and stop if the signal is lost.
 *
 * Hook the marked stub functions into your STM32 board code:
 * - ADC sampling for the field sensors
 * - PWM outputs for the motors
 * - Optional UART debug output
 */

static robot_state_t g_state = ROBOT_AUTO_FOLLOW;

enum
{
    BASE_SPEED = 600,
    MAX_PWM = 1000,
    MIN_SIGNAL = 80,
    DEADBAND = 15,
    KP = 2
};

static int lowpass(int previous, int sample)
{
    return (3 * previous + sample) / 4;
}

static int clamp_pwm(int value)
{
    if (value < 0)
    {
        return 0;
    }

    if (value > MAX_PWM)
    {
        return MAX_PWM;
    }

    return value;
}

static void motor_stop(void)
{
    /*
     * Replace with your motor driver call, for example:
     * motor_set(0, 0);
     */
}

static void motor_set_forward(int left_pwm, int right_pwm)
{
    (void)left_pwm;
    (void)right_pwm;

    /*
     * Replace with your motor driver call, for example:
     * motor_set(left_pwm, right_pwm);
     */
}

void field_sensor_reset(field_data_t *sensors)
{
    sensors->left_raw = 0;
    sensors->right_raw = 0;
    sensors->left_filtered = 0;
    sensors->right_filtered = 0;
}

void field_sensor_update(field_data_t *sensors, int left_sample, int right_sample)
{
    sensors->left_raw = left_sample;
    sensors->right_raw = right_sample;

    sensors->left_filtered = lowpass(sensors->left_filtered, sensors->left_raw);
    sensors->right_filtered = lowpass(sensors->right_filtered, sensors->right_raw);
}

void robot_auto_mode_init(void)
{
    g_state = ROBOT_AUTO_FOLLOW;
}

void robot_auto_mode_step(field_data_t *sensors)
{
    int error;
    int correction;
    int left_pwm;
    int right_pwm;

    switch (g_state)
    {
        case ROBOT_AUTO_FOLLOW:
            if (sensors->left_filtered < MIN_SIGNAL &&
                sensors->right_filtered < MIN_SIGNAL)
            {
                motor_stop();
                g_state = ROBOT_AUTO_LOST;
                break;
            }

            error = sensors->left_filtered - sensors->right_filtered;

            if (error < DEADBAND && error > -DEADBAND)
            {
                error = 0;
            }

            correction = KP * error;

            left_pwm = clamp_pwm(BASE_SPEED - correction);
            right_pwm = clamp_pwm(BASE_SPEED + correction);

            motor_set_forward(left_pwm, right_pwm);
            break;

        case ROBOT_AUTO_LOST:
        case ROBOT_STOPPED:
        default:
            motor_stop();
            break;
    }
}
