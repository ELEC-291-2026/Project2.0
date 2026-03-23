#include "robot_auto_mode.h"

/*
 * Auto-mode starter based on the project slides:
 *
 * - Left field detector -> ADC
 * - Right field detector -> ADC
 * - Intersection detector -> ADC
 * - Two H-bridges -> signed motor commands
 *
 * The main goal is to keep the robot centered by driving until d1 == d2.
 * Path decisions are made from the selected path and the number of distinct
 * intersections the robot has crossed.
 */

static robot_state_t g_state = ROBOT_AUTO_FOLLOW;
static path_action_t g_active_action = PATH_STRAIGHT;

static const path_action_t k_path_table[3][8] =
{
    { PATH_STRAIGHT, PATH_LEFT,     PATH_LEFT,     PATH_STRAIGHT,
      PATH_RIGHT,    PATH_LEFT,     PATH_RIGHT,    PATH_STOP },
    { PATH_LEFT,     PATH_RIGHT,    PATH_LEFT,     PATH_RIGHT,
      PATH_STRAIGHT, PATH_STRAIGHT, PATH_STOP,     PATH_STOP },
    { PATH_RIGHT,    PATH_STRAIGHT, PATH_RIGHT,    PATH_LEFT,
      PATH_RIGHT,    PATH_LEFT,     PATH_STRAIGHT, PATH_STOP }
};

enum
{
    BASE_SPEED = 600,
    MAX_PWM = 1000,
    MIN_SIGNAL = 80,
    DEADBAND = 15,
    KP = 2,
    INTERSECTION_THRESHOLD = 140,
    TURN_SPEED = 500
};

static int lowpass(int previous, int sample)
{
    return (3 * previous + sample) / 4;
}

static int clamp_pwm(int value)
{
    if (value < -MAX_PWM)
    {
        return -MAX_PWM;
    }

    if (value > MAX_PWM)
    {
        return MAX_PWM;
    }

    return value;
}

static motor_command_t make_motor_command(int left_command, int right_command)
{
    motor_command_t command;

    command.left_command = clamp_pwm(left_command);
    command.right_command = clamp_pwm(right_command);

    return command;
}

static void motor_stop(void)
{
    motor_command_t command = make_motor_command(0, 0);
    hbridge_motor_apply(&command);
}

static void motor_set_signed(int left_command, int right_command)
{
    motor_command_t command = make_motor_command(left_command, right_command);
    hbridge_motor_apply(&command);
}

static int intersection_detected(const field_data_t *sensors)
{
    return sensors->intersection_filtered >= INTERSECTION_THRESHOLD;
}

static int intersection_started(path_context_t *context, int detected_now)
{
    if (detected_now && !context->intersection_active)
    {
        context->intersection_active = 1;
        return 1;
    }

    if (!detected_now)
    {
        context->intersection_active = 0;
    }

    return 0;
}

static void run_follow_controller(const field_data_t *sensors)
{
    int error;
    int correction;
    int left_pwm;
    int right_pwm;

    error = sensors->left_filtered - sensors->right_filtered;

    if (error < DEADBAND && error > -DEADBAND)
    {
        error = 0;
    }

    correction = KP * error;

    left_pwm = BASE_SPEED - correction;
    right_pwm = BASE_SPEED + correction;

    motor_set_signed(left_pwm, right_pwm);
}

static void run_path_action(path_action_t action)
{
    switch (action)
    {
        case PATH_LEFT:
            motor_set_signed(-TURN_SPEED, TURN_SPEED);
            break;

        case PATH_RIGHT:
            motor_set_signed(TURN_SPEED, -TURN_SPEED);
            break;

        case PATH_STOP:
            motor_stop();
            break;

        case PATH_STRAIGHT:
        default:
            motor_set_signed(BASE_SPEED, BASE_SPEED);
            break;
    }
}

void field_sensor_reset(field_data_t *sensors)
{
    sensors->left_raw = 0;
    sensors->right_raw = 0;
    sensors->intersection_raw = 0;
    sensors->left_filtered = 0;
    sensors->right_filtered = 0;
    sensors->intersection_filtered = 0;
}

void field_sensor_update(field_data_t *sensors,
    int left_sample,
    int right_sample,
    int intersection_sample)
{
    sensors->left_raw = left_sample;
    sensors->right_raw = right_sample;
    sensors->intersection_raw = intersection_sample;

    sensors->left_filtered = lowpass(sensors->left_filtered, sensors->left_raw);
    sensors->right_filtered = lowpass(sensors->right_filtered, sensors->right_raw);
    sensors->intersection_filtered = lowpass(
        sensors->intersection_filtered,
        sensors->intersection_raw);
}

void path_context_reset(path_context_t *context, path_id_t selected_path)
{
    context->selected_path = selected_path;
    context->intersection_count = 0;
    context->intersection_active = 0;
}

path_action_t path_context_on_intersection(path_context_t *context)
{
    if (context->intersection_count >= 8)
    {
        return PATH_STOP;
    }

    return k_path_table[context->selected_path][context->intersection_count++];
}

void robot_auto_mode_init(path_context_t *context, path_id_t selected_path)
{
    g_state = ROBOT_AUTO_FOLLOW;
    g_active_action = PATH_STRAIGHT;
    path_context_reset(context, selected_path);
}

void robot_auto_mode_step(field_data_t *sensors, path_context_t *context)
{
    int detected_now;

    detected_now = intersection_detected(sensors);

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

            if (intersection_started(context, detected_now))
            {
                g_active_action = path_context_on_intersection(context);
                g_state = ROBOT_AUTO_INTERSECTION;
                run_path_action(g_active_action);
                break;
            }

            run_follow_controller(sensors);
            break;

        case ROBOT_AUTO_INTERSECTION:
            if (!detected_now)
            {
                context->intersection_active = 0;

                if (g_active_action == PATH_STOP)
                {
                    g_state = ROBOT_AUTO_STOP;
                    motor_stop();
                    break;
                }

                g_state = ROBOT_AUTO_FOLLOW;
            }
            else
            {
                run_path_action(g_active_action);
            }
            break;

        case ROBOT_AUTO_LOST:
        case ROBOT_AUTO_STOP:
        default:
            motor_stop();
            break;
    }
}
