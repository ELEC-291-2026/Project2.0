#include "robot_auto_mode.h"
#include "field_sensor_adc_config.h"
#include "hbridge_motor.h"

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
    { PATH_STRAIGHT, PATH_LEFT,     PATH_RIGHT,    PATH_STOP,
      PATH_STOP,     PATH_STOP,     PATH_STOP,     PATH_STOP },
    { PATH_LEFT,     PATH_RIGHT,    PATH_LEFT,     PATH_RIGHT,
      PATH_STRAIGHT, PATH_STRAIGHT, PATH_STOP,     PATH_STOP },
    { PATH_RIGHT,    PATH_STRAIGHT, PATH_RIGHT,    PATH_LEFT,
      PATH_RIGHT,    PATH_LEFT,     PATH_STRAIGHT, PATH_STOP }
};

enum
{
    BASE_SPEED = 420,           /* forward drive speed (out of MAX_PWM) */
    SLOW_SPEED = 0,             /* inner-wheel speed during a steer â€” full pivot */
    MAX_PWM = 1000,
    STEER_DEADBAND = 80,        /* raw ADC diff: below this = go straight */
    TRACK_THRESHOLD = 200,      /* raw ADC: either sensor above = wire visible */
    INTERSECTION_THRESHOLD = 100,  /* raw ADC center sensor = intersection */
    FILTER_KEEP_COUNT = 3,
    BASELINE_IDLE_KEEP_COUNT = 15,
    BASELINE_STARTUP_KEEP_COUNT = 7,
    STARTUP_SETTLE_SAMPLES = 16,
    TURN_SPEED = 600
};

static int mix_samples(int previous, int sample, int keep_count)
{
    return (keep_count * previous + sample) / (keep_count + 1);
}

static int absolute_difference(int left_value, int right_value)
{
    if (left_value >= right_value)
    {
        return left_value - right_value;
    }

    return right_value - left_value;
}

static int detection_active(int was_detected,
    int signal,
    int filtered,
    int entry_signal,
    int exit_signal,
    int startup_filtered_threshold)
{
    if (was_detected)
    {
        return signal >= exit_signal;
    }

    return signal >= entry_signal || filtered >= startup_filtered_threshold;
}

static int baseline_keep_count(unsigned int samples_seen)
{
    if (samples_seen < STARTUP_SETTLE_SAMPLES)
    {
        return BASELINE_STARTUP_KEEP_COUNT;
    }

    return BASELINE_IDLE_KEEP_COUNT;
}

static void update_sensor_channel(unsigned int samples_seen,
    int sample,
    int entry_signal,
    int exit_signal,
    int startup_filtered_threshold,
    int *raw_value,
    int *filtered_value,
    int *baseline_value,
    int *signal_value,
    int *detected_value)
{
    int filtered_sample;
    int next_signal;
    int next_detected;

    *raw_value = sample;
    filtered_sample = mix_samples(*filtered_value, sample, FILTER_KEEP_COUNT);
    next_signal = absolute_difference(filtered_sample, *baseline_value);
    next_detected = detection_active(*detected_value,
        next_signal,
        filtered_sample,
        entry_signal,
        exit_signal,
        startup_filtered_threshold);
    if (!next_detected)
    {
        *baseline_value = mix_samples(*baseline_value,
            filtered_sample,
            baseline_keep_count(samples_seen));
    }
    next_signal = absolute_difference(filtered_sample, *baseline_value);
    next_detected = detection_active(*detected_value,
        next_signal,
        filtered_sample,
        entry_signal,
        exit_signal,
        startup_filtered_threshold);

    *filtered_value = filtered_sample;
    *signal_value = next_signal;
    *detected_value = next_detected;
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
    return sensors->intersection_raw > INTERSECTION_THRESHOLD;
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
    int diff = sensors->left_raw - sensors->right_raw;

    if (diff > STEER_DEADBAND)
    {
        /* Wire is to the left â€” slow right wheel to steer left */
        motor_set_signed(SLOW_SPEED, BASE_SPEED);
    }
    else if (diff < -STEER_DEADBAND)
    {
        /* Wire is to the right â€” slow left wheel to steer right */
        motor_set_signed(BASE_SPEED, SLOW_SPEED);
    }
    else
    {
        motor_set_signed(BASE_SPEED, BASE_SPEED);
    }
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
    sensors->left_baseline = 0;
    sensors->right_baseline = 0;
    sensors->intersection_baseline = 0;
    sensors->left_signal = 0;
    sensors->right_signal = 0;
    sensors->intersection_signal = 0;
    sensors->left_detected = 0;
    sensors->right_detected = 0;
    sensors->intersection_detected = 0;
    sensors->samples_seen = 0U;
}

void field_sensor_update(field_data_t *sensors,
    int left_sample,
    int right_sample,
    int intersection_sample)
{
    update_sensor_channel(sensors->samples_seen,
        left_sample,
        FIELD_SENSOR_TRACK_ENTRY_SIGNAL,
        FIELD_SENSOR_TRACK_EXIT_SIGNAL,
        FIELD_SENSOR_TRACK_STARTUP_MIN_FILTERED,
        &sensors->left_raw,
        &sensors->left_filtered,
        &sensors->left_baseline,
        &sensors->left_signal,
        &sensors->left_detected);
    update_sensor_channel(sensors->samples_seen,
        right_sample,
        FIELD_SENSOR_TRACK_ENTRY_SIGNAL,
        FIELD_SENSOR_TRACK_EXIT_SIGNAL,
        FIELD_SENSOR_TRACK_STARTUP_MIN_FILTERED,
        &sensors->right_raw,
        &sensors->right_filtered,
        &sensors->right_baseline,
        &sensors->right_signal,
        &sensors->right_detected);
    update_sensor_channel(sensors->samples_seen,
        intersection_sample,
        FIELD_SENSOR_INTERSECTION_ENTRY_SIGNAL,
        FIELD_SENSOR_INTERSECTION_EXIT_SIGNAL,
        FIELD_SENSOR_INTERSECTION_STARTUP_MIN_FILTERED,
        &sensors->intersection_raw,
        &sensors->intersection_filtered,
        &sensors->intersection_baseline,
        &sensors->intersection_signal,
        &sensors->intersection_detected);

    if (sensors->samples_seen < 0xFFFFFFFFU)
    {
        ++sensors->samples_seen;
    }
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

void robot_auto_mode_set_path(path_context_t *context, path_id_t selected_path)
{
    context->selected_path = selected_path;
    context->intersection_count = 0;
}

robot_state_t robot_auto_mode_get_state(void)
{
    return g_state;
}

void robot_auto_mode_step(field_data_t *sensors, path_context_t *context)
{
    int detected_now;

    detected_now = intersection_detected(sensors);

    switch (g_state)
    {
        case ROBOT_AUTO_FOLLOW:
            if (intersection_started(context, detected_now))
            {
                g_active_action = path_context_on_intersection(context);
                motor_stop();
                delayms(400);
                if (g_active_action == PATH_LEFT || g_active_action == PATH_RIGHT)
                {
                    run_path_action(g_active_action);
                    delayms(900);
                    motor_stop();
                    g_state = ROBOT_AUTO_FOLLOW;
                }
                else if (g_active_action == PATH_STOP)
                {
                    g_state = ROBOT_AUTO_STOP;
                }
                else
                {
                    g_state = ROBOT_AUTO_FOLLOW;
                }
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
            /* Keep steering â€” never stop while searching for the wire */
            run_follow_controller(sensors);
            if (sensors->left_raw > TRACK_THRESHOLD ||
                sensors->right_raw > TRACK_THRESHOLD)
            {
                g_state = ROBOT_AUTO_FOLLOW;
            }
            break;

        case ROBOT_AUTO_STOP:
        default:
            motor_stop();
            break;
    }
}
