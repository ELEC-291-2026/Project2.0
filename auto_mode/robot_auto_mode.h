#ifndef ROBOT_AUTO_MODE_H
#define ROBOT_AUTO_MODE_H

typedef enum
{
    ROBOT_AUTO_STOP = 0,
    ROBOT_AUTO_FOLLOW,
    ROBOT_AUTO_INTERSECTION,
    ROBOT_AUTO_LOST
} robot_state_t;

typedef enum
{
    PATH_STRAIGHT = 0,
    PATH_LEFT,
    PATH_RIGHT,
    PATH_STOP
} path_action_t;

typedef enum
{
    PATH_ID_1 = 0,
    PATH_ID_2,
    PATH_ID_3
} path_id_t;

typedef struct
{
    int left_raw;
    int right_raw;
    int intersection_raw;
    int left_filtered;
    int right_filtered;
    int intersection_filtered;
    int left_baseline;
    int right_baseline;
    int intersection_baseline;
    int left_signal;
    int right_signal;
    int intersection_signal;
    int left_detected;
    int right_detected;
    int intersection_detected;
    unsigned int samples_seen;
} field_data_t;

typedef struct
{
    int left_command;
    int right_command;
} motor_command_t;

typedef struct
{
    path_id_t selected_path;
    int intersection_count;
    int intersection_active;
} path_context_t;

void field_sensor_reset(field_data_t *sensors);
void field_sensor_update(field_data_t *sensors,
    int left_sample,
    int right_sample,
    int intersection_sample);
void path_context_reset(path_context_t *context, path_id_t selected_path);
path_action_t path_context_on_intersection(path_context_t *context);
void hbridge_motor_apply(const motor_command_t *command);
void robot_auto_mode_init(path_context_t *context, path_id_t selected_path);
void robot_auto_mode_step(field_data_t *sensors, path_context_t *context);

#endif
