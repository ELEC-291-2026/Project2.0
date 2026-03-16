#ifndef ROBOT_AUTO_MODE_H
#define ROBOT_AUTO_MODE_H

typedef enum
{
    ROBOT_STOPPED = 0,
    ROBOT_AUTO_FOLLOW,
    ROBOT_AUTO_LOST
} robot_state_t;

typedef struct
{
    int left_raw;
    int right_raw;
    int left_filtered;
    int right_filtered;
} field_data_t;

void field_sensor_reset(field_data_t *sensors);
void field_sensor_update(field_data_t *sensors, int left_sample, int right_sample);
void robot_auto_mode_init(void);
void robot_auto_mode_step(field_data_t *sensors);

#endif
