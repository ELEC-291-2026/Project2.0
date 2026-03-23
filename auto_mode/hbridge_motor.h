#ifndef HBRIDGE_MOTOR_H
#define HBRIDGE_MOTOR_H

#include "robot_auto_mode.h"

void hbridge_motor_init(void);
void hbridge_motor_stop_all(void);
void hbridge_motor_apply(const motor_command_t *command);

#endif
