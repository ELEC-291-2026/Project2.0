#ifndef HBRIDGE_MOTOR_CONFIG_H
#define HBRIDGE_MOTOR_CONFIG_H

/*
 * Update these macros to match your PWM setup.
 *
 * The standalone auto-mode firmware in this folder uses TIM2 CH1..CH4
 * on PA0..PA3 by default, matching the motor test target.
 *
 * Example below:
 * - one timer provides four PWM outputs
 * - each motor uses two control inputs
 * - positive command  -> forward pin gets PWM, reverse pin is off
 * - negative command  -> reverse pin gets PWM, forward pin is off
 */

#define HBRIDGE_COMMAND_MAX 1000U

#define HBRIDGE_LEFT_FORWARD_TIMER    htim2
#define HBRIDGE_LEFT_FORWARD_CHANNEL  TIM_CHANNEL_1

#define HBRIDGE_LEFT_REVERSE_TIMER    htim2
#define HBRIDGE_LEFT_REVERSE_CHANNEL  TIM_CHANNEL_2

#define HBRIDGE_RIGHT_FORWARD_TIMER   htim2
#define HBRIDGE_RIGHT_FORWARD_CHANNEL TIM_CHANNEL_3

#define HBRIDGE_RIGHT_REVERSE_TIMER   htim2
#define HBRIDGE_RIGHT_REVERSE_CHANNEL TIM_CHANNEL_4

#endif
