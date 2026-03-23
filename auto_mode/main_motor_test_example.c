#include "main.h"
#include "hbridge_motor.h"

/*
 * Ready-to-paste example for STM32L051K8 using:
 * - PA0 -> TIM2_CH1 -> left motor forward
 * - PA1 -> TIM2_CH2 -> left motor reverse
 * - PA2 -> TIM2_CH3 -> right motor forward
 * - PA3 -> TIM2_CH4 -> right motor reverse
 *
 * Assumptions:
 * - CubeMX generated MX_GPIO_Init() and MX_TIM2_Init()
 * - htim2 exists in your project
 * - hbridge_motor.c/.h/.config.h were added to the build
 */

static void motor_test_left_only(void)
{
    motor_command_t command = { 600, 0 };
    hbridge_motor_apply(&command);
}

static void motor_test_right_only(void)
{
    motor_command_t command = { 0, 600 };
    hbridge_motor_apply(&command);
}

static void motor_test_both_forward(void)
{
    motor_command_t command = { 600, 600 };
    hbridge_motor_apply(&command);
}

static void motor_stop(void)
{
    motor_command_t command = { 0, 0 };
    hbridge_motor_apply(&command);
}

int main(void)
{
    HAL_Init();
    SystemClock_Config();

    MX_GPIO_Init();
    MX_TIM2_Init();

    hbridge_motor_init();

    while (1)
    {
        motor_test_left_only();
        HAL_Delay(3000);

        motor_stop();
        HAL_Delay(1000);

        motor_test_right_only();
        HAL_Delay(3000);

        motor_stop();
        HAL_Delay(1000);

        motor_test_both_forward();
        HAL_Delay(3000);

        motor_stop();
        HAL_Delay(2000);
    }
}
