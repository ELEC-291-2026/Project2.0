#include <stdint.h>
#include "../header_files/board.h"
#include "../header_files/hbridge_motor.h"
#include "../header_files/hbridge_motor_config.h"

#if HBRIDGE_COMMAND_MAX == 0
#error "HBRIDGE_COMMAND_MAX must be greater than zero."
#endif

typedef struct
{
    TIM_HandleTypeDef *timer;
    uint32_t channel;
} hbridge_pwm_channel_t;

static const hbridge_pwm_channel_t k_left_forward =
{
    &HBRIDGE_LEFT_FORWARD_TIMER,
    HBRIDGE_LEFT_FORWARD_CHANNEL
};

static const hbridge_pwm_channel_t k_left_reverse =
{
    &HBRIDGE_LEFT_REVERSE_TIMER,
    HBRIDGE_LEFT_REVERSE_CHANNEL
};

static const hbridge_pwm_channel_t k_right_forward =
{
    &HBRIDGE_RIGHT_FORWARD_TIMER,
    HBRIDGE_RIGHT_FORWARD_CHANNEL
};

static const hbridge_pwm_channel_t k_right_reverse =
{
    &HBRIDGE_RIGHT_REVERSE_TIMER,
    HBRIDGE_RIGHT_REVERSE_CHANNEL
};

static motor_command_t g_last_command = { 0, 0 };

static unsigned int command_magnitude(int command)
{
    if (command < 0)
    {
        return (unsigned int)(-command);
    }

    return (unsigned int)command;
}

static unsigned int clamp_command(int command)
{
    unsigned int magnitude = command_magnitude(command);

    if (magnitude > HBRIDGE_COMMAND_MAX)
    {
        return HBRIDGE_COMMAND_MAX;
    }

    return magnitude;
}

static uint32_t command_to_compare(const hbridge_pwm_channel_t *output, unsigned int magnitude)
{
    uint32_t period = __HAL_TIM_GET_AUTORELOAD(output->timer);

    if (magnitude == 0U || period == 0U)
    {
        return 0U;
    }

    return (uint32_t)(((uint64_t)magnitude * (uint64_t)period) / HBRIDGE_COMMAND_MAX);
}

static void set_pwm_compare(const hbridge_pwm_channel_t *output, uint32_t compare)
{
    __HAL_TIM_SET_COMPARE(output->timer, output->channel, compare);
}

static void motor_apply_single(const hbridge_pwm_channel_t *forward,
    const hbridge_pwm_channel_t *reverse,
    int command)
{
    unsigned int magnitude = clamp_command(command);

    if (command > 0)
    {
        set_pwm_compare(forward, command_to_compare(forward, magnitude));
        set_pwm_compare(reverse, 0U);
    }
    else if (command < 0)
    {
        set_pwm_compare(forward, 0U);
        set_pwm_compare(reverse, command_to_compare(reverse, magnitude));
    }
    else
    {
        set_pwm_compare(forward, 0U);
        set_pwm_compare(reverse, 0U);
    }
}

void hbridge_motor_init(void)
{
    HAL_TIM_PWM_Start(k_left_forward.timer, k_left_forward.channel);
    HAL_TIM_PWM_Start(k_left_reverse.timer, k_left_reverse.channel);
    HAL_TIM_PWM_Start(k_right_forward.timer, k_right_forward.channel);
    HAL_TIM_PWM_Start(k_right_reverse.timer, k_right_reverse.channel);

    hbridge_motor_stop_all();
}

void hbridge_motor_stop_all(void)
{
    motor_apply_single(&k_left_forward, &k_left_reverse, 0);
    motor_apply_single(&k_right_forward, &k_right_reverse, 0);
    g_last_command.left_command = 0;
    g_last_command.right_command = 0;
}

void hbridge_motor_apply(const motor_command_t *command)
{
    if (command == 0)
    {
        hbridge_motor_stop_all();
        return;
    }

    g_last_command.left_command = command->left_command;
    g_last_command.right_command = command->right_command;

    motor_apply_single(&k_left_forward, &k_left_reverse, command->left_command);
    motor_apply_single(&k_right_forward, &k_right_reverse, command->right_command);
}

motor_command_t hbridge_motor_get_last_command(void)
{
    return g_last_command;
}
