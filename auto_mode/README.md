# Auto Mode Starter

This folder contains a minimal starting point for the robot's autonomous mode
based on the project slides.

The software assumes:

- two field detectors are used for left/right tracking
- one field detector is used for intersection detection
- all three detector outputs go to ADC inputs
- each motor is controlled through an H-bridge

The initial goal is to:

1. Read the three detector voltages through the ADC.
2. Keep the robot centered over the guide wire.
3. Detect intersections.
4. Choose the next action from the selected path and the number of times an intersection has been met.
5. Leave a clean place to plug in obstacle handling.

Suggested module split for the full firmware:

- `board.c`: clocks, GPIO, timer, ADC, I2C init
- `field_sensor_adc_config.h`: editable ADC pin and channel mapping
- `field_sensor_adc.c`: STM32 HAL ADC reads for the 3 field sensors
- `hbridge_motor.c`: signed motor command to GPIO/PWM mapping
- `field_sensor.c`: ADC reads and filtering for left/right/intersection
- `robot_auto_mode.c`: state machine and steering logic
- `path_manager.c`: path selection and intersection-count lookup
- `vl53l0x.c`: obstacle sensing
- `ir_rx.c`: remote command handling

Recommended control-loop flow:

1. Update field sensors.
2. Check for obstacle or signal-loss conditions.
3. Detect whether a new intersection has started.
4. Run one state-machine step.
5. Apply signed motor outputs to the H-bridges.

Current starter states:

- `ROBOT_AUTO_FOLLOW`
- `ROBOT_AUTO_INTERSECTION`
- `ROBOT_AUTO_STOP`
- `ROBOT_AUTO_LOST`

Current starter abstractions:

- `field_sensor_adc_config.h` is where you change ADC pins and channels
- `field_sensor_adc_update(...)` reads the 3 ADC channels and updates filtering
- `field_sensor_update(...)` applies filtering to three samples
- `hbridge_motor_apply(...)` is the motor integration point
- `hbridge_motor_config.h` is where you change the STM32 PWM timer/channel mapping
- `hbridge_motor_init()` starts the four PWM outputs used by the two H-bridges
- `path_context_t` tracks the selected path and how many intersections have been crossed

Important behavior:

- The path action is chosen when a new intersection starts.
- The code does not count the same intersection multiple times while the robot is still physically over it.
- `field_sensor_adc.c` uses placeholder STM32 HAL constants and may need small family-specific tweaks before it compiles in your real project.

Minimal motor bring-up:

```c
#include "hbridge_motor.h"

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_TIM2_Init(); /* Replace with your actual PWM timer init. */

    hbridge_motor_init();

    while (1)
    {
        motor_command_t forward = { 600, 600 };
        hbridge_motor_apply(&forward);
    }
}
```

Before using the example above:

1. Configure four PWM outputs in CubeMX.
2. Update `hbridge_motor_config.h` to match your timer/channel names.
3. Set the PWM period so a command of `1000` is full speed.
