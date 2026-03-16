# Auto Mode Starter

This folder contains a minimal starting point for the robot's autonomous mode.

The design is intentionally small:

- `robot_auto_mode.c` contains a simple follow-the-wire state machine.
- `robot_auto_mode.h` exposes the data types and entry points.

The initial goal is not to solve the whole project at once. It is to:

1. Read the left and right field sensors.
2. Keep the robot centered over the guide wire.
3. Stop safely when the signal is lost.

After this works on the bench, the next layers are:

1. Add intersection detection using a third sensor.
2. Add pre-configured paths.
3. Add VL53L0X obstacle detection.
4. Add remote control integration.

Suggested module split for the full firmware:

- `board.c`: clocks, GPIO, timer, ADC, I2C init
- `motor.c`: H-bridge direction and PWM control
- `field_sensor.c`: ADC reads and filtering
- `robot_auto_mode.c`: state machine and steering logic
- `intersection.c`: third-sensor intersection detection
- `path_manager.c`: stored route steps
- `vl53l0x.c`: obstacle sensing
- `ir_rx.c`: remote command handling

Recommended control-loop flow:

1. Update field sensors.
2. Check safety conditions.
3. Run one state-machine step.
4. Update motor outputs.
