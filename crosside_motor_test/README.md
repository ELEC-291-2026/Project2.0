# CrossIDE Motor Test

This folder is a bare-metal STM32L051K8 motor test for the course CrossIDE
toolchain.  It does not use STM32 HAL or CubeMX.

Pin mapping:

- `PA0` (pin 6) -> left motor forward
- `PA1` (pin 7) -> left motor reverse
- `PA2` (pin 8) -> right motor forward
- `PA3` (pin 9) -> right motor reverse

The test sequence in `main.c` is:

1. Left `+600`, right `-600` for 3 seconds
2. Left `-600`, right `+600` for 3 seconds
3. Stop for 2 seconds

Build on macOS/Linux:

```sh
cd crosside_motor_test
make -f makefile.mac
```

Build with CrossIDE/Windows:

- Open `motor_test.mk`, or run `make -f motor_test.mk`

Generated output:

- `motor_test.elf`
- `motor_test.hex`

Notes:

- The support files in `../Common` came from the course STM32L051 bundle.
- TIM2 is used as a periodic interrupt source for software PWM on `PA0..PA3`,
  matching the example-style motor drive approach.
- This test uses opposite left/right command signs because the two motors
  need opposite polarity to move the robot in the same overall direction.
- If a motor spins backward, swap the motor wires or change the sign in
  `set_motor_outputs(...)`.
- `makefile.mac` includes an optional `flash` target, but it expects a local
  `stm32flash` tool path.
