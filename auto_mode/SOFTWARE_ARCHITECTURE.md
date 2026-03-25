# Auto Mode Software Architecture

This document describes the standalone `auto_mode` firmware target used to test
magnetic-wire following on the STM32L051 robot.

## Purpose

The `auto_mode` folder is a self-contained firmware slice for one specific job:

- read three field-detector voltages
- follow the magnetic guide wire using the left/right detector pair
- detect intersections using the third detector
- choose actions from a preconfigured path table
- drive the two motors through the H-bridge PWM outputs

It is not yet the full project firmware for the entire robot. Remote-control
firmware and other robot subsystems live outside this target. This folder is
specifically the field-detection and wire-following slice.

## Layered Structure

The code is organized in layers from low-level hardware access up to behavior:

```text
main.c
  -> board_init()
  -> field_sensor_adc_update()
  -> robot_auto_mode_step()
  -> hbridge_motor_apply()

board.c / main.h
  -> STM32L051 register setup
  -> timer PWM
  -> ADC
  -> SysTick delay

field_sensor_adc.c / field_sensor_adc_config.h
  -> sensor GPIO setup
  -> ADC channel selection
  -> oversampled sensor reads

robot_auto_mode.c / robot_auto_mode.h
  -> field signal filtering
  -> baseline tracking
  -> follow / intersection / lost / stop state machine
  -> path table lookup

hbridge_motor.c / hbridge_motor_config.h
  -> signed motor commands
  -> PWM compare outputs for H-bridge inputs
```

## File Roles

### Entry Point

- `main.c`
  - Starts the standalone firmware.
  - Initializes board peripherals and modules.
  - Warms up field sensors.
  - Repeats the control loop every 10 ms.

### Hardware Support

- `main.h`
  - Shared hardware-facing definitions used by the standalone target.
  - Declares lightweight STM32 helper types and HAL-style wrapper functions.

- `board.c`
  - Bare-metal STM32L051 setup.
  - Configures:
    - TIM2 PWM for motor outputs
    - ADC1 for field sensor sampling
    - SysTick-based millisecond delay
  - Provides wrapper functions used by the sensor and motor modules.

### Field Sensor Input

- `field_sensor_adc_config.h`
  - Hardware mapping and placeholder thresholds.
  - Defines:
    - ADC handle
    - ADC timeout
    - oversampling count
    - left/right/intersection GPIO ports and pins
    - ADC channel numbers
    - field detection thresholds tied to analog gain

- `field_sensor_adc.h`
  - Public declarations for field sensor ADC functions.

- `field_sensor_adc.c`
  - Initializes the field sensor GPIO pins as analog inputs.
  - Reads the three field sensor ADC channels.
  - Averages repeated readings for noise reduction.
  - Passes samples into the field signal processing logic.

### Control Logic

- `robot_auto_mode.h`
  - Shared data structures for the auto mode subsystem.
  - Defines:
    - robot states
    - path actions
    - path IDs
    - `field_data_t`
    - `path_context_t`
    - motor command structure

- `robot_auto_mode.c`
  - Converts raw ADC samples into stable signal information.
  - Maintains:
    - filtered values
    - baselines
    - signal magnitudes
    - detected flags
  - Implements the state machine:
    - `ROBOT_AUTO_FOLLOW`
    - `ROBOT_AUTO_INTERSECTION`
    - `ROBOT_AUTO_LOST`
    - `ROBOT_AUTO_STOP`
  - Contains the preconfigured path table.

### Motor Output

- `hbridge_motor_config.h`
  - Maps motor directions to timer channels.

- `hbridge_motor.h`
  - Public declarations for motor control functions.

- `hbridge_motor.c`
  - Converts signed motor commands into PWM duty cycles.
  - Drives forward/reverse channels for each motor.

### Build Support

- `auto_mode.mk`
  - Build script for the standalone firmware target.
  - Compiles the `auto_mode` sources plus the shared startup code from
    `../Common`.
  - Produces:
    - `auto_mode.elf`
    - `auto_mode.hex`

## Runtime Control Flow

At startup:

1. `main()` calls `board_init()`.
2. `field_sensor_adc_init()` configures sensor pins.
3. `hbridge_motor_init()` prepares PWM outputs.
4. `field_sensor_reset()` clears sensor state.
5. A short warm-up loop lets the detector filters and baselines settle.
6. `robot_auto_mode_init()` selects the initial path and resets path state.

During each loop iteration:

1. `field_sensor_adc_update(&sensors)`
   - reads left, right, and intersection detector voltages
   - applies oversampling
   - updates filtered and baseline-compensated sensor state

2. `robot_auto_mode_step(&sensors, &path_context)`
   - checks wire presence
   - checks intersection state
   - runs follow logic or path action logic
   - computes signed motor commands

3. `hbridge_motor_apply(...)`
   - updates TIM2 PWM compare values for the H-bridge

4. `delayms(10)`
   - enforces the loop period

## Sensor Interpretation Model

The firmware treats the three field detectors as:

- left tracker
- right tracker
- intersection detector

The left and right detectors are used as a pair:

- if left signal is stronger than right, steer to reduce the error
- if right signal is stronger than left, steer the other way
- if they are close, drive straight

This implements the project goal of keeping the robot centered over the guide
wire by trying to keep the two detector responses balanced.

The intersection detector is handled separately:

- it does not steer the robot during normal follow mode
- it indicates that a new route decision point has been reached

## Preconfigured Paths

The route is not discovered from the ADC values alone.  The route is chosen
from a predefined table in `robot_auto_mode.c`.

Sensor data only tells the firmware:

- where the wire is relative to the robot
- whether an intersection is currently present

The selected path table then tells the robot what to do at each new
intersection:

- straight
- left
- right
- stop

## Default Placeholder Hardware Mapping

The standalone target currently assumes:

- motor PWM outputs:
  - `PA0` -> TIM2_CH1
  - `PA1` -> TIM2_CH2
  - `PA2` -> TIM2_CH3
  - `PA3` -> TIM2_CH4

- field detector ADC inputs:
  - left tracker -> `PB0` -> ADC channel 8
  - right tracker -> `PB1` -> ADC channel 9
  - intersection detector -> `PA4` -> ADC channel 4

These are placeholder defaults and should be updated to match the real robot
wiring.

## Build and Run

Build the firmware:

```sh
cd auto_mode
make -f auto_mode.mk
```

Clean build outputs:

```sh
make -f auto_mode.mk clean
```

The generated files are:

- `auto_mode.elf`
- `auto_mode.hex`

## Design Notes

- The hardware setup in `board.c` is based on the STM32L051 course reference
  code, especially the ADC and timer examples.
- The sensor thresholds are intentionally placeholders because the real values
  depend on detector gain, geometry, and analog front-end behavior.
- The code is split into small modules so a future full robot firmware can
  reuse the same sensor, control, and motor layers under a larger top-level
  application.
