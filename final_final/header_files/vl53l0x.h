#ifndef VL53L0X_H
#define VL53L0X_H

#include <stdbool.h>
#include <stdint.h>

#define VL53L0X_OUT_OF_RANGE (8190)

bool vl53l0x_init(void);

/**
 * Does a single range measurement — BLOCKING, slow (~30-100ms).
 * Only use for testing. Use continuous mode in the robot control loop.
 */
bool vl53l0x_read_range_single(uint16_t *range);

/**
 * Start continuous back-to-back ranging.
 * Call once before entering the robot control loop.
 * Sensor draws ~20mA in this mode.
 */
bool vl53l0x_start_continuous(void);

/**
 * Stop continuous ranging. Call when auto mode ends to save battery.
 */
bool vl53l0x_stop_continuous(void);

/**
 * Non-blocking check — returns true if a new measurement is ready.
 * Call this every iteration of the control loop.
 */
bool vl53l0x_measurement_ready(void);

/**
 * Read the latest range from continuous mode.
 * Only call after vl53l0x_measurement_ready() returns true.
 * @param range distance in mm, or VL53L0X_OUT_OF_RANGE if nothing detected
 */
bool vl53l0x_read_range_continuous(uint16_t *range);

#endif /* VL53L0X_H */
