#ifndef BMI088_SENSOR_H
#define BMI088_SENSOR_H

#include "imu_sample.h"
#include <stdbool.h>
#include <stddef.h>

typedef enum
{
    BMI088_SENSOR_ACCEL,
    BMI088_SENSOR_GYRO
} Bmi088Device_e;

/** Task-only, synchronous complete SPI transaction. Keep CS asserted for all
 * bytes, copy RX before returning, return false on timeout/error. No retained
 * pointers. The port must bound its wait and serialize ownership of the bus. */
typedef bool (*Bmi088Transfer_f)(void *context, Bmi088Device_e device,
                               const uint8_t *tx, uint8_t *rx, size_t length);

typedef enum
{
    BMI088_SENSOR_BOOT,
    BMI088_SENSOR_RESET_WAIT,
    BMI088_SENSOR_PROBE,
    BMI088_SENSOR_CONFIG_WRITE,
    BMI088_SENSOR_CONFIG_VERIFY,
    BMI088_SENSOR_READY,
    BMI088_SENSOR_FAULT
} Bmi088State_e;

enum
{
    BMI088_SENSOR_ERROR_IO = 1U,
    BMI088_SENSOR_ERROR_ID = 2U,
    BMI088_SENSOR_ERROR_CONFIG = 4U,
    BMI088_SENSOR_ERROR_TIMEOUT = 8U,
    BMI088_SENSOR_ERROR_RANGE = 16U,
    BMI088_SENSOR_ERROR_DEVICE = 32U
};

typedef struct
{
    Bmi088Transfer_f transfer;
    void *context;
    Bmi088State_e state;
    uint32_t fault;
    uint32_t initialized_ms;
    uint32_t state_ms;
    uint32_t last_poll_ms;
    uint32_t last_sample_ms;
    uint32_t sample_count;
    unsigned config_index;
} Bmi088Sensor_s;

/** Pure C, caller-owned instance; one task calls Init/Step. Init performs no IO.
 * Fixed profile: accel +/-6 g, 800 Hz; gyro +/-2000 deg/s, 1000 Hz.
 * Initialization is time-stepped with readback and a 2 s deadline, no sleeps,
 * allocation, automatic retry loop, heater, DMA or global driver registry.
 * A fault latches until explicit Init (which restarts the reference chain). */
bool Bmi088SensorInit(Bmi088Sensor_s *sensor, Bmi088Transfer_f transfer,
                      void *context, uint32_t now_ms);
/** Call about every 2 ms with the same monotonic clock as ImuSample.timestamp_ms.
 * Returns true ONLY when a complete new sample has been copied to sample.
 * On false sample is unchanged; inspect state/fault to distinguish startup,
 * no fresh accel data, and failure. The two sensors are not hardware synced.
 * Gyro DRDY is a short pulse: this polling profile samples at <=500 Hz instead
 * of incorrectly treating that pulse as a latched new-data bit. */
bool Bmi088SensorStep(Bmi088Sensor_s *sensor, uint32_t now_ms, ImuSample_s *sample);

#endif
