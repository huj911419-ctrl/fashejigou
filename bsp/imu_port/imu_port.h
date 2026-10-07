#ifndef IMU_PORT_H
#define IMU_PORT_H

#include "bmi088_sensor.h"

/** H723/MC02 binding: exclusive SPI2, ACC_CS PC0, GYRO_CS PC3.
 * Call after CubeMX GPIO/SPI init, before starting the one owning IMU task.
 * Does not register callbacks, start DMA, enable DRDY outputs or heat the IMU. */
bool ImuPortInit(void);
/** Synchronous task-only transfer, at most 10 bytes, HAL timeout = 1 ms.
 * Interrupts remain enabled; timeout includes HAL tick quantization/preemption.
 * CS is released on both success and failure. Not a shared SPI bus manager. */
bool ImuPortTransfer(void *context, Bmi088Device_e device,
                     const uint8_t *tx, uint8_t *rx, size_t length);

#endif
