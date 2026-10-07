#ifndef TEST_FAKE_BMI088_H
#define TEST_FAKE_BMI088_H

#include "bmi088_sensor.h"

/* Register-level SPI fake shared by the sensor and real application tests.
 * Not a model of MEMS dynamics, bus timings, CPU scheduling or DMA/cache. */
typedef struct
{
    uint8_t registers[2][128];
    bool fail_io;
    bool corrupt_writes;
    unsigned transfers;
    unsigned reset_count;
    uint32_t tick;
} FakeBmi088_s;

void FakeBmi088Init(FakeBmi088_s *fake);
void FakeBmi088Raw(FakeBmi088_s *fake, Bmi088Device_e device, unsigned axis, int16_t raw);
bool FakeBmi088Transfer(void *context, Bmi088Device_e device,
                       const uint8_t *tx, uint8_t *rx, size_t length);

#endif
