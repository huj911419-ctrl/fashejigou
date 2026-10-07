#include "fake_bmi088.h"
#include <assert.h>
#include <string.h>

void FakeBmi088Raw(FakeBmi088_s *fake, Bmi088Device_e device, unsigned axis, int16_t raw)
{
    assert(axis < 3U);
    unsigned address = (device == BMI088_SENSOR_ACCEL ? 0x12U : 0x02U) + axis * 2U;
    fake->registers[device][address] = (uint8_t)(uint16_t)raw;
    fake->registers[device][address + 1U] = (uint8_t)((uint16_t)raw >> 8);
}

void FakeBmi088Init(FakeBmi088_s *fake)
{
    memset(fake, 0, sizeof(*fake));
    fake->registers[BMI088_SENSOR_ACCEL][0] = 0x1EU;
    fake->registers[BMI088_SENSOR_GYRO][0] = 0x0FU;
    fake->registers[BMI088_SENSOR_ACCEL][3] = 0x80U;
    FakeBmi088Raw(fake, BMI088_SENSOR_ACCEL, 2U, 5461); /* Approx. 1 g. */
}

bool FakeBmi088Transfer(void *context, Bmi088Device_e device,
                       const uint8_t *tx, uint8_t *rx, size_t length)
{
    FakeBmi088_s *fake = context;
    assert(device == BMI088_SENSOR_ACCEL || device == BMI088_SENSOR_GYRO);
    assert(length >= 2U && length <= 10U);
    ++fake->transfers;
    if (fake->fail_io)
    {
        return false;
    }
    memset(rx, 0xA5, length); /* Deliberately invalid address/dummy bytes. */
    unsigned address = tx[0] & 0x7FU;
    if (tx[0] & 0x80U)
    {
        size_t prefix = device == BMI088_SENSOR_ACCEL ? 2U : 1U;
        assert(length > prefix && address + length - prefix <= 128U);
        memcpy(rx + prefix, fake->registers[device] + address, length - prefix);
    }
    else
    {
        assert(length == 2U);
        if ((device == BMI088_SENSOR_ACCEL && address == 0x7EU) ||
            (device == BMI088_SENSOR_GYRO && address == 0x14U))
        {
            assert(tx[1] == 0xB6U);
            ++fake->reset_count;
        }
        else
        {
            fake->registers[device][address] = fake->corrupt_writes ? tx[1] ^ 1U : tx[1];
        }
    }
    return true;
}
