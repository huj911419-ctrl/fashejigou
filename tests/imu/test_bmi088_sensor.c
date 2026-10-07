#include "fake_bmi088.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

static void Ready(Bmi088Sensor_s *sensor, FakeBmi088_s *fake)
{
    assert(Bmi088SensorInit(sensor, FakeBmi088Transfer, fake, fake->tick));
    ImuSample_s sample;
    for (unsigned i = 0U; i < 200U && sensor->state != BMI088_SENSOR_READY; ++i)
    {
        assert(!Bmi088SensorStep(sensor, fake->tick, &sample));
        assert(sensor->state != BMI088_SENSOR_FAULT);
        fake->tick += 2U;
    }
    assert(sensor->state == BMI088_SENSOR_READY && fake->reset_count == 2U);
    assert(fake->registers[BMI088_SENSOR_ACCEL][0x40] == 0xABU);
    assert(fake->registers[BMI088_SENSOR_ACCEL][0x41] == 1U);
    assert(fake->registers[BMI088_SENSOR_GYRO][0x10] == 0x82U);
}

static void TestDecode(void)
{
    FakeBmi088_s fake;
    Bmi088Sensor_s sensor;
    FakeBmi088Init(&fake);
    fake.tick = UINT32_MAX - 60U;
    Ready(&sensor, &fake);
    FakeBmi088Raw(&fake, BMI088_SENSOR_ACCEL, 0U, -16384);
    FakeBmi088Raw(&fake, BMI088_SENSOR_GYRO, 0U, 16384);
    FakeBmi088Raw(&fake, BMI088_SENSOR_GYRO, 1U, -16384);
    fake.registers[BMI088_SENSOR_ACCEL][0x22] = 0xFFU;
    fake.registers[BMI088_SENSOR_ACCEL][0x23] = 0xE0U; /* Signed 11-bit -1. */
    ImuSample_s sample;
    assert(Bmi088SensorStep(&sensor, fake.tick, &sample));
    assert(sample.timestamp_ms == fake.tick && sensor.sample_count == 1U);
    assert(fabsf(sample.accel_m_s2[0] + 3.0f * 9.80665f) < 0.0001f);
    assert(fabsf(sample.gyro_rad_s[0] - 17.4532925f) < 0.0001f);
    assert(fabsf(sample.gyro_rad_s[1] + 17.4532925f) < 0.0001f);
    assert(sample.temperature_c == 22.875f);
    unsigned transfers = fake.transfers;
    assert(!Bmi088SensorStep(&sensor, fake.tick, &sample) && fake.transfers == transfers);
    ImuSample_s previous = sample;
    fake.registers[BMI088_SENSOR_ACCEL][3] = 0U;
    fake.tick += 2U;
    assert(!Bmi088SensorStep(&sensor, fake.tick, &sample));
    assert(memcmp(&sample, &previous, sizeof(sample)) == 0);
    fake.tick += 51U;
    assert(!Bmi088SensorStep(&sensor, fake.tick, &sample));
    assert(sensor.fault == BMI088_SENSOR_ERROR_TIMEOUT);
}

static void TestStartupFaults(void)
{
    for (unsigned fault = 0U; fault < 4U; ++fault)
    {
        FakeBmi088_s fake;
        Bmi088Sensor_s sensor;
        ImuSample_s sample = {.temperature_c = 42.0f};
        FakeBmi088Init(&fake);
        assert(Bmi088SensorInit(&sensor, FakeBmi088Transfer, &fake, 0U));
        if (fault == 0U)
        {
            fake.fail_io = true;
        }
        else if (fault == 1U)
        {
            fake.registers[BMI088_SENSOR_ACCEL][0] = 0U;
        }
        else if (fault == 2U)
        {
            fake.corrupt_writes = true;
        }
        else
        {
            fake.tick = 2001U;
        }
        for (unsigned i = 0U; i < 1100U && sensor.state != BMI088_SENSOR_FAULT; ++i)
        {
            assert(!Bmi088SensorStep(&sensor, fake.tick, &sample));
            fake.tick += 2U;
        }
        assert(sensor.state == BMI088_SENSOR_FAULT && sample.temperature_c == 42.0f);
        unsigned transfers = fake.transfers;
        fake.fail_io = false;
        assert(!Bmi088SensorStep(&sensor, fake.tick + 100U, &sample));
        assert(fake.transfers == transfers); /* No automatic retry/hang. */
    }
}

static void TestReadFaults(void)
{
    for (unsigned fault = 0U; fault < 4U; ++fault)
    {
        FakeBmi088_s fake;
        Bmi088Sensor_s sensor;
        ImuSample_s sample = {.temperature_c = 42.0f};
        FakeBmi088Init(&fake);
        Ready(&sensor, &fake);
        if (fault == 0U)
        {
            fake.fail_io = true;
        }
        else if (fault == 1U)
        {
            fake.registers[BMI088_SENSOR_GYRO][0] = 0xFFU;
        }
        else if (fault == 2U)
        {
            fake.registers[BMI088_SENSOR_ACCEL][2] = 1U;
        }
        else
        {
            FakeBmi088Raw(&fake, BMI088_SENSOR_GYRO, 0U, INT16_MIN);
        }
        assert(!Bmi088SensorStep(&sensor, fake.tick, &sample));
        assert(sensor.state == BMI088_SENSOR_FAULT && sample.temperature_c == 42.0f);
    }
}

int main(void)
{
    TestDecode();
    TestStartupFaults();
    TestReadFaults();
    puts("PASS: sensor startup/readback, SPI dummy bytes, signed SI/temperature conversion, fresh data and fault latching");
    return 0;
}
