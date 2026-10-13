#include "bmi088_sensor.h"
#include <string.h>

enum
{
    CHIP_ID = 0x00U,
    ACC_DATA = 0x12U,
    ACC_TEMPERATURE = 0x22U,
    ACC_RESET = 0x7EU,
    GYRO_RESET = 0x14U,
    RESET_COMMAND = 0xB6U,
    ACC_ID = 0x1EU,
    GYRO_ID = 0x0FU,
    ACC_DATA_READY = 0x80U,
    ACC_ERROR_MASK = 0x1DU,
    POWER_WAIT_MS = 50U,
    RESET_WAIT_MS = 80U,
    CONFIG_WAIT_MS = 5U,
    INIT_TIMEOUT_MS = 2000U,
    SAMPLE_PERIOD_MS = 2U,
    SAMPLE_TIMEOUT_MS = 50U
};

typedef struct
{
    Bmi088Device_e device;
    uint8_t address;
    uint8_t value;
} RegisterSetting_s;

/* Bosch BMI088 datasheet section 5. Readback includes the required reserved
 * bit 7 of ACC_CONF and GYRO_BANDWIDTH. Interrupt outputs stay unmapped. */
static const RegisterSetting_s settings[] =
{
    {BMI088_SENSOR_ACCEL, 0x7CU, 0x00U}, /* Active power mode. */
    {BMI088_SENSOR_ACCEL, 0x7DU, 0x04U}, /* Accel + temperature on. */
    {BMI088_SENSOR_ACCEL, 0x40U, 0xABU}, /* Normal bandwidth, 800 Hz. */
    {BMI088_SENSOR_ACCEL, 0x41U, 0x01U}, /* +/-6 g. */
    {BMI088_SENSOR_GYRO, 0x0FU, 0x00U},  /* +/-2000 deg/s. */
    {BMI088_SENSOR_GYRO, 0x10U, 0x82U},  /* 1000 Hz ODR / 116 Hz bandwidth. */
    {BMI088_SENSOR_GYRO, 0x11U, 0x00U}   /* Normal power mode. */
};

static void Fail(Bmi088Sensor_s *sensor, uint32_t fault)
{
    sensor->fault |= fault;
    sensor->state = BMI088_SENSOR_FAULT;
}

static bool Transfer(Bmi088Sensor_s *sensor, Bmi088Device_e device,
                     const uint8_t *tx, uint8_t *rx, size_t length)
{
    if (!sensor->transfer(sensor->context, device, tx, rx, length))
    {
        Fail(sensor, BMI088_SENSOR_ERROR_IO);
        return false;
    }
    return true;
}

static bool Read(Bmi088Sensor_s *sensor, Bmi088Device_e device,
                 uint8_t address, uint8_t *data, size_t length)
{
    uint8_t tx[10] = {0};
    uint8_t rx[10] = {0};
    size_t prefix = device == BMI088_SENSOR_ACCEL ? 2U : 1U;
    if (length == 0U || length > sizeof(tx) - prefix)
    {
        Fail(sensor, BMI088_SENSOR_ERROR_CONFIG);
        return false;
    }
    tx[0] = address | 0x80U;
    if (!Transfer(sensor, device, tx, rx, length + prefix))
    {
        return false;
    }
    memcpy(data, rx + prefix, length);
    return true;
}

static bool Write(Bmi088Sensor_s *sensor, Bmi088Device_e device, uint8_t address, uint8_t value)
{
    const uint8_t tx[2] = {address, value};
    uint8_t rx[2];
    return Transfer(sensor, device, tx, rx, sizeof(tx));
}

static int32_t Signed16(const uint8_t *bytes)
{
    int32_t value = (int32_t)bytes[0] | ((int32_t)bytes[1] << 8);
    return value >= 32768 ? value - 65536 : value;
}

static bool ReadSample(Bmi088Sensor_s *sensor, uint32_t now, ImuSample_s *sample)
{
    uint8_t accel_header[4]; /* ID, reserved, error, latched accel DRDY. */
    uint8_t accel[6];
    uint8_t gyro[8]; /* ID, reserved, six data bytes. */
    uint8_t temperature[2];
    if (!Read(sensor, BMI088_SENSOR_ACCEL, CHIP_ID, accel_header, sizeof(accel_header)))
    {
        return false;
    }
    if (accel_header[0] != ACC_ID || (accel_header[2] & ACC_ERROR_MASK) != 0U)
    {
        Fail(sensor, BMI088_SENSOR_ERROR_DEVICE);
        return false;
    }
    if ((accel_header[3] & ACC_DATA_READY) == 0U)
    {
        return false;
    }
    if (!Read(sensor, BMI088_SENSOR_ACCEL, ACC_DATA, accel, sizeof(accel)) ||
        !Read(sensor, BMI088_SENSOR_GYRO, CHIP_ID, gyro, sizeof(gyro)) ||
        !Read(sensor, BMI088_SENSOR_ACCEL, ACC_TEMPERATURE, temperature, sizeof(temperature)))
    {
        return false;
    }
    if (gyro[0] != GYRO_ID)
    {
        Fail(sensor, BMI088_SENSOR_ERROR_ID);
        return false;
    }
    ImuSample_s next = {.timestamp_ms = now};
    for (unsigned i = 0U; i < 3U; ++i)
    {
        int32_t raw_accel = Signed16(accel + 2U * i);
        int32_t raw_gyro = Signed16(gyro + 2U + 2U * i);
        if (raw_accel <= -32760 || raw_accel >= 32760 || raw_gyro <= -32760 || raw_gyro >= 32760)
        {
            Fail(sensor, BMI088_SENSOR_ERROR_RANGE);
            return false;
        }
        next.accel_m_s2[i] = (float)raw_accel * (6.0f * 9.80665f / 32768.0f);
        next.gyro_rad_s[i] = (float)raw_gyro * (2000.0f * 0.017453292519943295f / 32768.0f);
    }
    int32_t raw_temperature = ((int32_t)temperature[0] << 3) | (temperature[1] >> 5);
    if (raw_temperature > 1023)
    {
        raw_temperature -= 2048;
    }
    next.temperature_c = (float)raw_temperature * 0.125f + 23.0f;
    *sample = next;
    sensor->last_sample_ms = now;
    ++sensor->sample_count;
    return true;
}

bool Bmi088SensorInit(Bmi088Sensor_s *sensor, Bmi088Transfer_f transfer,
                      void *context, uint32_t now_ms)
{
    if (sensor == NULL)
    {
        return false;
    }
    memset(sensor, 0, sizeof(*sensor));
    sensor->transfer = transfer;
    sensor->context = context;
    sensor->initialized_ms = now_ms;
    sensor->state_ms = now_ms;
    if (transfer == NULL)
    {
        Fail(sensor, BMI088_SENSOR_ERROR_CONFIG);
        return false;
    }
    return true;
}

bool Bmi088SensorStep(Bmi088Sensor_s *sensor, uint32_t now_ms, ImuSample_s *sample)
{
    if (sensor == NULL || sample == NULL || sensor->state == BMI088_SENSOR_FAULT)
    {
        return false;
    }
    if (sensor->state != BMI088_SENSOR_READY && now_ms - sensor->initialized_ms > INIT_TIMEOUT_MS)
    {
        Fail(sensor, BMI088_SENSOR_ERROR_TIMEOUT);
        return false;
    }
    uint8_t value = 0U;
    switch (sensor->state)
    {
        case BMI088_SENSOR_BOOT:
            if (now_ms - sensor->state_ms < POWER_WAIT_MS)
            {
                break;
            }
            /* First accel transaction selects SPI. After reset select it again. */
            if (Read(sensor, BMI088_SENSOR_ACCEL, CHIP_ID, &value, 1U) &&
                Write(sensor, BMI088_SENSOR_ACCEL, ACC_RESET, RESET_COMMAND) &&
                Write(sensor, BMI088_SENSOR_GYRO, GYRO_RESET, RESET_COMMAND))
            {
                sensor->state = BMI088_SENSOR_RESET_WAIT;
                sensor->state_ms = now_ms;
            }
            break;
        case BMI088_SENSOR_RESET_WAIT:
            if (now_ms - sensor->state_ms >= RESET_WAIT_MS &&
                Read(sensor, BMI088_SENSOR_ACCEL, CHIP_ID, &value, 1U))
            {
                sensor->state = BMI088_SENSOR_PROBE;
            }
            break;
        case BMI088_SENSOR_PROBE:
            if (!Read(sensor, BMI088_SENSOR_ACCEL, CHIP_ID, &value, 1U))
            {
                break;
            }
            if (value != ACC_ID)
            {
                Fail(sensor, BMI088_SENSOR_ERROR_ID);
                break;
            }
            if (Read(sensor, BMI088_SENSOR_GYRO, CHIP_ID, &value, 1U))
            {
                if (value != GYRO_ID)
                {
                    Fail(sensor, BMI088_SENSOR_ERROR_ID);
                }
                else
                {
                    sensor->state = BMI088_SENSOR_CONFIG_WRITE;
                }
            }
            break;
        case BMI088_SENSOR_CONFIG_WRITE:
        {
            const RegisterSetting_s *setting = &settings[sensor->config_index];
            if (Write(sensor, setting->device, setting->address, setting->value))
            {
                sensor->state = BMI088_SENSOR_CONFIG_VERIFY;
                sensor->state_ms = now_ms;
            }
            break;
        }
        case BMI088_SENSOR_CONFIG_VERIFY:
        {
            const RegisterSetting_s *setting = &settings[sensor->config_index];
            if (now_ms - sensor->state_ms < CONFIG_WAIT_MS ||
                !Read(sensor, setting->device, setting->address, &value, 1U))
            {
                break;
            }
            if (value != setting->value)
            {
                Fail(sensor, BMI088_SENSOR_ERROR_CONFIG);
                break;
            }
            ++sensor->config_index;
            sensor->state = sensor->config_index == sizeof(settings) / sizeof(settings[0]) ?
                            BMI088_SENSOR_READY : BMI088_SENSOR_CONFIG_WRITE;
            sensor->last_sample_ms = now_ms;
            sensor->last_poll_ms = now_ms;
            break;
        }
        case BMI088_SENSOR_READY:
            if (now_ms - sensor->last_sample_ms > SAMPLE_TIMEOUT_MS)
            {
                Fail(sensor, BMI088_SENSOR_ERROR_TIMEOUT);
                break;
            }
            if (now_ms - sensor->last_poll_ms >= SAMPLE_PERIOD_MS)
            {
                sensor->last_poll_ms = now_ms;
                return ReadSample(sensor, now_ms, sample);
            }
            break;
        default:
            Fail(sensor, BMI088_SENSOR_ERROR_CONFIG);
            break;
    }
    return false;
}
