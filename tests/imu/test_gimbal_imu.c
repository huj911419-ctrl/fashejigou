#include "gimbal_imu.h"
#include "gimbal_imu_config.h"
#include "imu_port.h"
#include "control_platform.h"
#include "runtime_events.h"
#include "fake_bmi088.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>

static FakeBmi088_s fake;
static bool port_ok = true;
static bool locked;

uint32_t ControlPlatformNowMs(void)
{
    return fake.tick;
}

uint32_t ControlPlatformEnter(void)
{
    assert(!locked);
    locked = true;
    return 0U;
}

void ControlPlatformExit(uint32_t previous)
{
    assert(locked && previous == 0U);
    locked = false;
}

bool ImuPortInit(void)
{
    assert(BENCH_ENABLE_IMU);
    return port_ok;
}

bool ImuPortTransfer(void *context, Bmi088Device_e device,
                     const uint8_t *tx, uint8_t *rx, size_t length)
{
    (void)context;
    assert(BENCH_ENABLE_IMU);
    assert(!locked); /* Never block on SPI with interrupts masked. */
    return FakeBmi088Transfer(&fake, device, tx, rx, length);
}

static GimbalImuStatus_s Run(unsigned steps, bool calibration_allowed)
{
    GimbalImuStatus_s status;
    for (unsigned i = 0U; i < steps; ++i)
    {
        fake.tick += GIMBAL_IMU_PERIOD_MS;
        GimbalImuTask(calibration_allowed);
    }
    GimbalImuGetStatus(&status);
    return status;
}

static void Initialize(void)
{
    FakeBmi088Init(&fake);
    port_ok = true;
    assert(GimbalImuInit(fake.tick));
}

int main(void)
{
    Initialize();
    if (!BENCH_ATTITUDE_ACTIVE)
    {
        GimbalImuStatus_s isolated = Run(2400U, true);
        assert(!isolated.attitude.valid && !isolated.valid_for_control);
        assert(isolated.attitude.calibration_count == 0U);
        if (BENCH_ENABLE_IMU)
        {
            assert(isolated.sensor_state == BMI088_SENSOR_READY && isolated.sample_count > 1000U);
            assert(isolated.sensor_sample.accel_m_s2[2] > 9.0f);
        }
        else
        {
            assert(isolated.sample_count == 0U && !isolated.port_ready);
        }
        puts("PASS: raw-only or disabled IMU cannot run calibration or authorize attitude control");
        return 0;
    }
    GimbalImuStatus_s status = Run(1200U, false);
    assert(status.sensor_state == BMI088_SENSOR_READY && status.sensor_fault == 0U);
    assert(!status.attitude.valid && status.attitude.calibration_count == 0U);
    status = Run(1000U, true);
    assert(status.attitude.valid && status.attitude.state == ATTITUDE_TRACKING);
    assert(status.timing.samples == 2200U && status.timing.max_interval_ms == 2U);
    assert(!status.mounting_confirmed && !status.valid_for_control);
    FakeBmi088Raw(&fake, BMI088_SENSOR_GYRO, 2U, 939); /* About +1 rad/s. */
    status = Run(500U, false);
    assert(fabs(status.attitude.yaw_unwrapped_rad - 1.0003) < 0.005);
    assert(status.attitude.yaw_continuous && !status.valid_for_control);

    /* A dead task must not leave a fresh-looking snapshot for other tasks. */
    fake.tick += 21U;
    GimbalImuGetStatus(&status);
    assert(!status.attitude.valid && (status.attitude.fault & ATTITUDE_FAULT_TIMING));
    status = Run(1U, false);
    assert(!status.attitude.valid && status.attitude.state == ATTITUDE_LOST);
    assert(status.timing.late_count == 1U && status.timing.max_interval_ms == 23U);
    status = Run(30U, true);
    assert(!status.attitude.valid); /* Never silently choose a new yaw reference. */

    Initialize();
    status = Run(1200U, true);
    assert(status.attitude.valid);
    fake.fail_io = true;
    status = Run(1U, true);
    assert(!status.attitude.valid && !status.valid_for_control);
    assert(status.sensor_fault == BMI088_SENSOR_ERROR_IO);
    assert(status.attitude.fault & ATTITUDE_FAULT_SOURCE);
    EventHistorySnapshot_s history;
    RuntimeEventsGet(&history);
    bool recorded_spi_fault = false;
    for (size_t i = 0U; i < history.count; ++i)
    {
        recorded_spi_fault |= history.entries[i].key == RUNTIME_IMU_FAULT &&
                              history.entries[i].value == BMI088_SENSOR_ERROR_IO;
    }
    assert(recorded_spi_fault);

    port_ok = false;
    assert(!GimbalImuInit(fake.tick));
    status = Run(1U, true);
    assert(!status.port_ready && !status.attitude.valid);
    puts("PASS: real sensor/attitude/application chain, calibration permission, mount gate, stale snapshots and IO loss");
    return 0;
}
