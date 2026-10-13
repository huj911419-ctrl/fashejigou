#include "gimbal_imu.h"
#include "../config/gimbal_imu_config.h"
#include "imu_port.h"
#include "control_platform.h"
#include "runtime_events.h"
#include <string.h>

static Bmi088Sensor_s sensor;
static Attitude_s attitude;
static ImuSample_s sample;
static GimbalImuStatus_s published;
static bool port_ready;
static LoopStats_s timing;

static void Publish(void)
{
    GimbalImuStatus_s status =
    {
        .attitude = attitude.output,
        .timing = timing,
        .sensor_sample = sample,
        .sensor_state = sensor.state,
        .sensor_fault = sensor.fault,
        .sample_count = sensor.sample_count,
        .port_ready = port_ready,
        .mounting_confirmed = GIMBAL_IMU_MOUNTING_CONFIRMED != 0,
        .valid_for_control = BENCH_ATTITUDE_ACTIVE && port_ready && sensor.state == BMI088_SENSOR_READY &&
                             attitude.output.valid && attitude.output.heading_valid &&
                             attitude.output.yaw_continuous && GIMBAL_IMU_ON_GIMBAL &&
                             GIMBAL_IMU_MOUNTING_CONFIRMED
    };
    uint32_t lock = ControlPlatformEnter();
    published = status;
    ControlPlatformExit(lock);
    RuntimeEventsObserve(RUNTIME_IMU_STAGE, status.sensor_state);
    RuntimeEventsObserve(RUNTIME_IMU_FAULT, status.sensor_fault);
    if (BENCH_ATTITUDE_ACTIVE)
    {
        RuntimeEventsObserve(RUNTIME_ATTITUDE_STAGE, status.attitude.state);
        RuntimeEventsObserve(RUNTIME_ATTITUDE_FAULT, status.attitude.fault);
        RuntimeEventsObserve(RUNTIME_ATTITUDE_CONTROL_VALID, status.valid_for_control);
    }
}

bool GimbalImuInit(uint32_t now_ms)
{
    memset(&published, 0, sizeof(published));
    if (!BENCH_ENABLE_IMU)
    {
        return true;
    }
    const AttitudeConfig_s config = GIMBAL_ATTITUDE_CONFIG;
    memset(&attitude, 0, sizeof(attitude));
    memset(&sample, 0, sizeof(sample));
    memset(&timing, 0, sizeof(timing));
    port_ready = ImuPortInit();
    bool sensor_ok = Bmi088SensorInit(&sensor, ImuPortTransfer, NULL, now_ms);
    bool attitude_ok = !BENCH_ATTITUDE_ACTIVE || AttitudeInit(&attitude, &config);
    if (BENCH_ATTITUDE_ACTIVE && !port_ready)
    {
        AttitudeInvalidate(&attitude, ATTITUDE_FAULT_SOURCE);
    }
    Publish();
    return port_ready && sensor_ok && attitude_ok;
}

void GimbalImuTask(bool calibration_allowed)
{
    if (!BENCH_ENABLE_IMU)
    {
        return;
    }
    const AttitudeConfig_s config = GIMBAL_ATTITUDE_CONFIG;
    LoopStatsObserve(&timing, ControlPlatformNowMs(), config.max_step_ms);
    if (port_ready && Bmi088SensorStep(&sensor, ControlPlatformNowMs(), &sample))
    {
        if (BENCH_ATTITUDE_ACTIVE)
        {
            (void)AttitudeUpdate(&attitude, &sample, ControlPlatformNowMs(), calibration_allowed);
        }
    }
    if (BENCH_ATTITUDE_ACTIVE && (!port_ready || sensor.state == BMI088_SENSOR_FAULT))
    {
        AttitudeInvalidate(&attitude, ATTITUDE_FAULT_SOURCE);
    }
    if (BENCH_ATTITUDE_ACTIVE)
    {
        AttitudeCheckFresh(&attitude, ControlPlatformNowMs());
    }
    Publish();
}

void GimbalImuGetStatus(GimbalImuStatus_s *status)
{
    uint32_t lock = ControlPlatformEnter();
    *status = published;
    ControlPlatformExit(lock);
    const AttitudeConfig_s config = GIMBAL_ATTITUDE_CONFIG;
    if (status->attitude.valid && ControlPlatformNowMs() - status->attitude.sample_ms > config.stale_ms)
    {
        status->attitude.valid = false;
        status->attitude.heading_valid = false;
        status->attitude.yaw_continuous = false;
        status->attitude.accel_correction_used = false;
        status->attitude.state = ATTITUDE_LOST;
        status->attitude.fault |= ATTITUDE_FAULT_TIMING;
        status->valid_for_control = false;
    }
}
