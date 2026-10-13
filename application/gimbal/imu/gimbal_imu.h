#ifndef GIMBAL_IMU_H
#define GIMBAL_IMU_H

#include "attitude.h"
#include "bmi088_sensor.h"
#include "loop_stats.h"

typedef struct
{
    AttitudeOutput_s attitude;
    LoopStats_s timing;
    ImuSample_s sensor_sample;
    Bmi088State_e sensor_state;
    uint32_t sensor_fault;
    uint32_t sample_count;
    bool port_ready;
    bool mounting_confirmed;
    bool valid_for_control;
} GimbalImuStatus_s;

/** Once before task start. SPI setup is deferred to the IMU task, so boot is
 * not blocked by sensor delays or calibration. No actuator commands here. */
bool GimbalImuInit(uint32_t now_ms);
/** Single IMU task owner. calibration_allowed is supplied by task composition
 * from the latest actuator state. Motor control never waits for this function. */
void GimbalImuTask(bool calibration_allowed);
/** Task-only, bounded coherent copy; applies an additional age check at read.
 * valid_for_control also requires confirmed mounting and continuous heading.
 * Angles are relative; loss/reinitialization invalidates stored angle targets. */
void GimbalImuGetStatus(GimbalImuStatus_s *status);

#endif
