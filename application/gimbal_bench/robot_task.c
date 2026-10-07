#include "../bench_features.h"
#include "robot_task.h"
#include "gimbal.h"
#include "gimbal_config.h"
#include "gimbal_imu.h"
#include "gimbal_imu_config.h"
#include "bsp_log.h"
#include "FreeRTOS.h"
#include "task.h"
#include "iwdg.h"
#include "diagnostics.h"

static void YawTask(void *argument)
{
    (void)argument;
    TickType_t wake = xTaskGetTickCount();
    for (;;)
    {
        if (BENCH_ENABLE_YAW || BENCH_ENABLE_REMOTE)
        {
            GimbalTask();
        }
        HAL_IWDG_Refresh(&hiwdg1);
        if ((TickType_t)(xTaskGetTickCount() - wake) >= pdMS_TO_TICKS(GIMBAL_CONTROL_PERIOD_MS))
        {
            wake = xTaskGetTickCount(); /* Do not replay missed control periods. */
        }
        vTaskDelayUntil(&wake, pdMS_TO_TICKS(GIMBAL_CONTROL_PERIOD_MS));
    }
}

#if GIMBAL_IMU_ENABLED
static void ImuTask(void *argument)
{
    (void)argument;
    TickType_t wake = xTaskGetTickCount();
    for (;;)
    {
        GimbalStatus_s yaw;
        GimbalGetStatus(&yaw);
        GimbalImuTask(!yaw.axis.enabled && !yaw.axis.motor_enabled);
        if ((TickType_t)(xTaskGetTickCount() - wake) >= pdMS_TO_TICKS(GIMBAL_IMU_PERIOD_MS))
        {
            wake = xTaskGetTickCount();
        }
        vTaskDelayUntil(&wake, pdMS_TO_TICKS(GIMBAL_IMU_PERIOD_MS));
    }
}
#endif

static void DiagnosticTask(void *argument)
{
    (void)argument;
    for (;;)
    {
        GimbalStatus_s status;
        GimbalGetStatus(&status);
        if (BENCH_ENABLE_YAW || BENCH_ENABLE_REMOTE)
        {
            LOGINFO("[yaw] rc=%u en=%u state=%u id=%x fault=%u io=%u ref_mrad=%d vel_mrad=%d fb=%u",
                    status.remote.online, status.axis.enabled, status.axis.motor_state, status.axis.master_id,
                    (unsigned)status.axis.motor_fault, (unsigned)status.axis.transport_faults,
                    (int)(status.axis.reference_rad_s * 1000.0f), (int)(status.axis.feedback.velocity_rad_s * 1000.0f),
                    (unsigned)status.axis.feedback.count);
        }
        else
        {
            LOGINFO("[yaw/remote] DISABLED");
        }
#if GIMBAL_IMU_ENABLED
        GimbalImuStatus_s imu;
        GimbalImuGetStatus(&imu);
        LOGINFO("[imu] sensor=%u sf=%u attitude_enabled=%u attitude=%u af=%u cal=%u valid=%u mount=%u rpy_mrad=%d,%d,%d temp_c=%d",
                imu.sensor_state, (unsigned)imu.sensor_fault, BENCH_ATTITUDE_ACTIVE, imu.attitude.state,
                (unsigned)imu.attitude.fault, (unsigned)imu.attitude.calibration_count,
                imu.attitude.valid, imu.mounting_confirmed,
                (int)(imu.attitude.roll_rad * 1000.0f), (int)(imu.attitude.pitch_rad * 1000.0f),
                (int)(imu.attitude.yaw_rad * 1000.0f), (int)imu.attitude.temperature_c);
#endif
        vTaskDelay(pdMS_TO_TICKS(GIMBAL_LOG_PERIOD_MS));
    }
}

void RobotOSTaskInit(void)
{
    if (BENCH_ENABLE_USB)
    {
        DiagnosticsTaskStart();
    }
#if GIMBAL_IMU_ENABLED
    if (xTaskCreate(ImuTask, "GimbalIMU", 768U, NULL, tskIDLE_PRIORITY + 2U, NULL) != pdPASS)
    {
        Error_Handler();
    }
#endif
    if (xTaskCreate(YawTask, "BenchService", 512U, NULL, tskIDLE_PRIORITY + 3U, NULL) != pdPASS ||
        (BENCH_ENABLE_RTT_LOG &&
         xTaskCreate(DiagnosticTask, "YawLog", 512U, NULL, tskIDLE_PRIORITY + 1U, NULL) != pdPASS))
    {
        Error_Handler();
    }
}
