#include "../bench_features.h"
#include "robot.h"
#include "robot_task.h"
#include "gimbal.h"
#include "gimbal_imu.h"
#include "gimbal_imu_config.h"
#include "bsp_dwt.h"
#include "bsp_log.h"
#include "main.h"
#include "diagnostics.h"
#include "gimbal_diagnostics.h"
#include "runtime_events.h"

void RobotInit(void)
{
    /* 1. 准备计时和日志。此函数在开机时调用一次。 */
    DWT_Init(SystemCoreClock / 1000000U);
    if (BENCH_ENABLE_RTT_LOG)
    {
        BSPLogInit();
    }
    RuntimeEventsInit();
    /* 2. 准备遥控器、CAN 和水平轴控制，初始禁止电机转动。 */
    bool ready = GimbalInit(HAL_GetTick());
    if (BENCH_ENABLE_RTT_LOG)
    {
        LOGINFO("[gimbal] port=%d yaw=%d remote=%d; outputs disabled", ready, BENCH_ENABLE_YAW, BENCH_ENABLE_REMOTE);
    }
#if GIMBAL_IMU_ENABLED
    /* 3. 准备 IMU；采样和姿态计算在后面的独立任务中进行。 */
    bool imu_ready = GimbalImuInit(HAL_GetTick());
    if (BENCH_ENABLE_RTT_LOG)
    {
        LOGINFO("[imu] init=%d; BMI088 startup deferred to IMU task", imu_ready);
    }
#endif
    /* 4. 注册 USB 诊断，只读取状态；USB 外设由系统默认任务启动。 */
    if (BENCH_ENABLE_USB && !DiagnosticsInit("gimbal_bench", GimbalDiagnosticsReport))
    {
        Error_Handler();
    }
    /* 5. 创建循环任务；调度器启动后，云台每 2 ms 处理一轮控制。 */
    RobotOSTaskInit();
}
