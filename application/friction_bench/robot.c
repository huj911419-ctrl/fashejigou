#include "../bench_features.h"
#include "robot.h"
#include "robot_task.h"
#include "friction.h"
#include "friction_debug.h"
#include "bsp_dwt.h"
#include "bsp_log.h"
#include "main.h"
#include "diagnostics.h"
#include "friction_diagnostics.h"
#include "runtime_events.h"

void RobotInit(void)
{
    /* Derive DWT frequency from the clock actually selected by main.c. */
    DWT_Init(SystemCoreClock / 1000000U);
    if (BENCH_ENABLE_RTT_LOG || BENCH_ENABLE_RTT_COMMAND)
    {
        BSPLogInit();
    }
    RuntimeEventsInit();
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    uint8_t ready = FrictionInit();
    __set_PRIMASK(primask);
    if (BENCH_ENABLE_VOFA)
    {
        Vofa_Port_s port = FrictionDebugPort();
        FrictionDebugInit(&port);
    }
    if (BENCH_ENABLE_USB && !DiagnosticsInit("friction_bench", FrictionDiagnosticsReport))
    {
        Error_Handler();
    }
    RobotOSTaskInit();
    if (BENCH_ENABLE_RTT_LOG)
    {
        LOGINFO("[friction_bench] config=%d mask=%d; outputs disabled", ready, BENCH_FRICTION_MASK);
    }
}
