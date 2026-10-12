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
#include "friction_usb_log.h"
#include "runtime_events.h"
#include "friction_remote_bench.h"

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
    if (BENCH_ENABLE_USB && !DiagnosticsInitWithCommands("friction_bench", FrictionDiagnosticsReport,
                                                        FrictionUsbConsoleCommand))
    {
        Error_Handler();
    }
    if (BENCH_ENABLE_USB)
    {
        FrictionUsbLogInit();
    }
    /* FrictionInit has prepared OFF; transport initialization keeps its owner. */
    FrictionRemoteBenchInit();
    RobotOSTaskInit();
    if (BENCH_ENABLE_RTT_LOG)
    {
        LOGINFO("[friction_bench] config=%d mask=%d; outputs disabled", ready, BENCH_FRICTION_MASK);
    }
}
