#include "../bench_features.h"
#include "robot_task.h"
#include "friction.h"
#include "friction_debug.h"
#include "friction_config.h"
#include "friction_remote_bench.h"
#include "diagnostics.h"
#include "FreeRTOS.h"
#include "task.h"
#include "iwdg.h"
#include "bsp_log.h"
#include "SEGGER_RTT.h"
#include <stdlib.h>
#include <errno.h>
#include <ctype.h>
#include <string.h>

static void MotorTask(void *argument)
{
    (void)argument;
    TickType_t wake = xTaskGetTickCount();
    for (;;)
    {
        if (BENCH_FRICTION_ACTIVE)
        {
            FrictionRemoteBenchPoll(HAL_GetTick());
            FrictionTask();
        }
        /* Only the control task feeds IWDG: a stalled controller must reset. */
        HAL_IWDG_Refresh(&hiwdg1);
        if ((TickType_t)(xTaskGetTickCount() - wake) >= pdMS_TO_TICKS(FRICTION_PERIOD_MS))
            wake = xTaskGetTickCount(); /* Do not replay missed periods. */
        vTaskDelayUntil(&wake, pdMS_TO_TICKS(FRICTION_PERIOD_MS));
    }
}

/* RTT channel 0 commands: off\n or set MASK RPM1 RPM2 RPM3\n.
 * No background keepalive: enabled commands must actually be renewed. */
static void HandleLine(const char *line)
{
    if (strcmp(line, "off") == 0)
    {
        const Friction_Command_s stopped = {0};
        (void)FrictionSubmitFrom(FRICTION_SOURCE_RTT, &stopped, HAL_GetTick());
        return;
    }
    if (FrictionDebugOwnsCommands())
    {
        return; /* Keep RTT off, reserve run for VOFA. */
    }
    long values[4] = {0};
    uint8_t valid = strncmp(line, "set ", 4) == 0;
    const char *cursor = valid ? line + 4 : line;
    for (unsigned i = 0; valid && i < 4; ++i)
    {
        char *end;
        errno = 0;
        values[i] = strtol(cursor, &end, 10);
        if (end == cursor || errno == ERANGE || (*end && !isspace((unsigned char)*end)))
        {
            valid = 0;
        }
        cursor = end;
    }
    while (isspace((unsigned char)*cursor))
    {
        ++cursor;
    }
    if (valid && !*cursor && values[0] >= 0 && values[0] <= 7)
    {
        Friction_Command_s command =
        {
            .enable_mask = (uint8_t)values[0],
            .target_rpm = {(float)values[1], (float)values[2], (float)values[3]},
        };
        Friction_Submit_Result_e result = FrictionSubmitFrom(FRICTION_SOURCE_RTT, &command, HAL_GetTick());
        if (result != FRICTION_SUBMIT_ACCEPTED)
        {
            LOGWARNING("[friction_bench] RTT command rejected: %u", (unsigned)result);
        }
    }
    else
    {
        FrictionDisable();
        LOGWARNING("[friction_bench] invalid command; stopped");
    }
}

static void DiagnosticTask(void *argument)
{
    (void)argument;
    char line[80];
    unsigned length = 0;
    uint8_t overflow = 0;
    uint32_t last_log = HAL_GetTick();
    for (;;)
    {
        if (BENCH_ENABLE_VOFA)
        {
            FrictionDebugPoll(HAL_GetTick());
        }
        /* Bound work per iteration, including malformed input. */
        for (unsigned count = 0; BENCH_ENABLE_RTT_COMMAND && count < 80 && SEGGER_RTT_HasKey(); ++count)
        {
            int key = SEGGER_RTT_GetKey();
            if (key < 0) break;
            if (key == '\r') continue;
            if (key == '\n')
            {
                line[length] = 0;
                if (!overflow) HandleLine(line);
                length = 0;
                overflow = 0;
            }
            else if (length < sizeof(line) - 1 && !overflow)
                line[length++] = (char)key;
            else
            {
                overflow = 1;
                FrictionDisable();
            }
        }
        if (BENCH_ENABLE_RTT_LOG && HAL_GetTick() - last_log >= FRICTION_LOG_PERIOD_MS)
        {
            Friction_Status_s status;
            FrictionGetStatus(&status);
            last_log = HAL_GetTick();
            LOGINFO("[FRIC] fault=%u online=%u enabled=%u age=%u dt=%u",
                    (unsigned)status.fault, status.online_mask, status.enabled_mask,
                    (unsigned)status.command_age_ms, (unsigned)status.step_ms);
            for (unsigned i = 0; i < FRICTION_MOTOR_COUNT; ++i)
            {
                if (!(BENCH_FRICTION_MASK & (1U << i)))
                {
                    LOGINFO("[F%d] DISABLED", i + 1U);
                    continue;
                }
                LOGINFO("[F%d] ref=%d rpm=%d iq_raw=%d temp=%d", i + 1,
                        (int)status.reference_rpm[i], (int)status.feedback[i].rpm,
                        status.feedback[i].current_raw, status.feedback[i].temperature_c);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

void RobotOSTaskInit(void)
{
    if (BENCH_ENABLE_USB)
    {
        DiagnosticsTaskStart();
    }
    configASSERT(pdMS_TO_TICKS(FRICTION_PERIOD_MS) > 0);
    BaseType_t motor_ok = xTaskCreate(MotorTask, "BenchService", 512, NULL, tskIDLE_PRIORITY + 3, NULL);
    BaseType_t diag_ok = pdPASS;
    if (BENCH_ENABLE_VOFA || BENCH_ENABLE_RTT_COMMAND || BENCH_ENABLE_RTT_LOG)
    {
        diag_ok = xTaskCreate(DiagnosticTask, "fric_diag", 768, NULL, tskIDLE_PRIORITY + 1, NULL);
    }
    if (motor_ok != pdPASS || diag_ok != pdPASS) Error_Handler();
}
