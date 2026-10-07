#include "diagnostics.h"
#include "control_platform.h"
#include "main.h"
#include "FreeRTOS.h"
#include "task.h"

static void UsbDiagnosticTask(void *argument)
{
    (void)argument;
    for (;;)
    {
        DiagnosticsPoll(ControlPlatformNowMs());
        vTaskDelay(pdMS_TO_TICKS(2U));
    }
}

void DiagnosticsTaskStart(void)
{
    configASSERT(pdMS_TO_TICKS(2U) > 0U);
    if (xTaskCreate(UsbDiagnosticTask, "UsbDiag", 1024U, NULL, tskIDLE_PRIORITY + 1U, NULL) != pdPASS)
    {
        Error_Handler();
    }
}
