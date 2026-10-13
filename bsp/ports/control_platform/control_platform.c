#include "control_platform.h"
#include "main.h"

uint32_t ControlPlatformNowMs(void)
{
    return HAL_GetTick();
}

uint32_t ControlPlatformEnter(void)
{
    uint32_t previous = __get_PRIMASK();
    __disable_irq();
    return previous;
}

void ControlPlatformExit(uint32_t previous_state)
{
    __set_PRIMASK(previous_state);
}
