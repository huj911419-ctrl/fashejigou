#ifndef CONTROL_PLATFORM_H
#define CONTROL_PLATFORM_H

#include <stdint.h>

/** Platform contract for application mailboxes and snapshots.
 * All application/driver/IRQ timestamps use this SAME modulo-2^32 ms clock.
 * Enter returns prior interrupt state; Exit restores that exact state.
 * Only bounded copies inside locks: no RTOS, logging, initialization or I/O. */
uint32_t ControlPlatformNowMs(void);
uint32_t ControlPlatformEnter(void);
void ControlPlatformExit(uint32_t previous_state);

#endif
