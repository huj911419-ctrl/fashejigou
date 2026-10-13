#ifndef FRICTION_DEBUG_H
#define FRICTION_DEBUG_H

#include "vofa.h"

/** Initialize once before starting tasks. Port may be NULL (disconnected). */
void FrictionDebugInit(const Vofa_Port_s *port);
/** Diagnostic task only, normally every 1 ms; never in motor task or ISR. */
void FrictionDebugPoll(uint32_t now_ms);
/** An attached RX port reserves run commands for VOFA; RTT off still works. */
uint8_t FrictionDebugOwnsCommands(void);
/** Hardware placeholder: implement in friction_debug_port.c when port is known. */
Vofa_Port_s FrictionDebugPort(void);

#endif
