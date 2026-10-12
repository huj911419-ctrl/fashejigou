#ifndef FRICTION_STATUS_TEXT_H
#define FRICTION_STATUS_TEXT_H

#include "text_console.h"

/** Chinese status presentation for the diagnostic task only. Reads snapshots;
 * no control, source, PID, fault recovery or command-lease mutations.
 * All speeds are motor rotor RPM; current values are raw C620 units.
 * Bounded integer formatting requires no embedded printf-float support. */
void FrictionStatusTextReport(TextConsole_s *console, uint32_t now_ms);
/** Same read-only contract; full Chinese details, requested manually only. */
void FrictionStatusTextDetailReport(TextConsole_s *console, uint32_t now_ms);

#endif
