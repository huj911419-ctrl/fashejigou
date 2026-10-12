#ifndef FRICTION_DIAGNOSTICS_H
#define FRICTION_DIAGNOSTICS_H

#include "text_console.h"
#include "friction.h"

/** Diagnostic task only; reads snapshots, never changes actuator state. */
void FrictionDiagnosticsReport(TextConsole_s *console, uint32_t now_ms);
/** Explain a supplied snapshot; never submits a command or clears a fault. */
void FrictionDiagnosticsExplain(TextConsole_s *console, const Friction_Telemetry_s *telemetry,
                                uint32_t now_ms);

#endif
