#ifndef FRICTION_DIAGNOSTICS_H
#define FRICTION_DIAGNOSTICS_H

#include "text_console.h"

/** Diagnostic task only; reads snapshots, never changes actuator state. */
void FrictionDiagnosticsReport(TextConsole_s *console, uint32_t now_ms);

#endif
