#ifndef DIAGNOSTICS_H
#define DIAGNOSTICS_H

#include "text_console.h"
#include "loop_stats.h"

typedef void (*DiagnosticsReport_f)(TextConsole_s *console, uint32_t now_ms);

/** One composition per firmware. Call once before USB init/task start.
 * profile_name/report have static lifetime. Report reads snapshots only;
 * <=5000 bytes per report, no blocking IO, no control mutations.
 * Console is exclusively owned by DiagnosticsPoll's task. */
bool DiagnosticsInit(const char *profile_name, DiagnosticsReport_f report);
void DiagnosticsPoll(uint32_t now_ms);
/** Creates one low-priority task; composition calls before scheduler starts. */
void DiagnosticsTaskStart(void);
/** Shared formatting for invocation-spacing statistics; diagnostic task only. */
void DiagnosticsPrintTiming(TextConsole_s *console, const char *name, const LoopStats_s *stats);

#endif
