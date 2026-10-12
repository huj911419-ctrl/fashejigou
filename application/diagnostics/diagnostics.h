#ifndef DIAGNOSTICS_H
#define DIAGNOSTICS_H

#include "text_console.h"
#include "loop_stats.h"

typedef void (*DiagnosticsReport_f)(TextConsole_s *console, uint32_t now_ms);
/* Application commands are called only by the diagnostic task. Return true
 * when handled. Each application decides which actuator commands it exposes. */
typedef bool (*DiagnosticsCommand_f)(TextConsole_s *console, const char *line, uint32_t now_ms);

/** One composition per firmware. Call once before USB init/task start.
 * profile_name/report have static lifetime. Report reads snapshots only;
 * <=6500 bytes per report, no blocking IO, no control mutations.
 * Console is exclusively owned by DiagnosticsPoll's task. */
bool DiagnosticsInit(const char *profile_name, DiagnosticsReport_f report);
bool DiagnosticsInitWithCommands(const char *profile_name, DiagnosticsReport_f report,
                                 DiagnosticsCommand_f command);
void DiagnosticsPoll(uint32_t now_ms);
/** Optional read-only application presentation for status/normal streaming.
 * Task owner only, or before scheduler start; same report budget as Init.
 * interval_ms: 200..10000, rejects invalid values without changing settings.
 * NULL restores the original board/profile presentation. status raw always
 * uses that original presentation. Custom periodic CSV interval is unaffected. */
bool DiagnosticsSetStatusReport(DiagnosticsReport_f report, uint32_t interval_ms);
/** Diagnostic-task owner only. NULL restores normal status streaming. A custom
 * periodic report uses its own interval; status/events/period retain their meaning.
 * Skips a sample when TX has backlog; never waits or changes motor commands. */
void DiagnosticsSetPeriodicReport(DiagnosticsReport_f report, uint32_t interval_ms);
uint32_t DiagnosticsSkippedReports(void);
/** Creates one low-priority task; composition calls before scheduler starts. */
void DiagnosticsTaskStart(void);
/** Shared formatting for invocation-spacing statistics; diagnostic task only. */
void DiagnosticsPrintTiming(TextConsole_s *console, const char *name, const LoopStats_s *stats);

#endif
