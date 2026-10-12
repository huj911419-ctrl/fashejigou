#ifndef FRICTION_USB_LOG_H
#define FRICTION_USB_LOG_H

#include "text_console.h"

/** One instance in the diagnostic task, no allocation or IO in Init.
 * Mode 1: human-readable status/causes. Mode 2: timestamped, versioned CSV.
 * Does not own motor state, change PID, or alter UART telemetry. */
void FrictionUsbLogInit(void);
/** Application handler for DiagnosticsInitWithCommands. Existing motor command
 * syntax is preserved; log commands only affect diagnostic output. */
bool FrictionUsbConsoleCommand(TextConsole_s *console, const char *line, uint32_t now_ms);

#endif
