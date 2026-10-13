#ifndef FRICTION_USB_COMMAND_H
#define FRICTION_USB_COMMAND_H

#include <stdint.h>

typedef enum
{
    FRICTION_USB_QUEUED = 0,
    FRICTION_USB_SYNTAX,
    FRICTION_USB_RANGE,
    FRICTION_USB_WRONG_SOURCE,
    FRICTION_USB_REARM_REQUIRED,
    FRICTION_USB_BUSY,
    FRICTION_USB_STALE,
    FRICTION_USB_UNAVAILABLE
} FrictionUsbResult_e;

/** Diagnostic task only. Preserves set/pid/ramp syntax and mailbox semantics.
 * line is NUL-terminated, without CR/LF. Arguments require spaces/tabs;
 * mask/wheel are unsigned decimal digits. Nonfinite/out-of-range float
 * conversions and malformed tokens return SYNTAX without submitting work.
 * QUEUED acknowledges input only; motor status/applied tuning are separate.
 * off is a global sticky stop; selected USB ownership is retained.
 * The compatibility boolean API remains declared in friction.h. */
FrictionUsbResult_e FrictionUsbExecute(const char *line, uint32_t now_ms);
const char *FrictionUsbResultName(FrictionUsbResult_e result);

#endif
