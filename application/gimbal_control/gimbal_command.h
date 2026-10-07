#ifndef GIMBAL_COMMAND_H
#define GIMBAL_COMMAND_H

#include <stdbool.h>
#include <stdint.h>

/** Source-independent request, copied between the command and subsystem layers.
 * All rates are MOTOR-side rad/s in the configured positive direction.
 * received_ms is the original input timestamp, never the polling time; use the
 * same monotonic millisecond clock as GimbalControlStep (uint32 wrap allowed).
 * valid means a completely decoded/validated source input, NOT authorization.
 * arm requests operation; the subsystem still requires healthy hardware and
 * OFF -> ON with rate_rad_s == 0. No angle mode is implied by this contract.
 * stop_requested retains ANY intervening invalid/OFF input until one Step.
 * A producer must not overwrite that stop with a later RUN in the same batch.
 * One composition task owns both producer and consumer. gimbal_arbiter provides
 * source selection and epoch checks upstream; concurrent producers still need
 * synchronized mailboxes. sequence is diagnostic only, not authorization. */
typedef struct
{
    uint32_t received_ms;
    uint32_t sequence;
    float rate_rad_s;
    bool valid;
    bool arm;
    bool stop_requested;
} GimbalCommand_s;

#endif
