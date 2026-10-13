#include "friction.h"
#include "friction_usb_command.h"
#include <assert.h>
#include <errno.h>
#include <math.h>
#include <stdio.h>

static unsigned submits;
static unsigned stops;
static unsigned tunes;
static Friction_Command_s submitted;
static Friction_Tuning_s queued;
static Friction_Telemetry_s telemetry;
static const uint32_t received_ms = UINT32_MAX - 2U;

void FrictionGetTelemetry(Friction_Telemetry_s *value)
{
    *value = telemetry;
}

void FrictionStopFromUsb(void)
{
    ++stops;
}

Friction_Submit_Result_e FrictionSubmitFrom(Friction_Command_Source_e source,
                                           const Friction_Command_s *command,
                                           uint32_t timestamp)
{
    assert(source == FRICTION_SOURCE_USB && timestamp == received_ms);
    ++submits;
    submitted = *command;
    return FRICTION_SUBMIT_ACCEPTED;
}

Friction_Tune_Result_e FrictionQueueTuning(const Friction_Tuning_s *value,
                                         uint16_t sequence, uint16_t base_sequence)
{
    assert(sequence != 0U && base_sequence == telemetry.applied_sequence);
    ++tunes;
    queued = *value;
    return FRICTION_TUNE_QUEUED;
}

int main(void)
{
    static const char *const invalid[] = {
        "set 1+500+0+0", "set 1-500 0 0", "set 1 500+0+0",
        "set 1 500 0-0", "set -0 0 0 0", "set +1 500 0 0",
        "set -4294967295 500 0 0", "set 4294967297 500 0 0",
        "set 18446744073709551617 500 0 0", "set 65536 0 0 0",
        "set 8 0 0 0", "set 1.0 500 0 0", "set 0x1 500 0 0",
        "set 1 500 0", "set 1 500 0 0 0", "set 1 500x 0 0",
        "set 1 nan 0 0", "set 1 inf 0 0", "set 1 -inf 0 0",
        "set 1 1e9999 0 0", "set 1 1e-9999 0 0",
        "set 1 500 0 0\n", "pid 1+2+0.1+0+12000",
        "pid -0 2 0.1 0 12000", "pid +1 2 0.1 0 12000",
        "pid -4294967295 2 0.1 0 12000", "pid 4 2 0.1 0 12000",
        "pid 1 2 0.1 0", "pid 1 2 0.1 0 12000junk",
        "pid 1 2 0.1 0 12000 1", "pid 1 2 1e-9999 0 12000",
        "ramp 1e-9999", "ramp 1e9999", "ramp nan", "ramp 100+1",
        "ramp", "ramp 100 1", "off junk", ""
    };
    telemetry.applied_sequence = 42U;
    telemetry.tuning.motor[2].kp = 9.0f;
    telemetry.tuning.ramp_rpm_s = 1000.0f;

    for (unsigned i = 0U; i < sizeof(invalid) / sizeof(invalid[0]); ++i)
    {
        FrictionUsbResult_e result = FrictionUsbExecute(invalid[i], received_ms);
        if (result != FRICTION_USB_SYNTAX)
        {
            fprintf(stderr, "Malformed command accepted: %s (%s)\n",
                    invalid[i], FrictionUsbResultName(result));
        }
        assert(result == FRICTION_USB_SYNTAX);
        assert(submits == 0U && stops == 0U && tunes == 0U);
    }
    assert(FrictionUsbExecute(NULL, received_ms) == FRICTION_USB_UNAVAILABLE);

    /* A prior libc failure must not poison a later valid conversion. */
    errno = ERANGE;
    assert(FrictionUsbExecute("set  007  +5e2  -2.5e1  .5  ", received_ms) == FRICTION_USB_QUEUED);
    assert(submits == 1U && submitted.enable_mask == 7U);
    assert(submitted.target_rpm[0] == 500.0f && submitted.target_rpm[1] == -25.0f);
    assert(submitted.target_rpm[2] == 0.5f);
    assert(FrictionUsbExecute("set\t0\t0\t-0\t0\t", received_ms) == FRICTION_USB_QUEUED);
    assert(submits == 2U && submitted.enable_mask == 0U);
    assert(FrictionUsbExecute("pid 2 2.2 0.03 0 12000", received_ms) == FRICTION_USB_QUEUED);
    assert(tunes == 1U && fabsf(queued.motor[1].kp - 2.2f) < 1e-6f);
    assert(queued.motor[2].kp == 9.0f && queued.ramp_rpm_s == 1000.0f);
    assert(FrictionUsbExecute("ramp 1.2e3", received_ms) == FRICTION_USB_QUEUED);
    assert(tunes == 2U && queued.ramp_rpm_s == 1200.0f);
    assert(FrictionUsbExecute("off", received_ms) == FRICTION_USB_QUEUED);
    assert(stops == 1U);
    puts("PASS: malformed USB numbers have no command/tuning/stop side effects; valid syntax retained");
    return 0;
}
