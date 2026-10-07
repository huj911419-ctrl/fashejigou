#include "speed_ready.h"
#include "friction_monitor.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>

static const SpeedReady_Config_s config = {
    .minimum_target = 100.0f,
    .reference_tolerance = 1.0f,
    .enter_absolute_error = 10.0f,
    .enter_relative_error = 0.02f,
    .exit_absolute_error = 20.0f,
    .exit_relative_error = 0.04f,
    .dwell_ms = 200U,
    .maximum_sample_gap_ms = 5U,
};

static SpeedReady_Input_s Sample(float target)
{
    SpeedReady_Input_s sample = {
        .target = target,
        .reference = target,
        .measured = target,
        .enabled = 1,
        .feedback_valid = 1,
    };
    return sample;
}

static uint32_t Qualify(SpeedReady_s *detector, SpeedReady_Input_s *sample, uint32_t now)
{
    assert(!SpeedReadyUpdate(detector, sample, now).ready);
    for (unsigned i = 1; i <= config.dwell_ms; ++i)
    {
        SpeedReady_Status_s result = SpeedReadyUpdate(detector, sample, now + i);
        assert(result.ready == (i == config.dwell_ms));
        assert(result.stable_ms == i);
    }
    return now + config.dwell_ms;
}

static void TestScalar(void)
{
    SpeedReady_s detector;
    assert(SpeedReadyInit(&detector, &config));
    SpeedReady_Input_s sample = Sample(500);
    uint32_t now = Qualify(&detector, &sample, 0);

    /* Entry band is 10 rpm; exit band is 20 rpm. */
    sample.measured = 480;
    assert(SpeedReadyUpdate(&detector, &sample, ++now).ready);
    sample.measured = 479;
    assert(!SpeedReadyUpdate(&detector, &sample, ++now).ready);
    sample.measured = 485;
    for (unsigned i = 0; i < 250; ++i)
    {
        assert(!SpeedReadyUpdate(&detector, &sample, ++now).ready);
    }
    sample.measured = 490;
    now = Qualify(&detector, &sample, now + 1);

    sample.target = 501;
    sample.reference = sample.measured = 501;
    assert(!SpeedReadyUpdate(&detector, &sample, ++now).ready);
    SpeedReadyReset(&detector);
    now = Qualify(&detector, &sample, now + 1);
    assert(!SpeedReadyUpdate(&detector, &sample, now).ready); /* duplicate time */
    SpeedReadyReset(&detector);
    now = Qualify(&detector, &sample, now + 1);
    assert(!SpeedReadyUpdate(&detector, &sample, now + 6).ready);

    /* A perfectly followed ramp must never qualify before the final target. */
    SpeedReadyReset(&detector);
    sample = Sample(1000);
    sample.reference = sample.measured = 500;
    for (unsigned i = 0; i < 300; ++i)
    {
        assert(!SpeedReadyUpdate(&detector, &sample, ++now).ready);
    }

    sample = Sample(50);
    for (unsigned i = 0; i < 300; ++i)
    {
        assert(!SpeedReadyUpdate(&detector, &sample, ++now).ready);
    }
    sample = Sample(-500);
    sample.measured = 500;
    assert(!SpeedReadyUpdate(&detector, &sample, ++now).ready);
    sample.measured = -500;
    now = Qualify(&detector, &sample, now + 1); /* reusable detector supports reverse */
    sample.feedback_valid = 0;
    assert(!SpeedReadyUpdate(&detector, &sample, ++now).ready);
    sample.feedback_valid = 1;
    now = Qualify(&detector, &sample, now + 1);
    sample.enabled = 0;
    assert(!SpeedReadyUpdate(&detector, &sample, ++now).ready);
    sample.enabled = 1;
    sample.measured = NAN;
    assert(!SpeedReadyUpdate(&detector, &sample, ++now).ready);
    sample = Sample(500);
    SpeedReadyReset(&detector);
    now = Qualify(&detector, &sample, UINT32_MAX - 100U);
    assert(now == 99U);

    SpeedReady_Config_s invalid = config;
    invalid.exit_absolute_error = 1;
    assert(!SpeedReadyInit(&detector, &invalid));
    assert(!SpeedReadyUpdate(&detector, &sample, ++now).ready);
    invalid = config;
    invalid.enter_relative_error = INFINITY;
    assert(!SpeedReadyInit(&detector, &invalid));
    assert(!SpeedReadyInit(&detector, NULL));
    puts("PASS: dwell boundary, hysteresis, ramp exclusion, minimum/direction, target/reset invalidation, feedback/enable loss, invalid config/data, sample gaps and clock wrap");
}

static void TestGroup(void)
{
    Friction_Monitor_s monitor;
    assert(FrictionMonitorInit(&monitor, &config));
    Friction_Status_s status = {0};
    FrictionMonitorUpdate(&monitor, &status, 0);
    assert(monitor.status.phase == FRICTION_PHASE_STOPPED);

    status.enabled_mask = status.online_mask = FRICTION_ALL_MOTORS;
    for (unsigned i = 0; i < FRICTION_MOTOR_COUNT; ++i)
    {
        status.requested_rpm[i] = 500 + 100 * i; /* no forced equal-speed assumption */
        status.reference_rpm[i] = status.requested_rpm[i];
        status.feedback[i].rpm = status.requested_rpm[i];
        status.feedback[i].valid = 1;
    }
    uint32_t now;
    for (now = 1; now <= 201; ++now)
    {
        FrictionMonitorUpdate(&monitor, &status, now);
        assert(monitor.status.all_ready == (now == 201));
    }
    assert(monitor.status.phase == FRICTION_PHASE_READY);

    status.feedback[1].rpm -= 100;
    FrictionMonitorUpdate(&monitor, &status, ++now);
    assert(!monitor.status.all_ready && monitor.status.ready_mask == 5);
    assert(monitor.status.phase == FRICTION_PHASE_RECOVERING);
    status.feedback[1].rpm = status.requested_rpm[1];
    for (unsigned i = 0; i <= config.dwell_ms; ++i)
    {
        FrictionMonitorUpdate(&monitor, &status, ++now);
        assert(monitor.status.all_ready == (i == config.dwell_ms));
    }

    status.requested_rpm[2] += 1;
    status.reference_rpm[2] = status.feedback[2].rpm = status.requested_rpm[2];
    FrictionMonitorUpdate(&monitor, &status, ++now);
    assert(!monitor.status.ready_mask && monitor.status.phase == FRICTION_PHASE_SPINUP);
    status.enabled_mask = 1;
    for (unsigned i = 0; i <= config.dwell_ms; ++i)
    {
        FrictionMonitorUpdate(&monitor, &status, ++now);
    }
    assert(monitor.status.ready_mask == 1 && !monitor.status.all_ready);
    assert(monitor.status.phase == FRICTION_PHASE_BENCH);

    status.enabled_mask = FRICTION_ALL_MOTORS;
    status.requested_rpm[0] = status.reference_rpm[0] = status.feedback[0].rpm = -500;
    for (unsigned i = 0; i <= config.dwell_ms; ++i)
    {
        FrictionMonitorUpdate(&monitor, &status, ++now);
    }
    assert(monitor.status.ready_mask == FRICTION_ALL_MOTORS && !monitor.status.all_ready);
    assert(monitor.status.phase == FRICTION_PHASE_BENCH);
    status.fault = FRICTION_FAULT_OFFLINE;
    FrictionMonitorUpdate(&monitor, &status, ++now);
    assert(!monitor.status.ready_mask && monitor.status.phase == FRICTION_PHASE_FAULT);
    status.fault = 0;
    status.enabled_mask = 0;
    FrictionMonitorUpdate(&monitor, &status, ++now);
    assert(monitor.status.phase == FRICTION_PHASE_STOPPED);
    puts("PASS: independent wheel targets, group dwell, recovery, target change, partial/reverse bench isolation and faults");
}

int main(void)
{
    TestScalar();
    TestGroup();
    return 0;
}
