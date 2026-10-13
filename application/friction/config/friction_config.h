#ifndef FRICTION_CONFIG_H
#define FRICTION_CONFIG_H

/* Fill in enabled wheels' physical C620 IDs (1..8). Select wheels in
 * application/bench_features.h; a disabled wheel may retain ID=0.
 * Zero on an enabled wheel deliberately prevents group initialization.
 * Same ID on different buses is legal; duplicate IDs on one bus are rejected.
 * Direction +1/-1 maps positive RPM to the desired contact-surface direction.
 * Gains use degree/s error, current limits use C620 command units, NOT amperes.
 * Startup defaults retain the 2026-10-11 bench tuning: F1/F3 Ki=0.10,
 * F2 Ki=0.02, and a 2000 RPM/s ramp. Manual single shots were tested at
 * 5000/6000 RPM; projectile-speed and sustained thermal validation remain.
 * These defaults do not enable motors or restore a previous speed command.
 */
#ifndef FRICTION_MOTOR_CONFIGS
#define FRICTION_MOTOR_CONFIGS { \
    {.can_bus = 2, .id = 1, .direction = 1, .kp = 2.1f, .ki = 0.10f, .current_limit = 14000.0f}, \
    {.can_bus = 2, .id = 2, .direction = 1, .kp = 2.1f, .ki = 0.02f, .current_limit = 14000.0f}, \
    {.can_bus = 2, .id = 3, .direction = 1, .kp = 2.1f, .ki = 0.10f, .current_limit = 14000.0f}  \
}
#endif

#define FRICTION_CONTROL_CONFIG { \
    .max_rpm = 7500.0f, .ramp_rpm_s = 2000.0f, \
    .command_timeout_ms = 250U, .feedback_timeout_ms = 50U, \
    .max_step_ms = 5U, .max_temperature_c = 70U \
}

#define FRICTION_PERIOD_MS 1U
#define FRICTION_LOG_PERIOD_MS 200U
#define FRICTION_DIAGNOSTICS_PERIOD_MS 5000U /* Human log 1; independent of CSV/VOFA. */

/* Online tuning envelope. Current may never exceed each motor's startup limit.
 * Gains remain in degree/s units. These are input bounds, not recommended gains. */
#define FRICTION_TUNE_KP_MAX 100.0f
#define FRICTION_TUNE_KI_MAX 1000.0f
#define FRICTION_TUNE_KD_MAX 1.0f
#define FRICTION_TUNE_RAMP_MAX 2000.0f
#define FRICTION_VOFA_PERIOD_MS 20U /* 50 Hz, 180 bytes/frame */

/* Bench speed-stability thresholds, NOT calibrated launch thresholds.
 * Reaching the final target is required; tracking a still-rising ramp is not.
 * Hysteresis: enter at max(10 rpm, 2%), leave at max(20 rpm, 4%). */
#define FRICTION_READY_CONFIG { \
    .minimum_target = 100.0f, .reference_tolerance = 1.0f, \
    .enter_absolute_error = 10.0f, .enter_relative_error = 0.02f, \
    .exit_absolute_error = 20.0f, .exit_relative_error = 0.04f, \
    .dwell_ms = 200U, .maximum_sample_gap_ms = 5U \
}

#endif
