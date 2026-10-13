#ifndef GIMBAL_BENCH_CONFIG_H
#define GIMBAL_BENCH_CONFIG_H

/* Proven reference: CtrBoard-H7_CAN/User/dm_motor.* and User/dbus.*.
 * CAN1 / UART5 are bound only in bsp/ports/gimbal_port, not in portable modules. */
#define GIMBAL_YAW_ESC_ID 0x000U
#define GIMBAL_YAW_CHANNEL 0U
#define GIMBAL_ARM_S1_VALUE 1U
#define GIMBAL_YAW_DIRECTION 1
#if GIMBAL_YAW_DIRECTION != 1 && GIMBAL_YAW_DIRECTION != -1
#error "GIMBAL_YAW_DIRECTION must be the integer +1 or -1"
#endif
#define GIMBAL_CONTROL_PERIOD_MS 2U
#define GIMBAL_REMOTE_TIMEOUT_MS 100U
#define GIMBAL_LOG_PERIOD_MS 200U
#define GIMBAL_MAX_RATE_RAD_S 1.0f
#define GIMBAL_STICK_DEADBAND 0.04f

#define GIMBAL_REMOTE_CONFIG                        \
{                                                   \
    .timeout_ms = GIMBAL_REMOTE_TIMEOUT_MS,         \
    .max_rate_rad_s = GIMBAL_MAX_RATE_RAD_S,        \
    .deadband = GIMBAL_STICK_DEADBAND,              \
    .yaw_channel = GIMBAL_YAW_CHANNEL,              \
    .arm_s1_value = GIMBAL_ARM_S1_VALUE,            \
    .yaw_direction = GIMBAL_YAW_DIRECTION           \
}

/* Initial commissioning values, not tuned performance claims. Rates here are
 * MOTOR-side rad/s; adjust mapping if a transmission is added. */
#define GIMBAL_YAW_RATE_CONFIG                      \
{                                                   \
    .max_rate_rad_s = GIMBAL_MAX_RATE_RAD_S,        \
    .acceleration_rad_s2 = 2.0f,                    \
    .max_step_ms = 10U                              \
}

#define GIMBAL_DM_CONFIG                            \
{                                                   \
    .esc_id = GIMBAL_YAW_ESC_ID,                    \
    .max_rate_rad_s = GIMBAL_MAX_RATE_RAD_S,        \
    .position_range_rad = 12.5f,                    \
    .velocity_range_rad_s = 30.0f,                  \
    .torque_range_nm = 10.0f,                       \
    .max_temperature_c = 70U,                       \
    .power_up_ms = 2000U,                           \
    .setup_timeout_ms = 3000U,                      \
    .feedback_timeout_ms = 100U,                    \
    .command_period_ms = 10U                        \
}

#endif
