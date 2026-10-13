#include "../../bench_features.h"
#include "gimbal_diagnostics.h"
#include "../gimbal.h"
#include "../config/gimbal_config.h"
#include "gimbal_port.h"
#include "../imu/gimbal_imu.h"
#include "../config/gimbal_imu_config.h"
#include "control_platform.h"
#include "diagnostics.h"

static const char *MotorState(DmVelocityState_e state)
{
    static const char *const names[] = {"BOOT", "DISCOVER", "STOPPING", "SET_MODE", "CHECK_MODE",
                                        "READY", "STARTING", "RUNNING", "FAULT"};
    return (unsigned)state < sizeof(names) / sizeof(names[0]) ? names[state] : "UNKNOWN";
}

static void YawReport(TextConsole_s *console, uint32_t now_ms)
{
    if (!BENCH_ENABLE_YAW && !BENCH_ENABLE_REMOTE)
    {
        TextConsolePrint(console, "[DBUS] DISABLED");
        TextConsolePrint(console, "[DM-J4310] DISABLED");
        return;
    }
    GimbalStatus_s yaw;
    GimbalPortStats_s io;
    const DmVelocityConfig_s config = GIMBAL_DM_CONFIG;
    GimbalGetStatus(&yaw);
    GimbalPortGetStats(&io);
    now_ms = ControlPlatformNowMs();
    const char *owner = BENCH_ENABLE_YAW ? "yaw" : "remote";
    DiagnosticsPrintTiming(console, owner, &yaw.axis.timing);
    /* Read-time age is mandatory: a stalled owner must not look healthy. */
    bool fresh = now_ms - yaw.axis.sample_ms <= GIMBAL_CONTROL_PERIOD_MS * 5U;
    bool remote_fresh = yaw.remote.input.valid &&
                        now_ms - yaw.remote.input.received_ms <= GIMBAL_REMOTE_TIMEOUT_MS;
    bool motor_fresh = yaw.axis.feedback.valid &&
                       now_ms - yaw.axis.feedback.received_ms <= config.feedback_timeout_ms;
    TextConsolePrint(console, "[%s_task] %s sample_age_ms=%lu port_ready=%u io_fault_history=0x%lx",
                     owner, fresh ? "UPDATING" : "STALE", (unsigned long)(now_ms - yaw.axis.sample_ms),
                     yaw.axis.port_ready, (unsigned long)yaw.axis.transport_faults);
    if (BENCH_ENABLE_COMMAND_AUTHORITY)
    {
        static const char *const sources[] = {"NONE", "REMOTE", "DEBUG", "AUTO"};
        unsigned selected = (unsigned)yaw.authority.selected;
        TextConsolePrint(console, "[yaw_authority] snapshot=%s source=%s epoch=%lu online=%u permitted=%u reason=0x%lx; actuator checks remain separate",
                         fresh ? "FRESH" : "STALE", selected < 4U ? sources[selected] : "UNKNOWN",
                         (unsigned long)yaw.authority.epoch, yaw.authority.online,
                         yaw.authority.authorized, (unsigned long)yaw.authority.reason);
    }
    else
    {
        TextConsolePrint(console, "[yaw_authority] DISABLED; direct remote path with actuator protections");
    }
    if (BENCH_ENABLE_REMOTE)
    {
        TextConsolePrint(console, "[DBUS] %s UART5=PD2/100000/8E1/inverted valid_frames=%lu valid_age_ms=%lu raw_bytes=%lu raw_age_ms=%lu",
                         remote_fresh && fresh ? "ONLINE" : io.uart_bytes == 0U ? "NO_BYTES" : "NO_FRESH_VALID_FRAME",
                         (unsigned long)yaw.remote.input.sequence, (unsigned long)(now_ms - yaw.remote.input.received_ms),
                         (unsigned long)io.uart_bytes, (unsigned long)(now_ms - io.uart_last_ms));
        TextConsolePrint(console, "[RC] channels_milli=%ld,%ld,%ld,%ld S1=%u S2=%u keys=0x%04x arm_switch=%u",
                         (long)TextConsoleScaled(yaw.remote.input.channel[0], 1000.0),
                         (long)TextConsoleScaled(yaw.remote.input.channel[1], 1000.0),
                         (long)TextConsoleScaled(yaw.remote.input.channel[2], 1000.0),
                         (long)TextConsoleScaled(yaw.remote.input.channel[3], 1000.0),
                         yaw.remote.input.s1, yaw.remote.input.s2, yaw.remote.input.keys, yaw.remote.arm_switch);
    }
    else
    {
        TextConsolePrint(console, "[DBUS] DISABLED");
    }
    TextConsolePrint(console, "[gimbal_io] CAN1_rx=%lu filtered=%lu rx_age_ms=%lu tx_queued=%lu tx_rejected=%lu tx_aborted=%lu UARTerr=%lu CANerr=%lu overflow=%lu",
                     (unsigned long)io.can_frames, (unsigned long)io.can_filtered,
                     (unsigned long)(now_ms - io.can_last_ms), (unsigned long)io.tx_accepted,
                     (unsigned long)io.tx_rejected, (unsigned long)io.tx_aborted,
                     (unsigned long)io.uart_errors, (unsigned long)io.can_errors, (unsigned long)io.overflows);
    if (!BENCH_ENABLE_YAW)
    {
        TextConsolePrint(console, "[DM-J4310] DISABLED");
        return;
    }
    TextConsolePrint(console, "[DM-J4310] link=%s stage=%s CAN1 esc=0x%03x master=0x%03x fault=0x%lx feedback_count=%lu age_ms=%lu",
                     motor_fresh && fresh ? "ONLINE" : yaw.axis.feedback.count == 0U ? "NO_FEEDBACK" : "STALE",
                     MotorState(yaw.axis.motor_state), GIMBAL_YAW_ESC_ID, yaw.axis.master_id,
                     (unsigned long)yaw.axis.motor_fault, (unsigned long)yaw.axis.feedback.count,
                     (unsigned long)(now_ms - yaw.axis.feedback.received_ms));
    TextConsolePrint(console, "[yaw] app_enable=%u motor_on=%u drive_state=%u target_mrad_s=%ld ref_mrad_s=%ld measured_mrad_s=%ld",
                     yaw.axis.enabled, yaw.axis.motor_enabled, yaw.axis.feedback.motor_state,
                     (long)TextConsoleScaled(yaw.axis.target_rad_s, 1000.0),
                     (long)TextConsoleScaled(yaw.axis.reference_rad_s, 1000.0),
                     (long)TextConsoleScaled(yaw.axis.feedback.velocity_rad_s, 1000.0));
    uint32_t inhibit = yaw.axis.inhibit;
    TextConsolePrint(console, "[yaw_inhibit] snapshot=%s mask=0x%lx config=%u command=%u actuator=%u timing=%u rate=%u switch_off=%u rearm=%u start_nonzero=%u",
                     fresh ? "FRESH" : "STALE", (unsigned long)inhibit,
                     (inhibit & YAW_INHIBIT_CONFIG) != 0U,
                     (inhibit & YAW_INHIBIT_COMMAND) != 0U,
                     (inhibit & YAW_INHIBIT_ACTUATOR) != 0U,
                     (inhibit & YAW_INHIBIT_TIMING) != 0U,
                     (inhibit & YAW_INHIBIT_RATE) != 0U,
                     (inhibit & YAW_INHIBIT_SWITCH_OFF) != 0U,
                     (inhibit & YAW_INHIBIT_REARM_REQUIRED) != 0U,
                     (inhibit & YAW_INHIBIT_START_NOT_ZERO) != 0U);
    TextConsolePrint(console, "[DM_feedback] raw_position_mrad=%ld torque_mNm=%ld mos_C=%u coil_C=%u; position is protocol-wrapped",
                     (long)TextConsoleScaled(yaw.axis.feedback.position_rad, 1000.0),
                     (long)TextConsoleScaled(yaw.axis.feedback.torque_nm, 1000.0),
                     yaw.axis.feedback.mos_temperature_c, yaw.axis.feedback.coil_temperature_c);
    if (BENCH_ENABLE_REMOTE && !remote_fresh)
    {
        TextConsolePrint(console, "[hint RC] %s", io.uart_bytes == 0U ?
                         "check receiver power, PD2 wiring, transmitter link and DBUS output" :
                         "bytes alone are not DBUS: check 100k/8E1, inversion, protocol and frame freshness");
    }
    if (yaw.axis.motor_fault != 0U)
    {
        TextConsolePrint(console, "[hint DM] fault bits: 1=config 2=TX 4=timeout 8=motor 16=command; inspect power/ID/mode; faults latch");
    }
}

#if GIMBAL_IMU_ENABLED
static void ImuReport(TextConsole_s *console, uint32_t now_ms)
{
    static const char *const sensor_names[] = {"BOOT", "RESET_WAIT", "PROBE", "CONFIG_WRITE", "VERIFY", "READY", "FAULT"};
    static const char *const attitude_names[] = {"CALIBRATING", "TRACKING", "LOST", "BAD_CONFIG"};
    GimbalImuStatus_s imu;
    GimbalImuGetStatus(&imu);
    now_ms = ControlPlatformNowMs();
    const AttitudeOutput_s *a = &imu.attitude;
    DiagnosticsPrintTiming(console, "imu", &imu.timing);
    const AttitudeConfig_s config = GIMBAL_ATTITUDE_CONFIG;
    bool sensor_fresh = imu.sample_count != 0U && now_ms - imu.sensor_sample.timestamp_ms <= config.stale_ms;
    TextConsolePrint(console, "[BMI088] %s SPI2 port=%u stage=%s fault=0x%lx samples=%lu age_ms=%lu temp_cC=%ld",
                     imu.sensor_fault != 0U ? "FAULT" : sensor_fresh ? "ONLINE" : imu.sample_count == 0U ? "STARTING" : "STALE",
                     imu.port_ready, (unsigned)imu.sensor_state < 7U ? sensor_names[imu.sensor_state] : "UNKNOWN",
                     (unsigned long)imu.sensor_fault, (unsigned long)imu.sample_count,
                     (unsigned long)(now_ms - imu.sensor_sample.timestamp_ms),
                     (long)TextConsoleScaled(imu.sensor_sample.temperature_c, 100.0));
    TextConsolePrint(console, "[imu_sensor] gyro_mrad_s=%ld,%ld,%ld accel_mm_s2=%ld,%ld,%ld",
                     (long)TextConsoleScaled(imu.sensor_sample.gyro_rad_s[0], 1000.0),
                     (long)TextConsoleScaled(imu.sensor_sample.gyro_rad_s[1], 1000.0),
                     (long)TextConsoleScaled(imu.sensor_sample.gyro_rad_s[2], 1000.0),
                     (long)TextConsoleScaled(imu.sensor_sample.accel_m_s2[0], 1000.0),
                     (long)TextConsoleScaled(imu.sensor_sample.accel_m_s2[1], 1000.0),
                     (long)TextConsoleScaled(imu.sensor_sample.accel_m_s2[2], 1000.0));
    if (!BENCH_ATTITUDE_ACTIVE)
    {
        TextConsolePrint(console, "[attitude] DISABLED; raw BMI088 sampling remains active");
        return;
    }
    TextConsolePrint(console, "[attitude] %s fault=0x%lx cal=%lu/%lu valid=%u heading=%u continuous=%u for_control=%u mount_confirmed=%u",
                     (unsigned)a->state < 4U ? attitude_names[a->state] : "UNKNOWN", (unsigned long)a->fault,
                     (unsigned long)a->calibration_count, (unsigned long)config.calibration_samples,
                     a->valid, a->heading_valid, a->yaw_continuous, imu.valid_for_control, imu.mounting_confirmed);
    TextConsolePrint(console, "[attitude] RPY_mrad=%ld,%ld,%ld relative_yaw_total_mrad=%ld gravity_correction=%u frame=%s",
                     (long)TextConsoleScaled(a->roll_rad, 1000.0), (long)TextConsoleScaled(a->pitch_rad, 1000.0),
                     (long)TextConsoleScaled(a->yaw_rad, 1000.0), (long)TextConsoleScaled(a->yaw_unwrapped_rad, 1000.0),
                     a->accel_correction_used, imu.mounting_confirmed ? "BODY_CONFIRMED" : "UNCONFIRMED_SENSOR_AXES");
    TextConsolePrint(console, "[attitude] q_wxyz_ppm=%ld,%ld,%ld,%ld gyro_bias_urad_s=%ld,%ld,%ld",
                     (long)TextConsoleScaled(a->quaternion[0], 1000000.0), (long)TextConsoleScaled(a->quaternion[1], 1000000.0),
                     (long)TextConsoleScaled(a->quaternion[2], 1000000.0), (long)TextConsoleScaled(a->quaternion[3], 1000000.0),
                     (long)TextConsoleScaled(a->gyro_bias_rad_s[0], 1000000.0),
                     (long)TextConsoleScaled(a->gyro_bias_rad_s[1], 1000000.0),
                     (long)TextConsoleScaled(a->gyro_bias_rad_s[2], 1000000.0));
    if (a->state == ATTITUDE_CALIBRATING)
    {
        TextConsolePrint(console, "[hint IMU] keep board physically still with motor disabled until calibration finishes; yaw is relative and drifts");
    }
    if (imu.sensor_fault != 0U)
    {
        TextConsolePrint(console, "[hint BMI088] fault bits: 1=SPI 2=chip ID 4=config readback 8=no fresh data 16=range 32=device status");
    }
}
#endif

void GimbalDiagnosticsReport(TextConsole_s *console, uint32_t now_ms)
{
    TextConsolePrint(console, "[modules_gimbal] remote=%u yaw=%u imu=%u attitude_requested=%u attitude_active=%u",
                     BENCH_ENABLE_REMOTE, BENCH_ENABLE_YAW, BENCH_ENABLE_IMU,
                     BENCH_ENABLE_ATTITUDE, BENCH_ATTITUDE_ACTIVE);
    YawReport(console, now_ms);
#if GIMBAL_IMU_ENABLED
    ImuReport(console, now_ms);
#else
    TextConsolePrint(console, "[BMI088] DISABLED; [attitude] DISABLED (requires IMU)");
#endif
    TextConsolePrint(console, "[profile] yaw=%s; angle_hold=NOT_IMPLEMENTED; friction=NOT_IN_THIS_FIRMWARE",
                     BENCH_ENABLE_YAW ? "manual_velocity" : "DISABLED");
}
