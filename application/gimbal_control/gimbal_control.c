#include "gimbal_control.h"
#include <string.h>

bool GimbalControlInit(GimbalControl_s *control, const GimbalControlConfig_s *config,
                       DmVelocitySend_f send, void *context, uint32_t now_ms)
{
    if (control == NULL)
    {
        return false;
    }
    memset(control, 0, sizeof(*control));
    if (config == NULL)
    {
        control->motor.fault = DM_VELOCITY_FAULT_CONFIG;
        control->motor.state = DM_VELOCITY_FAULT;
        return false;
    }
    control->config = *config;
    bool yaw_ok = YawRateInit(&control->yaw, &config->yaw);
    bool motor_ok = DmVelocityInit(&control->motor, &config->motor, send, context, now_ms);
    control->configured = yaw_ok && motor_ok &&
                          config->command_timeout_ms > 0U && config->command_timeout_ms < 0x80000000U &&
                          config->yaw.max_rate_rad_s <= config->motor.max_rate_rad_s;
    if (!control->configured)
    {
        control->motor.fault |= DM_VELOCITY_FAULT_CONFIG;
        control->motor.state = DM_VELOCITY_FAULT;
    }
    return control->configured;
}

void GimbalControlReceiveCan(GimbalControl_s *control, const DmCanFrame_s *frame,
                             uint32_t now_ms)
{
    if (control->configured && frame != NULL &&
        now_ms - frame->received_ms <= control->config.motor.feedback_timeout_ms)
    {
        DmVelocityReceive(&control->motor, frame);
    }
}

void GimbalControlTransportFault(GimbalControl_s *control, uint32_t faults)
{
    if (faults != 0U)
    {
        control->transport_faults |= faults;
        control->transport_fault_pending = true;
        YawRateDisarm(&control->yaw);
    }
}

static void UpdateStatus(GimbalControl_s *control, const GimbalCommand_s *command,
                         uint32_t now, bool online, bool port_ready)
{
    const DmVelocity_s *motor = &control->motor;
    bool feedback_fresh = motor->feedback.valid &&
                          now - motor->feedback.received_ms <= control->config.motor.feedback_timeout_ms;
    control->status = (GimbalControlStatus_s)
    {
        .sample_ms = now,
        .timing = control->timing,
        .request = *command,
        .transport_faults = control->transport_faults,
        .motor_fault = motor->fault,
        .inhibit = control->configured ? control->yaw.inhibit : YAW_INHIBIT_CONFIG,
        .master_id = motor->master_id,
        .motor_state = motor->state,
        .feedback = motor->feedback,
        .target_rad_s = control->yaw.target_rad_s,
        .reference_rad_s = control->yaw.reference_rad_s,
        .command_online = online,
        .enabled = control->yaw.enabled,
        .motor_online = feedback_fresh,
        .motor_enabled = feedback_fresh && motor->feedback.motor_state == 1U,
        .port_ready = port_ready && control->configured
    };
}

void GimbalControlStep(GimbalControl_s *control, const GimbalCommand_s *command,
                       uint32_t now_ms, bool port_ready)
{
    const GimbalCommand_s stopped = {0};
    if (command == NULL)
    {
        command = &stopped;
    }
    LoopStatsObserve(&control->timing, now_ms, control->config.yaw.max_step_ms);
    if (!control->configured)
    {
        UpdateStatus(control, command, now_ms, false, false);
        return;
    }
    if (command->stop_requested)
    {
        YawRateDisarm(&control->yaw);
    }
    bool online = command->valid && now_ms - command->received_ms <= control->config.command_timeout_ms;
    bool ready = port_ready && !control->transport_fault_pending &&
                 DmVelocityAvailable(&control->motor, now_ms);
    YawRateStep(&control->yaw, now_ms, online, ready, command->arm, command->rate_rad_s);
    if (port_ready)
    {
        DmVelocityStep(&control->motor, now_ms, control->yaw.enabled, control->yaw.reference_rad_s);
        if (control->motor.fault != 0U)
        {
            uint32_t inhibit = control->yaw.inhibit;
            YawRateDisarm(&control->yaw);
            control->yaw.inhibit = inhibit | YAW_INHIBIT_ACTUATOR;
        }
    }
    control->transport_fault_pending = false;
    UpdateStatus(control, command, now_ms, online, port_ready);
}
