#include "gimbal_control.h"
#include "gimbal_config.h"
#include <assert.h>
#include <stdio.h>

typedef struct
{
    unsigned sent;
    uint16_t id;
} Transport_s;

static bool Send(void *context, const DmCanFrame_s *frame)
{
    Transport_s *transport = context;
    ++transport->sent;
    transport->id = frame->id;
    return true;
}

int main(void)
{
    GimbalControlConfig_s config =
    {
        .motor = GIMBAL_DM_CONFIG,
        .yaw = GIMBAL_YAW_RATE_CONFIG,
        .command_timeout_ms = 100U
    };
    GimbalControl_s first;
    GimbalControl_s second;
    Transport_s a = {0};
    Transport_s b = {0};
    assert(GimbalControlInit(&first, &config, Send, &a, 0U));
    config.motor.esc_id = 2U;
    assert(GimbalControlInit(&second, &config, Send, &b, 0U));
    config.motor.esc_id = 3U; /* Both instances own their copied configuration. */
    GimbalControlStep(&first, NULL, 2000U, true);
    GimbalControlStep(&second, NULL, 2000U, true);
    assert(a.sent == 1U && b.sent == 1U && a.id == 0U && b.id == 2U);
    GimbalControlTransportFault(&first, 4U);
    GimbalControlStep(&first, NULL, 2002U, true);
    GimbalControlStep(&second, NULL, 2002U, true);
    assert(first.status.transport_faults == 4U && second.status.transport_faults == 0U);
    assert(!first.status.enabled && !first.status.motor_enabled);
    assert(!second.status.enabled && !second.status.motor_online);
    config.command_timeout_ms = 0U;
    GimbalControl_s invalid;
    assert(!GimbalControlInit(&invalid, &config, Send, &a, 0U));
    unsigned sent = a.sent;
    GimbalControlStep(&invalid, NULL, 9000U, true);
    assert(a.sent == sent && !invalid.status.port_ready);
    assert(invalid.status.motor_fault & DM_VELOCITY_FAULT_CONFIG);
    puts("gimbal subsystem: independent instances, copied config, fail-closed validation passed");
    return 0;
}
