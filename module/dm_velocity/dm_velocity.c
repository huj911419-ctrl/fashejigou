#include "dm_velocity.h"
#include <float.h>
#include <math.h>
#include <string.h>

_Static_assert(sizeof(float) == 4 && FLT_RADIX == 2 && FLT_MANT_DIG == 24,
               "DM speed protocol requires IEEE-754 binary32");

enum
{
    DM_PARAMETER_ID = 0x7FF,
    DM_SPEED_ID_OFFSET = 0x200,
    DM_READ = 0x33,
    DM_WRITE = 0x55,
    DM_MASTER_ID_REGISTER = 0x07,
    DM_MODE_REGISTER = 0x0A,
    DM_SPEED_MODE = 3,
    DM_ENABLE = 0xFC,
    DM_DISABLE = 0xFD,
    DM_PARAMETER_RETRY_MS = 100
};

static void PutU32(uint8_t *bytes, uint32_t value)
{
    for (unsigned i = 0U; i < 4U; ++i)
    {
        bytes[i] = (uint8_t)(value >> (8U * i));
    }
}

static uint32_t GetU32(const uint8_t *bytes)
{
    return (uint32_t)bytes[0] | ((uint32_t)bytes[1] << 8) |
           ((uint32_t)bytes[2] << 16) | ((uint32_t)bytes[3] << 24);
}

static void Fault(DmVelocity_s *motor, uint32_t reason)
{
    motor->fault |= reason;
    motor->state = DM_VELOCITY_FAULT;
    motor->waiting_reply = false;
}

static void Enter(DmVelocity_s *motor, DmVelocityState_e state, uint32_t now)
{
    motor->state = state;
    motor->stage_ms = now;
    motor->waiting_reply = false;
}

static bool Transmit(DmVelocity_s *motor, DmCanFrame_s *frame, uint32_t now)
{
    motor->last_tx_ms = now;
    motor->sent = true;
    if (motor->send == NULL || !motor->send(motor->context, frame))
    {
        Fault(motor, DM_VELOCITY_FAULT_TX);
        return false;
    }
    return true;
}

static bool SendSpecial(DmVelocity_s *motor, uint8_t command, uint32_t now)
{
    DmCanFrame_s frame = {.id = motor->config.esc_id, .length = 8U};
    memset(frame.data, 0xFF, sizeof(frame.data));
    frame.data[7] = command;
    return Transmit(motor, &frame, now);
}

static void SendParameter(DmVelocity_s *motor, uint8_t opcode, uint8_t reg, uint32_t now)
{
    DmCanFrame_s frame = {.id = DM_PARAMETER_ID, .length = opcode == DM_READ ? 4U : 8U};
    frame.data[0] = (uint8_t)motor->config.esc_id;
    frame.data[1] = (uint8_t)(motor->config.esc_id >> 8);
    frame.data[2] = opcode;
    frame.data[3] = reg;
    if (opcode == DM_WRITE)
    {
        PutU32(&frame.data[4], DM_SPEED_MODE);
    }
    motor->waiting_reply = Transmit(motor, &frame, now);
}

static void SendSpeed(DmVelocity_s *motor, float rate, uint32_t now)
{
    DmCanFrame_s frame = {.id = DM_SPEED_ID_OFFSET + motor->config.esc_id, .length = 4U};
    uint32_t bits;
    memcpy(&bits, &rate, sizeof(bits));
    PutU32(frame.data, bits);
    (void)Transmit(motor, &frame, now);
}

bool DmVelocityInit(DmVelocity_s *motor, const DmVelocityConfig_s *config,
                    DmVelocitySend_f send, void *context, uint32_t now_ms)
{
    memset(motor, 0, sizeof(*motor));
    motor->master_id = UINT16_MAX;
    motor->send = send;
    motor->context = context;
    motor->boot_ms = now_ms;
    if (config == NULL)
    {
        Fault(motor, DM_VELOCITY_FAULT_CONFIG);
        return false;
    }
    motor->config = *config;
    /* Avoid the shared parameter ID and the other command mode's ID space. */
    if (send == NULL || config->esc_id > 0xFFU ||
        !isfinite(config->max_rate_rad_s) || config->max_rate_rad_s <= 0.0f ||
        !isfinite(config->position_range_rad) || config->position_range_rad <= 0.0f ||
        !isfinite(config->velocity_range_rad_s) || config->velocity_range_rad_s <= 0.0f ||
        !isfinite(config->torque_range_nm) || config->torque_range_nm <= 0.0f ||
        config->max_rate_rad_s > config->velocity_range_rad_s ||
        config->command_period_ms == 0U || config->command_period_ms > 20U ||
        config->feedback_timeout_ms < 2U * config->command_period_ms ||
        config->feedback_timeout_ms > 1000U || config->setup_timeout_ms < 1000U ||
        config->setup_timeout_ms > 10000U || config->power_up_ms > 10000U ||
        config->max_temperature_c == 0U)
    {
        Fault(motor, DM_VELOCITY_FAULT_CONFIG);
        return false;
    }
    return true;
}

static bool ConsumeParameter(DmVelocity_s *motor, const DmCanFrame_s *frame)
{
    const uint8_t *b = frame->data;
    uint16_t echo = b[0] | ((uint16_t)b[1] << 8);
    if (echo != motor->config.esc_id || (b[2] != DM_READ && b[2] != DM_WRITE))
    {
        return false;
    }
    if (!motor->waiting_reply)
    {
        return true; /* Never interpret late parameter replies as feedback. */
    }
    uint32_t value = GetU32(&b[4]);
    if (motor->state == DM_VELOCITY_DISCOVER && b[2] == DM_READ && b[3] == DM_MASTER_ID_REGISTER)
    {
        if (value <= 0x7FFU && frame->id == value)
        {
            motor->master_id = (uint16_t)value;
            Enter(motor, DM_VELOCITY_STOPPING, frame->received_ms);
        }
    }
    else if (frame->id == motor->master_id && b[3] == DM_MODE_REGISTER)
    {
        if (motor->state == DM_VELOCITY_SET_MODE && b[2] == DM_WRITE && value == DM_SPEED_MODE)
        {
            Enter(motor, DM_VELOCITY_CHECK_MODE, frame->received_ms);
        }
        else if (motor->state == DM_VELOCITY_CHECK_MODE && b[2] == DM_READ && value == DM_SPEED_MODE)
        {
            Enter(motor, DM_VELOCITY_READY, frame->received_ms);
        }
    }
    return true;
}

void DmVelocityReceive(DmVelocity_s *motor, const DmCanFrame_s *frame)
{
    if (frame == NULL || frame->length != 8U || frame->id > 0x7FFU)
    {
        return;
    }
    if (ConsumeParameter(motor, frame) || frame->id != motor->master_id ||
        (frame->data[0] & 0x0FU) != (motor->config.esc_id & 0x0FU))
    {
        return;
    }
    const uint8_t *b = frame->data;
    DmVelocityFeedback_s *feedback = &motor->feedback;
    uint16_t position = ((uint16_t)b[1] << 8) | b[2];
    uint16_t velocity = ((uint16_t)b[3] << 4) | (b[4] >> 4);
    uint16_t torque = ((uint16_t)(b[4] & 0x0FU) << 8) | b[5];
    feedback->position_rad = ((float)position * (2.0f / 65535.0f) - 1.0f) * motor->config.position_range_rad;
    feedback->velocity_rad_s = ((float)velocity * (2.0f / 4095.0f) - 1.0f) * motor->config.velocity_range_rad_s;
    feedback->torque_nm = ((float)torque * (2.0f / 4095.0f) - 1.0f) * motor->config.torque_range_nm;
    feedback->motor_state = b[0] >> 4;
    feedback->mos_temperature_c = b[6];
    feedback->coil_temperature_c = b[7];
    feedback->received_ms = frame->received_ms;
    feedback->valid = true;
    ++feedback->count;
    if (feedback->motor_state > 1U || b[6] >= motor->config.max_temperature_c ||
        b[7] >= motor->config.max_temperature_c)
    {
        Fault(motor, DM_VELOCITY_FAULT_MOTOR);
    }
    else if (motor->state == DM_VELOCITY_STOPPING && feedback->motor_state == 0U)
    {
        Enter(motor, DM_VELOCITY_SET_MODE, frame->received_ms);
    }
    else if (motor->state == DM_VELOCITY_STARTING && feedback->motor_state == 1U)
    {
        Enter(motor, DM_VELOCITY_RUNNING, frame->received_ms);
    }
}

bool DmVelocityAvailable(const DmVelocity_s *motor, uint32_t now_ms)
{
    return motor->fault == 0U && motor->state >= DM_VELOCITY_READY &&
           motor->state <= DM_VELOCITY_RUNNING && motor->feedback.valid &&
           now_ms - motor->feedback.received_ms <= motor->config.feedback_timeout_ms;
}

void DmVelocityStep(DmVelocity_s *motor, uint32_t now, bool enable, float rate)
{
    if (motor->fault & DM_VELOCITY_FAULT_CONFIG)
    {
        return; /* An invalid ID must never be transmitted. */
    }
    if (!isfinite(rate) || fabsf(rate) > motor->config.max_rate_rad_s)
    {
        Fault(motor, DM_VELOCITY_FAULT_COMMAND);
    }
    if (motor->state == DM_VELOCITY_BOOT && now - motor->boot_ms < motor->config.power_up_ms)
    {
        return;
    }
    if (motor->state > DM_VELOCITY_BOOT && motor->state < DM_VELOCITY_READY &&
        now - motor->boot_ms > motor->config.power_up_ms + motor->config.setup_timeout_ms)
    {
        Fault(motor, DM_VELOCITY_FAULT_TIMEOUT);
    }
    if (motor->state == DM_VELOCITY_RUNNING && (!DmVelocityAvailable(motor, now) ||
        motor->feedback.motor_state != 1U))
    {
        Fault(motor, DM_VELOCITY_FAULT_MOTOR);
    }
    if (motor->state == DM_VELOCITY_STARTING && now - motor->stage_ms > motor->config.feedback_timeout_ms)
    {
        Fault(motor, DM_VELOCITY_FAULT_TIMEOUT);
    }
    bool stopping = !enable && (motor->state == DM_VELOCITY_RUNNING || motor->state == DM_VELOCITY_STARTING);
    if (stopping)
    {
        Enter(motor, DM_VELOCITY_READY, now);
        (void)SendSpecial(motor, DM_DISABLE, now); /* Bypass periodic throttling. */
        return;
    }
    if (motor->sent && now - motor->last_tx_ms < motor->config.command_period_ms)
    {
        return;
    }
    switch (motor->state)
    {
        case DM_VELOCITY_BOOT:
            if (SendSpecial(motor, DM_DISABLE, now))
            {
                Enter(motor, DM_VELOCITY_DISCOVER, now);
            }
            break;
        case DM_VELOCITY_DISCOVER:
        case DM_VELOCITY_SET_MODE:
        case DM_VELOCITY_CHECK_MODE:
            if (!motor->waiting_reply || now - motor->last_tx_ms >= DM_PARAMETER_RETRY_MS)
            {
                uint8_t opcode = motor->state == DM_VELOCITY_SET_MODE ? DM_WRITE : DM_READ;
                uint8_t reg = motor->state == DM_VELOCITY_DISCOVER ? DM_MASTER_ID_REGISTER : DM_MODE_REGISTER;
                SendParameter(motor, opcode, reg, now);
            }
            break;
        case DM_VELOCITY_STOPPING:
        case DM_VELOCITY_FAULT:
            (void)SendSpecial(motor, DM_DISABLE, now);
            break;
        case DM_VELOCITY_READY:
            if (enable && DmVelocityAvailable(motor, now) && motor->feedback.motor_state == 0U)
            {
                /* Overwrite any old speed BEFORE enable. Enable is next cycle. */
                SendSpeed(motor, 0.0f, now);
                if (motor->fault == 0U)
                {
                    Enter(motor, DM_VELOCITY_STARTING, now);
                }
            }
            else
            {
                (void)SendSpecial(motor, DM_DISABLE, now);
            }
            break;
        case DM_VELOCITY_STARTING:
            (void)SendSpecial(motor, DM_ENABLE, now);
            break;
        case DM_VELOCITY_RUNNING:
            SendSpeed(motor, rate, now);
            break;
    }
}
