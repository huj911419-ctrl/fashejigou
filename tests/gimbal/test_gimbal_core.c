#include "dbus_input.h"
#include "dm_velocity.h"
#include "yaw_rate.h"
#include "gimbal_config.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

static DmCanFrame_s transmitted;
static unsigned tx_count;
static bool tx_ok = true;

static bool Send(void *context, const DmCanFrame_s *frame)
{
    (void)context;
    transmitted = *frame;
    ++tx_count;
    return tx_ok;
}

static void MakeRemote(uint8_t *bytes, uint16_t ch0, uint8_t s1)
{
    uint64_t bits = (uint64_t)ch0 | ((uint64_t)1024U << 11) |
                    ((uint64_t)1024U << 22) | ((uint64_t)1024U << 33) |
                    ((uint64_t)1U << 44) | ((uint64_t)s1 << 46);
    memset(bytes, 0, DBUS_FRAME_SIZE);
    for (unsigned i = 0U; i < 6U; ++i)
    {
        bytes[i] = (uint8_t)(bits >> (8U * i));
    }
}

static bool PushRemote(DbusDecoder_s *decoder, const uint8_t *bytes, uint32_t now)
{
    bool result = false;
    for (unsigned i = 0U; i < DBUS_FRAME_SIZE; ++i)
    {
        result = DbusDecoderPush(decoder, bytes[i], now);
    }
    return result;
}

static void TestRemote(void)
{
    DbusDecoder_s decoder;
    uint8_t bytes[DBUS_FRAME_SIZE];
    DbusDecoderInit(&decoder);
    MakeRemote(bytes, 1684U, 1U);
    assert(!PushRemote(&decoder, bytes, 0U)); /* Startup must synchronize. */
    assert(PushRemote(&decoder, bytes, 10U));
    assert(decoder.input.channel[0] == 1.0f && decoder.input.s1 == 1U);
    MakeRemote(bytes, 364U, 2U);
    assert(PushRemote(&decoder, bytes, 20U));
    assert(decoder.input.channel[0] == -1.0f && decoder.input.s1 == 2U);
    MakeRemote(bytes, 363U, 1U);
    assert(!PushRemote(&decoder, bytes, 30U) && !decoder.input.valid);
    MakeRemote(bytes, 1024U, 0U);
    assert(!PushRemote(&decoder, bytes, 40U));
    MakeRemote(bytes, 1024U, 1U);
    bytes[12] = 2U;
    assert(!PushRemote(&decoder, bytes, 50U));
    bytes[12] = 0U;
    for (unsigned i = 0U; i < 8U; ++i)
    {
        assert(!DbusDecoderPush(&decoder, bytes[i], 60U));
    }
    assert(PushRemote(&decoder, bytes, 70U)); /* Truncated frame, then gap. */
    DbusDecoderInvalidate(&decoder);
    assert(!decoder.input.valid);
    assert(!PushRemote(&decoder, bytes, 71U));
    assert(PushRemote(&decoder, bytes, 80U));
    decoder.last_byte_ms = UINT32_MAX - 3U;
    assert(PushRemote(&decoder, bytes, 4U));
}

static void TestYaw(void)
{
    YawRateConfig_s config = GIMBAL_YAW_RATE_CONFIG;
    YawRate_s yaw;
    assert(YawRateInit(&yaw, &config));
    assert(yaw.inhibit == YAW_INHIBIT_REARM_REQUIRED);
    YawRateStep(&yaw, 0U, true, true, true, 0.0f);
    assert(!yaw.enabled); /* Boot with switch on never starts. */
    assert(yaw.inhibit == YAW_INHIBIT_REARM_REQUIRED);
    YawRateStep(&yaw, 2U, true, true, false, 0.0f);
    assert(yaw.inhibit == YAW_INHIBIT_SWITCH_OFF);
    YawRateStep(&yaw, 4U, true, true, true, 0.5f);
    assert(!yaw.enabled); /* ON with nonzero requested rate requires another OFF. */
    assert(yaw.inhibit == YAW_INHIBIT_START_NOT_ZERO);
    YawRateStep(&yaw, 6U, true, true, true, 0.0f);
    assert(!yaw.enabled);
    assert(yaw.inhibit == YAW_INHIBIT_REARM_REQUIRED);
    YawRateStep(&yaw, 8U, true, true, false, 0.0f);
    YawRateStep(&yaw, 10U, true, true, true, 0.0f);
    assert(yaw.enabled);
    assert(yaw.inhibit == 0U);
    YawRateStep(&yaw, 12U, true, true, true, 1.0f);
    assert(fabsf(yaw.reference_rad_s - 0.004f) < 0.000001f);
    YawRateStep(&yaw, 14U, true, true, true, -1.0f);
    assert(fabsf(yaw.reference_rad_s) < 0.000001f);
    YawRateStep(&yaw, 16U, true, true, true, 0.02f);
    assert(yaw.target_rad_s == 0.02f); /* Core no longer applies an RC deadband. */
    YawRateStep(&yaw, 18U, false, true, true, 1.0f);
    assert(!yaw.enabled && yaw.reference_rad_s == 0.0f);
    assert(yaw.inhibit == YAW_INHIBIT_COMMAND);
    YawRateStep(&yaw, 20U, true, true, true, 0.0f);
    assert(!yaw.enabled);
    YawRateStep(&yaw, 22U, true, true, false, 0.0f);
    YawRateStep(&yaw, 24U, true, true, true, 0.0f);
    assert(yaw.enabled);
    YawRateStep(&yaw, 50U, true, true, true, 0.0f);
    assert(!yaw.enabled); /* Missed deadline. */
    assert(yaw.inhibit == YAW_INHIBIT_TIMING);
    YawRateStep(&yaw, 52U, true, true, false, 0.0f);
    YawRateStep(&yaw, 54U, true, true, true, NAN);
    assert(!yaw.enabled);
    assert(yaw.inhibit == YAW_INHIBIT_RATE);
    assert(YawRateInit(&yaw, &config));
    YawRateStep(&yaw, UINT32_MAX - 1U, true, true, false, 0.0f);
    YawRateStep(&yaw, 0U, true, true, true, 0.0f);
    assert(yaw.enabled);
    YawRateStep(&yaw, 2U, true, false, true, 0.0f);
    assert(!yaw.enabled);
    assert(yaw.inhibit == YAW_INHIBIT_ACTUATOR);
    YawRateStep(&yaw, 2U, false, false, true, NAN);
    assert(yaw.inhibit == (YAW_INHIBIT_COMMAND | YAW_INHIBIT_ACTUATOR |
                           YAW_INHIBIT_TIMING | YAW_INHIBIT_RATE));
    config.max_rate_rad_s = 0.0f;
    assert(!YawRateInit(&yaw, &config));
    assert(yaw.inhibit == YAW_INHIBIT_CONFIG);
}

static void Parameter(DmVelocity_s *motor, uint16_t can_id, uint8_t op, uint8_t reg,
                      uint32_t value, uint32_t now)
{
    DmCanFrame_s frame = {.id = can_id, .length = 8U, .received_ms = now};
    frame.data[0] = (uint8_t)motor->config.esc_id;
    frame.data[1] = (uint8_t)(motor->config.esc_id >> 8);
    frame.data[2] = op;
    frame.data[3] = reg;
    for (unsigned i = 0U; i < 4U; ++i)
    {
        frame.data[i + 4U] = (uint8_t)(value >> (8U * i));
    }
    DmVelocityReceive(motor, &frame);
}

static void Feedback(DmVelocity_s *motor, uint8_t state, uint32_t now)
{
    DmCanFrame_s frame = {.id = motor->master_id, .length = 8U, .received_ms = now,
                         .data = {0U, 0x80U, 0U, 0x80U, 0U, 0U, 25U, 25U}};
    frame.data[0] = (state << 4) | (motor->config.esc_id & 0xFU);
    DmVelocityReceive(motor, &frame);
}

static void Setup(DmVelocity_s *motor)
{
    DmVelocityConfig_s config = GIMBAL_DM_CONFIG;
    tx_ok = true;
    tx_count = 0U;
    assert(DmVelocityInit(motor, &config, Send, NULL, 0U));
    DmVelocityStep(motor, 1999U, false, 0.0f);
    assert(tx_count == 0U);
    DmVelocityStep(motor, 2000U, false, 0.0f);
    assert(transmitted.id == 0U && transmitted.data[7] == 0xFDU);
    DmVelocityStep(motor, 2010U, false, 0.0f);
    assert(transmitted.id == 0x7FFU && transmitted.length == 4U && transmitted.data[3] == 7U);
    Parameter(motor, 0x10U, 0x55U, 7U, 0x10U, 2011U);
    assert(motor->state == DM_VELOCITY_DISCOVER);
    Parameter(motor, 0x10U, 0x33U, 7U, 0x10U, 2012U);
    assert(motor->state == DM_VELOCITY_STOPPING);
    DmVelocityStep(motor, 2020U, false, 0.0f);
    Feedback(motor, 0U, 2021U);
    DmVelocityStep(motor, 2030U, false, 0.0f);
    assert(transmitted.data[2] == 0x55U && transmitted.data[3] == 0x0AU && transmitted.data[4] == 3U);
    Parameter(motor, 0x10U, 0x55U, 0x0AU, 3U, 2031U);
    DmVelocityStep(motor, 2040U, false, 0.0f);
    assert(transmitted.data[2] == 0x33U && transmitted.data[3] == 0x0AU);
    Parameter(motor, 0x10U, 0x33U, 0x0AU, 2U, 2041U);
    assert(motor->state == DM_VELOCITY_CHECK_MODE);
    Parameter(motor, 0x10U, 0x33U, 0x0AU, 3U, 2042U);
    assert(motor->state == DM_VELOCITY_READY && DmVelocityAvailable(motor, 2042U));
}

static void Start(DmVelocity_s *motor)
{
    DmVelocityStep(motor, 2050U, true, 0.5f);
    assert(transmitted.id == 0x200U && transmitted.length == 4U);
    assert(memcmp(transmitted.data, "\0\0\0\0", 4U) == 0); /* Clear old target first. */
    DmVelocityStep(motor, 2060U, true, 0.5f);
    assert(transmitted.data[7] == 0xFCU);
    Feedback(motor, 1U, 2061U);
    DmVelocityStep(motor, 2070U, true, 0.5f);
    assert(motor->state == DM_VELOCITY_RUNNING);
    const uint8_t half[4] = {0U, 0U, 0U, 0x3FU};
    assert(memcmp(transmitted.data, half, 4U) == 0);
}

static void TestMotor(void)
{
    DmVelocity_s motor;
    Setup(&motor);
    Start(&motor);
    DmCanFrame_s wrong = {.id = 0x123U, .length = 8U};
    uint32_t count = motor.feedback.count;
    DmVelocityReceive(&motor, &wrong);
    wrong.id = motor.master_id;
    wrong.length = 7U;
    DmVelocityReceive(&motor, &wrong);
    assert(motor.feedback.count == count);
    DmVelocityStep(&motor, 2071U, false, 0.0f);
    assert(transmitted.data[7] == 0xFDU && motor.state == DM_VELOCITY_READY);
    Setup(&motor);
    Start(&motor);
    DmVelocityStep(&motor, 2200U, true, 0.5f);
    assert(motor.state == DM_VELOCITY_FAULT && transmitted.data[7] == 0xFDU);
    Feedback(&motor, 1U, 2201U);
    DmVelocityStep(&motor, 2210U, true, 0.5f);
    assert(motor.state == DM_VELOCITY_FAULT); /* Feedback recovery cannot rearm. */
    Setup(&motor);
    Start(&motor);
    tx_ok = false;
    DmVelocityStep(&motor, 2080U, true, 0.5f);
    assert(motor.fault & DM_VELOCITY_FAULT_TX);
    tx_ok = true;
    DmVelocityStep(&motor, 2090U, true, 0.5f);
    assert(transmitted.data[7] == 0xFDU);
    Setup(&motor);
    Start(&motor);
    Feedback(&motor, 0xBU, 2071U);
    assert(motor.fault & DM_VELOCITY_FAULT_MOTOR);
    Setup(&motor);
    DmVelocityStep(&motor, 2050U, true, INFINITY);
    assert(motor.fault & DM_VELOCITY_FAULT_COMMAND);
    DmVelocityConfig_s config = GIMBAL_DM_CONFIG;
    assert(DmVelocityInit(&motor, &config, Send, NULL, 0U));
    DmVelocityStep(&motor, 2000U, false, 0.0f);
    DmVelocityStep(&motor, 5010U, false, 0.0f);
    assert(motor.fault & DM_VELOCITY_FAULT_TIMEOUT);
    config.esc_id = 0xFFFFU;
    assert(!DmVelocityInit(&motor, &config, Send, NULL, 0U));
    count = tx_count;
    DmVelocityStep(&motor, 9000U, true, 1.0f);
    assert(tx_count == count);
}

int main(void)
{
    TestRemote();
    TestYaw();
    TestMotor();
    puts("gimbal core: DBUS framing, yaw interlocks and DM handshake/faults passed");
    return 0;
}
