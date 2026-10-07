/* Deterministic fault/input stress. Exercises production sources, no plant
 * model and no claim about physical stability or actual hardware timing. */
#include "yaw_rate.h"
#include "friction_control.h"
#include "dbus_input.h"
#include "vofa.h"
#include <assert.h>
#include <float.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static uint32_t random_state;
static uint8_t incoming[VOFA_RX_BUDGET];
static size_t incoming_size;
static bool lose_rx;
static unsigned calls;
static unsigned faults;

static uint32_t Random(void)
{
    random_state ^= random_state << 13;
    random_state ^= random_state >> 17;
    random_state ^= random_state << 5;
    return random_state;
}

static void TestNumericLimits(void)
{
    YawRateConfig_s config = {FLT_MAX, FLT_MAX, 100U};
    YawRate_s yaw;
    assert(YawRateInit(&yaw, &config));
    YawRateStep(&yaw, 0U, true, true, false, 0.0f);
    YawRateStep(&yaw, 1U, true, true, true, 0.0f);
    YawRateStep(&yaw, 101U, true, true, true, FLT_MAX);
    assert(isfinite(yaw.reference_rad_s));
    YawRateStep(&yaw, 201U, true, true, true, -FLT_MAX);
    assert(isfinite(yaw.reference_rad_s));
}

static void TestControl(void)
{
    const YawRateConfig_s yaw_config = {1.0f, 2.0f, 10U};
    const Friction_Control_Config_s wheel_config = {1000.0f, 300.0f, 250U, 50U, 5U, 70U};
    YawRate_s yaw;
    Friction_Control_s wheel;
    uint32_t now = UINT32_MAX - 100U;
    assert(YawRateInit(&yaw, &yaw_config));
    FrictionControlInit(&wheel, now);
    unsigned enabled_samples = 0U;
    unsigned wheel_enabled_samples = 0U;
    for (unsigned step = 0U; step < 100000U; ++step)
    {
        uint32_t bits = Random();
        uint32_t dt = bits % 20U == 0U ? bits % 32U : 2U;
        now += dt;
        bool valid = bits % 29U != 0U;
        bool ready = bits % 31U != 0U;
        bool arm = step % 64U != 0U;
        float stick = step % 64U < 2U ? 0.0f : ((float)(Random() % 2001U) - 1000.0f) / 1000.0f;
        if (bits % 97U == 0U)
        {
            stick = NAN;
        }
        float previous = yaw.reference_rad_s;
        bool previously_enabled = yaw.enabled;
        YawRateStep(&yaw, now, valid, ready, arm, stick);
        assert(isfinite(yaw.reference_rad_s) && fabsf(yaw.reference_rad_s) <= 1.00001f);
        if (!valid || !ready || !arm || !isfinite(stick))
        {
            assert(!yaw.enabled && yaw.reference_rad_s == 0.0f);
        }
        if (yaw.enabled && previously_enabled)
        {
            assert(fabsf(yaw.reference_rad_s - previous) <= 2.0f * (float)dt * 0.001f + 0.00001f);
        }
        enabled_samples += yaw.enabled;

        Friction_Command_s command = {.enable_mask = (uint8_t)(Random() % 16U)};
        Friction_Feedback_s feedback[FRICTION_MOTOR_COUNT] = {0};
        for (unsigned i = 0U; i < FRICTION_MOTOR_COUNT; ++i)
        {
            command.target_rpm[i] = (float)(Random() % 2401U) - 1200.0f;
            feedback[i].valid = (uint8_t)(Random() & 1U);
            feedback[i].rpm = (float)(Random() % 2401U) - 1200.0f;
            feedback[i].temperature_c = (uint8_t)(Random() % 100U);
            feedback[i].last_rx_ms = now - Random() % 100U;
        }
        uint32_t age = Random() % 400U;
        /* Alternate healthy running windows with faults; random invalid data
         * alone would let an always-disabled implementation pass this test. */
        if (step % 64U < 32U)
        {
            command.enable_mask = step % 64U == 0U ? 0U : FRICTION_ALL_MOTORS;
            age = 0U;
            for (unsigned i = 0U; i < FRICTION_MOTOR_COUNT; ++i)
            {
                command.target_rpm[i] = 800.0f;
                feedback[i].valid = 1U;
                feedback[i].temperature_c = 25U;
                feedback[i].last_rx_ms = now;
            }
        }
        else if (bits % 7U == 0U)
        {
            command.target_rpm[0] = NAN;
        }
        FrictionControlStep(&wheel, &wheel_config, &command, now - age, feedback, now, 0U);
        wheel_enabled_samples += wheel.status.enabled_mask != 0U;
        assert((wheel.status.enabled_mask & ~FRICTION_ALL_MOTORS) == 0U);
        if (wheel.status.fault != 0U || age > 250U || dt == 0U || dt > 5U)
        {
            assert(wheel.status.enabled_mask == 0U);
        }
        for (unsigned i = 0U; i < FRICTION_MOTOR_COUNT; ++i)
        {
            assert(isfinite(wheel.status.reference_rpm[i]));
            assert(fabsf(wheel.status.reference_rpm[i]) <= 1000.001f);
            if (!(wheel.status.enabled_mask & (1U << i)))
            {
                assert(wheel.status.reference_rpm[i] == 0.0f);
            }
        }
    }
    assert(enabled_samples > 100U); /* Do not accept a vacuous always-off test. */
    assert(wheel_enabled_samples > 100U);
    printf("Active samples: yaw=%u, friction=%u\n", enabled_samples, wheel_enabled_samples);
}

static int Read(void *context, uint8_t *data, size_t capacity)
{
    (void)context;
    if (lose_rx)
    {
        lose_rx = false;
        incoming_size = 0U;
        return -1;
    }
    assert(incoming_size <= capacity);
    memcpy(data, incoming, incoming_size);
    int size = (int)incoming_size;
    incoming_size = 0U;
    return size;
}

static int Write(void *context, const uint8_t *data, size_t length)
{
    (void)context;
    assert(length >= 20U && length <= (VOFA_MAX_CHANNELS + 1U) * 4U);
    const uint8_t tail[4] = {0U, 0U, 0x80U, 0x7FU};
    assert(memcmp(data + length - 4U, tail, 4U) == 0);
    return (Random() & 1U) != 0U;
}

static Vofa_Result_e Get(void *context, uint16_t id, float *value)
{
    (void)context;
    *value = (float)id;
    ++calls;
    return VOFA_OK;
}

static Vofa_Result_e Set(void *context, uint16_t sequence, uint16_t id, float value)
{
    (void)context;
    (void)id;
    assert(sequence > 0U && isfinite(value));
    ++calls;
    return VOFA_QUEUED;
}

static Vofa_Result_e Call(void *context, uint16_t sequence, uint16_t id, const float *args, size_t count)
{
    (void)context;
    (void)id;
    assert(sequence > 0U && count <= VOFA_MAX_ARGS);
    for (size_t i = 0U; i < count; ++i)
    {
        assert(isfinite(args[i]));
    }
    ++calls;
    return VOFA_OK;
}

static void RxFault(void *context)
{
    (void)context;
    ++faults;
}

static void TestProtocols(void)
{
    struct
    {
        uint64_t before;
        Vofa_s vofa;
        uint64_t after;
    } guarded = {.before = UINT64_C(0xADDC0FFE12345678), .after = UINT64_C(0x56781234DEADC0DE)};
    const Vofa_Port_s port = {Read, Write, NULL};
    const Vofa_Handler_s handler = {Get, Set, Call, RxFault, NULL};
    VofaInit(&guarded.vofa, &port, &handler);
    uint32_t now = UINT32_MAX - 100U;
    for (unsigned step = 0U; step < 10000U; ++step)
    {
        incoming_size = Random() % (sizeof(incoming) + 1U);
        for (size_t i = 0U; i < incoming_size; ++i)
        {
            incoming[i] = (uint8_t)Random();
        }
        if (step % 8U == 0U)
        {
            const char command[] = "\nget 1 2\nset 2 3 1.5\ncall 3 4 -2.25 8e2\n";
            incoming_size = sizeof(command) - 1U;
            memcpy(incoming, command, incoming_size);
        }
        lose_rx = step % 17U == 0U;
        now += Random() % 150U;
        VofaPoll(&guarded.vofa, now);
        assert(guarded.vofa.length < VOFA_LINE_SIZE);
        assert(guarded.before == UINT64_C(0xADDC0FFE12345678));
        assert(guarded.after == UINT64_C(0x56781234DEADC0DE));
        float channels[VOFA_MAX_CHANNELS] = {0};
        (void)VofaSend(&guarded.vofa, channels, Random() % (VOFA_MAX_CHANNELS + 1U));
    }
    assert(calls > 100U && faults > 100U);

    DbusDecoder_s dbus;
    DbusDecoderInit(&dbus);
    for (unsigned byte = 0U; byte < 500000U; ++byte)
    {
        now += Random() % 4U;
        if (DbusDecoderPush(&dbus, (uint8_t)Random(), now))
        {
            for (unsigned i = 0U; i < DBUS_CHANNEL_COUNT; ++i)
            {
                assert(isfinite(dbus.input.channel[i]) && fabsf(dbus.input.channel[i]) <= 1.0f);
            }
            assert(dbus.input.s1 >= 1U && dbus.input.s1 <= 3U);
        }
        assert(dbus.count < DBUS_FRAME_SIZE);
    }
}

int main(int argc, char **argv)
{
    random_state = argc > 1 ? (uint32_t)strtoul(argv[1], NULL, 10) : 1U;
    assert(random_state != 0U);
    printf("Reproducible stress seed=%lu\n", (unsigned long)random_state);
    fflush(stdout);
    TestNumericLimits();
    TestControl();
    TestProtocols();
    puts("PASS: 100000 control steps, 10000 VOFA polls, 500000 DBUS bytes; software invariants only");
    return 0;
}
