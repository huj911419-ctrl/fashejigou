#include "../../application/bench_features.h"
#include "friction.h"
#include "friction_debug.h"
#include "friction_profile_service.h"
#include "dji_motor.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

FDCAN_HandleTypeDef hfdcan1 = {1}, hfdcan2 = {2}, hfdcan3 = {3};
static uint32_t now_ms;
static CANInstance instances[3];
static unsigned registered;
static uint8_t frames[3][2][8];
static unsigned sent;
static uint8_t bus_off, fifo_full;
static unsigned switch_on_clock_read;
static Friction_Command_Source_e injected_source;
static Friction_Submit_Result_e injected_switch_result;

uint32_t HAL_GetTick(void)
{
    if (switch_on_clock_read != 0U && --switch_on_clock_read == 0U)
    {
        injected_switch_result = FrictionSelectSource(injected_source);
    }
    return now_ms;
}
float DWT_GetDeltaT(uint32_t *last) { *last = now_ms; return 0.001f; }
CANInstance *CANRegister(CAN_Init_Config_s *config)
{
    assert(registered < 3);
    CANInstance *instance = &instances[registered++];
    instance->can_handle = config->can_handle;
    instance->tx_id = config->tx_id;
    instance->rx_id = config->rx_id;
    instance->rx_len = 8;
    instance->can_module_callback = config->can_module_callback;
    instance->id = config->id;
    return instance;
}
HAL_StatusTypeDef CANTransmit(CANInstance *instance, float timeout)
{ (void)instance; (void)timeout; assert(0 && "bench must use nonblocking guarded path"); return HAL_ERROR; }
uint32_t HAL_FDCAN_GetTxFifoFreeLevel(FDCAN_HandleTypeDef *bus)
{ (void)bus; return fifo_full ? 0 : 1; }
HAL_StatusTypeDef HAL_FDCAN_GetProtocolStatus(FDCAN_HandleTypeDef *bus, FDCAN_ProtocolStatusTypeDef *status)
{ (void)bus; status->BusOff = bus_off; return HAL_OK; }
HAL_StatusTypeDef HAL_FDCAN_AddMessageToTxFifoQ(FDCAN_HandleTypeDef *bus, FDCAN_TxHeaderTypeDef *header, uint8_t *data)
{
    assert(header->Identifier == 0x200 || header->Identifier == 0x1ff);
    assert(header->DataLength == 8);
    memcpy(frames[bus->bus - 1][header->Identifier == 0x200], data, 8);
    ++sent;
    return HAL_OK;
}
static int16_t Current(unsigned bus, uint32_t id, unsigned slot)
{
    uint8_t *data = frames[bus - 1][id == 0x200];
    return (int16_t)((uint16_t)data[slot * 2] << 8 | data[slot * 2 + 1]);
}
static void Feed(unsigned i, int16_t rpm, uint8_t temperature)
{
    CANInstance *instance = &instances[i];
    instance->rx_buff[2] = (uint16_t)rpm >> 8;
    instance->rx_buff[3] = rpm & 0xff;
    instance->rx_buff[6] = temperature;
    instance->can_module_callback(instance);
}
static void FeedAll(void) { for (unsigned i = 0; i < 3; ++i) Feed(i, 0, 25); }
static Friction_Status_s Status(void) { Friction_Status_s s; FrictionGetStatus(&s); return s; }
static void Tick(void) { ++now_ms; FrictionTask(); }
static void Rearm(Friction_Command_s *cmd)
{
    const Friction_Command_s stopped = {0};
    FrictionSubmit(&stopped); Tick();
    FeedAll(); FrictionSubmit(cmd); Tick();
    assert(Status().enabled_mask == cmd->enable_mask);
}

static uint8_t debug_rx[256], debug_tx[256];
static size_t debug_length, debug_tx_length;
static int DebugRead(void *context, uint8_t *bytes, size_t capacity)
{
    (void)context;
    size_t n = debug_length < capacity ? debug_length : capacity;
    memcpy(bytes, debug_rx, n);
    memmove(debug_rx, debug_rx + n, debug_length - n);
    debug_length -= n;
    return (int)n;
}
static int DebugWrite(void *context, const uint8_t *bytes, size_t n)
{
    (void)context;
    assert(n <= sizeof(debug_tx)); memcpy(debug_tx, bytes, n); debug_tx_length = n; return 1;
}
static float Channel(unsigned i)
{
    uint32_t bits = (uint32_t)debug_tx[i * 4] | (uint32_t)debug_tx[i * 4 + 1] << 8 |
        (uint32_t)debug_tx[i * 4 + 2] << 16 | (uint32_t)debug_tx[i * 4 + 3] << 24;
    float value; memcpy(&value, &bits, 4); return value;
}
static void Remote(const char *command)
{
    debug_length = strlen(command); assert(debug_length <= sizeof(debug_rx));
    memcpy(debug_rx, command, debug_length);
    FrictionDebugPoll(now_ms);
    for (unsigned i = 0; i < 21; ++i) { FeedAll(); Tick(); FrictionDebugPoll(now_ms); }
}
static void TestRemote(void)
{
    Vofa_Port_s port = {DebugRead, DebugWrite, NULL};
    FrictionDebugInit(&port);
    assert(FrictionDebugOwnsCommands());
    Remote("call 1000 1\n"); /* A newly selected VOFA owner must explicitly stop first. */
    Remote("get 1 10\n");
    assert(debug_tx_length == 180 && Channel(0) == 1 && Channel(1) == VOFA_OK && Channel(3) == 0.5f);
    assert(Channel(42) == FRICTION_PHASE_STOPPED && Channel(43) == 0);
    Remote("set 2 10 0.75\n");
    assert(Channel(1) == VOFA_QUEUED && Channel(9) == 2 && Status().enabled_mask == 0);
    Friction_Telemetry_s t;
    FrictionGetTelemetry(&t); assert(t.tuning.motor[0].kp == 0.75f);
    Remote("call 3 3 1 1.25 0 0 1000\n");
    FrictionGetTelemetry(&t);
    assert(t.tuning.motor[0].kp == 1.25f && t.tuning.motor[0].ki == 0 && t.tuning.motor[0].current_limit == 1000);
    Remote("set 4 13 1600\n"); assert(Channel(1) == VOFA_RANGE && Channel(9) == 3);
    Remote("set 5 10 0.5\nset 6 11 0.1\n");
    assert(Channel(0) == 6 && Channel(1) == VOFA_BUSY && Channel(9) == 5);
    Remote("get 7 11\n"); assert(Channel(3) == 0);
    Remote("call 8 2 1 300 0 0\n");
    assert(Status().enabled_mask == 1 && Channel(8) == 1 && Channel(16) > 0);
    assert(Channel(42) == FRICTION_PHASE_BENCH && Channel(43) == 0);
    Remote("set 9 10 nan\n"); assert(Status().enabled_mask == 0 && Channel(1) == VOFA_SYNTAX);
    Remote("call 10 2 1 300 0 0\n"); assert(Status().enabled_mask == 1);
    /* Tuning/readback traffic is not a motor keepalive. */
    for (unsigned i = 0; i < 13; ++i) Remote("get 11 10\n");
    assert(Status().enabled_mask == 0 && (Status().fault & FRICTION_FAULT_COMMAND));
    Remote("call 12 1\n");
    Remote("call 13 3 1 0.5 0.1 0 1500\n");
    assert(Status().enabled_mask == 0 && Channel(9) == 13);
    /* Stale snapshots cannot overwrite a newer pending/applied transaction. */
    FrictionGetTelemetry(&t);
    assert(FrictionQueueTuning(&t.tuning, 14, t.applied_sequence) == FRICTION_TUNE_QUEUED);
    assert(FrictionQueueTuning(&t.tuning, 15, t.applied_sequence) == FRICTION_TUNE_BUSY);
    Tick();
    assert(FrictionQueueTuning(&t.tuning, 15, t.applied_sequence) == FRICTION_TUNE_BUSY);
    FrictionDebugInit(NULL);
    assert(!FrictionDebugOwnsCommands());
    /* Opt-in continuous PID preserves integral holding torque at exact target;
     * the legacy controller path remains unchanged. */
    PIDInstance pid;
    PID_Init_Config_s cfg = {.Kp=1, .Ki=1, .MaxOut=1000, .IntegralLimit=1000,
        .Improve=PID_Integral_Limit | PID_Continuous};
    PIDInit(&pid, &cfg); pid.Iout = 123;
    assert(PIDCalculate(&pid, 500, 500) == 123);
    cfg.Improve = PID_Integral_Limit;
    PIDInit(&pid, &cfg); pid.Iout = 123;
    assert(PIDCalculate(&pid, 500, 500) == 0);
    puts("PASS: VOFA adapter readback, atomic online PID, current bounds, mailbox busy/stale rejection, wave channels, stop and no implicit keepalive, continuous PID opt-in");
}
static void TestControlEdges(void)
{
    Friction_Control_Config_s cfg = {1000, 300, 250, 50, 5, 70};
    Friction_Control_s c;
    Friction_Command_s cmd = {0};
    Friction_Feedback_s f[3] = {0};
    uint32_t t = UINT32_MAX - 2;
    FrictionControlInit(&c, t);
    FrictionControlStep(&c, &cfg, &cmd, t, f, t + 1, 0);
    cmd.enable_mask = 7;
    cmd.target_rpm[0] = cmd.target_rpm[1] = cmd.target_rpm[2] = 300;
    for (unsigned i = 0; i < 3; ++i) { f[i].valid = 1; f[i].last_rx_ms = t; }
    FrictionControlStep(&c, &cfg, &cmd, t, f, 0, 0);
    assert(c.status.enabled_mask == 7 && fabsf(c.status.reference_rpm[0] - 0.6f) < 0.001f);
    FrictionControlStep(&c, &cfg, &cmd, 0, f, 10, 0);
    assert(c.status.enabled_mask == 0 && c.status.fault & FRICTION_FAULT_TIMING);
    cmd.enable_mask = 0;
    FrictionControlStep(&c, &cfg, &cmd, 10, f, 11, 0);
    cmd.enable_mask = 7; cmd.target_rpm[1] = NAN;
    FrictionControlStep(&c, &cfg, &cmd, 11, f, 12, 0);
    assert(c.status.enabled_mask == 0 && c.status.fault & FRICTION_FAULT_COMMAND);
}

static void TestProfiles(void)
{
    Vofa_Port_s port = {DebugRead, DebugWrite, NULL};
    FrictionDebugInit(&port);
    if (!BENCH_ENABLE_PARAMETERS)
    {
        Remote("call 200 10\n");
        assert(Channel(1) == VOFA_UNAVAILABLE);
        Remote("get 201 109\n");
        assert(Channel(1) == VOFA_UNAVAILABLE);
        Remote("set 202 10 0.5\n");
        assert(Channel(1) == VOFA_QUEUED);
        FrictionDebugInit(NULL);
        return;
    }
    const Friction_Command_s stopped = {0};
    assert(FrictionSubmitFrom(FRICTION_SOURCE_VOFA, &stopped, now_ms) == FRICTION_SUBMIT_ACCEPTED);
    Tick();
    Friction_Telemetry_s before, after;
    FrictionGetTelemetry(&before);
    Friction_Command_s run = {.enable_mask = 1, .target_rpm = {300, 0, 0}};
    assert(FrictionSubmitFrom(FRICTION_SOURCE_VOFA, &run, now_ms) == FRICTION_SUBMIT_ACCEPTED);
    /* Pending RUN, while published status is still stopped. */
    assert(FrictionProfileAdmission() == FRICTION_TUNE_BUSY);
    assert(FrictionQueueProfile(&before.tuning, 200, before.tuning_revision) == FRICTION_TUNE_BUSY);
    FrictionDisable();
    Tick();
    assert(FrictionQueueProfile(&before.tuning, 200, before.tuning_revision + 1U) == FRICTION_TUNE_BUSY);
    assert(FrictionQueueProfile(&before.tuning, 200, before.tuning_revision) == FRICTION_TUNE_QUEUED);
    assert(FrictionSubmitFrom(FRICTION_SOURCE_VOFA, &run, now_ms) == FRICTION_SUBMIT_BUSY);
    /* No start can pass a queued profile before publication. */
    Tick();
    FrictionGetTelemetry(&after);
    assert(after.status.enabled_mask == 0 && after.tuning_revision == before.tuning_revision + 1U);

    FrictionProfile_s profile = {.values = {400, 1, 2, 0, 1200, 1, 2, 0, 1200, 1, 2, 0, 1200}};
    uint32_t crc = FrictionProfileChecksum(&profile);
    char line[120];
    Remote("call 201 10\n");
    assert(Channel(1) == VOFA_OK);
    Remote("get 202 109\n");
    assert(Channel(1) == VOFA_OK && Channel(3) == 0); /* Flash remains unbound. */
    snprintf(line, sizeof(line), "call 203 11 1 1 1 13 %u %u\n", (unsigned)(crc & 0xffffU), (unsigned)(crc >> 16));
    Remote(line);
    assert(Channel(1) == VOFA_OK);
    Remote("set 204 10 5\n");
    assert(Channel(1) == VOFA_BUSY);
    Remote("call 205 2 1 300 0 0\n");
    assert(Channel(1) == VOFA_BUSY && Status().enabled_mask == 0);
    Remote("call 206 13 203\n");
    assert(Channel(1) == VOFA_RANGE); /* Incomplete: no partial application. */
    FrictionGetTelemetry(&before);
    for (unsigned i = 0U; i < FRICTION_PROFILE_FIELDS; ++i)
    {
        snprintf(line, sizeof(line), "call %u 12 203 %u %.9g\n", 210U + i, i, (double)profile.values[i]);
        Remote(line);
        assert(Channel(1) == VOFA_OK);
    }
    FrictionGetTelemetry(&after);
    assert(after.tuning_revision == before.tuning_revision);
    Remote("call 230 13 203\n");
    assert(Channel(1) == VOFA_QUEUED);
    Remote("get 231 108\n");
    assert(Channel(3) == PROFILE_APPLIED);
    FrictionGetTelemetry(&after);
    assert(after.tuning_revision == before.tuning_revision + 1U && after.applied_sequence == 230);
    assert(after.tuning.ramp_rpm_s == 400 && after.tuning.motor[2].ki == 2);
    assert(Status().enabled_mask == 0);
    Remote("get 232 200\n");
    assert(Channel(3) == 300); /* Old export stays frozen across application. */
    Remote("call 233 10\n");
    Remote("get 234 200\n");
    assert(Channel(3) == 400);
    /* Restore original tuning so the existing ramp/readiness scenario is unchanged. */
    assert(FrictionQueueTuning(&before.tuning, 235, after.applied_sequence) == FRICTION_TUNE_QUEUED);
    Tick();
    FrictionDebugInit(NULL);
    puts("PASS: real profile protocol, pending RUN rejection, whole batch application and feature isolation");
}

static void FeedWorkingSpeed(void)
{
    Feed(0, 300, 25);
    Feed(1, -300, 25);
    Feed(2, 300, 25);
}

static void TestReadinessIntegration(void)
{
    Friction_Command_s command = {
        .enable_mask = FRICTION_ALL_MOTORS,
        .target_rpm = {300, 300, 300},
    };
    Rearm(&command);
    Friction_Telemetry_s snapshot;
    for (unsigned i = 0; i < 1350; ++i)
    {
        FeedWorkingSpeed();
        FrictionSubmit(&command);
        Tick();
        FrictionGetTelemetry(&snapshot);
        if (i < 900)
        {
            assert(!snapshot.readiness.all_ready);
        }
    }
    assert(snapshot.readiness.all_ready);
    assert(snapshot.readiness.phase == FRICTION_PHASE_READY);

    assert(FrictionQueueTuning(&snapshot.tuning, 100, snapshot.applied_sequence) == FRICTION_TUNE_QUEUED);
    Tick();
    FrictionGetTelemetry(&snapshot);
    assert(!snapshot.readiness.ready_mask && snapshot.status.enabled_mask == FRICTION_ALL_MOTORS);
    for (unsigned i = 0; i < 210; ++i)
    {
        FeedWorkingSpeed();
        FrictionSubmit(&command);
        Tick();
    }
    FrictionGetTelemetry(&snapshot);
    assert(snapshot.readiness.all_ready);

    for (unsigned i = 0; i < 10; ++i)
    {
        Feed(0, 0, 25);
        Feed(1, -300, 25);
        Feed(2, 300, 25);
        Tick();
    }
    FrictionGetTelemetry(&snapshot);
    assert(!snapshot.readiness.all_ready && snapshot.readiness.phase == FRICTION_PHASE_RECOVERING);

    fifo_full = 1;
    Tick();
    fifo_full = 0;
    FrictionGetTelemetry(&snapshot);
    assert(snapshot.readiness.phase == FRICTION_PHASE_FAULT && !snapshot.readiness.ready_mask);
    FrictionDisable();
    Tick();
    FrictionGetTelemetry(&snapshot);
    assert(snapshot.readiness.phase == FRICTION_PHASE_STOPPED && !snapshot.readiness.all_ready);
    puts("PASS: real adapter readiness waits for ramp/dwell, tuning invalidates readiness, speed droop recovers, driver failure revokes and stop clears");
}

static void TestCommandOwnership(void)
{
    const Friction_Command_s stopped = {0};
    Friction_Command_s run = {.enable_mask = 1U, .target_rpm = {300.0f, 0.0f, 0.0f}};
    Rearm(&run);
    Friction_Telemetry_s snapshot;
    FrictionGetTelemetry(&snapshot);
    assert(snapshot.command_source == FRICTION_SOURCE_RTT && !snapshot.source_requires_off);
    uint32_t epoch = snapshot.source_epoch;
    uint32_t command_age = snapshot.status.command_age_ms;

    assert(FrictionSubmitFrom(FRICTION_SOURCE_VOFA, &run, now_ms) == FRICTION_SUBMIT_WRONG_SOURCE);
    assert(FrictionSubmitFrom(FRICTION_SOURCE_RTT, &run, now_ms - 251U) == FRICTION_SUBMIT_STALE);
    assert(FrictionSubmitFrom(FRICTION_SOURCE_RTT, &run, now_ms + 1U) == FRICTION_SUBMIT_STALE);
    FeedAll();
    Tick();
    assert(Status().enabled_mask == 1U && Status().command_age_ms == command_age + 1U);
    assert(FrictionSubmitFrom(FRICTION_SOURCE_RTT, &run, now_ms - 100U) == FRICTION_SUBMIT_ACCEPTED);
    FeedAll();
    Tick();
    assert(Status().command_age_ms == 101U); /* Admission never rewrites the source timestamp. */

    /* Inject selection after the motor task sampled RUN but before it emits output. */
    injected_source = FRICTION_SOURCE_VOFA;
    switch_on_clock_read = 1U;
    FeedAll();
    Tick();
    assert(injected_switch_result == FRICTION_SUBMIT_ACCEPTED && Status().enabled_mask == 0U);
    assert(Current(3, 0x200, 0) == 0);
    FrictionGetTelemetry(&snapshot);
    assert(snapshot.command_source == FRICTION_SOURCE_VOFA && snapshot.source_requires_off);
    assert(snapshot.source_epoch == epoch + 1U);
    FrictionSubmit(&run); /* Compatibility API cannot bypass the selected owner. */
    Tick();
    assert(Status().enabled_mask == 0U && (Status().fault & FRICTION_FAULT_COMMAND));
    assert(FrictionSubmitFrom(FRICTION_SOURCE_VOFA, &run, now_ms) == FRICTION_SUBMIT_REARM_REQUIRED);
    FrictionDisable();
    Tick();
    assert(FrictionSubmitFrom(FRICTION_SOURCE_RTT, &stopped, now_ms) == FRICTION_SUBMIT_ACCEPTED);
    Tick();
    assert(FrictionSubmitFrom(FRICTION_SOURCE_VOFA, &stopped, now_ms - 251U) == FRICTION_SUBMIT_STALE);
    Tick();
    assert(Status().fault & FRICTION_FAULT_COMMAND);
    assert(FrictionSubmitFrom(FRICTION_SOURCE_VOFA, &run, now_ms) == FRICTION_SUBMIT_REARM_REQUIRED);

    assert(FrictionSubmitFrom(FRICTION_SOURCE_VOFA, &stopped, now_ms) == FRICTION_SUBMIT_ACCEPTED);
    assert(FrictionSubmitFrom(FRICTION_SOURCE_VOFA, &run, now_ms) == FRICTION_SUBMIT_ACCEPTED);
    FeedAll();
    Tick();
    assert(Status().enabled_mask == 0U); /* Same-batch RUN cannot overwrite OFF. */
    assert(FrictionSubmitFrom(FRICTION_SOURCE_VOFA, &run, now_ms) == FRICTION_SUBMIT_ACCEPTED);
    FeedAll();
    Tick();
    assert(Status().enabled_mask == 1U);
    assert(FrictionSelectSource(FRICTION_SOURCE_VOFA) == FRICTION_SUBMIT_ACCEPTED);
    FeedAll();
    Tick();
    FrictionGetTelemetry(&snapshot);
    assert(snapshot.source_epoch == epoch + 1U && Status().enabled_mask == 1U);

    /* Traffic from the unselected source is never a keepalive or automatic fallback. */
    for (unsigned i = 0U; i < 252U; ++i)
    {
        assert(FrictionSubmitFrom(FRICTION_SOURCE_RTT, &run, now_ms) == FRICTION_SUBMIT_WRONG_SOURCE);
        FeedAll();
        Tick();
    }
    assert(FrictionGetSource() == FRICTION_SOURCE_VOFA);
    assert(Status().enabled_mask == 0U && (Status().fault & FRICTION_FAULT_COMMAND));
    assert(FrictionSubmitFrom(FRICTION_SOURCE_VOFA, &run, now_ms) == FRICTION_SUBMIT_ACCEPTED);
    Tick();
    assert(Status().enabled_mask == 0U); /* Fresh RUN alone cannot clear actuator recovery. */

    assert(FrictionSubmitFrom(FRICTION_SOURCE_VOFA, &stopped, now_ms) == FRICTION_SUBMIT_ACCEPTED);
    Tick();
    FrictionGetTelemetry(&snapshot);
    assert(FrictionQueueProfile(&snapshot.tuning, 2000U, snapshot.tuning_revision) == FRICTION_TUNE_QUEUED);
    assert(FrictionSelectSource(FRICTION_SOURCE_RTT) == FRICTION_SUBMIT_BUSY);
    assert(FrictionGetSource() == FRICTION_SOURCE_VOFA);
    assert(FrictionSelectSource(FRICTION_SOURCE_VOFA) == FRICTION_SUBMIT_ACCEPTED);
    assert(FrictionSubmitFrom(FRICTION_SOURCE_VOFA, &run, now_ms) == FRICTION_SUBMIT_BUSY);
    Tick();
    assert(FrictionSelectSource(FRICTION_SOURCE_NONE) == FRICTION_SUBMIT_ACCEPTED);
    Tick();
    assert(FrictionSubmitFrom(FRICTION_SOURCE_VOFA, &run, now_ms) == FRICTION_SUBMIT_WRONG_SOURCE);
    assert(FrictionSubmitFrom(FRICTION_SOURCE_NONE, &run, now_ms) == FRICTION_SUBMIT_WRONG_SOURCE);
    assert(FrictionSelectSource((Friction_Command_Source_e)99) == FRICTION_SUBMIT_UNAVAILABLE);
    assert(FrictionGetSource() == FRICTION_SOURCE_NONE && Status().enabled_mask == 0U);
    assert(FrictionSelectSource(FRICTION_SOURCE_RTT) == FRICTION_SUBMIT_ACCEPTED);
    FrictionSubmit(&stopped);
    Tick();
    puts("PASS: explicit source ownership, original leases, handoff during calculation, owner OFF rearm, sticky stop and profile exclusion");
}

static void TestWithoutRtt(void)
{
    assert(FrictionInit());
    assert(FrictionGetSource() == (BENCH_FRICTION_REMOTE_ACTIVE ? FRICTION_SOURCE_REMOTE :
                                  (BENCH_FRICTION_ACTIVE && BENCH_ENABLE_USB ?
                                   FRICTION_SOURCE_USB : FRICTION_SOURCE_NONE)));
    Friction_Command_s run = {.enable_mask = BENCH_FRICTION_MASK, .target_rpm = {300.0f, 300.0f, 300.0f}};
    const Friction_Command_s stopped = {0};
    assert(FrictionSelectSource(FRICTION_SOURCE_RTT) == FRICTION_SUBMIT_UNAVAILABLE);
    FrictionDisable();
    Tick();
    assert(FrictionSubmitFrom(FRICTION_SOURCE_RTT, &run, now_ms) == FRICTION_SUBMIT_UNAVAILABLE);
    assert(FrictionSubmitFrom((Friction_Command_Source_e)99, &run, now_ms) == FRICTION_SUBMIT_UNAVAILABLE);
    assert(FrictionGetSource() == FRICTION_SOURCE_NONE);
    FrictionSubmit(&run);
    Tick();
    assert(Status().enabled_mask == 0U);
    Vofa_Port_s port = {DebugRead, DebugWrite, NULL};
    FrictionDebugInit(&port);
    assert(FrictionGetSource() == FRICTION_SOURCE_NONE); /* Transport init has no authority side effect. */
    if (BENCH_ENABLE_VOFA && BENCH_FRICTION_ACTIVE)
    {
        assert(FrictionSelectSource(FRICTION_SOURCE_VOFA) == FRICTION_SUBMIT_ACCEPTED);
        assert(FrictionGetSource() == FRICTION_SOURCE_VOFA);
        assert(FrictionSubmitFrom(FRICTION_SOURCE_VOFA, &stopped, now_ms) == FRICTION_SUBMIT_ACCEPTED);
        Tick();
        for (unsigned i = 0U; i < registered; ++i)
        {
            Feed(i, 0, 25);
        }
        assert(FrictionSubmitFrom(FRICTION_SOURCE_VOFA, &run, now_ms) == FRICTION_SUBMIT_ACCEPTED);
        Tick();
        assert(Status().enabled_mask == BENCH_FRICTION_MASK);
    }
    FrictionDebugInit(NULL);
    assert(FrictionGetSource() == (BENCH_ENABLE_VOFA && BENCH_FRICTION_ACTIVE ?
                                  FRICTION_SOURCE_VOFA : FRICTION_SOURCE_NONE));
    FrictionDisable();
    Tick();
    assert(FrictionGetSource() == FRICTION_SOURCE_NONE && Status().enabled_mask == 0U);
    puts("PASS: RTT-disabled build cannot fall back to an unavailable command source");
}

int main(void)
{
    TestControlEdges();
#ifdef TEST_UNCONFIGURED
    assert(!FrictionInit());
    assert(registered == 0);
    Tick();
    assert(Status().fault & FRICTION_FAULT_CONFIG);
    assert(sent == 0);
    puts("PASS: explicitly unconfigured IDs cannot register or drive motors");
    return 0;
#endif
    if (!BENCH_ENABLE_RTT_COMMAND)
    {
        TestWithoutRtt();
        return 0;
    }
    if (BENCH_FRICTION_MASK != 7U || !BENCH_ENABLE_VOFA || !BENCH_ENABLE_SPEED_READY)
    {
        assert(FrictionInit() && FrictionInit());
        unsigned expected = BENCH_FRICTION_ACTIVE * (BENCH_ENABLE_F1 + BENCH_ENABLE_F2 + BENCH_ENABLE_F3);
        assert(registered == expected);
        Friction_Command_s command = {.enable_mask = BENCH_FRICTION_MASK, .target_rpm = {300, 300, 300}};
        FrictionDisable();
        Tick();
        for (unsigned step = 0U; step < 20U; ++step)
        {
            for (unsigned i = 0U; i < registered; ++i)
            {
                Feed(i, 0, 25);
            }
            FrictionSubmit(&command);
            Tick();
        }
        assert(Status().enabled_mask == BENCH_FRICTION_MASK);
        Friction_Telemetry_s snapshot;
        FrictionGetTelemetry(&snapshot);
        if (BENCH_FRICTION_ACTIVE)
        {
            assert(sent > 0U);
            const int ids[3] = {1, 3, 8};
            const int buses[3] = {3, 3, 2};
            unsigned compact = 0U;
            for (unsigned i = 0U; i < 3U; ++i)
            {
                if (BENCH_FRICTION_MASK & (1U << i))
                {
                    assert(instances[compact].rx_id == (unsigned)(0x200 + ids[i]));
                    assert(instances[compact].can_handle->bus == buses[i]);
                    assert(snapshot.output[i] > 0.0f);
                    snapshot.tuning.motor[i].kp = (float)(i + 1U);
                    ++compact;
                }
                else
                {
                    assert(!snapshot.status.feedback[i].valid && snapshot.output[i] == 0.0f);
                }
            }
            assert(FrictionQueueTuning(&snapshot.tuning, 1U, 0U) == FRICTION_TUNE_QUEUED);
            Tick();
            compact = 0U;
            for (unsigned i = 0U; i < 3U; ++i)
            {
                if (BENCH_FRICTION_MASK & (1U << i))
                {
                    DJIMotorInstance *motor = instances[compact++].id;
                    assert(motor->motor_controller.speed_PID.Kp == (float)(i + 1U));
                    assert(motor->motor_controller.speed_PID.Improve & PID_ConditionalIntegration);
                }
            }
            if (BENCH_FRICTION_MASK != 7U)
            {
                command.enable_mask = 7U;
                FrictionSubmit(&command);
                Tick();
                assert(Status().enabled_mask == 0U && (Status().fault & FRICTION_FAULT_COMMAND));
            }
        }
        else
        {
            assert(sent == 0U && Status().fault == 0U && snapshot.sample_id == 0U);
            assert(FrictionQueueTuning(&snapshot.tuning, 1U, 0U) == FRICTION_TUNE_UNAVAILABLE);
        }
        if (!BENCH_ENABLE_VOFA)
        {
            Vofa_Port_s port = {DebugRead, DebugWrite, NULL};
            FrictionDebugInit(&port);
            debug_length = 4U;
            FrictionDebugPoll(now_ms);
            assert(!FrictionDebugOwnsCommands() && debug_length == 4U && debug_tx_length == 0U);
        }
        if (!BENCH_SPEED_READY_ACTIVE)
        {
            assert(!snapshot.readiness.all_ready && snapshot.readiness.ready_mask == 0U);
        }
        puts("PASS: selected registration, fixed wheel numbering/tuning, excluded command rejection and disabled transport");
        return 0;
    }
#ifndef TEST_UNCONFIGURED
    assert(FrictionInit());
    assert(registered == 3);
    for (unsigned i = 0U; i < registered; ++i)
    {
        const DJIMotorInstance *motor = instances[i].id;
        assert(motor->motor_controller.speed_PID.Improve & PID_ConditionalIntegration);
    }
    assert(instances[0].rx_id == 0x201 && instances[1].rx_id == 0x203 && instances[2].rx_id == 0x208);
    /* Clean startup prepares OFF but still rejects a non-owner RTT RUN. */
    for (unsigned i = 0U; i < 20U; ++i)
    {
        FeedAll();
        Tick();
    }
    Friction_Command_s first_run = {.enable_mask = 7U, .target_rpm = {300.0f, 300.0f, 300.0f}};
    FrictionSubmit(&first_run);
    Tick();
    assert(Status().enabled_mask == 0U);
    assert(FrictionGetSource() == (BENCH_FRICTION_REMOTE_ACTIVE ? FRICTION_SOURCE_REMOTE :
                                  FRICTION_SOURCE_USB) && Status().fault == 0U);
    assert(Current(3, 0x200, 0) == 0 && Current(2, 0x1FF, 3) == 0);
    FrictionDisable(); Tick();
    TestRemote();
    TestProfiles();
    TestReadinessIntegration();
    TestCommandOwnership();
    for (unsigned i = 0; i < 100; ++i)
    {
        FeedAll(); /* Settle the real driver's speed filter for the next scenario. */
        Tick();
    }
    now_ms += 51; Tick(); /* Let remote-test feedback expire before offline test. */
    Friction_Command_s cmd = {.enable_mask=7, .target_rpm={300, 300, 300}};
    FrictionSubmit(&cmd); Tick();
    assert(Status().fault & FRICTION_FAULT_OFFLINE);
    FeedAll(); FrictionSubmit(&cmd); Tick();
    assert(Status().enabled_mask == 0); /* feedback recovery must not restart */
    Rearm(&cmd);
    Tick(); Tick();
    assert(sent > 0 && Current(3, 0x200, 0) > 0 && Current(3, 0x200, 2) < 0 && Current(2, 0x1ff, 3) > 0);
    assert(Current(3, 0x200, 1) == 0 && Current(3, 0x200, 3) == 0);
    for (unsigned i = 0; i < 60; ++i) { Feed(0, 0, 25); Feed(1, 0, 25); FrictionSubmit(&cmd); Tick(); }
    assert(Status().enabled_mask == 0 && Status().fault & FRICTION_FAULT_OFFLINE);
    assert(Current(3, 0x200, 0) == 0 && Current(2, 0x1ff, 3) == 0);
    Rearm(&cmd);
    for (unsigned i = 0; i < 252; ++i) { FeedAll(); Tick(); }
    assert(Status().fault & FRICTION_FAULT_COMMAND);
    Rearm(&cmd);
    Feed(1, 0, 75); Tick();
    assert(Status().fault & FRICTION_FAULT_TEMPERATURE);
    Rearm(&cmd);
    FrictionDisable(); FrictionSubmit(&cmd); Tick(); Tick();
    assert(Status().enabled_mask == 0); /* cannot overwrite a pending stop */
    FrictionSubmit(&cmd); Tick();
    assert(Status().enabled_mask == 7);
    fifo_full = 1; Tick(); fifo_full = 0; FeedAll(); Tick();
    assert(Status().enabled_mask == 0 && Status().fault & FRICTION_FAULT_DRIVER);
    assert(Current(3, 0x200, 0) == 0);
    Rearm(&cmd);
    bus_off = 1; Tick();
    assert(Status().enabled_mask == 0 && Status().fault & FRICTION_FAULT_DRIVER);
    assert(Current(3, 0x200, 0) == 0);
    bus_off = 0;
    Rearm(&cmd);
    DJIMotorInstance *motor = instances[0].id;
    motor->motor_controller.speed_PID.Iout = 1000;
    FrictionDisable(); Tick();
    assert(motor->motor_controller.speed_PID.Iout == 0 && Current(3, 0x200, 0) == 0);
    /* Single-wheel mode does not require the other wheels online. */
    cmd.enable_mask = 1;
    for (unsigned i = 0; i < 60; ++i) { Feed(0, 0, 25); FrictionSubmit(&cmd); Tick(); }
    assert(Status().enabled_mask == 1 && Status().online_mask == 1);
    assert(Current(3, 0x200, 2) == 0 && Current(2, 0x1ff, 3) == 0);
    DJIMotorSetRef(motor, INFINITY);
    assert(!DJIMotorControlGuarded());
    assert(Current(3, 0x200, 0) == 0);
    motor->motor_controller.speed_PID.Kp = 1000000;
    DJIMotorSetRef(motor, 1e6f);
    assert(DJIMotorControlGuarded());
    assert(Current(3, 0x200, 0) <= 1500);
    /* Truncated frames must not refresh the feedback timestamp. */
    uint32_t old_rx = motor->last_rx_ms;
    ++now_ms; instances[0].rx_len = 3; Feed(0, 0, 25);
    assert(motor->last_rx_ms == old_rx);
    assert(Status().timing.samples > 0U && Status().timing.max_interval_ms > 0U);
    puts("PASS: three motor packing/direction, ramp, stale feedback/commands, rearm, stop, temperature, bus failures, PI reset, single motor, finite/current guards, tick wrap");
    return 0;
#endif
}
