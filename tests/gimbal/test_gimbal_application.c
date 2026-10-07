#include "../../application/bench_features.h"
#include "gimbal.h"
#include "gimbal_config.h"
#include "gimbal_port.h"
#include "control_platform.h"
#include "runtime_events.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

static uint32_t tick;
static bool advance_during_rx;
uint32_t ControlPlatformNowMs(void) { return tick; }
uint32_t ControlPlatformEnter(void) { return 0U; }
void ControlPlatformExit(uint32_t state) { (void)state; }
static uint32_t io_faults;
static bool tx_ok = true;
static bool motor_on;
static bool reply_pending;
static DmCanFrame_s reply;
static GimbalRcByte_s rc_bytes[18];
static unsigned rc_index = 18U;
static unsigned enables;
static unsigned disables;
static float last_speed;

bool GimbalPortInitModules(bool use_can, bool use_remote)
{
    assert(use_can == BENCH_ENABLE_YAW && use_remote == BENCH_ENABLE_REMOTE);
    return true;
}

bool GimbalPortSend(void *context, const DmCanFrame_s *frame)
{
    (void)context;
    assert(BENCH_ENABLE_YAW);
    if (!tx_ok)
    {
        return false;
    }
    memset(&reply, 0, sizeof(reply));
    reply.id = 0x10U;
    reply.length = 8U;
    reply.received_ms = tick;
    reply_pending = true;
    if (frame->id == 0x7FFU)
    {
        memcpy(reply.data, frame->data, frame->length);
        reply.data[4] = frame->data[3] == 7U ? 0x10U : 3U;
    }
    else
    {
        if (frame->length == 8U)
        {
            motor_on = frame->data[7] == 0xFCU;
            if (motor_on)
            {
                ++enables;
            }
            else
            {
                ++disables;
            }
        }
        else
        {
            memcpy(&last_speed, frame->data, sizeof(last_speed));
            assert(isfinite(last_speed) && fabsf(last_speed) <= 1.0f);
        }
        reply.data[0] = motor_on ? 0x10U : 0U;
        reply.data[1] = 0x80U;
        reply.data[3] = 0x80U;
        reply.data[6] = 25U;
        reply.data[7] = 25U;
    }
    return true;
}

bool GimbalPortPopCan(DmCanFrame_s *frame)
{
    assert(BENCH_ENABLE_YAW);
    if (!reply_pending)
    {
        return false;
    }
    *frame = reply;
    reply_pending = false;
    return true;
}

bool GimbalPortPopRc(GimbalRcByte_s *byte)
{
    assert(BENCH_ENABLE_REMOTE);
    if (rc_index == 18U)
    {
        return false;
    }
    if (advance_during_rx)
    {
        ++tick; /* Reproduce an IRQ arriving after task entry but during drain. */
        advance_during_rx = false;
        for (unsigned i = rc_index; i < 18U; ++i)
        {
            rc_bytes[i].received_ms = tick;
        }
    }
    *byte = rc_bytes[rc_index++];
    return true;
}

uint32_t GimbalPortTakeFaults(void)
{
    uint32_t result = io_faults;
    io_faults = 0U;
    if (result != 0U)
    {
        reply_pending = false;
        rc_index = 18U;
    }
    return result;
}

static void QueueRemote(uint8_t s1, uint16_t stick)
{
    uint64_t bits = (uint64_t)stick | ((uint64_t)1024U << 11) |
                    ((uint64_t)1024U << 22) | ((uint64_t)1024U << 33) |
                    ((uint64_t)1U << 44) | ((uint64_t)s1 << 46);
    for (unsigned i = 0U; i < 18U; ++i)
    {
        rc_bytes[i].byte = i < 6U ? (uint8_t)(bits >> (8U * i)) : 0U;
        rc_bytes[i].received_ms = tick;
    }
    rc_index = 0U;
}

static GimbalStatus_s RunUntil(uint32_t until, bool rc_online, uint8_t s1, uint16_t stick)
{
    for (; tick <= until; tick += 2U)
    {
        if (rc_online && tick % 10U == 0U)
        {
            QueueRemote(s1, stick);
        }
        GimbalTask();
    }
    GimbalStatus_s status;
    GimbalGetStatus(&status);
    return status;
}

int main(void)
{
    if (!BENCH_ENABLE_YAW || !BENCH_ENABLE_REMOTE)
    {
        assert(GimbalInit(0U));
        GimbalStatus_s isolated = RunUntil(2500U, true, 1U, 1684U);
        assert(!isolated.axis.enabled && enables == 0U);
        assert(isolated.remote.online == BENCH_ENABLE_REMOTE);
        if (!BENCH_ENABLE_YAW)
        {
            assert(disables == 0U && isolated.axis.motor_fault == 0U);
            assert(isolated.axis.port_ready && isolated.axis.timing.samples > 0U);
            io_faults = GIMBAL_PORT_UART_ERROR;
            GimbalTask();
            GimbalGetStatus(&isolated);
            assert(isolated.axis.transport_faults & GIMBAL_PORT_UART_ERROR);
        }
        else
        {
            assert(isolated.axis.motor_state == DM_VELOCITY_READY);
            assert(isolated.axis.inhibit & YAW_INHIBIT_COMMAND);
        }
        puts("PASS: isolated remote/motor ownership; disabled module receives no transport calls");
        return 0;
    }
    assert(GimbalInit(0U));
    GimbalStatus_s status = RunUntil(2300U, true, 1U, 1024U);
    assert(status.remote.online && !status.axis.enabled && enables == 0U);
    assert(status.axis.motor_state == DM_VELOCITY_READY && status.axis.master_id == 0x10U);
    assert(status.axis.inhibit == YAW_INHIBIT_REARM_REQUIRED);
    status = RunUntil(2320U, true, 2U, 1024U);
    assert(!status.axis.enabled);
    status = RunUntil(2380U, true, 1U, 1024U);
    assert(status.axis.enabled && motor_on && status.axis.motor_state == DM_VELOCITY_RUNNING);
    assert(status.axis.inhibit == 0U);
    status = RunUntil(2480U, true, 1U, 1684U);
    assert(status.axis.reference_rad_s > 0.15f && status.axis.reference_rad_s < 0.25f);
    assert(last_speed > 0.0f);
    unsigned previous_disables = disables;
    status = RunUntil(2600U, false, 1U, 1684U);
    assert(!status.remote.online && !status.axis.enabled && !motor_on && disables > previous_disables);
    assert(status.axis.inhibit & YAW_INHIBIT_COMMAND);
    unsigned previous_enables = enables;
    status = RunUntil(2700U, true, 1U, 1024U);
    assert(status.remote.online && !status.axis.enabled && enables == previous_enables);
    (void)RunUntil(2720U, true, 2U, 1024U);
    status = RunUntil(2780U, true, 1U, 1024U);
    assert(status.axis.enabled && motor_on);
    io_faults = GIMBAL_PORT_RX_OVERFLOW;
    status = RunUntil(2782U, false, 1U, 1024U);
    assert(!status.axis.enabled && !motor_on && status.axis.transport_faults == GIMBAL_PORT_RX_OVERFLOW);
    status = RunUntil(2840U, true, 1U, 1024U);
    assert(!status.axis.enabled);
    (void)RunUntil(2860U, true, 2U, 1024U);
    status = RunUntil(2920U, true, 1U, 1024U);
    assert(status.axis.enabled);
    tick += 30U; /* Task stall is detected independently of the RC timeout. */
    status = RunUntil(tick, true, 1U, 1024U);
    assert(!status.axis.enabled && !motor_on);
    assert(status.axis.timing.late_count > 0U && status.axis.timing.max_interval_ms >= 30U);
    assert(status.axis.inhibit & YAW_INHIBIT_TIMING);
    (void)RunUntil(3000U, true, 2U, 1024U);
    status = RunUntil(3060U, true, 1U, 1024U);
    assert(status.axis.enabled);
    /* Fresh RC received across a tick boundary must not appear UINT32_MAX ms old. */
    tick += 2U;
    QueueRemote(1U, 1024U);
    advance_during_rx = true;
    GimbalTask();
    GimbalGetStatus(&status);
    assert(status.axis.enabled && status.remote.online);
    ++tick; /* Return to even periodic ticks after the simulated IRQ. */
    tx_ok = false;
    status = RunUntil(3100U, true, 1U, 1024U);
    assert(!status.axis.enabled && (status.axis.motor_fault & DM_VELOCITY_FAULT_TX));
    assert(status.axis.inhibit & YAW_INHIBIT_ACTUATOR);
    tx_ok = true;
    previous_enables = enables;
    (void)RunUntil(3130U, true, 2U, 1024U);
    status = RunUntil(3200U, true, 1U, 1024U);
    assert(!status.axis.enabled && enables == previous_enables && !motor_on);
    EventHistorySnapshot_s history;
    RuntimeEventsGet(&history);
    bool recorded_tx_fault = false;
    bool recorded_actuator_inhibit = false;
    for (size_t i = 0U; i < history.count; ++i)
    {
        recorded_tx_fault |= history.entries[i].key == RUNTIME_YAW_FAULT &&
                             (history.entries[i].value & DM_VELOCITY_FAULT_TX) != 0U;
        recorded_actuator_inhibit |= history.entries[i].key == RUNTIME_YAW_INHIBIT &&
                                    (history.entries[i].value & YAW_INHIBIT_ACTUATOR) != 0U;
    }
    assert(recorded_tx_fault);
    assert(recorded_actuator_inhibit);
    puts("gimbal application: real control chain with mocked transport passed");
    return 0;
}
