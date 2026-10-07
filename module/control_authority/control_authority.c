#include "control_authority.h"

#include <stddef.h>
#include <string.h>

#define SERIAL_HALF_RANGE UINT32_C(0x80000000)

static bool IsProducer(ControlSource_t source)
{
    return source == CONTROL_SOURCE_REMOTE || source == CONTROL_SOURCE_DEBUG ||
           source == CONTROL_SOURCE_AUTO;
}

static bool IsLater(uint32_t next_ms, uint32_t previous_ms)
{
    uint32_t elapsed = next_ms - previous_ms;
    return elapsed != 0U && elapsed < SERIAL_HALF_RANGE;
}

static void Revoke(ControlAuthority_s *authority)
{
    authority->status.authorized = false;
    authority->off_seen = false;
}

bool ControlAuthorityInit(ControlAuthority_s *authority,
                          const ControlAuthorityConfig_s *config)
{
    if (authority == NULL)
    {
        return false;
    }
    memset(authority, 0, sizeof(*authority));
    authority->status.reason = CONTROL_AUTHORITY_CONFIG;
    if (config == NULL || (config->allowed_sources & ~CONTROL_SOURCE_ALL) != 0U)
    {
        return false;
    }
    for (unsigned int index = 0U; index < CONTROL_SOURCE_COUNT; ++index)
    {
        if ((config->allowed_sources & CONTROL_SOURCE_MASK(index + 1U)) != 0U &&
            (config->timeout_ms[index] == 0U ||
             config->timeout_ms[index] >= SERIAL_HALF_RANGE))
        {
            return false;
        }
    }
    authority->config = *config;
    authority->configured = true;
    authority->status.reason = CONTROL_AUTHORITY_NO_OWNER;
    return true;
}

bool ControlAuthoritySelect(ControlAuthority_s *authority, ControlSource_t source)
{
    if (authority == NULL || !authority->configured ||
        (source != CONTROL_SOURCE_NONE &&
         (!IsProducer(source) ||
          (authority->config.allowed_sources & CONTROL_SOURCE_MASK(source)) == 0U)))
    {
        return false;
    }
    if (source == authority->status.selected)
    {
        return true;
    }

    Revoke(authority);
    memset(&authority->input, 0, sizeof(authority->input));
    authority->have_input = false;
    authority->input_pending = false;
    authority->status.selected = source;
    authority->status.epoch++;
    authority->status.input_sequence = 0U;
    authority->status.online = false;
    authority->status.reason = source == CONTROL_SOURCE_NONE ?
                               CONTROL_AUTHORITY_NO_OWNER : CONTROL_AUTHORITY_NO_INPUT;
    if (authority->stop_pending)
    {
        authority->status.reason |= CONTROL_AUTHORITY_STOP;
    }
    return true;
}

bool ControlAuthoritySubmit(ControlAuthority_s *authority,
                            const ControlAuthorityInput_s *input)
{
    if (authority == NULL || input == NULL || !authority->configured ||
        !IsProducer(input->source) || input->source != authority->status.selected ||
        input->epoch != authority->status.epoch)
    {
        return false;
    }
    if (input->stop_requested)
    {
        ControlAuthorityStop(authority);
    }
    if (authority->have_input && !IsLater(input->received_ms, authority->input.received_ms))
    {
        return false;
    }
    if (!input->valid || !input->arm)
    {
        /* Preserve invalid input and a real OFF even if a later RUN replaces
         * the payload before Step. Duplicates do not synthesize STOP. */
        ControlAuthorityStop(authority);
    }
    authority->input = *input;
    authority->have_input = true;
    authority->input_pending = true;
    authority->status.input_sequence = input->sequence;
    return true;
}

void ControlAuthorityStop(ControlAuthority_s *authority)
{
    if (authority == NULL)
    {
        return;
    }
    Revoke(authority);
    authority->stop_pending = true;
    authority->status.reason |= CONTROL_AUTHORITY_STOP;
}

void ControlAuthorityStep(ControlAuthority_s *authority, uint32_t now_ms)
{
    if (authority == NULL)
    {
        return;
    }
    bool new_input = authority->input_pending;
    bool stopped = authority->stop_pending;
    authority->input_pending = false;
    authority->stop_pending = false;
    authority->status.online = false;
    uint32_t reason = stopped ? CONTROL_AUTHORITY_STOP : 0U;

    if (!authority->configured)
    {
        reason |= CONTROL_AUTHORITY_CONFIG;
    }
    else if (authority->status.selected == CONTROL_SOURCE_NONE)
    {
        reason |= CONTROL_AUTHORITY_NO_OWNER;
    }
    else if (!authority->have_input)
    {
        reason |= CONTROL_AUTHORITY_NO_INPUT;
    }
    else
    {
        uint32_t age_ms = now_ms - authority->input.received_ms;
        uint32_t timeout_ms = authority->config.timeout_ms[authority->status.selected - 1];
        if (!authority->input.valid)
        {
            reason |= CONTROL_AUTHORITY_INVALID_INPUT;
        }
        if (age_ms >= SERIAL_HALF_RANGE)
        {
            reason |= CONTROL_AUTHORITY_FUTURE_INPUT;
            /* Do not let a bad future timestamp become the lower bound for
             * subsequent input. Recovery still needs a new OFF -> ON. */
            authority->have_input = false;
        }
        else if (age_ms >= timeout_ms)
        {
            reason |= CONTROL_AUTHORITY_TIMEOUT;
        }
        authority->status.online = (reason & ~CONTROL_AUTHORITY_STOP) == 0U;
    }

    if (!authority->status.online)
    {
        Revoke(authority);
        authority->status.reason = reason;
        return;
    }
    if (stopped)
    {
        Revoke(authority);
    }
    if (!authority->input.arm)
    {
        authority->status.authorized = false;
        if (new_input)
        {
            authority->off_seen = true;
            authority->off_received_ms = authority->input.received_ms;
        }
        authority->status.reason = reason | CONTROL_AUTHORITY_SWITCH_OFF;
        return;
    }
    if (stopped)
    {
        authority->status.reason = reason | CONTROL_AUTHORITY_REARM_REQUIRED;
        return;
    }
    if (!authority->status.authorized)
    {
        bool edge = new_input && authority->off_seen &&
                    IsLater(authority->input.received_ms, authority->off_received_ms);
        if (!edge)
        {
            reason |= CONTROL_AUTHORITY_REARM_REQUIRED;
        }
        else if (!authority->input.neutral)
        {
            reason |= CONTROL_AUTHORITY_NOT_NEUTRAL;
        }
        else
        {
            authority->status.authorized = true;
        }
        /* Any ON attempt consumes OFF, including a non-neutral failed start. */
        authority->off_seen = false;
    }
    authority->status.reason = reason;
}
