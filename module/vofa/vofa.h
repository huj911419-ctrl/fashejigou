#ifndef VOFA_H
#define VOFA_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define VOFA_MAX_CHANNELS 48U
#define VOFA_LINE_SIZE 160U
#define VOFA_MAX_ARGS 8U
#define VOFA_RX_BUDGET 128U
#define VOFA_LINE_TIMEOUT_MS 100U

typedef enum {
    VOFA_OK = 0, VOFA_QUEUED = 1, VOFA_SYNTAX = 2, VOFA_RANGE = 3,
    VOFA_UNKNOWN = 4, VOFA_BUSY = 5, VOFA_UNAVAILABLE = 6, VOFA_RX_LOSS = 7
} Vofa_Result_e;

/** Task-context, nonblocking transport. read: 0=no data, -1=RX loss (flush
 * transport RX before returning -1), otherwise bytes copied <= capacity.
 * write: 1=ALL bytes copied to transport-owned storage, 0=busy/no bytes copied.
 * The write source expires on return: NEVER give it directly to async DMA.
 * Port must preserve packet order and handle DMA/cache/ISR synchronization.
 */
typedef struct {
    int (*read)(void *context, uint8_t *data, size_t capacity);
    int (*write)(void *context, const uint8_t *data, size_t length);
    void *context;
} Vofa_Port_s;

/** Parameters/operations belong to the application. No live-variable pointers.
 * set may return QUEUED; get must return the applied value. The application
 * publishes its applied sequence separately. All callbacks run in Poll's task.
 */
typedef struct {
    Vofa_Result_e (*get)(void *, uint16_t id, float *value);
    Vofa_Result_e (*set)(void *, uint16_t sequence, uint16_t id, float value);
    Vofa_Result_e (*call)(void *, uint16_t sequence, uint16_t id,
                          const float *args, size_t count);
    void (*rx_fault)(void *);
    void *context;
} Vofa_Handler_s;

typedef struct {
    Vofa_Port_s port;
    Vofa_Handler_s handler;
    char line[VOFA_LINE_SIZE];
    size_t length;
    uint32_t line_started_ms;
    uint32_t tx_dropped;
    uint8_t discard;
    float ack[4]; /* sequence, result, parameter/operation id, value */
    uint8_t frame[(VOFA_MAX_CHANNELS + 1U) * 4U];
} Vofa_s;

/** One instance has one task owner; initialize before use. No heap, HAL or OS. */
void VofaInit(Vofa_s *vofa, const Vofa_Port_s *port, const Vofa_Handler_s *handler);
/** Bounded RX processing, supports fragmented/coalesced LF or CRLF commands. */
void VofaPoll(Vofa_s *vofa, uint32_t now_ms);
/** Encode finite IEEE754 binary32 channels as little-endian JustFloat.
 * Prepends four ACK channels. Busy drops the whole frame, never waits/retries.
 * Return 1=accepted, 0=busy/invalid. At most VOFA_MAX_CHANNELS-4 user channels. */
int VofaSend(Vofa_s *vofa, const float *channels, size_t count);

#ifdef __cplusplus
}
#endif
#endif
