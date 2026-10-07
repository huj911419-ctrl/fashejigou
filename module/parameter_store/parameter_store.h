#ifndef PARAMETER_STORE_H
#define PARAMETER_STORE_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define PARAMETER_STORE_SLOT_COUNT 2U
#define PARAMETER_STORE_PROGRAM_UNIT 32U
#define PARAMETER_STORE_MAX_PAYLOAD 256U
#define PARAMETER_STORE_RECORD_SIZE 320U

typedef enum
{
    PARAMETER_STORE_OK = 0,
    PARAMETER_STORE_EMPTY,
    PARAMETER_STORE_CORRUPT,
    PARAMETER_STORE_IO_ERROR,
    PARAMETER_STORE_INCOMPATIBLE,
    PARAMETER_STORE_AMBIGUOUS,
    PARAMETER_STORE_INVALID_ARGUMENT
} ParameterStoreResult_t;

/** Byte-addressed, synchronous storage adapter. Each slot is an independent erase
 * region of at least RECORD_SIZE bytes; erase(slot) must not change the other slot.
 * read accepts arbitrary byte ranges. program receives exactly 32 bytes at a
 * 32-byte aligned slot offset, from a 32-byte aligned source buffer. Slots must
 * start on the real device's erase/program boundaries. The adapter owns physical
 * addresses, cache maintenance and hardware completion; true means all requested
 * bytes completed, false may leave a partially changed target slot. Data must be
 * copied/consumed before return; retaining pointers is forbidden. */
typedef struct
{
    void *context;
    bool (*read)(void *context, uint8_t slot, size_t offset, void *data, size_t size);
    bool (*program)(void *context, uint8_t slot, size_t offset,
                    const void *data, size_t size);
    bool (*erase)(void *context, uint8_t slot);
} ParameterStoreIo_s;

typedef struct
{
    uint32_t generation;
    uint8_t slot;
} ParameterStoreInfo_s;

typedef struct
{
    ParameterStoreIo_s io;
    uint32_t schema_id;
    size_t payload_length;
    bool initialized;
} ParameterStore_s;

/** Copies the adapter/configuration; performs no storage IO. Payloads are opaque
 * byte sequences, 1..256 bytes, exactly payload_length long. A schema_id identifies
 * the application serialization and its version; never persist native structs.
 * The instance and its two slots have one caller/owner; not ISR-safe, no internal
 * locking, allocation, scheduler or global state. Calls may block on the adapter;
 * run storage work outside the time-critical motor task. */
bool ParameterStoreInit(ParameterStore_s *store, const ParameterStoreIo_s *io,
                        uint32_t schema_id, size_t payload_length);

/** Loads the latest committed record after scanning both slots. Neither payload
 * nor optional info is modified on failure. A read error from either slot returns
 * IO_ERROR even if the other looks valid; EMPTY means both record areas erased.
 * CORRUPT means no valid record and at least one nonempty damaged record. Unknown
 * format/schema/length returns INCOMPATIBLE. See README for generation ordering. */
ParameterStoreResult_t ParameterStoreLoad(const ParameterStore_s *store,
                                         void *payload, size_t capacity,
                                         ParameterStoreInfo_s *info);

/** Writes the inactive slot, verifies header/payload, then writes the commit unit
 * last and reads it back. The previously latest record is never erased. Scans
 * both slots before mutation; IO_ERROR, INCOMPATIBLE, AMBIGUOUS or unrecoverable
 * CORRUPT prohibit all writes. A failed save may nevertheless have committed a new
 * record (e.g. final read failed); reload before assuming which generation exists.
 * On error info is unchanged. Only OK confirms durable readback through adapter;
 * real hardware power-loss properties remain the adapter/device's responsibility. */
ParameterStoreResult_t ParameterStoreSave(const ParameterStore_s *store,
                                         const void *payload, size_t length,
                                         ParameterStoreInfo_s *info);

#endif
