#include "parameter_store.h"

#include <string.h>

#define RECORD_MAGIC UINT32_C(0x31545350) /* PST1, little endian. */
#define COMMIT_MAGIC UINT32_C(0x454E4F44) /* DONE, little endian. */
#define FORMAT_VERSION 1U
#define HEADER_SIZE PARAMETER_STORE_PROGRAM_UNIT
#define PAYLOAD_OFFSET HEADER_SIZE
#define COMMIT_OFFSET (PAYLOAD_OFFSET + PARAMETER_STORE_MAX_PAYLOAD)
#define CRC_OFFSET 28U
#define SERIAL_HALF_RANGE UINT32_C(0x80000000)

typedef enum
{
    SLOT_EMPTY,
    SLOT_DAMAGED,
    SLOT_FOREIGN,
    SLOT_VALID
} SlotState_t;

typedef struct
{
    uint8_t bytes[PARAMETER_STORE_SLOT_COUNT][PARAMETER_STORE_RECORD_SIZE];
    SlotState_t state[PARAMETER_STORE_SLOT_COUNT];
    uint8_t latest;
} StoreScan_s;

static uint32_t Read32(const uint8_t *bytes)
{
    return (uint32_t)bytes[0] | ((uint32_t)bytes[1] << 8) |
           ((uint32_t)bytes[2] << 16) | ((uint32_t)bytes[3] << 24);
}

static void Write32(uint8_t *bytes, uint32_t value)
{
    for (unsigned i = 0U; i < 4U; ++i)
    {
        bytes[i] = (uint8_t)(value >> (8U * i));
    }
}

static uint32_t Crc32(const uint8_t *bytes, size_t length)
{
    uint32_t crc = UINT32_MAX;
    for (size_t i = 0U; i < length; ++i)
    {
        crc ^= bytes[i];
        for (unsigned bit = 0U; bit < 8U; ++bit)
        {
            crc = (crc >> 1) ^ ((crc & 1U) ? UINT32_C(0xEDB88320) : 0U);
        }
    }
    return ~crc;
}

static bool IsErased(const uint8_t *bytes, size_t length)
{
    for (size_t i = 0U; i < length; ++i)
    {
        if (bytes[i] != UINT8_MAX)
        {
            return false;
        }
    }
    return true;
}

static SlotState_t InspectSlot(const ParameterStore_s *store, const uint8_t *bytes)
{
    const uint8_t *commit = bytes + COMMIT_OFFSET;
    uint32_t length = Read32(bytes + 12U);
    if (IsErased(bytes, PARAMETER_STORE_RECORD_SIZE))
    {
        return SLOT_EMPTY;
    }
    if (Read32(bytes) != RECORD_MAGIC ||
        Read32(bytes + CRC_OFFSET) != Crc32(bytes, CRC_OFFSET))
    {
        return SLOT_DAMAGED;
    }
    if (Read32(bytes + 4U) != FORMAT_VERSION)
    {
        return SLOT_FOREIGN;
    }
    if (length == 0U || length > PARAMETER_STORE_MAX_PAYLOAD ||
        Read32(bytes + 24U) != HEADER_SIZE ||
        Read32(bytes + 20U) != Crc32(bytes + PAYLOAD_OFFSET, length) ||
        !IsErased(bytes + PAYLOAD_OFFSET + length,
                  PARAMETER_STORE_MAX_PAYLOAD - length) ||
        Read32(commit) != COMMIT_MAGIC ||
        Read32(commit + 4U) != Read32(bytes + 16U) ||
        Read32(commit + 8U) != Read32(bytes + CRC_OFFSET) ||
        Read32(commit + 12U) != Read32(bytes + 20U) ||
        Read32(commit + 16U) != 0U || Read32(commit + 20U) != 0U ||
        Read32(commit + 24U) != 0U ||
        Read32(commit + CRC_OFFSET) != Crc32(commit, CRC_OFFSET))
    {
        return SLOT_DAMAGED;
    }
    if (Read32(bytes + 8U) != store->schema_id || length != store->payload_length)
    {
        return SLOT_FOREIGN;
    }
    return SLOT_VALID;
}

static ParameterStoreResult_t Scan(const ParameterStore_s *store, StoreScan_s *scan)
{
    bool io_error = false;
    for (uint8_t slot = 0U; slot < PARAMETER_STORE_SLOT_COUNT; ++slot)
    {
        if (!store->io.read(store->io.context, slot, 0U, scan->bytes[slot],
                            PARAMETER_STORE_RECORD_SIZE))
        {
            io_error = true;
        }
        else
        {
            scan->state[slot] = InspectSlot(store, scan->bytes[slot]);
        }
    }
    if (io_error)
    {
        return PARAMETER_STORE_IO_ERROR;
    }
    if (scan->state[0] == SLOT_FOREIGN || scan->state[1] == SLOT_FOREIGN)
    {
        return PARAMETER_STORE_INCOMPATIBLE;
    }
    if (scan->state[0] != SLOT_VALID && scan->state[1] != SLOT_VALID)
    {
        return scan->state[0] == SLOT_EMPTY && scan->state[1] == SLOT_EMPTY ?
            PARAMETER_STORE_EMPTY : PARAMETER_STORE_CORRUPT;
    }
    if (scan->state[0] != SLOT_VALID || scan->state[1] != SLOT_VALID)
    {
        scan->latest = scan->state[0] == SLOT_VALID ? 0U : 1U;
        return PARAMETER_STORE_OK;
    }

    uint32_t delta = Read32(scan->bytes[1] + 16U) - Read32(scan->bytes[0] + 16U);
    if (delta == SERIAL_HALF_RANGE ||
        (delta == 0U && memcmp(scan->bytes[0], scan->bytes[1],
                             PARAMETER_STORE_RECORD_SIZE) != 0))
    {
        return PARAMETER_STORE_AMBIGUOUS;
    }
    scan->latest = delta != 0U && delta < SERIAL_HALF_RANGE ? 1U : 0U;
    return PARAMETER_STORE_OK;
}

bool ParameterStoreInit(ParameterStore_s *store, const ParameterStoreIo_s *io,
                        uint32_t schema_id, size_t payload_length)
{
    if (store == NULL)
    {
        return false;
    }
    memset(store, 0, sizeof(*store));
    if (io == NULL || io->read == NULL || io->program == NULL || io->erase == NULL ||
        payload_length == 0U || payload_length > PARAMETER_STORE_MAX_PAYLOAD)
    {
        return false;
    }
    store->io = *io;
    store->schema_id = schema_id;
    store->payload_length = payload_length;
    store->initialized = true;
    return true;
}

ParameterStoreResult_t ParameterStoreLoad(const ParameterStore_s *store,
                                         void *payload, size_t capacity,
                                         ParameterStoreInfo_s *info)
{
    if (store == NULL || !store->initialized || payload == NULL ||
        capacity < store->payload_length)
    {
        return PARAMETER_STORE_INVALID_ARGUMENT;
    }
    StoreScan_s scan;
    ParameterStoreResult_t result = Scan(store, &scan);
    if (result != PARAMETER_STORE_OK)
    {
        return result;
    }
    memcpy(payload, scan.bytes[scan.latest] + PAYLOAD_OFFSET, store->payload_length);
    if (info != NULL)
    {
        info->generation = Read32(scan.bytes[scan.latest] + 16U);
        info->slot = scan.latest;
    }
    return PARAMETER_STORE_OK;
}

static void BuildHeader(uint8_t *record, const ParameterStore_s *store,
                        const void *payload, uint32_t generation)
{
    memset(record, UINT8_MAX, PARAMETER_STORE_RECORD_SIZE);
    memcpy(record + PAYLOAD_OFFSET, payload, store->payload_length);
    Write32(record, RECORD_MAGIC);
    Write32(record + 4U, FORMAT_VERSION);
    Write32(record + 8U, store->schema_id);
    Write32(record + 12U, (uint32_t)store->payload_length);
    Write32(record + 16U, generation);
    Write32(record + 20U, Crc32(record + PAYLOAD_OFFSET, store->payload_length));
    Write32(record + 24U, HEADER_SIZE);
    Write32(record + CRC_OFFSET, Crc32(record, CRC_OFFSET));
}

static void BuildCommit(uint8_t *record)
{
    uint8_t *commit = record + COMMIT_OFFSET;
    memset(commit, 0, PARAMETER_STORE_PROGRAM_UNIT);
    Write32(commit, COMMIT_MAGIC);
    Write32(commit + 4U, Read32(record + 16U));
    Write32(commit + 8U, Read32(record + CRC_OFFSET));
    Write32(commit + 12U, Read32(record + 20U));
    Write32(commit + CRC_OFFSET, Crc32(commit, CRC_OFFSET));
}

ParameterStoreResult_t ParameterStoreSave(const ParameterStore_s *store,
                                         const void *payload, size_t length,
                                         ParameterStoreInfo_s *info)
{
    if (store == NULL || !store->initialized || payload == NULL ||
        length != store->payload_length)
    {
        return PARAMETER_STORE_INVALID_ARGUMENT;
    }
    StoreScan_s scan;
    ParameterStoreResult_t result = Scan(store, &scan);
    if (result != PARAMETER_STORE_OK && result != PARAMETER_STORE_EMPTY)
    {
        return result;
    }
    uint8_t target = result == PARAMETER_STORE_EMPTY ? 0U : (uint8_t)(1U - scan.latest);
    uint32_t generation = result == PARAMETER_STORE_EMPTY ? 1U :
        Read32(scan.bytes[scan.latest] + 16U) + 1U;
    _Alignas(PARAMETER_STORE_PROGRAM_UNIT) uint8_t record[PARAMETER_STORE_RECORD_SIZE];
    BuildHeader(record, store, payload, generation);
    if (!store->io.erase(store->io.context, target))
    {
        return PARAMETER_STORE_IO_ERROR;
    }
    size_t end = PAYLOAD_OFFSET + length;
    for (size_t offset = 0U; offset < end; offset += PARAMETER_STORE_PROGRAM_UNIT)
    {
        if (!store->io.program(store->io.context, target, offset, record + offset,
                               PARAMETER_STORE_PROGRAM_UNIT))
        {
            return PARAMETER_STORE_IO_ERROR;
        }
    }
    if (!store->io.read(store->io.context, target, 0U, scan.bytes[target],
                        PARAMETER_STORE_RECORD_SIZE))
    {
        return PARAMETER_STORE_IO_ERROR;
    }
    if (memcmp(scan.bytes[target], record, PARAMETER_STORE_RECORD_SIZE) != 0)
    {
        return PARAMETER_STORE_CORRUPT;
    }
    BuildCommit(record);
    if (!store->io.program(store->io.context, target, COMMIT_OFFSET,
                           record + COMMIT_OFFSET, PARAMETER_STORE_PROGRAM_UNIT) ||
        !store->io.read(store->io.context, target, 0U, scan.bytes[target],
                        PARAMETER_STORE_RECORD_SIZE))
    {
        return PARAMETER_STORE_IO_ERROR;
    }
    if (memcmp(scan.bytes[target], record, PARAMETER_STORE_RECORD_SIZE) != 0 ||
        InspectSlot(store, scan.bytes[target]) != SLOT_VALID)
    {
        return PARAMETER_STORE_CORRUPT;
    }
    if (info != NULL)
    {
        info->generation = generation;
        info->slot = target;
    }
    return PARAMETER_STORE_OK;
}
