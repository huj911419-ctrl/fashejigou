#include "parameter_store.h"

#include <assert.h>
#include <stdio.h>
#include <string.h>

#define SLOT_BYTES 512U
#define TEST_SCHEMA UINT32_C(0x31435046)

typedef struct
{
    uint8_t bytes[PARAMETER_STORE_SLOT_COUNT][SLOT_BYTES];
    long mutation_budget;
    unsigned reads;
    unsigned programs;
    unsigned erases;
    unsigned fail_read;
    unsigned corrupt_program;
    bool bad_erase;
} FakeNor_s;

static bool FakeRead(void *context, uint8_t slot, size_t offset, void *data, size_t size)
{
    FakeNor_s *nor = context;
    assert(slot < PARAMETER_STORE_SLOT_COUNT && offset + size <= SLOT_BYTES);
    ++nor->reads;
    if (nor->reads == nor->fail_read)
    {
        memset(data, 0, size / 2U); /* An IO failure is not an erased record. */
        return false;
    }
    memcpy(data, nor->bytes[slot] + offset, size);
    return true;
}

static bool MutationAllowed(FakeNor_s *nor)
{
    if (nor->mutation_budget == 0)
    {
        return false;
    }
    if (nor->mutation_budget > 0)
    {
        --nor->mutation_budget;
    }
    return true;
}

static bool FakeProgram(void *context, uint8_t slot, size_t offset,
                        const void *data, size_t size)
{
    FakeNor_s *nor = context;
    const uint8_t *source = data;
    assert(slot < PARAMETER_STORE_SLOT_COUNT && offset + size <= SLOT_BYTES);
    assert(offset % PARAMETER_STORE_PROGRAM_UNIT == 0U);
    assert((uintptr_t)data % PARAMETER_STORE_PROGRAM_UNIT == 0U);
    assert(size == PARAMETER_STORE_PROGRAM_UNIT);
    ++nor->programs;
    for (size_t i = 0U; i < size; ++i)
    {
        if (!MutationAllowed(nor) || (source[i] & nor->bytes[slot][offset + i]) != source[i])
        {
            return false;
        }
        nor->bytes[slot][offset + i] &= source[i];
    }
    if (nor->programs == nor->corrupt_program)
    {
        nor->bytes[slot][offset] ^= 1U; /* Driver lies: success with bad data. */
    }
    return true;
}

static bool FakeErase(void *context, uint8_t slot)
{
    FakeNor_s *nor = context;
    assert(slot < PARAMETER_STORE_SLOT_COUNT);
    ++nor->erases;
    for (size_t i = 0U; i < SLOT_BYTES; ++i)
    {
        if (!MutationAllowed(nor))
        {
            return false;
        }
        nor->bytes[slot][i] = UINT8_MAX;
    }
    if (nor->bad_erase)
    {
        nor->bytes[slot][287U] = 0U;
    }
    return true;
}

static void ClearFaults(FakeNor_s *nor)
{
    nor->mutation_budget = -1;
    nor->reads = 0U;
    nor->programs = 0U;
    nor->erases = 0U;
    nor->fail_read = 0U;
    nor->corrupt_program = 0U;
    nor->bad_erase = false;
}

static void EmptyNor(FakeNor_s *nor)
{
    memset(nor, 0, sizeof(*nor));
    memset(nor->bytes, UINT8_MAX, sizeof(nor->bytes));
    ClearFaults(nor);
}

static void OpenStore(ParameterStore_s *store, FakeNor_s *nor, size_t length)
{
    ParameterStoreIo_s io = {nor, FakeRead, FakeProgram, FakeErase};
    assert(ParameterStoreInit(store, &io, TEST_SCHEMA, length));
}

static void FillPayload(uint8_t *payload, size_t size, uint8_t seed)
{
    for (size_t i = 0U; i < size; ++i)
    {
        payload[i] = (uint8_t)(seed + i * 37U);
    }
}

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

/* Independent bitwise oracle, also checked against the standard CRC-32 vector. */
static uint32_t Crc32(const uint8_t *bytes, size_t size)
{
    uint32_t crc = UINT32_MAX;
    for (size_t i = 0U; i < size; ++i)
    {
        uint8_t value = bytes[i];
        for (unsigned bit = 0U; bit < 8U; ++bit)
        {
            bool feedback = ((crc ^ value) & 1U) != 0U;
            crc >>= 1;
            if (feedback)
            {
                crc ^= UINT32_C(0xEDB88320);
            }
            value >>= 1;
        }
    }
    return crc ^ UINT32_MAX;
}

static void RepairChecksums(uint8_t *record)
{
    Write32(record + 28U, Crc32(record, 28U));
    Write32(record + 292U, Read32(record + 16U));
    Write32(record + 296U, Read32(record + 28U));
    Write32(record + 300U, Read32(record + 20U));
    Write32(record + 316U, Crc32(record + 288U, 28U));
}

static void CheckUnchangedFailure(const ParameterStore_s *store,
                                  ParameterStoreResult_t expected)
{
    uint8_t output[PARAMETER_STORE_MAX_PAYLOAD];
    uint8_t untouched[PARAMETER_STORE_MAX_PAYLOAD];
    ParameterStoreInfo_s info = {123U, 7U};
    memset(output, 0xA5, sizeof(output));
    memcpy(untouched, output, sizeof(output));
    assert(ParameterStoreLoad(store, output, sizeof(output), &info) == expected);
    assert(memcmp(output, untouched, sizeof(output)) == 0);
    assert(info.generation == 123U && info.slot == 7U);
}

static void TestRoundTripAndArguments(void)
{
    FakeNor_s nor;
    ParameterStore_s store;
    uint8_t payload[PARAMETER_STORE_MAX_PAYLOAD];
    uint8_t output[PARAMETER_STORE_MAX_PAYLOAD];
    ParameterStoreInfo_s info;
    EmptyNor(&nor);
    OpenStore(&store, &nor, 97U);
    assert(nor.reads == 0U && nor.programs == 0U && nor.erases == 0U);
    CheckUnchangedFailure(&store, PARAMETER_STORE_EMPTY);
    assert(Crc32((const uint8_t *)"123456789", 9U) == UINT32_C(0xCBF43926));
    FillPayload(payload, sizeof(payload), 19U);
    assert(ParameterStoreSave(&store, payload, 97U, &info) == PARAMETER_STORE_OK);
    assert(info.generation == 1U && info.slot == 0U);
    assert(nor.programs == 6U); /* Header, four data units, commit. */
    assert(memcmp(nor.bytes[0], "PST1", 4U) == 0);
    assert(Read32(nor.bytes[0] + 20U) == Crc32(payload, 97U));
    assert(ParameterStoreLoad(&store, output, sizeof(output), &info) == PARAMETER_STORE_OK);
    assert(memcmp(payload, output, 97U) == 0);
    for (uint32_t generation = 2U; generation <= 40U; ++generation)
    {
        payload[0] = (uint8_t)generation;
        assert(ParameterStoreSave(&store, payload, 97U, &info) == PARAMETER_STORE_OK);
        assert(info.generation == generation && info.slot == (generation - 1U) % 2U);
        OpenStore(&store, &nor, 97U); /* Reboot loses all instance history. */
        assert(ParameterStoreLoad(&store, output, sizeof(output), &info) == PARAMETER_STORE_OK);
        assert(memcmp(payload, output, 97U) == 0);
    }
    unsigned writes = nor.programs + nor.erases;
    assert(ParameterStoreSave(&store, payload, 96U, NULL) == PARAMETER_STORE_INVALID_ARGUMENT);
    assert(ParameterStoreSave(&store, NULL, 97U, NULL) == PARAMETER_STORE_INVALID_ARGUMENT);
    assert(ParameterStoreLoad(&store, output, 96U, NULL) == PARAMETER_STORE_INVALID_ARGUMENT);
    assert(ParameterStoreLoad(&store, NULL, sizeof(output), NULL) == PARAMETER_STORE_INVALID_ARGUMENT);
    assert(writes == nor.programs + nor.erases);
    ParameterStoreIo_s io = {&nor, FakeRead, FakeProgram, FakeErase};
    assert(!ParameterStoreInit(NULL, &io, TEST_SCHEMA, 1U));
    assert(!ParameterStoreInit(&store, &io, TEST_SCHEMA, 0U));
    assert(!ParameterStoreInit(&store, &io, TEST_SCHEMA, 257U));
    assert(!ParameterStoreInit(&store, NULL, TEST_SCHEMA, 1U));
    assert(ParameterStoreSave(&store, payload, 1U, NULL) == PARAMETER_STORE_INVALID_ARGUMENT);
    io.program = NULL;
    assert(!ParameterStoreInit(&store, &io, TEST_SCHEMA, 1U));
    EmptyNor(&nor);
    OpenStore(&store, &nor, 1U);
    assert(ParameterStoreSave(&store, payload, 1U, NULL) == PARAMETER_STORE_OK);
    assert(ParameterStoreLoad(&store, output, 1U, NULL) == PARAMETER_STORE_OK);
    assert(payload[0] == output[0]);
}

static void SeedTwoRecords(FakeNor_s *nor, size_t size, const uint8_t *older,
                           const uint8_t *latest)
{
    ParameterStore_s store;
    EmptyNor(nor);
    OpenStore(&store, nor, size);
    assert(ParameterStoreSave(&store, older, size, NULL) == PARAMETER_STORE_OK);
    assert(ParameterStoreSave(&store, latest, size, NULL) == PARAMETER_STORE_OK);
    ClearFaults(nor);
}

static void TestEveryPowerCut(void)
{
    FakeNor_s seed;
    FakeNor_s trial;
    ParameterStore_s store;
    uint8_t older[PARAMETER_STORE_MAX_PAYLOAD];
    uint8_t latest[PARAMETER_STORE_MAX_PAYLOAD];
    uint8_t next[PARAMETER_STORE_MAX_PAYLOAD];
    uint8_t output[PARAMETER_STORE_MAX_PAYLOAD];
    FillPayload(older, sizeof(older), 3U);
    FillPayload(latest, sizeof(latest), 37U);
    FillPayload(next, sizeof(next), 71U);
    SeedTwoRecords(&seed, sizeof(older), older, latest);

    const long mutation_bytes = SLOT_BYTES + PARAMETER_STORE_RECORD_SIZE;
    for (long cut = 0; cut <= mutation_bytes; ++cut)
    {
        trial = seed;
        OpenStore(&store, &trial, sizeof(next));
        trial.mutation_budget = cut;
        ParameterStoreInfo_s info = {999U, 9U};
        ParameterStoreResult_t saved = ParameterStoreSave(&store, next, sizeof(next), &info);
        assert(saved == PARAMETER_STORE_IO_ERROR || saved == PARAMETER_STORE_OK);
        assert(memcmp(trial.bytes[1], seed.bytes[1], SLOT_BYTES) == 0);
        if (saved != PARAMETER_STORE_OK)
        {
            assert(info.generation == 999U && info.slot == 9U);
        }
        ClearFaults(&trial);
        OpenStore(&store, &trial, sizeof(output));
        assert(ParameterStoreLoad(&store, output, sizeof(output), &info) == PARAMETER_STORE_OK);
        assert(memcmp(output, latest, sizeof(output)) == 0 ||
               memcmp(output, next, sizeof(output)) == 0);
        if (saved == PARAMETER_STORE_OK)
        {
            assert(memcmp(output, next, sizeof(output)) == 0);
        }
        /* A torn inactive record remains reusable with the current one intact. */
        assert(ParameterStoreSave(&store, older, sizeof(older), NULL) == PARAMETER_STORE_OK);
        assert(ParameterStoreLoad(&store, output, sizeof(output), NULL) == PARAMETER_STORE_OK);
        assert(memcmp(output, older, sizeof(output)) == 0);
    }

    EmptyNor(&seed);
    for (long cut = 0; cut <= mutation_bytes; ++cut)
    {
        trial = seed;
        OpenStore(&store, &trial, sizeof(next));
        trial.mutation_budget = cut;
        (void)ParameterStoreSave(&store, next, sizeof(next), NULL);
        ClearFaults(&trial);
        OpenStore(&store, &trial, sizeof(next));
        ParameterStoreResult_t loaded = ParameterStoreLoad(&store, output, sizeof(output), NULL);
        assert(loaded == PARAMETER_STORE_OK || loaded == PARAMETER_STORE_EMPTY ||
               loaded == PARAMETER_STORE_CORRUPT);
        if (loaded == PARAMETER_STORE_OK)
        {
            assert(memcmp(output, next, sizeof(output)) == 0);
        }
        else if (loaded == PARAMETER_STORE_CORRUPT)
        {
            /* First save had no previous valid record. Do not silently erase
             * the evidence; explicit application recovery must provision it. */
            assert(ParameterStoreSave(&store, next, sizeof(next), NULL) == PARAMETER_STORE_CORRUPT);
            assert(trial.erases == 0U && trial.programs == 0U);
        }
    }
    printf("parameter_store: %ld byte-boundary power cuts verified\n", 2 * (mutation_bytes + 1));
}

static void TestCorruptionAndCompatibility(void)
{
    FakeNor_s seed;
    FakeNor_s nor;
    ParameterStore_s store;
    uint8_t older[97];
    uint8_t latest[97];
    uint8_t output[97];
    FillPayload(older, sizeof(older), 2U);
    FillPayload(latest, sizeof(latest), 22U);
    SeedTwoRecords(&seed, sizeof(older), older, latest);
    for (size_t byte = 0U; byte < PARAMETER_STORE_RECORD_SIZE; ++byte)
    {
        nor = seed;
        nor.bytes[1][byte] ^= 1U;
        OpenStore(&store, &nor, sizeof(older));
        assert(ParameterStoreLoad(&store, output, sizeof(output), NULL) == PARAMETER_STORE_OK);
        assert(memcmp(output, older, sizeof(output)) == 0);
    }
    nor = seed;
    nor.bytes[0][0] ^= 1U;
    nor.bytes[1][0] ^= 1U;
    OpenStore(&store, &nor, sizeof(older));
    CheckUnchangedFailure(&store, PARAMETER_STORE_CORRUPT);
    assert(ParameterStoreSave(&store, latest, sizeof(latest), NULL) == PARAMETER_STORE_CORRUPT);
    assert(nor.programs == 0U && nor.erases == 0U);

    /* A correctly checksummed but incompatible record must never be overwritten. */
    const size_t fields[] = {4U, 8U, 12U};
    for (size_t index = 0U; index < sizeof(fields) / sizeof(fields[0]); ++index)
    {
        nor = seed;
        size_t field = fields[index];
        Write32(nor.bytes[1] + field, Read32(nor.bytes[1] + field) + 1U);
        if (field == 12U)
        {
            Write32(nor.bytes[1] + 20U, Crc32(nor.bytes[1] + 32U, 98U));
        }
        RepairChecksums(nor.bytes[1]);
        OpenStore(&store, &nor, sizeof(older));
        CheckUnchangedFailure(&store, PARAMETER_STORE_INCOMPATIBLE);
        assert(ParameterStoreSave(&store, latest, sizeof(latest), NULL) == PARAMETER_STORE_INCOMPATIBLE);
        assert(nor.programs == 0U && nor.erases == 0U);
    }
    nor = seed;
    OpenStore(&store, &nor, 98U);
    CheckUnchangedFailure(&store, PARAMETER_STORE_INCOMPATIBLE);
    nor = seed;
    Write32(nor.bytes[1] + 12U, UINT32_MAX);
    RepairChecksums(nor.bytes[1]);
    OpenStore(&store, &nor, sizeof(older));
    assert(ParameterStoreLoad(&store, output, sizeof(output), NULL) == PARAMETER_STORE_OK);
    assert(memcmp(output, older, sizeof(output)) == 0);
}

static void TestIoErrorsAndLyingDriver(void)
{
    FakeNor_s seed;
    FakeNor_s nor;
    ParameterStore_s store;
    uint8_t older[97];
    uint8_t latest[97];
    uint8_t next[97];
    uint8_t output[97];
    FillPayload(older, sizeof(older), 12U);
    FillPayload(latest, sizeof(latest), 42U);
    FillPayload(next, sizeof(next), 72U);
    SeedTwoRecords(&seed, sizeof(older), older, latest);
    for (unsigned failing_read = 1U; failing_read <= 4U; ++failing_read)
    {
        nor = seed;
        nor.fail_read = failing_read;
        OpenStore(&store, &nor, sizeof(next));
        ParameterStoreInfo_s info = {444U, 8U};
        assert(ParameterStoreSave(&store, next, sizeof(next), &info) == PARAMETER_STORE_IO_ERROR);
        assert(info.generation == 444U && info.slot == 8U);
        if (failing_read <= 2U)
        {
            assert(nor.programs == 0U && nor.erases == 0U);
        }
        assert(memcmp(nor.bytes[1], seed.bytes[1], SLOT_BYTES) == 0);
        ClearFaults(&nor);
        assert(ParameterStoreLoad(&store, output, sizeof(output), NULL) == PARAMETER_STORE_OK);
        assert(memcmp(output, failing_read == 4U ? next : latest, sizeof(output)) == 0);
    }
    for (unsigned failing_read = 1U; failing_read <= 2U; ++failing_read)
    {
        nor = seed;
        nor.fail_read = failing_read;
        OpenStore(&store, &nor, sizeof(next));
        CheckUnchangedFailure(&store, PARAMETER_STORE_IO_ERROR);
        EmptyNor(&nor);
        nor.fail_read = failing_read;
        CheckUnchangedFailure(&store, PARAMETER_STORE_IO_ERROR);
    }
    for (unsigned failing_program = 1U; failing_program <= 6U; ++failing_program)
    {
        nor = seed;
        nor.corrupt_program = failing_program;
        OpenStore(&store, &nor, sizeof(next));
        assert(ParameterStoreSave(&store, next, sizeof(next), NULL) == PARAMETER_STORE_CORRUPT);
        ClearFaults(&nor);
        assert(ParameterStoreLoad(&store, output, sizeof(output), NULL) == PARAMETER_STORE_OK);
        assert(memcmp(output, latest, sizeof(output)) == 0);
    }
    nor = seed;
    nor.bad_erase = true;
    OpenStore(&store, &nor, sizeof(next));
    assert(ParameterStoreSave(&store, next, sizeof(next), NULL) == PARAMETER_STORE_CORRUPT);
    ClearFaults(&nor);
    assert(ParameterStoreLoad(&store, output, sizeof(output), NULL) == PARAMETER_STORE_OK);
    assert(memcmp(output, latest, sizeof(output)) == 0);
}

static void TestGenerationOrdering(void)
{
    FakeNor_s seed;
    FakeNor_s nor;
    ParameterStore_s store;
    uint8_t older[33];
    uint8_t latest[33];
    uint8_t output[33];
    ParameterStoreInfo_s info;
    FillPayload(older, sizeof(older), 3U);
    FillPayload(latest, sizeof(latest), 4U);
    SeedTwoRecords(&seed, sizeof(older), older, latest);
    nor = seed;
    Write32(nor.bytes[0] + 16U, UINT32_MAX);
    Write32(nor.bytes[1] + 16U, 0U);
    RepairChecksums(nor.bytes[0]);
    RepairChecksums(nor.bytes[1]);
    OpenStore(&store, &nor, sizeof(older));
    assert(ParameterStoreLoad(&store, output, sizeof(output), &info) == PARAMETER_STORE_OK);
    assert(info.slot == 1U && info.generation == 0U);
    assert(memcmp(output, latest, sizeof(output)) == 0);
    assert(ParameterStoreSave(&store, older, sizeof(older), &info) == PARAMETER_STORE_OK);
    assert(info.slot == 0U && info.generation == 1U);

    const uint32_t ambiguous[] = {1U, UINT32_C(0x80000001)};
    for (size_t index = 0U; index < sizeof(ambiguous) / sizeof(ambiguous[0]); ++index)
    {
        nor = seed;
        Write32(nor.bytes[1] + 16U, ambiguous[index]);
        RepairChecksums(nor.bytes[1]);
        OpenStore(&store, &nor, sizeof(older));
        CheckUnchangedFailure(&store, PARAMETER_STORE_AMBIGUOUS);
        assert(ParameterStoreSave(&store, latest, sizeof(latest), NULL) == PARAMETER_STORE_AMBIGUOUS);
        assert(nor.programs == 0U && nor.erases == 0U);
    }
    nor = seed;
    memcpy(nor.bytes[1], nor.bytes[0], SLOT_BYTES);
    OpenStore(&store, &nor, sizeof(older));
    assert(ParameterStoreLoad(&store, output, sizeof(output), &info) == PARAMETER_STORE_OK);
    assert(info.slot == 0U && info.generation == 1U);
    assert(ParameterStoreSave(&store, latest, sizeof(latest), &info) == PARAMETER_STORE_OK);
    assert(info.slot == 1U && info.generation == 2U);
}

int main(void)
{
    TestRoundTripAndArguments();
    TestEveryPowerCut();
    TestCorruptionAndCompatibility();
    TestIoErrorsAndLyingDriver();
    TestGenerationOrdering();
    puts("parameter_store: all tests passed");
    return 0;
}
