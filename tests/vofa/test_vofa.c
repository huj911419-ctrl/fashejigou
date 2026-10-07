#include "vofa.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

static uint8_t rx[4096], tx[512];
static size_t used, consumed, chunk = 128, tx_size;
static unsigned sets, calls, faults;
static int lost, busy;
static float parameter = 3;
static Vofa_s v;

static int Read(void *ctx, uint8_t *out, size_t capacity)
{
    (void)ctx;
    if (lost) { consumed = used; lost = 0; return -1; }
    size_t n = used - consumed;
    if (n > chunk) n = chunk;
    if (n > capacity) n = capacity;
    memcpy(out, rx + consumed, n); consumed += n;
    return (int)n;
}
static int Write(void *ctx, const uint8_t *in, size_t n)
{
    (void)ctx;
    if (busy) return 0;
    assert(n <= sizeof(tx)); memcpy(tx, in, n); tx_size = n; return 1;
}
static Vofa_Result_e Get(void *ctx, uint16_t id, float *out)
{ (void)ctx; if (id != 10) return VOFA_UNKNOWN; *out = parameter; return VOFA_OK; }
static Vofa_Result_e Set(void *ctx, uint16_t seq, uint16_t id, float value)
{
    (void)ctx; assert(seq);
    if (id != 10) return VOFA_UNKNOWN;
    if (value < 0 || value > 100) return VOFA_RANGE;
    ++sets; parameter = value; return VOFA_OK;
}
static Vofa_Result_e Call(void *ctx, uint16_t seq, uint16_t id, const float *a, size_t n)
{ (void)ctx; (void)seq; (void)id; (void)a; (void)n; ++calls; return VOFA_QUEUED; }
static void Fault(void *ctx) { (void)ctx; ++faults; }
static void Feed(const char *s, uint32_t now)
{
    size_t n = strlen(s);
    assert(used + n <= sizeof(rx)); memcpy(rx + used, s, n); used += n;
    while (consumed < used) VofaPoll(&v, now);
    used = consumed = 0;
}
int main(void)
{
    Vofa_Port_s port = {Read, Write, NULL};
    Vofa_Handler_s handler = {Get, Set, Call, Fault, NULL};
    VofaInit(&v, &port, &handler);
    chunk = 1;
    Feed("set 1 10 0.5\r\n", 0);
    assert(sets == 1 && parameter == 0.5f && v.ack[0] == 1);
    chunk = 128;
    Feed("set 2 10 2\nget 3 10\n", 1);
    assert(sets == 2 && v.ack[0] == 3 && v.ack[3] == 2);
    Feed("set 4 10 1e0\n", 1); assert(parameter == 1);
    Feed("set 5 10 -1\n", 1); assert(v.ack[1] == VOFA_RANGE && parameter == 1);
    unsigned before = sets;
    const char *invalid[] = {"set 6 10 nan\n", "set 6 10 inf\n", "set 6 10 1e99\n",
        "set 0 10 1\n", "set 65536 10 1\n", "set 6 10 1garbage\n", "set 6 10 1 extra\n",
        "set 6 10.0 1\n", "set 6 10 1\rgarbage\n", "get 6 10 extra\n"};
    for (unsigned i = 0; i < sizeof(invalid) / sizeof(invalid[0]); ++i) Feed(invalid[i], 2);
    assert(sets == before && faults == sizeof(invalid) / sizeof(invalid[0]));
    char huge[300]; memset(huge, 'A', sizeof(huge) - 1); huge[sizeof(huge) - 1] = 0;
    Feed(huge, 3); Feed("set 7 10 7\n", 3); assert(sets == before);
    Feed("set 8 10 8\n", 3); assert(parameter == 8);
    Feed("set 9 10 ", UINT32_MAX - 50U);
    VofaPoll(&v, 60); Feed("9\n", 60); assert(parameter == 8 && v.ack[1] == VOFA_RX_LOSS);
    Feed("get 10 10\n", 60); assert(v.ack[3] == 8);
    Feed("set 11 10 ", 60); lost = 1; VofaPoll(&v, 61);
    Feed("1\nget 12 10\n", 61); assert(parameter == 8 && v.ack[0] == 12);
    Feed("call 13 3 1 2 3 4 5 6 7 8\n", 61); assert(calls == 1);
    Feed("call 14 3 1 2 3 4 5 6 7 8 9\n", 61); assert(calls == 1);
    Feed("get 65535 10\n", 61);
    float ch[] = {1, -2.5f};
    assert(VofaSend(&v, ch, 2) && tx_size == 28);
    const uint8_t expected[] = {0, 0, 0x80, 0x3f, 0, 0, 0x20, 0xc0, 0, 0, 0x80, 0x7f};
    assert(!memcmp(tx + 16, expected, sizeof(expected)));
    busy = 1; assert(!VofaSend(&v, ch, 2) && v.tx_dropped == 1);
    busy = 0; ch[0] = INFINITY; assert(!VofaSend(&v, ch, 2) && v.tx_dropped == 2);
    assert(!VofaSend(&v, ch, VOFA_MAX_CHANNELS));
    VofaInit(&v, NULL, NULL); VofaPoll(&v, 0); ch[0] = 0;
    assert(!VofaSend(&v, ch, 2));
    puts("PASS: fragmentation/coalescing, strict grammar, finite/range validation, RX loss/timeout/overflow recovery, little endian framing and nonblocking TX");
    return 0;
}
