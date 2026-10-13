#include "vofa.h"
#include <float.h>
#include <math.h>
#include <string.h>

_Static_assert(sizeof(float) == 4 && FLT_RADIX == 2 && FLT_MANT_DIG == 24 &&
               FLT_MAX_EXP == 128, "VOFA requires IEEE754 binary32 float");

static void Fault(Vofa_s *v, Vofa_Result_e result)
{
    v->ack[0] = 0; v->ack[1] = (float)result;
    v->ack[2] = 0; v->ack[3] = 0;
    if (v->handler.rx_fault) v->handler.rx_fault(v->handler.context);
}

/* IDs and sequence numbers must be decimal integers, not rounded floats. */
static int U16(const char *s, uint16_t *value)
{
    uint32_t n = 0; /* Also rejects overflow on targets with 16-bit unsigned. */
    if (!*s) return 0;
    for (; *s; ++s) {
        if (*s < '0' || *s > '9') return 0;
        n = n * 10U + (unsigned)(*s - '0');
        if (n > 65535U) return 0;
    }
    *value = (uint16_t)n;
    return 1;
}

static int Number(const char *s, float *value)
{
    /* Bounded decimal parser: avoid libc strtof implementations that allocate
     * heap memory or pull in a large formatted-I/O runtime on embedded targets. */
    double mantissa = 0;
    int sign = 1, digits = 0, fraction = 0, exponent = 0, exponent_sign = 1;
    if (*s == '+' || *s == '-') { if (*s == '-') sign = -1; ++s; }
    while (*s >= '0' && *s <= '9') { mantissa = mantissa * 10 + (*s++ - '0'); ++digits; }
    if (*s == '.') {
        ++s;
        while (*s >= '0' && *s <= '9') {
            mantissa = mantissa * 10 + (*s++ - '0'); ++digits; ++fraction;
        }
    }
    if (!digits) return 0;
    if (*s == 'e' || *s == 'E') {
        ++s;
        if (*s == '+' || *s == '-') { if (*s == '-') exponent_sign = -1; ++s; }
        if (*s < '0' || *s > '9') return 0;
        while (*s >= '0' && *s <= '9') {
            exponent = exponent * 10 + (*s++ - '0');
            if (exponent > 400) return 0;
        }
    }
    if (*s || !isfinite(mantissa)) return 0;
    exponent = exponent * exponent_sign - fraction;
    uint8_t nonzero = mantissa != 0;
    while (exponent > 0) { mantissa *= 10; --exponent; }
    while (exponent < 0) { mantissa /= 10; ++exponent; }
    if (!isfinite(mantissa) || mantissa > FLT_MAX) return 0;
    *value = (float)(sign * mantissa);
    return isfinite(*value) && (!nonzero || *value != 0);
}

static void Parse(Vofa_s *v)
{
    char *tokens[VOFA_MAX_ARGS + 3U];
    size_t count = 0;
    char *p = v->line;
    while (*p) {
        while (*p == ' ' || *p == '\t') ++p;
        if (!*p) break;
        if (count == VOFA_MAX_ARGS + 3U) { Fault(v, VOFA_SYNTAX); return; }
        tokens[count++] = p;
        while (*p && *p != ' ' && *p != '\t') ++p;
        if (*p) *p++ = 0;
    }
    uint16_t seq, id;
    if (count < 3 || !U16(tokens[1], &seq) || !seq || !U16(tokens[2], &id)) {
        Fault(v, VOFA_SYNTAX); return;
    }
    float args[VOFA_MAX_ARGS] = {0}, value = 0;
    for (size_t i = 3; i < count; ++i)
        if (!Number(tokens[i], &args[i - 3])) { Fault(v, VOFA_SYNTAX); return; }
    Vofa_Result_e result;
    if (!strcmp(tokens[0], "get") && count == 3)
        result = v->handler.get ? v->handler.get(v->handler.context, id, &value) : VOFA_UNKNOWN;
    else if (!strcmp(tokens[0], "set") && count == 4) {
        value = args[0];
        result = v->handler.set ? v->handler.set(v->handler.context, seq, id, value) : VOFA_UNKNOWN;
    } else if (!strcmp(tokens[0], "call"))
        result = v->handler.call ? v->handler.call(v->handler.context, seq, id, args, count - 3) : VOFA_UNKNOWN;
    else { Fault(v, VOFA_SYNTAX); return; }
    v->ack[0] = (float)seq; v->ack[1] = (float)result;
    v->ack[2] = (float)id; v->ack[3] = isfinite(value) ? value : 0;
}

void VofaInit(Vofa_s *v, const Vofa_Port_s *port, const Vofa_Handler_s *handler)
{
    memset(v, 0, sizeof(*v));
    if (port) v->port = *port;
    if (handler) v->handler = *handler;
}

void VofaPoll(Vofa_s *v, uint32_t now_ms)
{
    if (!v) return;
    if (v->length && now_ms - v->line_started_ms > VOFA_LINE_TIMEOUT_MS) {
        v->length = 0; v->discard = 1; Fault(v, VOFA_RX_LOSS);
    }
    if (!v->port.read) return;
    uint8_t bytes[VOFA_RX_BUDGET];
    int n = v->port.read(v->port.context, bytes, sizeof(bytes));
    if (n < 0 || n > (int)sizeof(bytes)) {
        v->length = 0; v->discard = 1; Fault(v, VOFA_RX_LOSS); return;
    }
    for (int i = 0; i < n; ++i) {
        uint8_t b = bytes[i];
        if (b == '\n') {
            if (!v->discard && v->length) {
                if (v->line[v->length - 1] == '\r') --v->length;
                v->line[v->length] = 0;
                if (v->length) Parse(v);
            }
            v->length = 0; v->discard = 0;
        } else if (!v->discard) {
            if ((b < 32 && b != '\t' && b != '\r') || b > 126 ||
                v->length >= sizeof(v->line) - 1) {
                v->length = 0; v->discard = 1; Fault(v, VOFA_SYNTAX);
            } else {
                if (!v->length) v->line_started_ms = now_ms;
                v->line[v->length++] = (char)b;
            }
        }
    }
}

int VofaSend(Vofa_s *v, const float *channels, size_t count)
{
    if (!v || count > VOFA_MAX_CHANNELS - 4U || (count && !channels)) return 0;
    for (size_t i = 0; i < count + 4U; ++i) {
        float f = i < 4 ? v->ack[i] : channels[i - 4];
        if (!isfinite(f)) { ++v->tx_dropped; return 0; }
        uint32_t bits;
        memcpy(&bits, &f, sizeof(bits));
        for (unsigned b = 0; b < 4; ++b) v->frame[i * 4 + b] = (uint8_t)(bits >> (8U * b));
    }
    const uint8_t tail[] = {0, 0, 0x80, 0x7f};
    memcpy(v->frame + (count + 4U) * 4U, tail, 4);
    if (!v->port.write || !v->port.write(v->port.context, v->frame, (count + 5U) * 4U)) {
        ++v->tx_dropped; return 0;
    }
    return 1;
}
