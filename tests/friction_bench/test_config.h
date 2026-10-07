#include "../../application/bench_features.h"

#define FRICTION_MOTOR_CONFIGS { \
    {.can_bus=3, .id=BENCH_ENABLE_F1 ? 1 : 0, .direction=1, .kp=0.5f, .ki=0.1f, .current_limit=1500}, \
    {.can_bus=3, .id=BENCH_ENABLE_F2 ? 3 : 0, .direction=-1, .kp=0.5f, .ki=0.1f, .current_limit=1500}, \
    {.can_bus=2, .id=BENCH_ENABLE_F3 ? 8 : 0, .direction=1, .kp=0.5f, .ki=0.1f, .current_limit=1500} \
}
