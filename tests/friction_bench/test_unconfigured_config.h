/* Explicit invalid fixture: production CAN IDs are now populated. */
#define FRICTION_MOTOR_CONFIGS { \
    {.can_bus = 2, .id = 0, .direction = 1, .kp = 0.5f, .ki = 0.1f, .current_limit = 1500}, \
    {.can_bus = 2, .id = 0, .direction = 1, .kp = 0.5f, .ki = 0.1f, .current_limit = 1500}, \
    {.can_bus = 2, .id = 0, .direction = 1, .kp = 0.5f, .ki = 0.1f, .current_limit = 1500} \
}
