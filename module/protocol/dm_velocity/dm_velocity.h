#ifndef DM_VELOCITY_H
#define DM_VELOCITY_H

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
    uint16_t id;
    uint8_t length;
    uint8_t data[8];
    uint32_t received_ms;
} DmCanFrame_s;

/** Copy/enqueue the entire frame before returning true; false means rejected.
 * Must not wait for CAN space. Context and buffers belong to the caller.
 */
typedef bool (*DmVelocitySend_f)(void *context, const DmCanFrame_s *frame);

typedef enum
{
    DM_VELOCITY_BOOT,
    DM_VELOCITY_DISCOVER,
    DM_VELOCITY_STOPPING,
    DM_VELOCITY_SET_MODE,
    DM_VELOCITY_CHECK_MODE,
    DM_VELOCITY_READY,
    DM_VELOCITY_STARTING,
    DM_VELOCITY_RUNNING,
    DM_VELOCITY_FAULT
} DmVelocityState_e;

typedef enum
{
    DM_VELOCITY_FAULT_NONE = 0,
    DM_VELOCITY_FAULT_CONFIG = 1,
    DM_VELOCITY_FAULT_TX = 2,
    DM_VELOCITY_FAULT_TIMEOUT = 4,
    DM_VELOCITY_FAULT_MOTOR = 8,
    DM_VELOCITY_FAULT_COMMAND = 16
} DmVelocityFault_e;

typedef struct
{
    uint16_t esc_id;              /* 0 is valid; speed ID = 0x200 + esc_id. */
    float max_rate_rad_s;
    float position_range_rad;    /* Must match motor firmware PMAX. */
    float velocity_range_rad_s;  /* Must match motor firmware VMAX. */
    float torque_range_nm;       /* Must match motor firmware TMAX. */
    uint8_t max_temperature_c;
    uint32_t power_up_ms;
    uint32_t setup_timeout_ms;
    uint32_t feedback_timeout_ms;
    uint32_t command_period_ms;
} DmVelocityConfig_s;

typedef struct
{
    float position_rad;          /* Raw protocol range, NOT an unwrapped angle. */
    float velocity_rad_s;
    float torque_nm;
    uint8_t motor_state;         /* 0=off, 1=on; all others block operation. */
    uint8_t mos_temperature_c;
    uint8_t coil_temperature_c;
    uint32_t received_ms;
    uint32_t count;
    bool valid;
} DmVelocityFeedback_s;

typedef struct
{
    DmVelocityConfig_s config;
    DmVelocitySend_f send;
    void *context;
    DmVelocityState_e state;
    uint32_t fault;
    uint16_t master_id;
    uint32_t boot_ms;
    uint32_t stage_ms;
    uint32_t last_tx_ms;
    bool sent;
    bool waiting_reply;
    DmVelocityFeedback_s feedback;
} DmVelocity_s;

/** Pure single-owner driver. Call Init/Receive/Step from the same control task.
 * No memory allocation, HAL, RTOS, blocking delays or parameter flash writes.
 * Init starts disabled; discovers MASTER_ID, confirms OFF and mode 3 before
 * becoming ready. Runtime faults latch until an explicit reinitialization.
 */
bool DmVelocityInit(DmVelocity_s *motor, const DmVelocityConfig_s *config,
                    DmVelocitySend_f send, void *context, uint32_t now_ms);
void DmVelocityReceive(DmVelocity_s *motor, const DmCanFrame_s *frame);
/** Caller renews desired command every control cycle; transport loss must call
 * with enable=false. This API does not replace the caller's input watchdog.
 */
void DmVelocityStep(DmVelocity_s *motor, uint32_t now_ms, bool enable, float rate_rad_s);
bool DmVelocityAvailable(const DmVelocity_s *motor, uint32_t now_ms);

#endif
