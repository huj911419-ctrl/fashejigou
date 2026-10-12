#ifndef DJI_SPEED_GROUP_H
#define DJI_SPEED_GROUP_H

#include <stdbool.h>
#include <stdint.h>

#define DJI_SPEED_GROUP_CAPACITY 3U
/* Logged independently of tunable gains to distinguish control behavior. */
#define DJI_SPEED_GROUP_CONTROL_POLICY "speed_pid_conditional_v1"

typedef struct
{
    float kp;                 /* Error unit: motor degrees/s, as legacy PID. */
    float ki;                 /* Integral uses seconds. */
    float kd;
    float current_limit;      /* C620 command units, NOT amperes. */
} DjiSpeedTuning_s;

typedef struct
{
    uint8_t can_bus;           /* Board adapter bus index, currently 1..3. */
    uint8_t id;                /* C620 ID 1..8. Zero means unconfigured. */
    int8_t direction;          /* Exactly +1 or -1. */
    float kp;
    float ki;
    float current_limit;
} DjiSpeedMotorConfig_s;

typedef struct
{
    float rpm;                /* Logical motor-side RPM. */
    int16_t current_raw;       /* Physical direction, raw feedback units. */
    uint8_t temperature_c;
    uint8_t valid;
    uint32_t last_rx_ms;
} DjiSpeedFeedback_s;

typedef struct
{
    float output;
    float p;
    float i;
    float d;                  /* All normalized to logical direction. */
} DjiSpeedOutput_s;

typedef struct
{
    void *motor[DJI_SPEED_GROUP_CAPACITY]; /* Private driver handles; never dereference in app. */
    DjiSpeedMotorConfig_s config[DJI_SPEED_GROUP_CAPACITY];
    unsigned count;
    bool initialized;
} DjiSpeedGroup_s;

/** M3508/C620 adapter with a HAL-free public contract. Caller owns the group.
 * Init once before scheduler, copies config; outputs stay stopped. Do not copy
 * or reinitialize a registered group. Allocation remains in legacy DJI driver.
 * Runtime functions have one control-task owner. PID/units/driver internals
 * stay here; the application never reaches inside DJIMotorInstance.
 */
bool DjiSpeedGroupInit(DjiSpeedGroup_s *group, const DjiSpeedMotorConfig_s *config, unsigned count);
void DjiSpeedGroupRead(const DjiSpeedGroup_s *group, unsigned motor, DjiSpeedFeedback_s *feedback);
void DjiSpeedGroupGetOutput(const DjiSpeedGroup_s *group, unsigned motor, DjiSpeedOutput_s *output);
/** tuning has group->count entries, validated by the application as ONE
 * transaction: finite nonnegative gains, current_limit in (0, config limit].
 * All runtime APIs require non-null group/output storage. */
void DjiSpeedGroupTune(DjiSpeedGroup_s *group, const DjiSpeedTuning_s *tuning);
void DjiSpeedGroupSet(DjiSpeedGroup_s *group, unsigned motor, float rpm, bool enable);
void DjiSpeedGroupStop(DjiSpeedGroup_s *group);
/** Flushes ALL registered DJI motors through the legacy grouped sender.
 * Must be called by ONE system motor-task owner, not once per subsystem.
 * No waiting for CAN; false means the application must latch a driver fault.
 */
bool DjiSpeedGroupControl(void);

#endif
