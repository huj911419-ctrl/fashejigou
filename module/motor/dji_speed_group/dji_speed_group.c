#include "dji_speed_group.h"
#include "dji_motor.h"
#include <math.h>
#include <string.h>

#define DJI_DEGREES_PER_SECOND_PER_RPM 6.0f
#define C620_MAX_CURRENT_COMMAND 16384.0f

static FDCAN_HandleTypeDef *BusHandle(uint8_t bus)
{
    switch (bus)
    {
        case 1U: return &hfdcan1;
        case 2U: return &hfdcan2;
        case 3U: return &hfdcan3;
        default: return NULL;
    }
}

static bool ConfigValid(const DjiSpeedMotorConfig_s *config, unsigned count)
{
    if (config == NULL || count == 0U || count > DJI_SPEED_GROUP_CAPACITY)
    {
        return false;
    }
    for (unsigned i = 0U; i < count; ++i)
    {
        const DjiSpeedMotorConfig_s *motor = &config[i];
        if (BusHandle(motor->can_bus) == NULL || motor->id < 1U || motor->id > 8U ||
            (motor->direction != 1 && motor->direction != -1) ||
            !isfinite(motor->kp) || motor->kp < 0.0f || !isfinite(motor->ki) || motor->ki < 0.0f ||
            !isfinite(motor->current_limit) || motor->current_limit <= 0.0f ||
            motor->current_limit > C620_MAX_CURRENT_COMMAND)
        {
            return false;
        }
        for (unsigned j = 0U; j < i; ++j)
        {
            if (motor->can_bus == config[j].can_bus && motor->id == config[j].id)
            {
                return false;
            }
        }
    }
    return true;
}

bool DjiSpeedGroupInit(DjiSpeedGroup_s *group, const DjiSpeedMotorConfig_s *config, unsigned count)
{
    if (group == NULL)
    {
        return false;
    }
    memset(group, 0, sizeof(*group));
    if (!ConfigValid(config, count))
    {
        return false;
    }
    memcpy(group->config, config, count * sizeof(*config));
    for (unsigned i = 0U; i < count; ++i)
    {
        const DjiSpeedMotorConfig_s *cfg = &config[i];
        Motor_Init_Config_s init =
        {
            .motor_type = M3508,
            .can_init_config = {.can_handle = BusHandle(cfg->can_bus), .tx_id = cfg->id},
            .controller_setting_init_config =
            {
                .outer_loop_type = SPEED_LOOP,
                .close_loop_type = SPEED_LOOP,
                .angle_feedback_source = MOTOR_FEED,
                .speed_feedback_source = MOTOR_FEED,
                .motor_reverse_flag = MOTOR_DIRECTION_NORMAL
            },
            .controller_param_init_config.speed_PID =
            {
                .Kp = cfg->kp,
                .Ki = cfg->ki,
                .Kd = 0.0f,
                .MaxOut = cfg->current_limit,
                .IntegralLimit = cfg->current_limit,
                .Improve = PID_Integral_Limit | PID_Continuous
            }
        };
        group->motor[i] = DJIMotorInit(&init);
        if (group->motor[i] == NULL)
        {
            DjiSpeedGroupStop(group);
            return false;
        }
        ++group->count;
        DJIMotorStop(group->motor[i]);
    }
    group->initialized = true;
    return true;
}

void DjiSpeedGroupRead(const DjiSpeedGroup_s *group, unsigned index, DjiSpeedFeedback_s *feedback)
{
    memset(feedback, 0, sizeof(*feedback));
    if (index >= group->count)
    {
        return;
    }
    DJI_Motor_Feedback_s sample;
    DJIMotorGetFeedback(group->motor[index], &sample);
    feedback->rpm = sample.measure.speed_aps / DJI_DEGREES_PER_SECOND_PER_RPM * group->config[index].direction;
    feedback->current_raw = sample.measure.real_current;
    feedback->temperature_c = sample.measure.temperature;
    feedback->valid = sample.valid;
    feedback->last_rx_ms = sample.last_rx_ms;
}

void DjiSpeedGroupGetOutput(const DjiSpeedGroup_s *group, unsigned index, DjiSpeedOutput_s *output)
{
    memset(output, 0, sizeof(*output));
    if (index >= group->count)
    {
        return;
    }
    const DJIMotorInstance *motor = group->motor[index];
    const PIDInstance *pid = &motor->motor_controller.speed_PID;
    float sign = (float)group->config[index].direction;
    output->output = motor->stop_flag == MOTOR_STOP ? 0.0f : pid->Output * sign;
    output->p = pid->Pout * sign;
    output->i = pid->Iout * sign;
    output->d = pid->Dout * sign;
}

void DjiSpeedGroupTune(DjiSpeedGroup_s *group, const DjiSpeedTuning_s *tuning)
{
    for (unsigned i = 0U; i < group->count; ++i)
    {
        DJIMotorInstance *motor = group->motor[i];
        PIDInstance *pid = &motor->motor_controller.speed_PID;
        const DjiSpeedTuning_s *parameters = &tuning[i];
        pid->Kp = parameters->kp;
        pid->Ki = parameters->ki;
        pid->Kd = parameters->kd;
        pid->MaxOut = parameters->current_limit;
        pid->IntegralLimit = parameters->current_limit;
        /* Caller validates the complete transaction before applying. Keep I
         * bounded; Ki=0 clears it. Gain changes are not guaranteed bumpless. */
        pid->Iout = parameters->ki == 0.0f ? 0.0f :
                    fmaxf(-parameters->current_limit, fminf(parameters->current_limit, pid->Iout));
    }
}

void DjiSpeedGroupSet(DjiSpeedGroup_s *group, unsigned index, float rpm, bool enable)
{
    if (!group->initialized || index >= group->count)
    {
        return;
    }
    DJIMotorInstance *motor = group->motor[index];
    DJIMotorSetRef(motor, rpm * DJI_DEGREES_PER_SECOND_PER_RPM * group->config[index].direction);
    if (enable)
    {
        DJIMotorEnable(motor);
    }
    else
    {
        DJIMotorStop(motor);
    }
}

void DjiSpeedGroupStop(DjiSpeedGroup_s *group)
{
    for (unsigned i = 0U; i < group->count; ++i)
    {
        DJIMotorStop(group->motor[i]);
    }
}

bool DjiSpeedGroupControl(void)
{
    return DJIMotorControlGuarded() != 0U;
}
