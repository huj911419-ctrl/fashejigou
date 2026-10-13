# H723 composition of the portable subsystem; dedicated CAN/UART callbacks.
C_DEFS += -DROBOT_GIMBAL_BENCH
C_SOURCES := $(filter-out application/% module/% bsp/%,$(C_SOURCES))
C_SOURCES += \
    bsp/peripherals/dwt/bsp_dwt.c \
    bsp/peripherals/log/bsp_log.c \
    bsp/ports/control_platform/control_platform.c \
    bsp/ports/gimbal_port/gimbal_port.c \
    bsp/ports/imu_port/imu_port.c \
    module/sensor/bmi088_sensor/bmi088_sensor.c \
    module/sensor/attitude/attitude.c \
    module/protocol/dbus_input/dbus_input.c \
    module/control/yaw_rate/yaw_rate.c \
    module/control/control_authority/control_authority.c \
    module/control/yaw_angle/yaw_angle.c \
    module/protocol/dm_velocity/dm_velocity.c \
    application/gimbal/control/gimbal_control.c \
    application/commands/gimbal_remote.c \
    application/commands/gimbal_arbiter.c \
    application/gimbal/gimbal.c \
    application/gimbal/imu/gimbal_imu.c \
    application/gimbal/diagnostics/gimbal_diagnostics.c
C_INCLUDES += \
    -Ibsp/ports/control_platform \
    -Ibsp/ports/gimbal_port \
    -Ibsp/ports/imu_port \
    -Imodule/sensor/imu_sample \
    -Imodule/sensor/bmi088_sensor \
    -Imodule/sensor/attitude \
    -Imodule/protocol/dbus_input \
    -Imodule/control/yaw_rate \
    -Imodule/control/control_authority \
    -Imodule/control/yaw_angle \
    -Imodule/protocol/dm_velocity \
    -Iapplication/gimbal/control \
    -Iapplication/commands
C_INCLUDES := $(filter-out -Iapplication/legacy/chassis -Iapplication/legacy/cmd -Iapplication/legacy/gimbal -Iapplication/legacy/shoot,$(C_INCLUDES))
include application/services/diagnostics/build.mk

# Feature-local headers; no headers from the other application.
C_INCLUDES += -Iapplication/gimbal -Iapplication/gimbal/config -Iapplication/gimbal/control -Iapplication/gimbal/diagnostics -Iapplication/gimbal/imu
