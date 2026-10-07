# H723 composition of the portable subsystem; dedicated CAN/UART callbacks.
C_DEFS += -DROBOT_GIMBAL_BENCH
C_SOURCES := $(filter-out application/% module/% bsp/%,$(C_SOURCES))
C_SOURCES += \
    bsp/dwt/bsp_dwt.c \
    bsp/log/bsp_log.c \
    bsp/control_platform/control_platform.c \
    bsp/gimbal_port/gimbal_port.c \
    bsp/imu_port/imu_port.c \
    module/bmi088_sensor/bmi088_sensor.c \
    module/attitude/attitude.c \
    module/dbus_input/dbus_input.c \
    module/yaw_rate/yaw_rate.c \
    module/control_authority/control_authority.c \
    module/yaw_angle/yaw_angle.c \
    module/dm_velocity/dm_velocity.c \
    application/gimbal_control/gimbal_control.c \
    application/robot_cmd/gimbal_remote.c \
    application/robot_cmd/gimbal_arbiter.c \
    application/gimbal_bench/gimbal.c \
    application/gimbal_bench/gimbal_imu.c \
    application/gimbal_bench/gimbal_diagnostics.c
C_INCLUDES += \
    -Ibsp/control_platform \
    -Ibsp/gimbal_port \
    -Ibsp/imu_port \
    -Imodule/imu_sample \
    -Imodule/bmi088_sensor \
    -Imodule/attitude \
    -Imodule/dbus_input \
    -Imodule/yaw_rate \
    -Imodule/control_authority \
    -Imodule/yaw_angle \
    -Imodule/dm_velocity \
    -Iapplication/gimbal_control \
    -Iapplication/robot_cmd
C_INCLUDES := $(filter-out -Iapplication/chassis -Iapplication/cmd -Iapplication/gimbal -Iapplication/shoot,$(C_INCLUDES))
include application/diagnostics/build.mk
