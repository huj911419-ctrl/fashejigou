# The bench owns this source manifest; no sentry/infantry business is linked.
C_DEFS += -DROBOT_FRICTION_BENCH
C_SOURCES := $(filter-out application/% module/%,$(C_SOURCES))
C_SOURCES += \
    bsp/friction_remote_port/friction_remote_port.c \
    module/dbus_input/dbus_input.c \
    application/robot_cmd/friction_remote.c \
    application/friction_bench/friction_remote_bench.c \
    bsp/control_platform/control_platform.c \
    module/motor/DJImotor/dji_motor.c \
    module/motor/dji_speed_group/dji_speed_group.c \
    module/algorithm/controller.c \
    module/speed_ready/speed_ready.c \
    module/vofa/vofa.c \
    module/parameter_store/parameter_store.c \
    application/parameters/friction_profile.c \
    application/parameters/friction_profile_service.c \
    application/friction_bench/friction.c \
    application/friction_bench/friction_control.c \
    application/friction_bench/friction_monitor.c \
    application/friction_bench/friction_debug.c \
    application/friction_bench/friction_debug_port.c \
    application/friction_bench/friction_diagnostics.c \
    application/friction_bench/friction_status_text.c \
    application/friction_bench/friction_usb_command.c \
    application/friction_bench/friction_log_stats.c \
    application/friction_bench/friction_usb_log.c
C_INCLUDES += -Ibsp/control_platform -Imodule/motor/dji_speed_group -Imodule/speed_ready -Imodule/vofa
C_INCLUDES += -Imodule/parameter_store -Iapplication/parameters
C_INCLUDES += -Ibsp/friction_remote_port -Imodule/dbus_input -Iapplication/robot_cmd
C_INCLUDES := $(filter-out -Iapplication/chassis -Iapplication/cmd -Iapplication/gimbal -Iapplication/shoot,$(C_INCLUDES))
include application/diagnostics/build.mk
