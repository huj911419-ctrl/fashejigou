# The bench owns this source manifest; no sentry/infantry business is linked.
C_DEFS += -DROBOT_FRICTION_BENCH
C_SOURCES := $(filter-out application/% module/%,$(C_SOURCES))
C_SOURCES += \
    bsp/ports/friction_remote_port/friction_remote_port.c \
    module/protocol/dbus_input/dbus_input.c \
    application/commands/friction_remote.c \
    application/friction/input/friction_remote_bench.c \
    bsp/ports/control_platform/control_platform.c \
    module/motor/dji/dji_motor.c \
    module/motor/dji_speed_group/dji_speed_group.c \
    module/algorithm/controller.c \
    module/control/speed_ready/speed_ready.c \
    module/protocol/vofa/vofa.c \
    module/system/parameter_store/parameter_store.c \
    application/services/parameters/friction_profile.c \
    application/services/parameters/friction_profile_service.c \
    application/friction/friction.c \
    application/friction/control/friction_control.c \
    application/friction/control/friction_monitor.c \
    application/friction/input/friction_debug.c \
    application/friction/input/friction_debug_port.c \
    application/friction/diagnostics/friction_diagnostics.c \
    application/friction/diagnostics/friction_status_text.c \
    application/friction/input/friction_usb_command.c \
    application/friction/diagnostics/friction_log_stats.c \
    application/friction/diagnostics/friction_usb_log.c
C_INCLUDES += -Ibsp/ports/control_platform -Imodule/motor/dji_speed_group -Imodule/control/speed_ready -Imodule/protocol/vofa
C_INCLUDES += -Imodule/system/parameter_store -Iapplication/services/parameters
C_INCLUDES += -Ibsp/ports/friction_remote_port -Imodule/protocol/dbus_input -Iapplication/commands
C_INCLUDES := $(filter-out -Iapplication/legacy/chassis -Iapplication/legacy/cmd -Iapplication/legacy/gimbal -Iapplication/legacy/shoot,$(C_INCLUDES))
include application/services/diagnostics/build.mk

# Feature-local headers; no headers from the other application.
C_INCLUDES += -Iapplication/friction -Iapplication/friction/config -Iapplication/friction/control -Iapplication/friction/diagnostics -Iapplication/friction/input
