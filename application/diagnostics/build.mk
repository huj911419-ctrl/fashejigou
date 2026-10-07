# Optional -D overrides for isolated verification builds; use a separate BUILD_DIR.
C_DEFS += $(BENCH_DEFINES)

# Included only by the independent bench manifests. No legacy application changes.
C_SOURCES := $(filter-out bsp/usb_console_port/% bsp/board_status/%,$(C_SOURCES))
C_SOURCES += \
    module/loop_stats/loop_stats.c \
    module/event_history/event_history.c \
    application/runtime/runtime_events.c \
    module/text_console/text_console.c \
    bsp/usb_console_port/usb_console_port.c \
    bsp/board_status/board_status.c \
    application/diagnostics/diagnostics.c \
    application/diagnostics/diagnostics_task.c
C_INCLUDES += -Imodule/text_console -Ibsp/usb_console_port -Ibsp/board_status -Iapplication/diagnostics
C_INCLUDES += -Imodule/loop_stats -Imodule/event_history -Iapplication/runtime
