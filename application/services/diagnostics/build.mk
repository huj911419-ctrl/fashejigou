# Optional -D overrides for isolated verification builds; use a separate BUILD_DIR.
C_DEFS += $(BENCH_DEFINES)

# Included only by the independent bench manifests. No legacy application changes.
C_SOURCES := $(filter-out bsp/ports/usb_console_port/% bsp/board/%,$(C_SOURCES))
C_SOURCES += \
    module/system/loop_stats/loop_stats.c \
    module/system/event_history/event_history.c \
    application/services/runtime/runtime_events.c \
    module/protocol/text_console/text_console.c \
    bsp/ports/usb_console_port/usb_console_port.c \
    bsp/board/board_status.c \
    application/services/diagnostics/diagnostics.c \
    application/services/diagnostics/diagnostics_task.c
C_INCLUDES += -Imodule/protocol/text_console -Ibsp/ports/usb_console_port -Ibsp/board -Iapplication/services/diagnostics
C_INCLUDES += -Imodule/system/loop_stats -Imodule/system/event_history -Iapplication/services/runtime
