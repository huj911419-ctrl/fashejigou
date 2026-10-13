"""Check the maintained portable subset, not generated/vendor/legacy firmware.

The CMake host library also compiles this subset without HAL or RTOS headers.
This check catches forbidden dependencies even if someone adds an include path
to make such a dependency compile. Run from any current working directory.
"""
from pathlib import Path
import re
import sys

ROOT = Path(__file__).resolve().parents[1]
PURE_DIRS = (
    "module/control/control_authority",
    "module/system/parameter_store",
    "module/protocol/vofa", "module/control/speed_ready", "module/protocol/dbus_input",
    "module/control/yaw_rate", "module/protocol/dm_velocity", "application/gimbal/control",
    "module/sensor/imu_sample", "module/sensor/bmi088_sensor", "module/sensor/attitude",
    "module/protocol/text_console",
    "module/system/loop_stats", "module/system/event_history", "module/control/yaw_angle",
    "application/commands",
)
PURE_FILES = (
    "application/services/parameters/friction_profile.c",
    "application/services/parameters/friction_profile.h",
    "application/friction/control/friction_control.c",
    "application/friction/control/friction_control.h",
    "application/friction/control/friction_monitor.c",
    "application/friction/control/friction_monitor.h",
    "application/friction/diagnostics/friction_log_stats.c",
    "application/friction/diagnostics/friction_log_stats.h",
)
ADAPTER_FREE_BUSINESS = (
    "application/services/parameters/friction_profile_service.c",
    "application/services/parameters/friction_profile_service.h",
    "application/services/runtime/runtime_events.c",
    "application/services/runtime/runtime_events.h",
    "application/friction/friction.c",
    "application/friction/friction.h",
    "application/friction/input/friction_usb_command.c",
    "application/friction/input/friction_usb_command.h",
    "application/friction/diagnostics/friction_usb_log.c",
    "application/friction/diagnostics/friction_usb_log.h",
    "application/friction/input/friction_remote_bench.c",
    "application/friction/input/friction_remote_bench.h",
    "application/gimbal/gimbal.c",
    "application/gimbal/gimbal.h",
    "application/gimbal/imu/gimbal_imu.c",
    "application/gimbal/imu/gimbal_imu.h",
)
FORBIDDEN_HEADERS = re.compile(
    r"^(?:main\.h|stm32.*|FreeRTOS\.h|task\.h|cmsis.*|arm_math\.h|"
    r"fdcan\.h|usart\.h|spi\.h|bsp_dwt\.h|dji_motor\.h|controller\.h)$"
)
FORBIDDEN_SYMBOLS = re.compile(r"\b(?:HAL_\w+|__get_PRIMASK|__disable_irq|__set_PRIMASK|"
                               r"taskENTER_CRITICAL|taskEXIT_CRITICAL|PIDInstance|DJIMotorInstance)\b")


def code_text(path):
    text = path.read_text(encoding="utf-8-sig")
    # These sources have no comment delimiters inside string literals.
    return re.sub(r"/\*.*?\*/|//[^\n]*", lambda match: "\n" * match[0].count("\n"), text, flags=re.S)


def main():
    pure = {ROOT / name for name in PURE_FILES}
    for directory in PURE_DIRS:
        pure.update(p for p in (ROOT / directory).glob("*.[ch]"))
    files = pure | {ROOT / name for name in ADAPTER_FREE_BUSINESS}
    errors = []
    for path in sorted(files):
        text = code_text(path)
        label = path.relative_to(ROOT).as_posix()
        for line, source in enumerate(text.splitlines(), 1):
            include = re.search(r'#\s*include\s*[<"]([^>"]+)', source)
            if include and FORBIDDEN_HEADERS.match(include[1].split("/")[-1]):
                errors.append(f"{label}:{line}: forbidden hardware header {include[1]}")
            if FORBIDDEN_SYMBOLS.search(source):
                errors.append(f"{label}:{line}: hardware/driver internals in control business")
            if path in pure and include:
                header = include[1].split("/")[-1]
                if label.startswith("application/gimbal/control/") and header in (
                    "dbus_input.h", "gimbal_remote.h", "remote_control.h", "robot_cmd.h",
                    "vofa.h", "gimbal.h",
                ):
                    errors.append(f"{label}:{line}: command-source dependency in actuator core")
                if label.startswith("application/commands/") and header in (
                    "gimbal_control.h", "dm_velocity.h", "yaw_rate.h", "gimbal.h",
                    "dji_speed_group.h", "runtime_events.h", "friction.h", "friction_control.h",
                ):
                    errors.append(f"{label}:{line}: actuator/runtime dependency in command producer")
                if header.startswith("bsp_") or header in ("control_platform.h", "gimbal_port.h", "imu_port.h"):
                    errors.append(f"{label}:{line}: platform dependency in portable core")
    if errors:
        print("\n".join(errors), file=sys.stderr)
        return 1
    print(f"Architecture boundaries passed for {len(files)} maintained C sources/headers")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
