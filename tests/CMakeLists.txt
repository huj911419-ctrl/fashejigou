cmake_minimum_required(VERSION 3.21)
project(robot_control_host_tests LANGUAGES C)
enable_testing()
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)
get_filename_component(REPO "${CMAKE_CURRENT_LIST_DIR}/.." ABSOLUTE)

# This library deliberately has NO vendor, HAL, CMSIS or RTOS include paths.
# A hardware dependency in a portable component must fail this build.
add_library(portable_control STATIC
    ${REPO}/module/control_authority/control_authority.c
    ${REPO}/application/robot_cmd/gimbal_arbiter.c
    ${REPO}/module/parameter_store/parameter_store.c
    ${REPO}/application/parameters/friction_profile.c
    ${REPO}/module/yaw_angle/yaw_angle.c
    ${REPO}/module/loop_stats/loop_stats.c
    ${REPO}/module/event_history/event_history.c
    ${REPO}/module/text_console/text_console.c
    ${REPO}/module/bmi088_sensor/bmi088_sensor.c
    ${REPO}/module/attitude/attitude.c
    ${REPO}/module/vofa/vofa.c
    ${REPO}/module/speed_ready/speed_ready.c
    ${REPO}/module/dbus_input/dbus_input.c
    ${REPO}/module/yaw_rate/yaw_rate.c
    ${REPO}/module/dm_velocity/dm_velocity.c
    ${REPO}/application/gimbal_control/gimbal_control.c
    ${REPO}/application/robot_cmd/gimbal_remote.c
    ${REPO}/application/robot_cmd/friction_remote.c
    ${REPO}/application/friction_bench/friction_control.c
    ${REPO}/application/friction_bench/friction_monitor.c)
target_sources(portable_control PRIVATE ${REPO}/application/friction_bench/friction_log_stats.c)
target_include_directories(portable_control PUBLIC ${REPO}/module/motor/dji_speed_group)
target_include_directories(portable_control PUBLIC
    ${REPO}/module/control_authority
    ${REPO}/module/parameter_store
    ${REPO}/application/parameters
    ${REPO}/module/yaw_angle
    ${REPO}/module/loop_stats
    ${REPO}/module/event_history
    ${REPO}/module/text_console
    ${REPO}/module/imu_sample
    ${REPO}/module/bmi088_sensor
    ${REPO}/module/attitude
    ${REPO}/module/vofa
    ${REPO}/module/speed_ready
    ${REPO}/module/dbus_input
    ${REPO}/module/yaw_rate
    ${REPO}/module/dm_velocity
    ${REPO}/application/gimbal_control
    ${REPO}/application/robot_cmd
    ${REPO}/application/friction_bench)

if(CMAKE_C_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(portable_control PRIVATE -Wall -Wextra -Werror -pedantic)
    target_link_libraries(portable_control PUBLIC m)
else()
    message(FATAL_ERROR "Use a host GCC/Clang compiler; embedded ARM GCC cannot run host tests")
endif()

function(control_test name)
    add_executable(${name} ${ARGN})
    target_link_libraries(${name} PRIVATE portable_control)
    target_compile_options(${name} PRIVATE -Wall -Wextra -Werror -pedantic -UNDEBUG)
    add_test(NAME ${name} COMMAND ${name})
endfunction()

control_test(vofa ${REPO}/tests/vofa/test_vofa.c)
control_test(control_authority ${REPO}/tests/control_authority/test_control_authority.c)
control_test(gimbal_arbiter ${REPO}/tests/control_authority/test_gimbal_arbiter.c)
control_test(parameter_store ${REPO}/tests/parameter_store/test_parameter_store.c)
control_test(friction_profile ${REPO}/tests/parameters/test_friction_profile.c)
control_test(profile_service ${REPO}/tests/parameters/test_profile_service.c
    ${REPO}/application/parameters/friction_profile_service.c)
target_include_directories(profile_service PRIVATE ${REPO}/module/motor/dji_speed_group ${REPO}/application)
control_test(runtime_history ${REPO}/tests/runtime/test_runtime.c)
control_test(yaw_angle ${REPO}/tests/yaw_angle/test_yaw_angle.c)
control_test(text_console ${REPO}/tests/diagnostics/test_text_console.c)
control_test(friction_log_stats ${REPO}/tests/diagnostics/test_friction_log_stats.c)
control_test(usb_console_port ${REPO}/tests/diagnostics/test_usb_console_port.c ${REPO}/bsp/usb_console_port/usb_console_port.c)
target_include_directories(usb_console_port PRIVATE ${REPO}/tests/diagnostics/stubs ${REPO}/bsp/usb_console_port)
control_test(diagnostics
    ${REPO}/tests/diagnostics/test_diagnostics.c
    ${REPO}/application/diagnostics/diagnostics.c
    ${REPO}/application/gimbal_bench/gimbal_diagnostics.c
    ${REPO}/application/friction_bench/friction_diagnostics.c)
target_include_directories(diagnostics PRIVATE
    ${REPO}/application/diagnostics ${REPO}/application/gimbal_bench
    ${REPO}/bsp/board_status ${REPO}/bsp/usb_console_port ${REPO}/bsp/gimbal_port
    ${REPO}/bsp/control_platform ${REPO}/module/motor/dji_speed_group)
control_test(friction_usb_log
    ${REPO}/tests/diagnostics/test_friction_usb_log.c
    ${REPO}/application/diagnostics/diagnostics.c
    ${REPO}/application/runtime/runtime_events.c
    ${REPO}/application/friction_bench/friction_diagnostics.c
    ${REPO}/application/friction_bench/friction_status_text.c
    ${REPO}/application/friction_bench/friction_usb_command.c
    ${REPO}/application/friction_bench/friction_usb_log.c)
target_include_directories(friction_usb_log PRIVATE
    ${REPO}/application/diagnostics ${REPO}/application/runtime
    ${REPO}/bsp/board_status ${REPO}/bsp/usb_console_port
    ${REPO}/bsp/control_platform ${REPO}/module/motor/dji_speed_group)
control_test(friction_usb_control
    ${REPO}/tests/friction_bench/test_usb_control.c
    ${REPO}/application/friction_bench/friction.c
    ${REPO}/application/friction_bench/friction_usb_command.c
    ${REPO}/application/runtime/runtime_events.c)
target_include_directories(friction_usb_control PRIVATE
    ${REPO}/application/runtime ${REPO}/bsp/control_platform ${REPO}/module/motor/dji_speed_group)
add_test(NAME friction_usb_startup_idle COMMAND friction_usb_control --idle)
add_test(NAME friction_usb_startup_wrap COMMAND friction_usb_control --wrap)
add_test(NAME friction_usb_startup_offline COMMAND friction_usb_control --offline)
control_test(friction_remote ${REPO}/tests/friction_remote/test_remote.c)
control_test(friction_remote_application ${REPO}/tests/friction_remote/test_application.c
    ${REPO}/application/friction_bench/friction.c
    ${REPO}/application/friction_bench/friction_remote_bench.c
    ${REPO}/application/friction_bench/friction_usb_command.c
    ${REPO}/application/runtime/runtime_events.c)
target_include_directories(friction_remote_application PRIVATE
    ${REPO}/application/runtime ${REPO}/bsp/control_platform ${REPO}/bsp/friction_remote_port)
add_test(NAME friction_remote_application_wrap COMMAND friction_remote_application --wrap)
control_test(friction_remote_port ${REPO}/tests/friction_remote/test_port.c
    ${REPO}/bsp/friction_remote_port/friction_remote_port.c)
target_include_directories(friction_remote_port PRIVATE
    ${REPO}/tests/gimbal_port/stubs ${REPO}/bsp/friction_remote_port)
foreach(name diagnostics friction_usb_log)
    target_sources(${name} PRIVATE ${REPO}/tests/diagnostics/friction_remote_stub.c)
    target_include_directories(${name} PRIVATE ${REPO}/bsp/friction_remote_port)
endforeach()
control_test(speed_ready ${REPO}/tests/speed_ready/test_speed_ready.c)
control_test(gimbal_core ${REPO}/tests/gimbal/test_gimbal_core.c)
control_test(gimbal_instances ${REPO}/tests/gimbal/test_gimbal_instances.c)
control_test(gimbal_commands ${REPO}/tests/gimbal/test_gimbal_commands.c)
control_test(attitude ${REPO}/tests/imu/test_attitude.c)
control_test(bmi088_sensor ${REPO}/tests/imu/test_bmi088_sensor.c ${REPO}/tests/imu/fake_bmi088.c)
control_test(imu_port ${REPO}/tests/imu/test_imu_port.c ${REPO}/bsp/imu_port/imu_port.c)
target_include_directories(imu_port PRIVATE ${REPO}/tests/imu/stubs ${REPO}/bsp/imu_port)
control_test(gimbal_imu
    ${REPO}/tests/imu/test_gimbal_imu.c
    ${REPO}/tests/imu/fake_bmi088.c
    ${REPO}/application/gimbal_bench/gimbal_imu.c)
target_include_directories(gimbal_imu PRIVATE
    ${REPO}/application/gimbal_bench ${REPO}/bsp/imu_port ${REPO}/bsp/control_platform)
control_test(control_stress ${REPO}/tests/robustness/test_control_stress.c)
add_test(NAME control_stress_seed_20260929 COMMAND control_stress 20260929)
add_test(NAME control_stress_seed_max COMMAND control_stress 4294967295)
control_test(gimbal_application
    ${REPO}/tests/gimbal/test_gimbal_application.c
    ${REPO}/application/gimbal_bench/gimbal.c)
foreach(name gimbal_core gimbal_instances gimbal_application gimbal_commands)
    target_include_directories(${name} PRIVATE ${REPO}/application/gimbal_bench)
endforeach()
target_include_directories(gimbal_application PRIVATE ${REPO}/bsp/gimbal_port ${REPO}/bsp/control_platform)

control_test(gimbal_port
    ${REPO}/tests/gimbal_port/test_gimbal_port.c
    ${REPO}/bsp/gimbal_port/gimbal_port.c)
target_include_directories(gimbal_port PRIVATE
    ${REPO}/tests/gimbal_port/stubs ${REPO}/bsp/gimbal_port)
add_test(NAME gimbal_port_init_failure COMMAND gimbal_port init-failure)

add_executable(pid_saturation
    ${REPO}/tests/friction_bench/test_pid_saturation.c
    ${REPO}/module/algorithm/controller.c)
target_include_directories(pid_saturation PRIVATE
    ${REPO}/tests/friction_bench/stubs ${REPO}/module/algorithm)
target_compile_options(pid_saturation PRIVATE -Wall -Wextra -Werror -UNDEBUG)
target_link_libraries(pid_saturation PRIVATE m)
add_test(NAME pid_saturation COMMAND pid_saturation)

foreach(variant configured unconfigured)
    set(name friction_${variant})
    add_executable(${name}
        ${REPO}/tests/friction_bench/test_friction.c
        ${REPO}/application/friction_bench/friction.c
        ${REPO}/application/friction_bench/friction_usb_command.c
        ${REPO}/application/friction_bench/friction_debug.c
        ${REPO}/application/parameters/friction_profile_service.c
        ${REPO}/module/motor/dji_speed_group/dji_speed_group.c
        ${REPO}/module/motor/DJImotor/dji_motor.c
        ${REPO}/module/algorithm/controller.c
        ${REPO}/bsp/control_platform/control_platform.c)
    target_link_libraries(${name} PRIVATE portable_control)
    target_include_directories(${name} PRIVATE
        ${REPO}/tests/friction_bench/stubs
        ${REPO}/module/motor/dji_speed_group
        ${REPO}/module/motor/DJImotor
        ${REPO}/module/motor
        ${REPO}/module/algorithm
        ${REPO}/bsp/control_platform)
    # Legacy driver has known unused callbacks and GNU binary-literal enums.
    target_compile_options(${name} PRIVATE -Wall -Wextra -Wno-unused-function -Wno-unused-variable -UNDEBUG)
    if(variant STREQUAL "configured")
        target_compile_options(${name} PRIVATE -include ${REPO}/tests/friction_bench/test_config.h)
    else()
        target_compile_definitions(${name} PRIVATE TEST_UNCONFIGURED)
        target_compile_options(${name} PRIVATE -include ${REPO}/tests/friction_bench/test_unconfigured_config.h)
    endif()
    add_test(NAME ${name} COMMAND ${name})
endforeach()

foreach(name diagnostics gimbal_application gimbal_imu friction_configured friction_unconfigured runtime_history)
    target_sources(${name} PRIVATE ${REPO}/application/runtime/runtime_events.c)
    target_include_directories(${name} PRIVATE ${REPO}/application/runtime ${REPO}/bsp/control_platform)
endforeach()

find_package(Python3 REQUIRED COMPONENTS Interpreter)
add_test(NAME profile_client COMMAND ${Python3_EXECUTABLE} ${REPO}/tests/parameters/test_profiles.py)
add_test(NAME architecture_boundaries COMMAND ${Python3_EXECUTABLE} ${REPO}/tests/check_architecture.py)

# Build the same fixtures under isolated feature selections. Production control
# algorithms keep their dependencies; switches belong to application composition.
function(feature_test name base)
    get_target_property(sources ${base} SOURCES)
    get_target_property(includes ${base} INCLUDE_DIRECTORIES)
    get_target_property(options ${base} COMPILE_OPTIONS)
    add_executable(${name} ${sources})
    target_link_libraries(${name} PRIVATE portable_control)
    target_include_directories(${name} PRIVATE ${includes})
    target_compile_options(${name} PRIVATE ${options})
    target_compile_definitions(${name} PRIVATE ${ARGN})
    add_test(NAME ${name} COMMAND ${name})
endfunction()

feature_test(switch_remote_only gimbal_application BENCH_ENABLE_YAW=0)
feature_test(switch_friction_remote_off friction_remote_application BENCH_ENABLE_FRICTION_REMOTE=0)
feature_test(switch_friction_usb_default friction_usb_control BENCH_ENABLE_FRICTION_REMOTE=0)
feature_test(switch_friction_remote_f2 friction_remote_application BENCH_ENABLE_F1=0 BENCH_ENABLE_F3=0)
feature_test(switch_friction_remote_group_off friction_remote_application BENCH_ENABLE_FRICTION=0)
feature_test(switch_friction_remote_usb_off friction_remote_application BENCH_ENABLE_USB=0)
feature_test(switch_authority_off gimbal_application BENCH_ENABLE_COMMAND_AUTHORITY=0)
feature_test(switch_yaw_only gimbal_application BENCH_ENABLE_REMOTE=0)
feature_test(switch_gimbal_off gimbal_application BENCH_ENABLE_REMOTE=0 BENCH_ENABLE_YAW=0)
feature_test(switch_imu_raw gimbal_imu BENCH_ENABLE_ATTITUDE=0)
feature_test(switch_imu_off gimbal_imu BENCH_ENABLE_IMU=0)
feature_test(switch_f1 friction_configured BENCH_ENABLE_F2=0 BENCH_ENABLE_F3=0)
feature_test(switch_parameters_off friction_configured BENCH_ENABLE_PARAMETERS=0)
feature_test(switch_rtt_off friction_configured BENCH_ENABLE_RTT_COMMAND=0)
feature_test(switch_commands_off friction_configured BENCH_ENABLE_RTT_COMMAND=0 BENCH_ENABLE_VOFA=0)
feature_test(switch_f2 friction_configured BENCH_ENABLE_F1=0 BENCH_ENABLE_F3=0)
feature_test(switch_f3 friction_configured BENCH_ENABLE_F1=0 BENCH_ENABLE_F2=0)
feature_test(switch_friction_off friction_configured BENCH_ENABLE_FRICTION=0)
feature_test(switch_wheels_off friction_configured BENCH_ENABLE_F1=0 BENCH_ENABLE_F2=0 BENCH_ENABLE_F3=0)
feature_test(switch_friction_debug_off friction_configured BENCH_ENABLE_VOFA=0 BENCH_ENABLE_SPEED_READY=0)
feature_test(switch_disabled_reports diagnostics TEST_MODULE_SWITCHES
    BENCH_ENABLE_YAW=0 BENCH_ENABLE_REMOTE=0 BENCH_ENABLE_IMU=0
    BENCH_ENABLE_FRICTION=0 BENCH_ENABLE_EVENT_HISTORY=0 BENCH_ENABLE_BOARD_DIAGNOSTICS=0)
feature_test(switch_usb_off diagnostics TEST_MODULE_SWITCHES BENCH_ENABLE_USB=0 BENCH_ENABLE_EVENT_HISTORY=0)
feature_test(switch_usb_log_default2 friction_usb_log BENCH_FRICTION_USB_LOG_MODE=2)
foreach(selection remote-only can-only off)
    add_test(NAME switch_port_${selection} COMMAND gimbal_port ${selection})
endforeach()
