# Repository Guidelines

## 长期目标与架构约束（用户要求）

最终目标是用本框架实现 RoboMaster 云台控制和摩擦轮弹丸发射控制。
当前三电机台架、VOFA 波形及在线调参只是支撑该目标的开发工具，不是最终产品范围。
后续涉及控制、调试或框架调整时，均需遵循以下要求：

- 当前摩擦轮机构为 120° 布置的三个 DJI 电机驱动轮、中心进弹；电机 ID 和调试串口由用户确定，不擅自绑定。
- 当前已确认的 360° 水平轴是 DM-J4310。独立 `gimbal_bench` 使用参考工程的 CAN1/ESC_ID=0x000 速度模式和 UART5 DBUS；具体分层、测试与扩展边界见 `application/gimbal_bench/README.md`。不要把速度控制描述成已实现角度保持或 IMU 稳定控制。
- 依据实际开源源码的分层调整见 `docs/architecture/open-source-review.md`。`application/robot_cmd/gimbal_remote` 独立负责 DBUS、通道/开关、方向和死区；`gimbal_control` 仅消费带原始时间戳的 `GimbalCommand_s`（电机侧 rad/s），不得重新引入遥控协议依赖。`YawRateStep` 现收 rad/s，状态快照分 `axis`/`remote`。保持同批停机不丢失、旧命令不续期、单控制任务所有者；多来源仲裁尚未实现。
- 用户确认控制板随云台一起转动。板载 BMI088 的独立采集/姿态链见 `application/gimbal_bench/IMU.md`；轴向安装尚未确认，默认不可直接用于控制。纯 C 核心在 `module/bmi088_sensor`、`module/attitude` 和 `module/imu_sample`，SPI2 仅由 IMU 任务拥有；不与旧 INS/BMI088 同时驱动硬件。保持相对偏航、校准状态、过期和安装确认语义，不宣称六轴 IMU 提供无漂移绝对航向。
- 用户的云台/发射应用与现有哨兵应用保持独立入口、配置和任务组织。可以复用明确的底层接口，但不得引入对哨兵业务状态或任务的依赖。
- 两套 bench 已分别接入原生 USB CDC 文本诊断，见 `application/diagnostics/README.md`。`module/text_console` 为可移植核心，`bsp/usb_console_port` 独占 CDC 回调和缓冲；必须在 Cube USB 初始化前绑定，不能混用旧 USBInit/CDC_Transmit_HS 或 VOFA 二进制输出。诊断只读应用快照，不能借查询刷新控制租约、使能电机或清故障。区分外设初始化、有效反馈、过期、未配置和未监测；GPIO 电源指令不等于实测供电。
- 按职责分层：`bsp/` 管硬件与传输适配，`module/` 管可复用算法、驱动和协议，`application/` 管云台/摩擦轮业务及任务编排。通用算法和协议核心不依赖具体板卡、UART 编号或某个机器人应用。
- 运行准备见 `application/runtime/README.md`：两套 bench 已接入状态变化历史（USB `events`，RAM 最近 32 条）及调用间隔统计；这些是只读观察，不改变控制权或恢复条件。`module/yaw_angle` 是已测试的连续角度→角速度 PI 核心，尚未接入电机链；组合前必须确认坐标、方向、零点参考编号及使能条件，不把核心测试通过说成已完成实机角度保持。
- 第二轮开源对照见 `docs/architecture/open-source-review-20261002.md`。Yaw 的 `inhibit` 是最近控制决定的只读原因位，已接 USB/事件历史；0 只代表应用授权，不能当电机 ON。新观测不能改变输出或恢复条件；当前参数仍只存 RAM。
- 独立模块开关统一在 `application/bench_features.h`，见 `docs/architecture/module-switches.md`。开关控制组合/资源启动，不进入纯算法；被关闭模块诊断为 DISABLED。遥控/Yaw 独立选择，IMU/姿态分开；关闭轮不注册且不要求 ID，F1/F2/F3 编号固定，选择关闭轮的运行请求拒绝。全关仍保留 BenchService 喂 IWDG；USB=0 不初始化 CDC。改动后验证隔离配置，不只编译默认全开。
- 参数管理见 `application/parameters/README.md`：摩擦轮完整配置包有版本/单位/CRC，暂存后在停用状态由电机任务原子应用到 RAM，每次应用增加完整修订号；QUEUED 不等于 APPLIED。电脑工具只通过待绑定 VOFA 串口操作，USB 仍只读。`BENCH_ENABLE_PARAMETERS` 单独关闭参数包，不关闭旧逐项调参。`module/parameter_store` 是实例化双槽字节存储核心，真实 Flash 和启动加载未绑定，不能宣称掉电保存已经可用。存储 IO 不得放入电机任务或中断；导入不得包含使能/目标/CAN ID。
- 不把通信解析、PID 运算、设备访问、状态机和任务调度堆在一个大文件或大函数中。按清晰职责拆分，保持依赖方向明确；也不为尚未确定的功能预建复杂抽象。
- 持续保持代码格式一致、命名明确、可阅读性和可移植性优先。沿用项目的 4 空格缩进与大括号风格；避免多条独立语句挤在一行。协议 ID、通道编号及边界等重复数字应有具名定义或明确集中映射。
- 公共接口说明单位、有效范围、调用上下文、数据所有权和失败行为；配置集中管理。实时控制、命令接收和波形发送通过明确的邮箱/快照接口交接，避免通信侧直接改写控制器运行状态或阻塞控制任务。
- 修改现有模块时一并检查相关代码的组织与格式；重构限定在相关职责范围，保留既有应用行为并做相应验证。调试成功、编译通过和实际云台/发射性能验证须明确区分。

## Project Structure & Module Organization

This is STM32H723 embedded RoboMaster controller firmware built from a CubeMX-style layout. `Core/Inc` and `Core/Src` hold generated startup, peripheral, FreeRTOS, and interrupt code. Keep board support code in `bsp/<peripheral>/`, high-level robot behavior in `application/<feature>/`, and reusable device/control logic in `module/<domain>/`. Vendor code lives under `Drivers/`, `Middlewares/`, and `USB_DEVICE/`; avoid editing it unless regenerating or patching upstream code deliberately. Project metadata and linker configuration are in `Meta-Embedded-NG.ioc` and `STM32H723XG_FLASH.ld`. Module notes are usually stored beside code as `*.md`.

## Build, Test, and Development Commands

Build rules create their output directories with `mkdir -p`. Object, dependency and listing paths preserve source subdirectories to avoid basename collisions. Independent bench manifests live in `application/<bench>/build.mk`.

The firmware is built per robot variant via the `ROBOT` variable (default `sentry`):

- `make -j ROBOT=sentry` / `make -j ROBOT=infantry`: builds `build/<robot>/Meta-Embedded-NG-<robot>.elf`, `.hex`, `.bin`, and the map file with `arm-none-eabi-gcc`. The `-j` flag enables parallel compilation (pass `-j<N>` to cap the job count, e.g. `-j8`). Each variant compiles into its own `build/<robot>/` subdirectory, so switching robots needs no `make clean`.
- `make -j`: same as `make -j ROBOT=sentry` (the default variant).
- `make clean`: removes the current variant's `build/<robot>/` directory (run per `ROBOT`).
- `make -j GCC_PATH=/path/to/gcc-arm-none-eabi/bin`: builds with a specific GNU Arm toolchain without changing `PATH`.

The robot type is selected at build time. Legacy sentry/infantry use `application/robot_def.h` and their params headers. New independent robot applications should compose portable subsystems in their own entry/task files and source manifest; do not copy sentry business state. Add a Makefile selection branch pointing to that manifest. See `docs/architecture/control-system.md` for current ownership and migration contracts.

There is no repository-level flash target in the Makefile. Use the generated artifacts with the team debugger/programmer workflow.

## Coding Style & Naming Conventions

Use C with the existing firmware style: 4-space indentation, Doxygen file/function comments for public modules, and short inline comments for hardware-specific decisions. Prefer `static` for file-local helpers and state. Follow current names: macros in `UPPER_SNAKE_CASE`, config/data structs ending in `_s`, typedefs ending in `_t`, and init/control APIs like `ChassisInit()` or `DJIMotorControl()`. Keep each feature's `.c`, `.h`, and optional `.md` together in its module directory. Do not hand-edit generated CubeMX blocks unless the change is intentional and documented.

## Testing Guidelines

Focused host tests are available under `tests/vofa/`, `tests/speed_ready/`, `tests/friction_bench/`, and `tests/gimbal/`; each has a `run_tests.ps1` accepting a host GCC path. Run tests relevant to behavior changes and compile the affected firmware variant. For behavior changes, document any hardware smoke test actually performed, including board role, connected buses, affected motors/sensors, and observed logs or telemetry. Explicitly distinguish mocked host verification from hardware validation. Add focused tests or simulation harnesses for portable control/protocol logic as needed.

## Commit & Pull Request Guidelines

Recent history follows Conventional Commits: `feat:`, `fix:`, `chore:`, `doc:`, and `feat!:` for breaking changes. Keep commits scoped and mention tuned parameters or affected modules when relevant, for example `fix: update gimbal PID`. Pull requests should describe the change, list build/hardware validation, link issues when available, and call out CubeMX regeneration, linker changes, protocol changes, or calibration updates.

## Portable control boundaries

- `application/gimbal_control` and the maintained protocol/control modules compile as `portable_control` in `tests/CMakeLists.txt` without HAL/RTOS/vendor headers. Keep them instance-based and single-owner.
- `application/friction_bench/friction.c` uses `dji_speed_group` and `control_platform`; do not reach back into DJI/PID internals or add HAL/RTOS details there.
- Run `cmake -S tests -B build/host-tests`, build it and run `ctest --test-dir build/host-tests --output-on-failure` with a host toolchain. `tests/check_architecture.py` checks the maintained boundaries.
- Run affected firmware builds after edits to manifests/platform adapters. A shared root Makefile change also requires legacy variant build checks.
- `gimbal_port` still exclusively owns CAN1/UART5. `DjiSpeedGroupControl()` still flushes the legacy global DJI registry once per system motor task. Do not silently combine these into a multi-owner/shared-bus design.
