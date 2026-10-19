# fashejigou · RoboMaster 云台与三摩擦轮控制

## 项目详解

本项目面向 RoboMaster 云台与发射机构控制，基于 **DM-MC02 / STM32H723 / FreeRTOS** 和 Meta-Embedded-NG 框架扩展。当前重点是两个独立应用：三摩擦轮速度控制台架、DM-J4310 Yaw 速度控制台架。它们服务于后续整体控制集成，不能按“已经完成整套自动发射系统”理解。

初次阅读建议先看 [从这里开始](从这里开始.md)，再按目标选择专项文档。

### 当前可以使用的部分

| 子系统 | 当前能力 | 尚需区分的边界 |
| --- | --- | --- |
| [三摩擦轮](application/friction_bench/README.md) | 120° 三轮、C620/M3508 速度闭环、目标斜坡、遥控档位、调参、状态与故障处理 | 转速到位不等于完整发射许可；弹速与持续温升验收仍需单独完成 |
| [J4310 云台](application/gimbal_bench/README.md) | 遥控生成 rad/s 目标、速度斜坡、达妙模式确认、超时与故障状态 | 目前是速度控制，不是已接入 IMU 的角度保持或世界坐标稳定 |
| [BMI088 与姿态](application/gimbal_bench/IMU.md) | 独立采样、校准、姿态和状态快照 | 安装轴向仍需确认；六轴 IMU 不提供无漂移绝对航向 |
| [USB 诊断](application/diagnostics/README.md) | 快照、日志、禁止运行原因和事件历史 | 查询与观察不等于清故障或获得电机控制许可 |
| [参数管理](application/parameters/README.md) | 配置包版本/单位/CRC、电脑导入导出、停用时原子应用到 RAM | 双槽存储核心已有测试，真实 Flash 与启动加载尚未绑定 |
| [角度控制准备](application/runtime/README.md) | 事件历史、间隔统计与连续角度 PI 核心 | 算法核心测试通过不代表已接入实际电机链 |

### 目录与分层

| 层次 | 路径 | 责任 |
| --- | --- | --- |
| 板级与传输 | `bsp/` | CAN/UART/USB/SPI、队列、时间与平台适配 |
| 可复用核心 | `module/` | DBUS、达妙协议、速度/角度逻辑、姿态、参数、文本与波形协议 |
| 应用执行 | `application/gimbal_control/` | 接受带原始时间戳的角速度命令，不解析遥控协议 |
| 输入映射 | `application/robot_cmd/` | 遥控通道、方向、死区和控制请求 |
| 应用装配 | `application/friction_bench/`、`application/gimbal_bench/` | 独立配置、任务、入口与状态组织 |
| 生成与厂商代码 | `Core/`、`Drivers/`、`Middlewares/` | 外设启动、HAL、CMSIS、FreeRTOS |
| 验证与工具 | `tests/`、`tools/` | 主机测试、架构检查、参数和日志工具 |

主机可编译的控制/协议核心与 HAL/RTOS 适配分离，实时控制与调试通过邮箱或一致快照交接。两套台架各自独立构建，不依赖哨兵业务状态；共享总线与多设备组合仍须明确资源所有者。

### 选择构建目标

安装 GNU Arm Embedded 工具链和 Make，在仓库根目录运行：

```bash
git clone https://github.com/huj911419-ctrl/fashejigou.git
cd fashejigou

# 三摩擦轮独立台架
make -f Makefile.friction -j8

# J4310 Yaw 独立台架，另一个构建目标
make -f Makefile.gimbal -j8
```

也可使用 `make ROBOT=friction_bench -j8` 和 `make ROBOT=gimbal_bench -j8`。产物分别位于 `build/friction_bench/` 与 `build/gimbal_bench/`，文件名前缀为 `Meta-Embedded-NG-<目标>`。

普通 `make` 默认仍选择 `sentry`；保留的 `sentry/infantry` 机器人目标与独立台架是不同构建入口。仅编译不会烧录或改变硬件。

### 配置与调试入口

- [bench_features.h](application/bench_features.h)：逐模块选择遥控、电机、IMU、姿态、USB 和参数等功能。
- [摩擦轮配置](application/friction_bench/friction_config.h)：电机总线、ID、方向、速度与 PI/电流限制；以实际源码和接线为准。
- [云台配置](application/gimbal_bench/gimbal_config.h)：J4310 速度、时效、温度与遥控映射。
- [USB 日志](application/friction_bench/USB_LOG.md)：摩擦轮状态、故障原因及 `status`、`log` 等命令。
- [遥控摩擦轮](application/friction_bench/REMOTE.md)：来源选择、档位和停止后重新解锁约定。
- [模块开关](docs/architecture/module-switches.md)：关闭功能后的诊断语义与验证方法。

摩擦轮的 rpm 指电机转子转速，达妙速度目标单位是 rad/s，电流命令使用协议原始值。云台 UART5 用于 DBUS，不能把它当成待配置的波形调试串口。更改 ID、方向和阈值前先核对目标台架，避免沿用另一机器人参数。

### 不连接硬件的验证

准备主机 GCC/Clang、CMake 3.21+ 和 Python 3：

```bash
cmake -S tests -B build/host-tests
cmake --build build/host-tests --parallel
ctest --test-dir build/host-tests --output-on-failure
```

也可运行 `python3 tests/verify_offline.py`，由脚本管理临时构建与验证报告。覆盖范围与限制见 [主机测试](tests/README.md) 和 [离线验证方案](docs/verification/offline-validation.md)。

主机测试使用替身接口，检查可移植算法、协议、编排、配置和故障约束，不模拟完整电机物理响应、真实 IRQ 时序或供电故障。实机转速、动态响应、弹速和温升需另行验收。

### 阅读与维护

推荐顺序：[工程入口](从这里开始.md) → [控制结构](docs/architecture/control-system.md) → 所选台架 README → 配置 → 可移植模块 → 板级端口 → 测试。项目保留 MIT 许可证与上游框架致谢，见 [LICENSE](LICENSE) 和下文。

---

以下保留框架基座与原有中英文说明。新增台架的当前能力、配置和验证边界以本节及专项文档为准。

**第一次看代码：[整个工程怎样组织、怎样运行](从这里开始.md)。从目录分工、构建选择、启动流程和 FreeRTOS 任务开始。**

> 面向达妙 **DM-MC02（STM32H723VGT6）** 控制板的 RoboMaster 机器人电控固件。
> RoboMaster robot control firmware for the DaMiao **DM-MC02 (STM32H723VGT6)** board.

[![MCU](https://img.shields.io/badge/MCU-STM32H723VGT6-blue)](#)
[![Board](https://img.shields.io/badge/Board-DM--MC02-orange)](#)
[![RTOS](https://img.shields.io/badge/RTOS-FreeRTOS-green)](#)
[![License](https://img.shields.io/badge/License-MIT-lightgrey)](#license--致谢--license--acknowledgements)

**[简体中文](#简体中文) | [English](#english)**

当前独立控制扩展：
[云台 Yaw](application/gimbal_bench/README.md) ·
[板载姿态反馈](application/gimbal_bench/IMU.md) ·
[三摩擦轮](application/friction_bench/README.md) ·
[USB 硬件诊断](application/diagnostics/README.md) ·
[运行记录与角度控制准备](application/runtime/README.md) ·
[结构与控制路线](docs/architecture/control-system.md) ·
[主机测试](tests/README.md) ·
[逐层验证方案](docs/verification/offline-validation.md)。云台与摩擦轮使用独立入口，均不依赖哨兵业务。


---

## 简体中文

### 项目简介

**Meta-Embedded-NG** 是一套用于 RoboMaster 比赛机器人的嵌入式电控固件。它在湖南大学跃鹿战队开源框架 [`basic_framework`](https://github.com/HNUYueLuRM/basic_framework) 的基础上做了大量修改，将原本面向 STM32F407（大疆 C 板）的代码移植并适配到了基于 **STM32H723** 的 **达妙 DM-MC02** 控制板。

项目沿用了原框架优秀的设计思想：

- **三层分层架构** —— `bsp`（板级支持）→ `module`（可复用算法与设备驱动）→ `application`（机器人行为）。
- **消息中心（Message Center）发布-订阅** —— 各应用之间不直接相互包含，而是通过话题（topic）解耦通信。
- **离线守护（Daemon）** —— 统一监控各模块心跳，掉线时触发回调以进入安全状态。

### 硬件平台

| 项目 | 说明 |
| --- | --- |
| 主控 MCU | STM32H723VGT6（Cortex-M7，最高 550 MHz，1 MB Flash） |
| 控制板 | 达妙 DM-MC02 |
| 板载 IMU | BMI088（SPI 接口，含温度补偿与在线标定） |
| 链接脚本 | `STM32H723XG_FLASH.ld` |
| RTOS | FreeRTOS |

### 与 basic_framework 的差异

本项目是对 `basic_framework` 的**深度移植与裁剪**，主要区别如下：

| 方面 | basic_framework | Meta-Embedded-NG |
| --- | --- | --- |
| 主控 / 控制板 | STM32F407（大疆 C 板） | STM32H723（DM-MC02） |
| 模块目录名 | `modules/` | `module/` |
| 电机驱动 | DJI / DM / HT / LK / 舵机 / 步进 | DJI / DM / 舵机 / 步进 / **XM（新增）**，移除 HT、LK |
| 传感器/外设模块 | 含 IST8310 磁力计、OLED、蓝牙、TFminiPlus 激光雷达 | 已移除上述模块 |
| 裁判系统 | 原版 | 修复了裁判系统协议与任务 |

核心三层架构、消息中心设计与代码规范保持不变。

### 目录结构

```
Meta-Embedded-NG/
├── application/      # 机器人行为：底盘、云台、发射、指令分发、各机器人（哨兵/步兵）
├── module/           # 可复用算法与设备驱动（电机、IMU、消息中心等）
├── bsp/              # 板级支持包，封装 STM32H7 片上外设
├── Core/             # CubeMX 生成的外设/FreeRTOS/中断代码（Inc + Src）
├── Drivers/          # STM32H7xx HAL 与 CMSIS（厂商代码）
├── Middlewares/      # FreeRTOS、USB 设备库、CMSIS-DSP、SEGGER RTT
├── USB_DEVICE/       # USB 设备中间件（App + Target）
├── Makefile          # arm-none-eabi-gcc 构建脚本
├── Meta-Embedded-NG.ioc   # CubeMX 工程文件
└── STM32H723XG_FLASH.ld   # 链接脚本
```

### 软件架构

- **application/**：`chassis`（底盘运动学/功率限制）、`gimbal`（云台姿态控制）、`shoot`（摩擦轮 + 拨弹）、`cmd`（指令分发，消费遥控/键鼠输入并发布控制量），以及各机器人目录 `sentry`（哨兵）、`infantry`（步兵），每个机器人目录含统一入口 `robot.c`/`robot_task.c` 与专属参数 `<robot>_def.h`。机器人类型在构建时经 `make ROBOT=<robot>` 选择，通用消息契约与选择器见 `application/robot_def.h`。
- **module/**：电机驱动（`motor/`：DJI、DM、XM、舵机、步进）、`algorithm`（PID、卡尔曼滤波、LQR、EKF 姿态解算、CRC）、`message_center`、`daemon`、`BMI088`、`imu`、`encoder`、`can_comm`（多板 CAN 通信）、`super_cap`（超级电容）、`referee`（裁判系统/UI）、`remote`（DT7 遥控）、`master_machine`、`standard_cmd`、`unicomm`、`alarm`。
- **bsp/**：`can`、`usart`、`spi`、`iic`、`pwm`、`adc`、`gpio`、`log`、`usb`、`flash`、`dwt`。
- **FreeRTOS 任务**：INS 姿态解算（约 1 kHz）、电机控制、系统监控、机器人主逻辑等并行运行。

各子模块旁通常附带 `*.md` 说明文档（如 `module/message_center/message_center.md`、`module/motor/DJImotor/dji_motor.md`、`module/BMI088/bmi088.md`），开发前建议先阅读。

### 环境与构建

依赖 **GNU Arm Embedded 工具链**（`arm-none-eabi-gcc`）。

> 当前 Makefile 会自动创建构建输出目录。也可预先执行 `mkdir -p build`，重复执行不会报错。

```bash
# 可选：预先创建构建目录
mkdir -p build

# 按机器人种类构建（ROBOT 默认为 sentry）
# -j 启用并行编译；可用 -j<N> 限制并发数，如 -j8
make -j ROBOT=sentry      # 生成 build/sentry/Meta-Embedded-NG-sentry.elf / .hex / .bin / .map
make -j ROBOT=infantry    # 生成 build/infantry/Meta-Embedded-NG-infantry.elf / .hex / .bin / .map
make -j                   # 等价于 make -j ROBOT=sentry

# 清理当前机器人的 build/<robot>/ 目录
make clean

# 指定工具链路径而不修改 PATH
make -j ROBOT=infantry GCC_PATH=/path/to/gcc-arm-none-eabi/bin
```

> 每种机器人各自编译到独立的 `build/<robot>/` 子目录，切换机器人无需先 `make clean`。

> 仓库未提供烧录目标（flash target）。请使用团队的调试器/烧录器（如 OpenOCD + CMSIS-DAP、Ozone 等）将生成的 `.hex`/`.bin` 烧录到板子。

### 配置说明

- **机器人类型在构建时选择**：`make ROBOT=sentry|infantry` 会传入 `-DROBOT_SENTRY` / `-DROBOT_INFANTRY`，并编译对应的 `application/<robot>/` 目录。
- 各模块间通用的消息契约（底盘/云台/发射的枚举与结构体）与机器人选择器统一放在 `application/robot_def.h`；各机器人专属的物理参数（轮距、云台限位、拨盘参数等）放在 `application/<robot>/<robot>_def.h`。
- 新增一种机器人：把 `application/sentry/` 复制为 `application/<new>/`，在 `robot_def.h` 增加一条 `#elif defined(ROBOT_<NEW>)`，并在 Makefile 的机器人变体块中增加一条 `else ifeq ($(ROBOT),<new>)`。
- BMI088 支持上电在线标定，标定相关参数见 `module/BMI088/bmi088.md`。

### 代码规范与贡献

详见 [`AGENTS.md`](AGENTS.md)。要点：

- 遵循 Conventional Commits：`feat:`、`fix:`、`chore:`、`doc:`、`feat!:`（破坏性变更）。
- C 代码 4 空格缩进；宏 `UPPER_SNAKE_CASE`，配置结构体以 `_s` 结尾，typedef 以 `_t` 结尾，初始化/控制 API 形如 `ChassisInit()`、`DJIMotorControl()`。
- CAN/串口等协议结构体使用 `#pragma pack(1)`。
- 每次改动至少保证 `make` 通过；行为变更请记录硬件验证情况。
- 不要手动修改 CubeMX 生成块，除非是有意且记录在案的改动。

### License & 致谢

本项目以 **MIT License** 发布，Copyright (c) 2025 Meta-Team，详见 [`LICENSE`](LICENSE)。

衷心感谢 **湖南大学 RoboMaster 跃鹿战队** 开源的 [`basic_framework`](https://github.com/HNUYueLuRM/basic_framework)（MIT，Copyright (c) 2022 NeoZng / Even），本项目在其基础上开发。

---

## English

### Overview

**Meta-Embedded-NG** is embedded control firmware for RoboMaster competition robots. It is heavily modified from the Hunan University YueLu Team's open-source [`basic_framework`](https://github.com/HNUYueLuRM/basic_framework), porting and adapting code originally targeting the STM32F407 (DJI C-board) to the **STM32H723**-based **DaMiao DM-MC02** control board.

The project keeps the original framework's core design ideas:

- **Three-layer architecture** — `bsp` (board support) → `module` (reusable algorithms & device drivers) → `application` (robot behavior).
- **Message Center pub-sub** — applications never include each other directly; they communicate through decoupled topics.
- **Daemon offline watchdog** — monitors module heartbeats and triggers a callback to enter a safe state on timeout.

### Hardware

| Item | Detail |
| --- | --- |
| MCU | STM32H723VGT6 (Cortex-M7, up to 550 MHz, 1 MB Flash) |
| Board | DaMiao DM-MC02 |
| Onboard IMU | BMI088 (SPI, with temperature compensation & online calibration) |
| Linker script | `STM32H723XG_FLASH.ld` |
| RTOS | FreeRTOS |

### Differences from basic_framework

This project is a **deep port and trim** of `basic_framework`. Key differences:

| Aspect | basic_framework | Meta-Embedded-NG |
| --- | --- | --- |
| MCU / board | STM32F407 (DJI C-board) | STM32H723 (DM-MC02) |
| Modules folder | `modules/` | `module/` |
| Motor drivers | DJI / DM / HT / LK / servo / stepper | DJI / DM / servo / stepper / **XM (new)**; HT & LK removed |
| Sensor/peripheral modules | Includes IST8310 magnetometer, OLED, Bluetooth, TFminiPlus LiDAR | These modules removed |
| Referee system | Original | Referee protocol & task fixed |

The core three-layer architecture, Message Center design, and coding conventions are unchanged.

### Project Structure

```
Meta-Embedded-NG/
├── application/      # Robot behavior: chassis, gimbal, shoot, cmd dispatch, per-robot dirs (sentry/infantry)
├── module/           # Reusable algorithms & device drivers (motors, IMU, message center...)
├── bsp/              # Board support package wrapping STM32H7 on-chip peripherals
├── Core/             # CubeMX-generated peripheral/FreeRTOS/interrupt code (Inc + Src)
├── Drivers/          # STM32H7xx HAL & CMSIS (vendor code)
├── Middlewares/      # FreeRTOS, USB device library, CMSIS-DSP, SEGGER RTT
├── USB_DEVICE/       # USB device middleware (App + Target)
├── Makefile          # arm-none-eabi-gcc build script
├── Meta-Embedded-NG.ioc   # CubeMX project file
└── STM32H723XG_FLASH.ld   # Linker script
```

### Architecture

- **application/**: `chassis` (kinematics / power limiting), `gimbal` (attitude control), `shoot` (friction wheels + loader), `cmd` (command dispatch — consumes remote/keyboard input and publishes control setpoints), plus per-robot dirs `sentry` and `infantry` — each holds a common entry point `robot.c`/`robot_task.c` and its own params header `<robot>_def.h`. The robot type is selected at build time via `make ROBOT=<robot>`; the common message contract and variant selector live in `application/robot_def.h`.
- **module/**: motor drivers (`motor/`: DJI, DM, XM, servo, stepper), `algorithm` (PID, Kalman filter, LQR, EKF attitude solver, CRC), `message_center`, `daemon`, `BMI088`, `imu`, `encoder`, `can_comm` (multi-board CAN), `super_cap`, `referee` (referee system / UI), `remote` (DT7 receiver), `master_machine`, `standard_cmd`, `unicomm`, `alarm`.
- **bsp/**: `can`, `usart`, `spi`, `iic`, `pwm`, `adc`, `gpio`, `log`, `usb`, `flash`, `dwt`.
- **FreeRTOS tasks**: INS attitude estimation (~1 kHz), motor control, system monitoring, and the main robot logic run concurrently.

Most submodules ship with a companion `*.md` doc (e.g. `module/message_center/message_center.md`, `module/motor/DJImotor/dji_motor.md`, `module/BMI088/bmi088.md`) — read them before diving in.

### Build

Requires the **GNU Arm Embedded toolchain** (`arm-none-eabi-gcc`).

> The current Makefile creates output directories automatically. An optional `mkdir -p build` is safe to repeat.

```bash
# Optional: create the build directory in advance
mkdir -p build

# Build per robot variant (ROBOT defaults to sentry)
# -j enables parallel compilation; pass -j<N> to cap the job count, e.g. -j8
make -j ROBOT=sentry      # produces build/sentry/Meta-Embedded-NG-sentry.elf / .hex / .bin / .map
make -j ROBOT=infantry    # produces build/infantry/Meta-Embedded-NG-infantry.elf / .hex / .bin / .map
make -j                   # same as make -j ROBOT=sentry

# Clean the current variant's build/<robot>/ directory
make clean

# Use a specific toolchain without changing PATH
make -j ROBOT=infantry GCC_PATH=/path/to/gcc-arm-none-eabi/bin
```

> Each robot compiles into its own `build/<robot>/` subdirectory, so switching robots needs no `make clean`.

> There is no flash target in the Makefile. Use your team's debugger/programmer (e.g. OpenOCD + CMSIS-DAP, Ozone) to flash the generated `.hex`/`.bin` to the board.

### Configuration

- **Robot type is selected at build time**: `make ROBOT=sentry|infantry` passes `-DROBOT_SENTRY` / `-DROBOT_INFANTRY` and compiles the matching `application/<robot>/` directory.
- The message contract shared across modules (chassis/gimbal/shoot enums and structs) and the robot selector live in `application/robot_def.h`; each robot's own physical params (wheel geometry, gimbal limits, loader params, etc.) live in `application/<robot>/<robot>_def.h`.
- Adding a robot: copy `application/sentry/` to `application/<new>/`, add an `#elif defined(ROBOT_<NEW>)` arm in `robot_def.h`, and add an `else ifeq ($(ROBOT),<new>)` arm in the Makefile's robot-variant block.
- BMI088 supports power-on online calibration; see `module/BMI088/bmi088.md` for parameters.

### Conventions & Contributing

See [`AGENTS.md`](AGENTS.md). Highlights:

- Follow Conventional Commits: `feat:`, `fix:`, `chore:`, `doc:`, `feat!:` (breaking change).
- 4-space indentation; macros `UPPER_SNAKE_CASE`, config structs end in `_s`, typedefs end in `_t`, init/control APIs like `ChassisInit()` / `DJIMotorControl()`.
- Use `#pragma pack(1)` for CAN/serial protocol structs.
- A clean `make` is the minimum verification for every change; document hardware validation for behavior changes.
- Do not hand-edit generated CubeMX blocks unless the change is intentional and documented.

### License & Acknowledgements

Released under the **MIT License**, Copyright (c) 2025 Meta-Team. See [`LICENSE`](LICENSE).

Sincere thanks to the **Hunan University RoboMaster YueLu Team** for the open-source [`basic_framework`](https://github.com/HNUYueLuRM/basic_framework) (MIT, Copyright (c) 2022 NeoZng / Even), on which this project is built.
