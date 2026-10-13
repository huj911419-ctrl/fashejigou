# 模块独立开关与逐项验证

统一入口：[`application/bench_features.h`](../../application/bench_features.h)。
把对应 `#define` 的最后一个数字改成 **1 开启 / 0 关闭**，重新编译、烧录对应 bench 固件。
默认全部为 1，保持此前功能选择；开启电机模块仍需原有运行命令、有效反馈和解锁流程。
这是编译配置，不是 USB 命令或运行时热插拔。两个 bench 仍是两份固件，与旧哨兵/步兵独立。

## 开关表

| 开关 | 适用固件 | 关闭后的行为 |
| --- | --- | --- |
| `BENCH_ENABLE_USB` | 两套 | 不绑定 CDC 诊断、不创建 USB 诊断任务，也不调用 USB 设备初始化；电脑不会出现这份固件的 CDC COM 口 |
| `BENCH_ENABLE_BOARD_DIAGNOSTICS` | 两套 | USB 不读取板级外设/电源/系统诊断，显示 DISABLED；应用状态报告仍可使用 |
| `BENCH_ENABLE_RTT_LOG` | 两套 | 关闭应用周期 RTT 日志；旧底层驱动自身的初始化/错误日志仍受原日志系统配置管理 |
| `BENCH_ENABLE_EVENT_HISTORY` | 两套 | 不追加 RAM 状态历史；USB `events` 回复 DISABLED |
| `BENCH_ENABLE_REMOTE` | 云台 | 不启动 UART5 接收，不解码 DBUS；不会生成遥控运行命令 |
| `BENCH_ENABLE_FRICTION_REMOTE` | 摩擦轮 | 不启动 UART5 遥控 RX，保留 USB 控制；与云台遥控开关独立。默认开启后选 REMOTE，操作见 [REMOTE.md](../friction/REMOTE.md) |
| `BENCH_ENABLE_YAW` | 云台 | 不启动云台 CAN1 服务，不推进 J4310 控制/握手，不发送电机命令；遥控仍能单独观察 |
| `BENCH_ENABLE_IMU` | 云台 | 不创建 IMU 任务，不访问 BMI088；姿态解算也随之停用 |
| `BENCH_ENABLE_ATTITUDE` | 云台 | 保留 IMU 原始加速度/角速度/温度采集，关闭零偏校准和姿态解算 |
| `BENCH_ENABLE_FRICTION` | 摩擦轮 | 不注册任何 DJI 电机，不执行电机控制；调参返回不可用 |
| `BENCH_ENABLE_F1/F2/F3` | 摩擦轮 | 分别关闭对应轮的注册与控制，关闭轮无需填写有效 ID |
| `BENCH_ENABLE_SPEED_READY` | 摩擦轮 | 关闭整组到位观察，ready 标志保持假；逐轮速度 PID 与保护不受影响 |
| `BENCH_ENABLE_VOFA` | 摩擦轮 | 不绑定、读取或发送 VOFA 数据，也不占有运行命令来源；串口仍由用户另行绑定 |
| `BENCH_ENABLE_RTT_COMMAND` | 摩擦轮 | 不读取 RTT 控制命令；可与 RTT 日志单独选择 |
| `BENCH_ENABLE_PARAMETERS` | 摩擦轮 | 停用完整参数包导入/导出，返回 UNAVAILABLE；原有逐项 VOFA 调参仍可用，不自动写 Flash |

开关只对所属固件生效。例如在摩擦轮固件中设置 IMU=1，不会因此创建 IMU 任务。
`BENCH_ATTITUDE_ACTIVE`、`BENCH_FRICTION_MASK` 等是自动计算结果，不要手改。
USB 报告中的 `[modules]`、`[modules_gimbal]`、`[modules_friction]` 显示本次构建选择及有效状态。
关闭的模块显示 `DISABLED`，没有接入这份固件的设备仍为 `NOT_MONITORED` 或 `NOT_IN_THIS_FIRMWARE`。

## 推荐的验证顺序

先保持 USB=1，方便观察。云台使用 `make -f Makefile.gimbal`：

| 验证步骤 | REMOTE | YAW | IMU | ATTITUDE | 看什么 |
| --- | ---: | ---: | ---: | ---: | --- |
| 1. 只测电脑通信 | 0 | 0 | 0 | 0 | COM 枚举、ping、status、报告周期 |
| 2. 只测遥控 | 1 | 0 | 0 | 0 | DBUS 有效帧、摇杆、开关；电机 CAN1 服务不启动 |
| 3. 只测 IMU 原始值 | 0 | 0 | 1 | 0 | 芯片 ID、三轴数据、温度、采样计数与故障 |
| 4. 测姿态解算 | 0 | 0 | 1 | 1 | 静止校准、转动方向、姿态变化；安装确认仍单独设置 |
| 5. 只测 J4310 通信 | 0 | 1 | 0 | 0 | 原有禁用/参数握手、有效反馈；没有运行命令来源，不自动转动 |
| 6. 遥控 + 云台速度 | 1 | 1 | 0 | 0 | 原有回零、OFF→ON、斜坡、反馈和断连停止 |
| 7. 云台 + IMU 观察 | 1 | 1 | 1 | 1 | 两条任务链同时运行；仍未接入 IMU 角度闭环 |

表头缩写均指 `BENCH_ENABLE_` 后的名字。每次只增加一个待验证功能，记录日志后再组合。

摩擦轮使用 `make -f Makefile.friction`。例如只验证 **F2**：

```c
#define BENCH_ENABLE_FRICTION 1
#define BENCH_ENABLE_F1 0
#define BENCH_ENABLE_F2 1
#define BENCH_ENABLE_F3 0
```

在 [`friction_config.h`](../../application/friction/config/friction_config.h) 只填写 **第二项** 电机的实际 CAN/ID/方向。
F1、F3 的 ID 可以继续为 0。协议编号仍为 F1/F2/F3，F2 不会因为是唯一开启轮就变成 F1。
启用掩码使用 2（F1=1、F2=2、F3=4、全部=7）。选中了关闭轮的运行请求会拒绝，不会悄悄改成运行剩下的轮。
云台 USB 保持只读诊断。摩擦轮 USB 已支持 `off/set/pid/ramp`，并可用 `source usb` / `source remote`
停机切换输入；UART10 VOFA 当前只输出波形。查询/日志不续期，原超时与重新解锁规则继续生效。
若尚未绑定 VOFA，设 VOFA=0、RTT_COMMAND=1 即可单独使用 RTT。三轮都关闭与摩擦轮总开关关闭均不会注册/发送电机控制。

## 开关边界

- PID、有效性检查、命令/反馈超时、限流、故障锁定等属于控制链的必要条件，不提供绕过它们的调试开关。
- 云台角度核心目前没有接入电机，因此不增加一个看似能开启角度保持的空开关。Pitch、供弹、自瞄、裁判系统也尚未有可启动的模块。
- `Core/main.c` 的 Cube 外设基础配置、GPIO 初始状态仍执行；关闭应用模块不等于切断电源或撤销引脚初始化。
- 摩擦轮复用的旧 CAN 服务在首个电机注册时启动三个 CAN 控制器；关闭轮不注册接收实例、不计算 PID，控制帧可能仍以 C620 共用分组形式包含其零电流槽位。这些开关不是每根 CAN 总线的电源开关。
- 全部业务关闭时，保留 `BenchService` 基础周期任务喂已有 IWDG，避免仅测 USB 时不断复位。正常运行时仍由原控制任务路径喂狗，未另加后台任务掩盖控制任务卡死。
- IMU=1、ATTITUDE=0 可单独验证驱动；IMU=0 时，无论 ATTITUDE 请求值多少，有效解算状态均为关闭。关闭到位监测时，不能把零值当作已经允许发射。

## 移植与验证

开关位于应用组合层，纯 C 算法/协议核心不包含工程开关。换工程可以继续复用算法，按自己的应用选择初始化和任务。
`GimbalPortInitModules(use_can, use_remote)` 是一次性资源选择接口；不要运行中重复调用它充当开关。
现有 `GimbalPortInit()` 保留为两项均开启的兼容入口。

自动化隔离构建可使用 `BENCH_DEFINES`，并指定不同的 `BUILD_DIR`，例如：

```powershell
make -f Makefile.gimbal BUILD_DIR=build/imu-raw "BENCH_DEFINES=-DBENCH_ENABLE_YAW=0 -DBENCH_ENABLE_REMOTE=0 -DBENCH_ENABLE_ATTITUDE=0"
```

平时直接改头文件即可，Make 会根据头文件依赖重编译。命令行更换宏而复用旧构建目录时，Make 不会自动识别编译选项变化，因此要用独立目录或先清理对应构建。

本轮验证入口仍为 [`tests/verify_offline.py`](../../tests/verify_offline.py)，新增模块关闭、单轮注册、固定编号调参、关闭设备无收发、关闭状态显示等组合。
实际记录见 本轮验证报告（历史资源未随仓库提交：`../../.scratch/module-switches/verification.md`）。软件测试/构建不代替接板确认。
