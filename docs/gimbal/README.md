[文档导航](../README.md) · [源码入口](../../application/gimbal/README.md) · [新手阅读路线](../start/04-read-gimbal.md)

# 独立 Yaw 云台工程

本阶段实现 DM-J4310 水平旋转轴的遥控速度控制。开发位置是 Meta-Embedded-NG 基座；
`CtrBoard-H7_CAN` 是已经跑通的硬件/通信参考，未修改该参考工程。

已接入独立 [BMI088 采集、校准与姿态反馈](IMU.md)。用户确认控制板随云台转动，
安装轴向仍待实物确认。姿态目前供观测和后续角度控制使用，J4310 仍运行手动速度模式。

逐模块验证时只需修改 [`bench_features.h`](../../application/bench_features.h)：遥控、Yaw 电机、IMU 采集、姿态解算及调试功能可分别选择。见 [模块开关与验证顺序](../architecture/module-switches.md)。

已接入 [USB 硬件诊断](../../application/services/diagnostics/README.md)：原生 CDC 虚拟串口默认每秒报告
遥控器、CAN、达妙电机、BMI088、姿态与系统状态，支持只读查询和电脑日志保存。
电机没有运行时，先看 `[yaw_inhibit]` 的原因与 `events` 中的 `yaw.inhibit` 变化；
字段含义见 [禁止运行原因说明](../../application/services/diagnostics/README.md#云台为什么没有运行)。这不改变原来的解锁或故障恢复条件。

## 已实现的控制链

```text
UART5 字节中断 → 有界接收队列 → robot_cmd：DBUS 解码、CH0/S1、方向/死区
                                           ↓
                         GimbalCommand_s：角速度 rad/s、原始时间、停机标记
                                           ↓
                       gimbal_control：时效、使能条件、限速、速度斜坡
                                           ↓
CAN1 接收队列 → J4310 反馈/状态机 ← 速度目标 rad/s → CAN1 发送
                                           ↓
                              输入/执行状态快照 → RTT / USB 诊断
```

电机使用参考工程已经使用的 `CTRL_MODE=3`，内部闭合速度环。
MCU 当前负责目标生成、限速、斜坡、通信与使能管理，**没有再串一个速度 PID**。
松杆后目标减到零；这不等于锁定云台角度，也不等于世界坐标稳定控制。

## 文件与职责

| 路径 | 职责 | 移植是否修改 |
| --- | --- | --- |
| `module/protocol/dbus_input/` | 18 字节 DBUS 解码、帧间隔同步、字段合法性 | 保持协议时不改 |
| `application/commands/` | 遥控输入映射、方向/死区、生成带原始时间戳的角速度请求 | 换输入时实现对应命令源 |
| `module/control/yaw_rate/` | 零速解锁、使能条件、按真实时间变化的速度斜坡 | 通常只改配置 |
| `module/protocol/dm_velocity/` | CAN 编解码、参数确认、运行状态、反馈超时和故障锁定 | 绑定发送回调即可 |
| `bsp/ports/gimbal_port/` | CAN/UART 中断与队列、非阻塞发送、传输错误恢复 | 换板/外设时修改 |
| `application/gimbal/config/gimbal_config.h` | 通道、方向、转速、时间和温度阈值 | 用户配置入口 |
| `application/gimbal/control/` | 可实例化的 Yaw 执行子系统，无遥控协议/HAL/RTOS | 配置并喂入带时间戳的命令和 CAN 反馈 |
| `application/gimbal/gimbal.c` | 硬件队列与可移植子系统的组装、状态快照 | 目标板接入 |
| `bsp/ports/control_platform/` | 同源毫秒时钟、短临界区 | 换平台时实现三个接口 |
| `application/gimbal/robot_task.c` | 2 ms 控制任务、低优先级 RTT 诊断 | 系统任务编排 |

三个通用模块、命令层和 `application/gimbal/control` 均不包含 STM32、HAL、FreeRTOS，不分配堆内存。中断只收数据，解析与控制在同一个
控制任务执行；诊断通过 `GimbalGetStatus()` 获取一致的副本，不直接修改电机或控制器。

当前板级端口独占 CAN1/UART5 的 HAL 回调，所以独立构建不编译旧 `bsp_can.c` / `bsp_usart.c`。
以后组合其他设备时，在板级统一分发回调，把收到的帧交给模块；不要把两套 HAL 同名回调一起链接。
J4310 的 CAN 发送也应由同一个总线所有者调度。当前端口停止时可以取消 CAN1 待发帧，这一做法
依赖“独占 CAN1”；改为共享总线时必须按设备管理待发帧，不能取消其他设备的发送。

## 已确认的硬件映射

| 项目 | 当前值 | 依据 |
| --- | --- | --- |
| 水平轴电机 | DM-J4310，360° Yaw | 用户确认 |
| 电机总线 | CAN1，Classic CAN，1 Mbps | 参考工程与本基座配置 |
| ESC_ID | `0x000`，这是有效的 DM ID | 参考 `User/dm_motor.h` |
| MASTER_ID | 开机读寄存器 `0x07`，不凭空填写 | 参考参数查询逻辑 |
| 速度命令 | `0x200 + ESC_ID`，4 字节小端 float，rad/s | 参考 `User/dm_motor.c` |
| 接收机 | UART5/PD2，100000 baud，8E1，板上硬件反相 | 参考 `User/dbus.c` 与 main |
| 遥控映射 | CH0 控制 Yaw，S1 原始值 1 为使能请求 | 参考遥控映射 |

沿用本基座 GPIO 对 PC14/PC13/PC15 的供电初始化。UART5 是**遥控接收口**，不是 VOFA 调试口；
本次没有替用户指定调试串口，也没有改动摩擦轮电机的待配置 ID。
S2 暂不接任何动作，Pitch 未接入。

## 操作与状态

1. 上电等待 2 秒，不自动使能。随后发送失能、查询 MASTER_ID、确认电机 OFF、写入速度模式、
   读回模式为 3。写入为运行参数操作，不发保存到 Flash 指令。
2. 待 RTT 的 `state=5`（READY），遥控在线，先让 S1 离开原始值 1，摇杆回中，再切到原始值 1。
   上电时 S1 已在使能位不会运行；偏杆使能也不会运行，需回 OFF 后重试。
3. CH0 推动生成目标转速；正常变化按斜坡执行。切到 OFF 则立即撤销软件参考并发送失能。
4. 遥控超过 100 ms 没有有效帧、输入错误、队列丢数据或控制间隔超过 10 ms，撤销使能。
   遥控恢复且 S1 仍为 ON 不会重新运行，需要 OFF→ON。
5. 电机报错、过温、运行反馈丢失或发送失败会锁定 `state=8`（FAULT）。修复原因后重启本应用，
   重新完成模式确认和遥控使能；目前没有遥控清故障/自动清电机故障功能。

正常松杆使用速度斜坡；故障/禁用发送的是电机失能，不承诺机械瞬时停止，也不维持角度。
总线断开时失能命令可能无法送达，软件超时不能代替电机端通信看门狗。实机需确认电机的
通信超时停机设置；MCU 看门狗仅由控制任务喂狗，不能替代驱动器端这一行为。

驱动状态编号：BOOT=0、DISCOVER=1、STOPPING=2、SET_MODE=3、CHECK_MODE=4、READY=5、
STARTING=6、RUNNING=7、FAULT=8。使能时先写零速度，再发送使能，收到 ON 反馈后才发送运行速度。

## 调整参数

集中修改 [gimbal_config.h](../../application/gimbal/config/gimbal_config.h)：

- `GIMBAL_YAW_DIRECTION`：只能 整数 `+1` 或 `-1`，用于确认实际正方向。
- `GIMBAL_MAX_RATE_RAD_S`：初始 1 rad/s（约 57.3°/s），同时配置输入映射、Yaw 和 DM 上限。
- `acceleration_rad_s2`：初始 2 rad/s²，零到最大目标至少约 0.5 秒。
- `GIMBAL_STICK_DEADBAND`：命令层死区，初始 0.04，剩余行程重新映射到满量程。
- 100 ms 遥控/反馈超时、10 ms CAN 命令周期、70°C 温度阈值均是初始台架配置，需结合实测确认。

速度单位是**电机侧 rad/s**；有减速传动时应单独补传动比映射。
PMAX=12.5 rad、VMAX=30 rad/s、TMAX=10 N·m 沿用参考代码，必须与驱动固件配置匹配。
反馈位置目前只作为原始诊断值，未假定它是连续多圈角度，也没有按 2π 或 25 rad 擅自展开。
360° 水平轴不设固定角度目标边界；机械是否支持连续多圈旋转需与实际结构一致。

云台已提供只读 USB/RTT/状态快照，没有新增 Yaw 在线调参协议。现有独立 VOFA 模块和摩擦轮调参保留，
以后可从该快照取波形，再通过控制任务参数邮箱接入 Yaw 调参，不能从串口回调直接改控制状态。

## 构建与验证

```powershell
make -f Makefile.gimbal -j8
# 等价：make ROBOT=gimbal_bench -j8
./tests/gimbal/run_tests.ps1 -Compiler 'D:/CLion/CLion 2025.3.2/bin/mingw/bin/gcc.exe'
```

输出在 `build/gimbal_bench/`。`Makefile.friction` 仍只构建摩擦轮；默认 sentry 入口不变。
这个镜像不会初始化原哨兵业务、摩擦轮、拨弹机构或旧 INS 任务；使用本应用独立的 IMU 采集/姿态任务。

统一测试与移植检查见 [tests/README.md](../../tests/README.md)。

主机测试覆盖 DBUS 同步/截断/非法字段、时间回绕、使能边沿与回中、斜坡、遥控恢复不自启、
任务超时、DM 协议帧与模式读回、发送失败/反馈掉线/报错的故障锁定，以及实际应用组合后的链路。
应用链路测试只替换硬件传输，不模拟电机物理特性。编译和这些测试不代表实机联调完成。

## 后续云台扩展

先用这一版确认方向、遥控映射、实际速度与反馈缩放。之后明确 IMU 安装轴向、云台/电机传动比、
角度零点及反馈回绕行为，再接入已独立验证的 `module/control/yaw_angle` 角度控制器。

- 角度保持：角度目标生成 → 角度环 → 角速度目标 → 当前电机速度接口。
- 世界坐标稳定：云台 IMU 姿态/角速度 → 稳定控制器 → 电机输出接口。
- 若改 MIT 力矩模式：另建力矩适配与 MCU 速度环，并明确切换条件，不在现有速度模式上重复闭环。
- Pitch：独立轴配置、机械限位、零位与反馈，复用协议/算法而不是复制本应用的任务状态。

角度目标、传感器融合和电机协议各自保留职责，不把后续功能继续塞入 `gimbal.c`。

## 本轮结构调整

详见 [控制系统架构与后续路线](../architecture/control-system.md)。调度入口由早期的
`GimbalStep(now)` 改为 `GimbalTask()`，会在出队后采样时间，避免中途接收到的新数据被旧时间误判为超时。
2026-09-30 对照公开源码后，输入与执行已进一步拆开。需要移植/模拟时使用
`GimbalRemoteReceiveByte/TakeCommand` 生成请求，再交给 `GimbalControlStep`；纯执行测试可直接提供命令，
无需构造遥控帧。完整契约和迁移说明见 [命令层说明](../../application/commands/README.md)。
`GimbalStatus_s.axis.enabled` 是应用授权，`.axis.motor_online` 与 `.axis.motor_enabled` 分别表示反馈在线和电机确认状态；
遥控快照放在 `.remote`。依据和取舍见 [开源对照记录](../history/open-source-review.md)。
