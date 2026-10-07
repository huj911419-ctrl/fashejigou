# 云台与三摩擦轮：结构、移植边界和后续控制路线

本轮检查日期：2026-09-29。开发基座为本 Meta-Embedded-NG 目录。
`CtrBoard-H7_CAN` 仅作为已跑通的 J4310/DBUS 硬件参考，未修改。

2026-09-30 更新：已按 [实际开源源码对照](open-source-review.md) 拆分输入命令层与云台执行层。
下文路线继续适用；最新接口迁移见 [robot_cmd 说明](../../application/robot_cmd/README.md)。

## 当前判断

原先的 bsp/module/application 方向可以继续使用。问题在于：有分层目录，并不等于依赖已经隔离。
此前云台业务仍直接关中断，摩擦轮业务直接访问 DJI PID 内部成员，移植时需要到处寻找这些依赖。
本轮将这些接触点集中，并使用不带 HAL/RTOS 头文件的主机构建验证可移植核心。

| 部分 | 本轮后的移植程度 | 换平台需要做什么 |
| --- | --- | --- |
| VOFA、DBUS、速度到位、Yaw 斜坡、DM 协议状态机 | 高：纯 C、调用者持有状态 | 提供时间/数据/发送回调 |
| `application/gimbal_control` | 高：纯 C、多实例、无静态业务状态 | 传入轴配置和发送回调，按接口喂数据 |
| 摩擦轮状态机、到位组合 | 高：纯 C | 喂入反馈、命令和同一时钟时间 |
| BMI088 协议、六轴姿态核心 | 高：纯 C、多实例 | 传输回调、带时间戳的 SI 样本、坐标映射 |
| `text_console` 文本协议、队列与会话 | 高：纯 C、多实例 | 注入非阻塞 read/write/session 回调 |
| `yaw_angle` 角度外环核心 | 高：纯 C、多实例；尚未接入电机 | 提供同一连续坐标的目标/测量、有效性、时间和零点参考编号 |
| `event_history`、`loop_stats` | 高：纯 C、多实例 | 统一时钟、调用者提供同步；[运行准备说明](../../application/runtime/README.md) |
| USB 诊断报告与硬件观测 | 应用报告读快照，板级状态/CDC/任务单独适配 | 替换 `usb_console_port`、`board_status`、任务入口，见 [诊断说明](../../application/diagnostics/README.md) |
| 摩擦轮业务编排 | 中：无 HAL/RTOS 细节，但仍为一个台架实例 | 实现平台接口和 DJI 速度组接口 |
| `dji_speed_group` 实现、旧 DJI/PID 驱动 | 仍依赖现有 BSP/DWT/驱动 | 适配或替换这一层，保留外部单位约定 |
| `gimbal_port`、FreeRTOS 任务、CubeMX 外设 | 板级实现，不能直接跨 MCU 复制 | 根据目标板重接外设、IRQ、时钟、调度 |

这不是整车移植已经完成，也不是把整个旧框架改成硬件无关。当前可以复用的是经过隔离和验证的控制核心。

## 参照的开源结构

本轮查阅以下原项目及其文档，学习职责划分，不复制其机器人参数，也没有引入新的外部运行依赖：

1. [HNUYueLuRM/basic_framework](https://github.com/HNUYueLuRM/basic_framework)：
   bsp/module/app 分层，模块实例持有状态，应用通过命令与反馈交换数据。
   [云台实现](https://github.com/HNUYueLuRM/basic_framework/blob/master/application/gimbal/gimbal.c)
   将云台业务、电机和姿态反馈分别组织。
2. [ARUW Taproot Subsystem](https://aruw.gitlab.io/controls/taproot/api/classtap_1_1control_1_1_subsystem.html)：
   一个子系统封装其硬件资源，通过接口接受动作请求。
   [CommandScheduler](https://aruw.gitlab.io/controls/aruw-mcb/api/classtap_1_1control_1_1_command_scheduler.html)
   处理动作对同一资源的竞争。我们目前采用更小的“单控制任务所有者 + 命令邮箱/状态快照”，
   不为两个台架照搬完整 C++ 调度框架。
3. [ARUW 云台世界坐标控制器](https://aruw.gitlab.io/-/controls/aruw-mcb/-/jobs/15181085374/artifacts/public/api/classaruwsrc_1_1control_1_1turret_1_1algorithms_1_1_world_frame_turret_imu_cascade_pid_turret_controller.html)：
   角度、角速度控制分层，并明确依赖云台 IMU 的安装和坐标变换。
   这支持后续按坐标系和执行器输出类型拆分控制器，而不是把不同模式的 PID 混在一起。
4. [ARUW 主工程](https://github.com/uw-advanced-robotics/aruw-mcb)：
   区分硬件构建与主机测试。此处用独立 `tests/CMakeLists.txt` 达到同一目的，保持本项目 C/Make 工具链。

上述项目展示了可参考的结构，不能据此断言某个控制方法对本机构“最好”。实际参数和性能仍由实机数据决定。

## 本轮具体变化

```text
application/gimbal_bench/       组装配置、任务、诊断快照
    ├─ application/robot_cmd/        输入解释、方向/死区、带时戳角速度命令
    │     └─ module/dbus_input/      遥控协议
    └─ application/gimbal_control/   可移植 Yaw 执行子系统
          ├─ module/yaw_rate/        限速/斜坡与使能条件
          └─ module/dm_velocity/     电机协议与反馈状态

application/friction_bench/     命令/调参邮箱、三轮业务、到位与诊断
    ├─ friction_control/monitor      纯 C 状态与到位逻辑（同目录文件）
    └─ module/motor/dji_speed_group/  速度/电流单位、PID、DJI 驱动接触点

bsp/control_platform/          单调毫秒时钟、短临界区
bsp/gimbal_port/               H723 CAN1/UART5 队列和中断适配
tests/                         不带 HAL 的核心测试 + 带传输替身的组合测试
```

- `GimbalControl_s` 由调用者持有，可以创建互不影响的实例，配置复制到实例里；板级端口目前仍绑定一个 Yaw。
- `GimbalTask()` 仅负责有界接收、调用子系统、发布快照。时间在出队后采样，修复中途 IRQ 收到新帧时
  旧任务时间导致 unsigned 数据年龄下溢、误判掉线的问题。
- 状态分开表示应用授权 `enabled`、反馈在线 `motor_online`、电机确认 `motor_enabled`。
  收到使能请求不等于电机已经使能，历史传输故障也不等于当前在线状态。
- `FrictionTask()` 不再包含 HAL/FreeRTOS 头文件，也不访问 `DJIMotorInstance`/`PIDInstance` 内部结构。
  参数事务、P/I/D 遥测和 deg/s↔RPM 的转换由 `dji_speed_group` 接口统一承接。
- 修复首次启动空闲周期把零初始化邮箱误当作显式 OFF 的问题。现在没有收到任何命令时保留
  `FRICTION_FAULT_COMMAND`，先收到停机，再收到新的运行命令才能解除首次使能限制。
- 两个台架分别使用自己的 `build.mk`；根 Makefile 保留源路径生成 `.o/.d/.lst`，避免同名源文件相互覆盖，
  并把构建清单变化纳入增量构建依赖。输出 ELF/BIN 的位置不变。
- 新增统一 CMake/CTest 主机入口和架构依赖检查；已有各模块 PowerShell 测试入口继续可用。

## 现阶段怎样控制

### Yaw：先使用已验证的电机速度模式

DBUS CH0 → 死区/方向 → 电机侧角速度目标 → 加速度限制 → J4310 速度接口。
S1 与输入有效性决定是否允许使能；驱动还要确认模式、OFF/ON 反馈及电机健康状态。
其中 DBUS/死区/方向由命令层处理，执行核心仅接收带原始时间戳的 rad/s 请求。

J4310 速度环在驱动器内部。本阶段没有重复套一层 MCU 速度 PID。
松杆减速到零，不等于锁定角度。原始电机位置反馈也不能直接当成经过校准的世界坐标角度。

下一步若保留速度模式实现角度保持：

```text
手动输入/目标角度 → 角度目标管理 → 角度控制器 → 角速度限幅 → 当前 J4310 速度接口
                              ↑
                  明确坐标系、零点和回绕规则的角度反馈
```

若以后切换到 MIT 力矩模式，才由 MCU 的角度外环和角速度内环生成力矩输出；这需要独立的模式适配、
限幅、参数和实机测试。当前不自动切换，也不拿速度模式参数套用。

### 摩擦轮：三个速度环分别控制

整组命令 → 各轮目标/斜坡 → 三个独立速度 PI/PID → C620 电流指令。
到位模块观察最终目标、反馈、持续时间和迟滞；目标变化或调参会重新检查到位。
三轮速度到位只是一个条件，不能直接当作拨弹许可。

先测方向、反馈比例、空载阶跃和电流限制，再测实际负载下的速度跌落/恢复。
若测得稳定且可重复的负载规律，再加入前馈或补偿。现在保留已有 PI/PID 行为和参数单位，避免结构重构
同时引入未验证的控制算法变化。旧 PID 仍在电机适配内部依赖 DWT，这部分尚未完成跨平台替换。

## 成为整机项目还缺什么

| 工作 | 为什么需要 | 开始前必须确定 |
| --- | --- | --- |
| 云台坐标与传感器标定 | 角度保持/稳定控制不能只靠原始位置值 | IMU 安装轴向、零位、传动比、位置回绕 |
| Pitch 轴 | 实际云台需要独立限位与姿态控制 | 电机型号、驱动模式、ID、机械上下限 |
| 整机命令管理 | 遥控、调试、视觉不能同时争夺电机 | 模式优先级、切换条件、命令有效期 |
| 总线资源统一管理 | 当前两个台架不能简单合并 HAL 同名回调 | 最终 CAN/UART 分配、其他设备、总线负载 |
| 拨弹与互锁 | 转速合格仍不代表允许送弹 | 拨弹机构、反馈、裁判信息/限额、停止条件 |
| 参数生命周期 | 在线调参结果需要可靠读回和后续保存 | 参数版本、保存时机、存储布局；当前仍只改 RAM |
| 整机故障策略 | 区分暂时掉线、需人工恢复的故障和停机方式 | 电机端通信超时行为、机构惯性、恢复流程 |
| 实机验收数据 | 编译和 mock 测试不证明控制效果 | 角误差/转速误差、超调、稳定时间、负载恢复时间 |

这些是具体下一步，不是都已经实现。本轮没有把它们写成空模块或未经标定的控制器。

后续进展：已补充板载 BMI088 采集、静止零偏和六轴姿态反馈，详见
[IMU 实现与边界](../../application/gimbal_bench/IMU.md)。尚未完成实物轴向/温漂标定，
也未接入电机角度环；原表中的整机姿态控制和实机验收仍待完成。

2026-09-30：补充独立 `yaw_angle` PI 角度外环及离线测试，当前未接入 J4310 输出。
两套 bench 已接入 USB `events` 状态变化历史和任务调用间隔统计；它们不会改变既有停机/恢复策略。

## 组合与移植约束

1. 先组合可移植子系统，再在唯一的板级入口分配外设、回调和任务。新整机入口继续与哨兵分开。
2. 当前 `gimbal_port` 独占 CAN1/UART5，会在异常时取消 CAN1 待发帧。共享 CAN1 时必须先改成按设备
   管理待发帧，不能把它原样接入共享总线。
3. 旧 DJI 驱动统一打包全部已注册电机。因此 `DjiSpeedGroupControl()` 必须只由一个系统电机任务调用，
   不能让多个子系统各自调用一次。当前独立摩擦轮台架符合此约束。
4. 参数只能通过控制任务接收事务；诊断只读快照。时间戳与时钟必须同源，不能混用 RTOS tick 数与毫秒。
5. UART5 已知用于 DBUS。VOFA 调试串口与三个摩擦轮 ID 仍由用户确定，默认未绑定/未配置。
6. 重构不改变三轮参数的误差单位：现有 PID 使用电机侧 deg/s，业务/波形为 RPM；1 RPM=6 deg/s。

测试与构建方法见 [tests/README.md](../../tests/README.md)。尚未烧录或完成实机性能验证。

后续已启动独立、可删除的离线验证，范围、实测发现和分阶段验收见
[无硬件验证记录](../verification/offline-validation.md)。
