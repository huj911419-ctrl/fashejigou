> 历史记录：用于了解当时背景；当前目录、构建与功能状态以 [文档导航](../README.md) 为准。文中过去的验证数字不代表当前版本。

# 对照公开 RoboMaster 源码后的结构调整

查阅日期：2026-09-30。对象是 Meta 基座中的独立云台/三摩擦轮工程。
本记录将上游事实、适用范围和本地决定分开；仓库公开和比赛用途不等于每段示例都已完善，也不证明本机构的控制参数。

后续见 [2026-10-02 第二轮对照](open-source-review-20261002.md)：补充 RoboRTS、UICRM、INS 示例及 XRobot/LibXR，新增 Yaw 禁止运行原因诊断。

## 查了哪些实际源码

通过 GitHub API 固定提交，再读取下列文件；不是只看 README 或仓库热度。
本次 Gitee 网页获取失败，实际对照使用同团队 GitHub 源码。

| 项目、提交 | 具体入口 | 观察到的结构 |
| --- | --- | --- |
| 跃鹿 basic_framework，`1a136eb1ed2f101110348d5a5cd182272eada38e` | [robot_cmd.c](https://github.com/HNUYueLuRM/basic_framework/blob/1a136eb1ed2f101110348d5a5cd182272eada38e/application/cmd/robot_cmd.c)、[gimbal.c](https://github.com/HNUYueLuRM/basic_framework/blob/1a136eb1ed2f101110348d5a5cd182272eada38e/application/gimbal/gimbal.c)、[shoot.c](https://github.com/HNUYueLuRM/basic_framework/blob/1a136eb1ed2f101110348d5a5cd182272eada38e/application/shoot/shoot.c) | 命令层处理输入，发布云台/发射命令；各子系统执行命令并反馈状态 |
| 跃鹿 MC02 移植，`14ea0f490ba984015f775c39da75887b6cfe446c` | [gimbal.c](https://github.com/HNUYueLuRM/basic_framework_MC02/blob/14ea0f490ba984015f775c39da75887b6cfe446c/application/gimbal/gimbal.c)、[robot_task.h](https://github.com/HNUYueLuRM/basic_framework_MC02/blob/14ea0f490ba984015f775c39da75887b6cfe446c/application/robot_task.h) | 板卡移植后仍保留命令/应用/电机任务分工；云台示例执行器仍是 GM6020，不能因此套用到 J4310 |
| ARUW Taproot，`2e3b59b88520de92aa410763249dda20091e9982` | [subsystem.hpp](https://github.com/uw-advanced-robotics/taproot/blob/2e3b59b88520de92aa410763249dda20091e9982/src/tap/control/subsystem.hpp)、[command_scheduler.cpp](https://github.com/uw-advanced-robotics/taproot/blob/2e3b59b88520de92aa410763249dda20091e9982/src/tap/control/command_scheduler.cpp) | 子系统封装资源，调度器检查命令对资源的要求、结束冲突命令，并处理断连 |
| ARUW 整机代码，`681f847b5fe893f5419d0784ce915381c3f07a23` | [turret user command](https://github.com/uw-advanced-robotics/aruw-mcb/blob/681f847b5fe893f5419d0784ce915381c3f07a23/aruw-mcb-project/src/aruwsrc/control/turret/user/turret_user_world_relative_command.cpp)、[turret subsystem](https://github.com/uw-advanced-robotics/aruw-mcb/blob/681f847b5fe893f5419d0784ce915381c3f07a23/aruw-mcb-project/src/aruwsrc/control/turret/turret_subsystem.cpp)、[friction wheel subsystem](https://github.com/uw-advanced-robotics/aruw-mcb/blob/681f847b5fe893f5419d0784ce915381c3f07a23/aruw-mcb-project/src/aruwsrc/control/launcher/friction_wheel_subsystem.hpp) | 输入动作与云台资源分开；摩擦轮支持多个轮，各有速度 PID，目标有斜坡，并包含依赖实际弹速反馈的扩展 |

Meta 上游 [README](https://github.com/Meta-Team/Meta-Embedded-NG/blob/a0ebb9596b9cf5a827140eb4209af1628ea67525/README.md)
声明基于跃鹿框架。因此本地继续沿用 `bsp/module/application` 有继承依据；目录本身无需换一套名字。
ARUW 属于另一套 C++ 体系，适合对照职责和资源管理，不能整套直接搬进当前 C/HAL 工程。
本轮没有把上游实现复制进生产源码，也没有添加第三方运行依赖。下载的只读对照文件在 `.scratch/upstream-review/`，不参加固件构建。

## 判断与已经落地的修改

| 对照项 | 本地判断 | 本次处理 |
| --- | --- | --- |
| 命令生成与执行分开 | 之前云台核心持有 DBUS 解码器、通道和 S1 配置，换输入会牵动执行器 | 新增 `application/commands/gimbal_remote`；执行器接收 `GimbalCommand_s`，单位为电机侧 rad/s |
| 命令/反馈契约明确 | 输入在线、请求使能、电机确认运行不能混为一个状态 | 状态快照分为 `remote` 与 `axis`；命令保留原始时间戳、有效性、停机事件 |
| 资源只有一个执行者 | 当前一个来源、一个云台控制任务，不需要完整命令调度器 | 保持单任务所有者；接口中明确多个来源必须先仲裁，不新增线程和动态分配 |
| 控制核心独立测试 | 目录分层需能被编译和依赖检查验证 | 架构检查禁止云台核心包含 DBUS/遥控/VOFA，禁止命令源包含执行器实现；增加不经过遥控协议的命令控制测试 |
| 摩擦轮速度控制 | 多轮独立速度闭环和目标斜坡与所查实现一致 | 保留三路 PID、单轮反馈、整组到位条件；不因参考工程用了两个轮就改机械方案或轮数 |
| 云台闭环层次 | 上游 GM6020 角度/速度串级输出与本地 J4310 驱动内速度环不同 | 保留 J4310 速度接口；已准备的角度外环仍等待坐标和实物标定，不重复增加同类速度环 |

当前链路是：

```text
输入解释 application/commands
    → 带单位和原始时间戳的命令
    → 执行子系统 application/gimbal/control
    → 控制/协议 module
    → 板级收发 bsp

执行状态 + 输入状态 → 一致快照 → USB/RTT 诊断
```

对读代码的人，区别在于“遥控怎么映射”只看命令层，“电机何时允许动、怎样输出”看执行层。
两部分仍在同一个任务中顺序调用，没有因为分文件而增加并发复杂度。

## 为什么没有直接复制整个消息中心和控制参数

跃鹿的 [message_center.c](https://github.com/HNUYueLuRM/basic_framework/blob/1a136eb1ed2f101110348d5a5cd182272eada38e/modules/message_center/message_center.c)
使用动态分配和通用队列；我们当前固定数量的实例、明确类型的命令/快照已经可以表达交互。
后续需要跨任务时，应围绕实际所有者和消息容量实现同步，不能因为有发布/订阅名字就假定线程安全。

跃鹿所查云台文件仍有未收到命令的待办提示，发射示例也有需要实测填入的弹速目标。
这说明应学习其职责拆分，同时保留本地已有的命令有效期、断连撤销和恢复条件。
ARUW 摩擦轮文件包含具体平台的时步与弹速反馈约定；这些依赖不适用于尚无弹速测量的本地机构。
此处判断的是适配条件，不是对整个上游工程可靠性下结论。

## 当前还不能声称完成的部分

- 多输入控制权：接口已拆开，遥控/视觉/调试的模式优先级和切换流程尚未实现。
- 总线合并：`gimbal_port` 仍独占 CAN1/UART5，DJI 仍需一个任务统一发送；两个 bench 不能直接合并回调。
- 完整云台：Yaw 角度核心已有离线验证，实物轴向、零点、传动比和 Pitch 信息仍缺失。
- 完整发射：三轮速度合格不等于允许供弹，供弹机构、裁判限制与弹速标定要独立完成。

后续修改按这些实际边界推进。开源参数、算法复杂度和“编译通过”都不能代替本机构的测量结果。

## 本次验证

验证记录见 本轮报告（历史资源未随仓库提交：`../../.scratch/upstream-review/verification.md`）。
包括真实生产代码主机测试、两套 ARM 固件构建，以及重构前后同一故障/恢复场景的 CAN 输出逐条比较。
没有连接硬件或烧录；这些检查不证明电机响应、PID 效果、IMU 实物误差或发射性能。
