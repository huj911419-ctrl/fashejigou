# 03 顺着一轮摩擦轮控制读代码

[文档目录](../README.md) · [源码目录说明](../../application/friction/README.md)

第一轮只追踪“请求从哪里来、在哪里检查、怎样送到电机”。先看函数入口和它调用的名字，再看公式。

| 顺序 | 打开文件 | 关注点 |
| --- | --- | --- |
| 1 | [robot.c](../../application/friction/robot.c) | `RobotInit()` 组合控制、调试、遥控和诊断 |
| 2 | [robot_task.c](../../application/friction/robot_task.c) | `MotorTask` 周期调用 `FrictionRemoteBenchPoll()` 和 `FrictionTask()` |
| 3 | [friction.c](../../application/friction/friction.c) | 命令邮箱、来源切换、参数交接、状态快照；控制任务是运行状态的所有者 |
| 4 | [control/friction_control.c](../../application/friction/control/friction_control.c) | 根据命令、反馈、时间和故障推进控制；不依赖 HAL/RTOS |
| 5 | [dji_speed_group.c](../../module/motor/dji_speed_group/dji_speed_group.c) | 用组接口读取反馈、设置目标、提交控制输出 |
| 6 | [dji_motor.c](../../module/motor/dji/dji_motor.c) | DJI 电机协议和底层控制实现 |
| 7 | [bsp_can.c](../../bsp/peripherals/can/bsp_can.c) | 通过 STM32 CAN 外设传输数据 |

## 请求可以从哪里来

| 输入 | 接入文件 | 转换/解析后交到哪里 |
| --- | --- | --- |
| 遥控器 | [input/friction_remote_bench.c](../../application/friction/input/friction_remote_bench.c) | [commands/friction_remote.c](../../application/commands/friction_remote.c) 生成内部命令，再交给应用 |
| USB 文本 | [input/friction_usb_command.c](../../application/friction/input/friction_usb_command.c) | `FrictionSubmitFrom()` 或调参队列 |
| VOFA 调试协议 | [input/friction_debug.c](../../application/friction/input/friction_debug.c) | 由所选配置和当前接入方式决定可用请求 |
| RTT 调试 | [robot_task.c](../../application/friction/robot_task.c) | 按模块开关接收；源选择和控制保护仍生效 |

输入解析不等于电机已经执行。USB `QUEUED` 只是请求进队列；电机任务还要检查反馈、来源、有效期和故障。

## 状态显示是另一条路

`friction.c` 发布快照，`diagnostics/` 读取快照，生成中文状态或运行日志。查询不会续期运行命令。

- 想看为什么停机：读 [friction_status_text.c](../../application/friction/diagnostics/friction_status_text.c)。
- 想看日志字段：读 [USB 日志说明](../friction/USB_LOG.md)。
- 想看“已经到目标转速”的判断：读 [friction_monitor.c](../../application/friction/control/friction_monitor.c)。
- 想找参数：读 [配置位置](../guides/configure.md)。

这份独立应用负责三轮控制与观测。旧机器人的供弹业务位于 `application/legacy/shoot/`，两者不能因为名字相近就当成已经接好的完整系统。
