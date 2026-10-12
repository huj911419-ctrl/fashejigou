# 输入如何变成云台命令

这里负责把“人怎样操作遥控器”翻译成“希望电机怎样运动”。
具体依据和其他开源工程的取舍见 [开源对照记录](../../docs/architecture/open-source-review.md)。

当前实现是 `gimbal_remote.c`：接收 DBUS 字节，解析通道和 S1，处理方向和死区，
生成一份 `GimbalCommand_s`。它没有电机对象，也不能发送 CAN 或直接使能电机。

摩擦轮独立输入在 `friction_remote.[ch]`：S1/S2、CH1 死区和正向转速目标、
原始帧超时、物理 OFF + 回中解锁。输出自己的纯 C 请求类型，不依赖摩擦轮执行核心。
轮掩码和来源邮箱由 `friction_remote_bench` 接入；操作/移植见 [REMOTE.md](../friction_bench/REMOTE.md)。

```text
遥控字节
  → GimbalRemoteReceiveByte：得到通道、开关、原始接收时间
  → GimbalRemoteTakeCommand：转成电机侧 rad/s 请求
  → GimbalControlStep：检查时效、允许条件、限速、斜坡和电机状态
  → DM 协议 → 板级 CAN
```

当前这些调用按上述顺序在同一个 2 ms 云台任务里完成，新增文件不意味着新增线程。
每份命令通过明确类型的结构体交接；这里不需要另建动态话题注册或消息调度框架。

## 职责和单位

| 内容 | 所属位置 |
| --- | --- |
| DBUS 帧同步、18 字节解析 | `module/dbus_input` |
| CH0、S1、正反向、0.04 死区与满杆转速映射 | 本目录 `gimbal_remote` |
| 命令格式与原始时间戳契约 | `application/gimbal_control/gimbal_command.h` |
| 命令过期检查、OFF→ON、零速解锁、输出斜坡 | `gimbal_control`、`module/yaw_rate` |
| 电机模式读回、反馈、实际使能确认和故障锁定 | `module/dm_velocity` |
| 具体串口、CAN、队列与任务编排 | `bsp/gimbal_port`、`application/gimbal_bench` |

死区内请求严格为 0，超出后把剩余行程线性映射到满量程。
例如满杆为 1 rad/s、死区为 0.04 时，归一化摇杆 0.52 对应 0.5 rad/s。
这是电机侧请求，传动比和云台/世界坐标变换需要在以后明确机械关系后补入。

## 不应丢失的信息

1. `received_ms` 始终取原始有效帧时间。每 2 ms 读取旧帧不会延长它的有效期；云台核心还独立检查命令超时。
2. `stop_requested` 记录本批输入里是否出现过非法帧或 OFF。即使稍后又来 ON，也不能把前面的停机吞掉。
3. 启动、失联或异常后要重新经历 OFF→ON，且 ON 时请求为零；源恢复本身不等于电机获准重新运行。
4. `GimbalStatus_s.remote` 是遥控诊断，`.axis` 是执行器诊断。遥控在线、应用允许、电机反馈在线、电机确认 ON 是不同状态。

`GimbalRemoteTakeCommand` 会消费停机标记，因此一轮只调用一次，并在同一轮交给控制器。
USB/日志读取已经发布的快照，不能调用它来查询状态。传输出错时，组合层同时使输入失效并通知执行器。

## 后续接视觉或其他输入

执行器接口现在已经不依赖 DBUS，可以独立测试或移植。但这不等于已经实现视觉控制权管理。
加入第二个来源前，要在命令层确定唯一来源、优先级、切换停机/目标捕获规则和租约，
再由唯一控制任务消费。不能让遥控和视觉任务轮流直接调用 `GimbalControlStep`。
角度控制还需要坐标、零点、参考编号、反馈有效性及标定，不能把视觉角度直接填进 `rate_rad_s`。

## 接口迁移

- 旧 `GimbalControlReceiveByte` 改为 `GimbalRemoteReceiveByte`；云台核心只保留 CAN 反馈和命令执行。
- `GimbalControlStep` 新增 `const GimbalCommand_s *` 参数；NULL 视为无效输入并撤销允许，不能作为续期。
- `YawRateStep` 最后一个参数改为 **rad/s**，不再是归一化摇杆量。死区配置移至命令层。
- 应用诊断字段由 `status.enabled` 等移到 `status.axis.enabled`；遥控字段在 `status.remote`。

本仓库所有维护中的调用点、构建清单和测试已随此次变更更新；外部复制过旧模块的工程需按此迁移。
