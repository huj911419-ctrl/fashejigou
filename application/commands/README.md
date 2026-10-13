# 输入如何变成控制命令

[应用层](../README.md) · [完整项目地图](../../docs/start/01-project-map.md)

本目录把“人怎样操作”翻译成“希望电机怎样运动”。它只生成带单位和原始时间戳的请求，不拥有电机，不发送 CAN，不直接调用 HAL。

## 三组文件

| 文件 | 负责什么 | 下一步交给谁 |
| --- | --- | --- |
| [gimbal_remote.c](gimbal_remote.c) / [.h](gimbal_remote.h) | 云台 DBUS 输入、CH0 方向/死区、S1 允许条件；生成电机侧 rad/s 请求 | `gimbal_arbiter`，再交给云台控制器 |
| [gimbal_arbiter.c](gimbal_arbiter.c) / [.h](gimbal_arbiter.h) | 检查来源、切换代次、请求顺序和有效期，交出唯一有效请求 | [gimbal_control.c](../gimbal/control/gimbal_control.c) |
| [friction_remote.c](friction_remote.c) / [.h](friction_remote.h) | 摩擦轮 S1 停止/低速/高速组，S2 选择组内三档；生成转子 RPM 请求 | [friction_remote_bench.c](../friction/input/friction_remote_bench.c) 的来源与邮箱适配 |

底层 18 字节 DBUS 帧解析在 [module/protocol/dbus_input](../../module/protocol/dbus_input/dbus_input.h)，不在业务控制器里重复实现。

## 云台：沿着一轮调用读

[应用组合层 GimbalTask](../gimbal/gimbal.c) 在同一个 2 ms 云台任务中依次取出输入、生成请求、仲裁、调用控制器、发布状态快照。新增目录或文件不意味着新增线程。

1. `GimbalRemoteReceiveByte` 收集字节，保留原始接收时间。
2. `GimbalRemoteTakeCommand` 将通道映射为 [GimbalCommand_s](../gimbal/control/gimbal_command.h)。满杆速度、方向、死区来自 [gimbal_config.h](../gimbal/config/gimbal_config.h)。
3. 开启控制权开关时，`GimbalArbiterSubmit/Step` 检查来源及租约；当前应用仅接入遥控来源。
4. `GimbalControlStep` 检查执行许可、超时、斜坡和电机状态，最终通过 DM 协议与板级 CAN 发送。

死区内请求严格为 0，超出后线性映射剩余行程。例如满杆 1 rad/s、死区 0.04 时，归一化摇杆 0.52 对应 0.5 rad/s。这是电机侧角速度，不能直接当成世界坐标角度。

## 摩擦轮：开关选档，摇杆只供观察

S1 选择停止、低速组或高速组；S2 在运行组里选三档，共六档。摇杆既不控制转速，也不参与解锁。上电、失联或异常后，需要收到新的 S1 停止帧，再选择运行组。

档位表、停止条件和 USB 来源切换见 [六档遥控操作](../../docs/friction/REMOTE.md)；配置在 [friction_remote_config.h](../friction/config/friction_remote_config.h)。目标 RPM 不等于实际转速，更不等于弹速。

## 阅读时留意的边界

- 原始时间戳不随任务读取刷新；只有新输入可以延长运行有效期。
- 同一批数据中出现过 OFF 或非法帧，后来的 ON 不能吞掉停机请求。
- `Take` 会消费停机标记，一轮只调用一次；日志读取已发布的快照。
- 遥控在线、应用允许和电机实际 ON 是不同状态，分别查看输入和执行器诊断。
- 扩展视觉等来源时，先接入仲裁与同步邮箱；所有来源仍由唯一控制任务消费。当前没有完整视觉闭环，角度保持也尚未接入电机链。
