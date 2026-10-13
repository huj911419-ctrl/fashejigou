# 04 顺着一轮云台控制读代码

[文档目录](../README.md) · [源码目录说明](../../application/gimbal/README.md)

当前独立云台是水平轴速度控制。板载 IMU 有独立观察链，连续角度控制核心尚未接入电机链。

| 顺序 | 文件 | 看什么 |
| --- | --- | --- |
| 1 | [robot.c](../../application/gimbal/robot.c) | `RobotInit()` 初始化云台、IMU 和诊断 |
| 2 | [robot_task.c](../../application/gimbal/robot_task.c) | `YawTask`、`ImuTask` 各自做什么 |
| 3 | [gimbal.c](../../application/gimbal/gimbal.c) | `GimbalTask()` 取输入、推进控制、发布状态 |
| 4 | [commands/gimbal_remote.c](../../application/commands/gimbal_remote.c) | 把遥控通道、开关、死区转换成带时间戳的命令 |
| 5 | [control/gimbal_control.c](../../application/gimbal/control/gimbal_control.c) | 消费内部命令，处理执行器状态；不解析遥控协议 |
| 6 | [yaw_rate.c](../../module/control/yaw_rate/yaw_rate.c) | 角速度斜坡、使能和失效判断 |
| 7 | [dm_velocity.c](../../module/protocol/dm_velocity/dm_velocity.c) | 达妙速度模式协议与握手 |
| 8 | [gimbal_port.c](../../bsp/ports/gimbal_port/gimbal_port.c) | 此板 CAN1 / UART5 的收发和队列 |

## IMU 为什么另放一个目录

[imu/gimbal_imu.c](../../application/gimbal/imu/gimbal_imu.c) 组合传感器采集和姿态解算。`bsp/ports/imu_port/` 处理 SPI，`module/sensor/bmi088_sensor/` 处理传感器协议，`module/sensor/attitude/` 处理姿态。

IMU 任务独占 SPI2。默认安装方向尚未确认，诊断中的姿态不能直接当作已实现的稳定控制；六轴 IMU 的相对偏航也会漂移。具体状态含义见 [IMU 说明](../gimbal/IMU.md)。

先读 [config/gimbal_config.h](../../application/gimbal/config/gimbal_config.h) 认清控制单位，再读 [gimbal_command.h](../../application/gimbal/control/gimbal_command.h) 理解命令数据。
