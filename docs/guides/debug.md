# 调试时先看什么

[文档目录](../README.md)

先分清“没有收到请求”“请求被拒绝”“控制允许但硬件没有反馈”。诊断中的软件使能不等于轴已经转动。

| 问题 | 入口 |
| --- | --- |
| 摩擦轮 USB 日志怎么读、怎么少输出 | [USB_LOG.md](../friction/USB_LOG.md)：`status`、`status detail`、`log 1/2`、`stream on/off` |
| 遥控如何选档和恢复 | [REMOTE.md](../friction/REMOTE.md) |
| VOFA 串口数据是什么 | [VOFA.md](../friction/VOFA.md) |
| 云台停止原因或 IMU 状态 | [公共诊断](../../application/services/diagnostics/README.md)、[IMU](../gimbal/IMU.md) |
| 短暂故障刚才发生了什么 | [运行记录](../../application/services/runtime/README.md)：`events` 与周期统计 |
| 电脑怎样保存参数文件 | [参数工具](../../tools/FRICTION_PROFILES.md) |
| 想在电脑上读 USB 文本 | [tools/README.md](../../tools/README.md) |

摩擦轮的查询与日志不续期控制命令。`stream off` 停止日志，`off` 是电机停止请求；运行命令还受来源、反馈、故障和有效期控制。具体操作顺序以相应功能说明为准。

复现问题时记录：构建版本、所选应用、模块开关、接线与设备、完整命令、原始日志。这样才能把源码行为与硬件现象对应起来。
