# module：可复用组件

[首页](../README.md) · [项目地图](../docs/start/01-project-map.md)

| 分类 | 放什么 |
| --- | --- |
| [control](control/README.md) | 速度/角度控制、到位判断、控制权 |
| [algorithm](algorithm/algorithm.md) | 现有 PID、滤波、CRC 和数学工具 |
| [protocol](protocol/README.md) | 遥控、电机帧、VOFA、文本、裁判等协议 |
| [sensor](sensor/README.md) | BMI088、姿态、采样类型和旧 IMU 链 |
| [motor](motor/README.md) | DJI、DM、XM、电机组、舵机等组件 |
| [system](system/README.md) | 存储核心、运行统计、事件历史、消息与离线监控 |
| [device](device/README.md) | 蜂鸣器与超级电容 |

新的纯 C 控制/协议核心可由电脑测试验证；部分保留设备组件仍依赖 BSP，不应把整个 module 目录都理解成硬件无关代码。
