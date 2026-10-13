# 配置在哪里改

[文档目录](../README.md)

先找设置的负责人，再修改具体字段。目录迁移保留现有配置数值；文件归类不会自动改变电机接线和参数。

| 想设置什么 | 文件 | 需要理解什么 |
| --- | --- | --- |
| 哪些独立模块参与运行 | [bench_features.h](../../application/bench_features.h) | `BENCH_ENABLE_*`；开关不等于运行授权 |
| 摩擦轮 CAN、ID、方向、初始 PID、限流 | [friction_config.h](../../application/friction/config/friction_config.h) | F1/F2/F3 编号固定；电流字段是 C620 原始单位 |
| 摩擦轮遥控档位、开关、超时 | [friction_remote_config.h](../../application/friction/config/friction_remote_config.h) | 输入转换与重解锁条件 |
| 云台速度上限、死区、方向、超时 | [gimbal_config.h](../../application/gimbal/config/gimbal_config.h) | 电机侧角速度单位 rad/s |
| IMU 采样、校准、安装映射 | [gimbal_imu_config.h](../../application/gimbal/config/gimbal_imu_config.h) | 安装确认与观察有效性 |
| 具体 CAN/UART/SPI 的适配 | [bsp/ports](../../bsp/ports/README.md) | 收发资源和唯一所有者 |
| 旧整机物理参数 | [legacy/sentry](../../application/legacy/sentry/sentry_def.h)、[legacy/infantry](../../application/legacy/infantry/infantry_def.h) | 仅对应旧机器人构建 |

## 编译配置与运行参数

改 `.h` 后，需要重新编译和烧录才影响板子。USB/VOFA 的运行时参数在 RAM 中，是否已应用要查实际版本/状态。真实 Flash 保存尚未接入，不能把 RAM 调参当作掉电保存。

先查看 [模块开关](../architecture/module-switches.md)，再查看 [参数服务](../../application/services/parameters/README.md)。接线/方向/安装轴仍以自己的硬件确认结果为准。
