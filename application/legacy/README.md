# legacy：保留的旧机器人业务

[应用目录](../README.md)

| 目录/文件 | 内容 |
| --- | --- |
| [sentry](sentry) | 旧哨兵初始化、任务与物理参数 |
| [infantry](infantry) | 旧步兵初始化、任务与物理参数 |
| [chassis](chassis) | 旧底盘业务 |
| [cmd](cmd) | 旧遥控、自动和图传命令业务 |
| [gimbal](gimbal) | 旧双云台业务 |
| [shoot](shoot) | 旧摩擦轮/供弹业务 |
| [robot_def.h](robot_def.h) | 旧应用消息契约和机器人参数选择 |

`make sentry` / `make infantry` 显式编译旧应用，原 `make ROBOT=...` 也可用。它们继续使用公共 module/bsp 组件。

学习当前独立控制时，从 [friction](../friction/README.md) 或 [gimbal](../gimbal/README.md) 进入。
