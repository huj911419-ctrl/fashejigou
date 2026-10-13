# application：从功能入口开始

[返回首页](../README.md) · [阅读第一步](../从这里开始.md)

| 目录/文件 | 负责什么 | 建议入口 |
| --- | --- | --- |
| [friction](friction/README.md) | 当前独立三摩擦轮 | `robot.c` → `robot_task.c` → `friction.c` |
| [gimbal](gimbal/README.md) | 当前独立云台 | `robot.c` → `robot_task.c` → `gimbal.c` |
| [commands](commands/README.md) | 遥控转换与命令仲裁 | 命令结构和输入转换 |
| [services](services/README.md) | 两套应用共享的诊断、参数、运行记录 | 按需要查看 |
| [legacy](legacy/README.md) | 旧哨兵/步兵等整机业务 | 研究旧框架时查看 |
| [bench_features.h](bench_features.h) | 独立应用功能开关 | [配置指南](../docs/guides/configure.md) |

每次构建只选一套应用入口。应用决定“做什么”，具体协议和硬件分别由 `module/` 与 `bsp/` 提供。
