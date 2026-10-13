# friction：当前三摩擦轮应用

[应用目录](../README.md) · [逐步阅读路线](../../docs/start/03-read-friction.md) · [使用说明](../../docs/friction/README.md)

先按 `robot.c` → `robot_task.c` → `friction.c` 阅读。

| 文件/目录 | 负责什么 |
| --- | --- |
| [robot.c](robot.c) / [robot.h](robot.h) | 应用初始化入口 |
| [robot_task.c](robot_task.c) / [robot_task.h](robot_task.h) | 控制和调试任务，安排执行周期 |
| [friction.c](friction.c) / [friction.h](friction.h) | 命令来源、邮箱、调参交接、控制任务与快照接口 |
| [config](config/README.md) | 电机、控制和遥控配置 |
| [control](control/README.md) | 控制状态机和到位判断 |
| [input](input/README.md) | USB/遥控/VOFA 输入适配 |
| [diagnostics](diagnostics/README.md) | 中文状态、USB 日志、统计 |
| [build.mk](build.mk) | 本应用明确需要编译哪些源码 |

构建：在工程根目录运行 `make friction -j4`。输出在 `build/friction_bench/`；旧命令继续可用。
