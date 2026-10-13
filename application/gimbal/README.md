# gimbal：当前独立云台应用

[应用目录](../README.md) · [逐步阅读路线](../../docs/start/04-read-gimbal.md) · [使用说明](../../docs/gimbal/README.md)

先按 `robot.c` → `robot_task.c` → `gimbal.c` 阅读。这里是水平轴速度控制应用；旧双云台代码在 [legacy/gimbal](../legacy/gimbal)。

| 文件/目录 | 负责什么 |
| --- | --- |
| [robot.c](robot.c) / [robot.h](robot.h) | 应用初始化 |
| [robot_task.c](robot_task.c) / [robot_task.h](robot_task.h) | Yaw、IMU 和诊断任务 |
| [gimbal.c](gimbal.c) / [gimbal.h](gimbal.h) | 板级输入、命令和执行控制的组合 |
| [config](config/README.md) | 云台、遥控映射与 IMU 配置 |
| [control](control/README.md) | 内部命令契约与执行控制核心 |
| [imu](imu/README.md) | 独立采样、校准和姿态观察链 |
| [diagnostics](diagnostics/README.md) | 云台状态报告 |
| [build.mk](build.mk) | 本应用源码清单 |

构建：在工程根目录运行 `make gimbal -j4`。输出在 `build/gimbal_bench/`。角度核心测试通过不等于已经实现实机角度保持。
