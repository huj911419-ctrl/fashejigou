# 文档导航

按你现在的问题找入口。第一次阅读从 [从这里开始](../从这里开始.md) 进入。

| 我的问题 | 文档 |
| --- | --- |
| 整个文件夹怎么分工？ | [01 项目地图](start/01-project-map.md) |
| 从上电到循环，代码怎么走？ | [02 启动过程](start/02-startup.md) |
| 摩擦轮代码应该按什么顺序看？ | [03 摩擦轮阅读路线](start/03-read-friction.md) |
| 云台代码应该按什么顺序看？ | [04 云台阅读路线](start/04-read-gimbal.md) |
| Linux 上怎么编译、测试？ | [构建指南](guides/build.md) |
| 开关和参数在哪改？ | [配置位置](guides/configure.md) · [开关语义](architecture/module-switches.md) |
| 不运行、掉线、日志太多怎么办？ | [调试入口](guides/debug.md) |
| USB/遥控/VOFA 怎么用？ | [摩擦轮功能说明](friction/README.md) · [USB 日志](friction/USB_LOG.md) · [遥控](friction/REMOTE.md) · [VOFA](friction/VOFA.md) |
| IMU 当前做到哪一步？ | [云台说明](gimbal/README.md) · [IMU](gimbal/IMU.md) |
| 配置包和运行事件是什么？ | [参数服务](../application/services/parameters/README.md) · [运行记录](../application/services/runtime/README.md) |
| 为什么按这样分目录？ | [布局设计与开源参考](architecture/layout-design.md) · [控制边界](architecture/control-system.md) |
| 老文件搬到哪里了？ | [完整路径迁移表](reference/path-migration.md) |
| 这些术语是什么意思？ | [词汇表](reference/glossary.md) |
| 大改以后验证过什么？ | [结构重构验证](verification/layout-refactor.md) · [测试目录](../tests/README.md) |
| 想看过去的研究记录？ | [历史资料](history/README.md) |

`start/` 用于学习，`guides/` 用于操作，`reference/` 用于查找；`architecture/` 记录设计，`verification/` 记录验证，`history/` 保留历史背景。每份当前文档应链接到现有文件。
