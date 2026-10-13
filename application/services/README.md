# services：共享应用服务

[应用目录](../README.md)

| 服务 | 作用 |
| --- | --- |
| [diagnostics](diagnostics/README.md) | USB 控制台、状态报告与周期输出调度 |
| [parameters](parameters/README.md) | 完整参数包校验、排队和 RAM 应用 |
| [runtime](runtime/README.md) | 状态变化历史与调用间隔观察 |

这些服务帮助应用通信和观察。电机运行状态仍由相应控制任务拥有，不能通过状态查询续期运行命令。
