# 可移植 VOFA 波形与调参协议模块

只需复制 `vofa.c` 和 `vofa.h` 并加入编译。C11，无 HAL、RTOS、DJI、电机或具体串口依赖；
无动态内存分配，无 printf/strtof，所有容量固定，多个实例彼此独立。
要求 IEEE754 binary32 `float`；编译时检查格式，发送时显式编码小端字节序。
不是 VOFA 桌面程序插件，也不是自动整定器。

## 接入契约

1. 长期存储一个 `Vofa_s`，启动时调用 `VofaInit()`。
2. 提供 `Vofa_Port_s.read/write/context`。不需要修改协议核心。
3. 注册 `get/set/call/rx_fault` 业务回调，决定参数 ID、范围、生效和停止规则。
4. 一个低优先级任务或主循环周期调用 `VofaPoll(&instance, monotonic_ms)`，
   再按所需频率调用 `VofaSend(&instance, values, count)`。同一实例禁止并发、递归调用。
5. 控制任务与通信任务通过业务自己的邮箱和快照交接。协议库不直接写控制器变量。

`read(context, destination, capacity)` 必须立即返回：

- 正数：实际复制字节数，不能超过 capacity；每次 Poll 最多处理 128 字节。
- 0：暂无数据。
- -1：丢字节/溢出/传输中断。端口先清空接收队列再返回 -1；核心会通知业务停止，
  丢弃输入至下一行 LF，避免把残缺命令拼接成新命令。

UART 中断/DMA 回调只向端口接收队列写入，不调用解析器或业务调参函数。
队列读写的临界区、内存屏障和并发保证由端口实现。连接重建时应清空旧队列并报告 RX loss。

`write(context, data, length)` 也必须立即返回：

- 1：已将**整帧复制**到端口拥有的缓冲区；随后可异步发送。
- 0：忙，**一个字节也未发送/入队**；当前帧直接丢弃，`tx_dropped` 增加。

data 仅在回调期间有效，严禁把它直接交给异步 DMA/USB 驱动后返回成功。
端口应使用自己的双缓冲或帧队列，按顺序完整发送；不能出现半帧发送后返回 busy。
以 STM32H7 为例，DMA 可访问内存、D-cache 对齐/维护、DMA 完成标记均属于端口职责。
不要在发送回调中使用阻塞式 UART 发送或等待 DMA 完成。

两种回调均可留空。当前工程的空接入位置是
`application/friction/input/friction_debug_port.c`，没有默认 UART、波特率或引脚。

## 下行：ASCII 命令

```text
get SEQ ID
set SEQ ID VALUE
call SEQ OP [ARG1 ... ARG8]
```

每条以真实 LF（0x0A）或 CRLF 结尾。SEQ 是 1..65535 的十进制整数，ID/OP 为 0..65535。
调试端自行分配序号；0 专用于无法解析/初始状态。序号用于关联回执，不是去重或身份验证机制。
`get/set` 参数表和 `call` 操作表分别由业务解释。数值支持正负十进制、小数、科学计数法；
拒绝 NaN、Inf、十六进制、溢出及下溢到零的非零数值。空行忽略。

最长 159 字节（不含 LF；CR 占 1 字节）；超过长度、控制字符和错误语法通知 rx_fault，
丢弃至下一行。半条命令从开始处理起超过 100 ms 未完成也丢弃至下一行。
单次工作量有界；持续输入速度必须不超过 Poll 和端口队列的处理能力。
这套文本下行是本模块定义的协议，**JustFloat 本身只规定上行波形**。

`get` 回调返回已生效参数；`set` 可返回 QUEUED 表示已排队，业务需另提供生效序号或状态。
同一帧中只保留**最近一条命令**回执。客户端按回执串行调参，不要批量发送后假设每条都成功。
连续运行命令也可能覆盖回执；可用业务的已生效序号确认修改，查询时需安排发送顺序。

## 上行：JustFloat

每帧是 N 个小端 float32，后跟 `00 00 80 7F`，符合
[VOFA 官方 JustFloat 文档](https://www.vofa.plus/docs/learning/dataengines/justfloat/)。
最多 48 通道（包括前 4 个回执通道）。一个连接固定通道数，日志不要混入此字节流。

| 0 起通道编号 | 内容 |
| --- | --- |
| 0 | 最新回执 SEQ，0 表示初始或无法归属到命令的错误 |
| 1 | 结果码，见下表 |
| 2 | 参数 ID 或操作 OP；根据发送的命令解释 |
| 3 | get：已生效值；set：请求值，**不是生效确认**；call：0 |
| 4 开始 | 业务波形数组 |

| 结果码 | 含义 |
| --- | --- |
| 0 OK | 查询/同步修改成功 |
| 1 QUEUED | 请求已排队，尚不代表执行或电机已启动 |
| 2 SYNTAX | 命令格式或数值格式错误 |
| 3 RANGE | 参数超出业务范围/操作参数不合法 |
| 4 UNKNOWN | 未知参数或操作，不支持的读写 |
| 5 BUSY | 上一事务待处理或基准快照过期，稍后重新读取/提交 |
| 6 UNAVAILABLE | 当前业务未就绪，例如电机 ID 未配置 |
| 7 RX_LOSS | 端口丢数据或半条命令超时 |

非有限波形值会拒绝整帧并计入 tx_dropped，避免 Inf 与帧尾冲突；业务应另外保留故障诊断渠道。
默认 ACK=0/0/0/0 只表示尚无回执。必须匹配非零 SEQ 才能确认某条命令。

## 移植最小框架

```c
static Vofa_s link;

void DebugInit(void)
{
    Vofa_Port_s port = { MyReadRxQueue, MyCopyTxFrame, &my_port };
    Vofa_Handler_s handlers = {
        .get = MyGetAppliedParameter,
        .set = MyQueueParameter,
        .call = MyQueueOperation,
        .rx_fault = MyRequestStop,
        .context = &my_application
    };
    VofaInit(&link, &port, &handlers);
}

void DebugService(uint32_t now_ms)
{
    VofaPoll(&link, now_ms);
    /* 实际工程按独立节拍发送，先取得完整业务快照。 */
    if (TelemetryDue(now_ms)) {
        float channels[] = { ReadTargetSnapshot(), ReadSpeedSnapshot() };
        VofaSend(&link, channels, 2); /* 上位机总共 6 个通道 */
    }
}
```

以上 My* 函数由新工程实现，函数签名见 `vofa.h`；不必复制三摩擦轮应用、FreeRTOS 或 DJI 驱动。
业务回调必须有界且不阻塞。涉及控制器参数时建议整组邮箱提交，由控制任务生效。

## 验证

`tests/vofa/run_tests.ps1 -Compiler <主机gcc>`：严格 C11 编译，覆盖拆包、粘包、非法数值、
语法、长度溢出、跨 tick 回绕的半行超时、RX 丢失恢复、JustFloat 字节序和 TX busy。
实际串口尚未指定，因此没有验证实体端口、DMA、实际吞吐或桌面 VOFA 连板显示。
