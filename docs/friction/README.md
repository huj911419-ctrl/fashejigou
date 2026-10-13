[文档导航](../README.md) · [源码入口](../../application/friction/README.md) · [新手阅读路线](../start/03-read-friction.md)

# 三 DJI 电机独立台架

模块可单独开启/关闭：修改 [`bench_features.h`](../../application/bench_features.h)。可只注册 F1、F2 或 F3，关闭轮不必填 ID，编号保持不变。具体操作见 [逐项验证配置](../architecture/module-switches.md)。

本目标只控制 120° 布置的三只 M3508/C620。它有自己的应用入口、任务和参数，
不编译或调用哨兵、步兵、底盘、云台、拨弹或自动发射应用。
复用 CubeMX 外设启动、FreeRTOS、BSP 和 DJI 驱动；没有另复制一套底层驱动。

已提供 [转速就绪与运行状态](READINESS.md)：通用单通道判定与三轮状态组合分离，
完成连续稳定时间、迟滞、掉速恢复及单轮测试隔离，可供后续发射协调层读取。
就绪仅表示摩擦轮速度条件，不等于完整发射许可。

已接入 [USB 硬件诊断](../../application/services/diagnostics/README.md)：原生 CDC 可报告三电机反馈、
转速/PID 分项、到位状态、CAN 和系统信息；默认简报周期为 5 秒；不占用待定 VOFA UART，也不通过查询使能电机。

摩擦轮 USB `off/set/pid/ramp` 命令保持现有语法。新增 [USB 故障诊断与 AI 运行日志](USB_LOG.md)：
`log 1` 默认每5秒显示少量中文：控制方式、停机原因/处理方法和三轮目标/实际RPM；`status` 查询一次，`status detail` 查看遥控/PID/温度/CAN详情，`status raw` 保留完整原始字段。`log 2` 输出带单位/配置/时间的逐轮数据与每秒统计摘要。
默认模式在 `bench_features.h` 的 `BENCH_FRICTION_USB_LOG_MODE` 选择 1 或 2。

已接入[六档遥控摩擦轮](REMOTE.md)，默认开启且上电默认 REMOTE。S1选择停止／低速／高速，S2选择组内三档；目标为1000/2000/3000和5000/6000/7000转子RPM。启动、断线或故障恢复必须先S1停止再选运行组；摇杆不影响转速。

USB控制需先发 `source usb`、`off`，待停机处理后每100ms发送原 `set` 命令。`source remote`切回遥控后需S1停止再启动。`off/set/pid/ramp`语法和UART10波形保留；USB `off`能停止遥控。遥控模块关闭时，USB开启则恢复默认USB来源。

## 构建

在项目根目录执行：

```sh
make friction -j4
```

构建会自动创建输出目录。也可使用 `make ROBOT=friction_bench -j8`。
独立入口 `Makefile.friction` 固定选择本目标，产物位于：

```text
build/friction_bench/Meta-Embedded-NG-friction_bench.elf
build/friction_bench/Meta-Embedded-NG-friction_bench.hex
build/friction_bench/Meta-Embedded-NG-friction_bench.bin
```

普通 `make` 显示帮助；完整说明见 [构建指南](../guides/build.md)。

## 填写配置

编辑 [friction_config.h](../../application/friction/config/friction_config.h) 的三行电机配置，依次对应 F1/F2/F3：

| 配置 | 意义 |
| --- | --- |
| `can_bus` | 1、2、3 对应 FDCAN1、2、3 |
| `id` | C620 的物理 ID，1..8；当前 CAN2 的 F1/F2/F3 为 1/2/3。启用轮填 0 会禁止初始化 |
| `direction` | +1 或 -1；让逻辑正转对应接触面向出口送弹的方向 |
| `kp` / `ki` | 速度 PI，误差单位为电机转子的度/秒 |
| `current_limit` | C620 电流命令原始单位，不是 A |

同总线 ID 不允许重复；不同总线允许相同 ID。未启用的 CAN 分组不会发送。
已启用分组中未注册的槽位发送零，因此本台架应独占所选电流命令分组，
不要让另一控制器同时驱动同组电调。

当前控制上限7500 rpm、默认斜坡2000 rpm/s、电流命令限幅14000。
三轮Kp=2.1，F1/F3 Ki=0.10、F2 Ki=0.02，Kd=0；已按2026-10-11日志固化，尚未完成弹速和持续温升验收。遥控最高档7000 rpm。
默认温度门槛 70°C，需要按实际电机、机构和试验要求确认。
所有转速均指电机转子，配置里没有隐含减速比或轮径换算。

板上电源引脚保持原 CubeMX `MX_GPIO_Init()` 的配置，台架不另外切换电源。
填完配置只允许开始接收反馈；不会自动启动电机。

## 调用方式

在任务上下文通过带来源的邮箱提交完整指令，包括三个目标及使能掩码。
成功初始化时已显式准备OFF；默认遥控还需要物理S1停止解锁。选择USB来源后需先OFF再RUN。
运行中切换来源或故障恢复时，仍需提交 OFF，让控制任务至少执行一次关闭，再提交新的 RUN：

```c
Friction_Command_s command = {
    .enable_mask = 1,                 // 1=F1, 2=F2, 4=F3, 7=全部
    .target_rpm = {200, 0, 0},
};
(void)FrictionSubmitFrom(FRICTION_SOURCE_USB, &command, received_ms);
```

从其他来源切到 USB 时，可用 `source usb` → `off` 完成前置步骤；仅在遥控关闭且上电默认来源为 USB 的配置下，干净上电不需要这两步。
兼容 `FrictionSubmit()` 仅代表 RTT 来源；
REMOTE/USB 已选中时 RTT RUN 会被拒绝，不能通过兼容接口绕过来源管理。
默认每 250 ms 内必须收到新的完整使能命令，建议命令源每 20～50 ms 发布一次。
没有自动续期：命令源停止时会进入故障并停止输出。

`enable_mask=0` / `FrictionDisable()` 是零电流滑行，不是主动刹车。
“使能但目标 0 rpm”仍然是速度闭环，可能产生制动电流，不能当作关闭。
支持有符号 rpm，便于逐只验证旋向；换向也经过目标斜坡。

`FrictionDisable()` 的请求不会被同周期随后的使能覆盖。控制任务处理关闭时，
会丢弃在处理前排队的使能，重新启动需再发一条新的使能命令。
命令和状态通过临界区整组复制，状态只能通过 `FrictionGetStatus()` 读取。

## 调试串口与 RTT 开发入口

协议框架见 [VOFA 波形与在线调参](VOFA.md)。当前用户配置已经把 UART10 绑定为 **仅波形输出**，
电机命令/逐项 PID 与斜坡调参通过 USB；本次遥控不改变 UART10。
完整参数的电脑保存、导出/导入见 [参数管理](../../application/services/parameters/README.md) 与 [操作说明](../../tools/FRICTION_PROFILES.md)。导入要求应用输出已停用，按整组应用到 RAM；可移植双槽存储核心已测试，但真实 Flash 仍未绑定。
通用可移植模块在 `module/protocol/vofa/`，硬件绑定文件为 `friction_debug_port.c`。
`Makefile.friction` 默认打开 VOFA、关闭 RTT 命令。下面的 RTT 接口仅供其他构建显式开启并选择 RTT 来源后使用。

诊断任务使用 RTT 通道 0，接收以换行结尾的 ASCII 命令：

```text
off
set 1 200 0 0
set 2 0 200 0
set 4 0 0 200
set 7 200 200 200
```

先发送 `off`，观察 `enabled=0`，再发送设速命令。运行时客户端必须持续发送，
一次手动输入仅在命令有效期内生效，之后超时停机。
格式错误或接收行过长会请求关闭。当前 DT7 与 USB 控制分别见 [遥控说明](REMOTE.md) 和 [USB 日志说明](USB_LOG.md)。

RTT 每 200 ms 输出状态，含故障位、在线/使能掩码、命令年龄、控制间隔，
以及每轮参考 rpm、反馈 rpm、电流原始值和温度。更完整状态通过 API 获取。

## 控制和故障行为

单个电机任务按 1 ms 绝对周期调度，顺序执行：

0. 有界读取 UART5 队列，遥控输入模块验证/解锁后提交带原始时间戳的命令。
1. 读取整组命令及带时间戳的反馈快照。
2. 检查配置、命令有效期、使能电机的反馈/温度和实际控制间隔。
3. 更新每轮转速斜坡，统一转换 rpm → 度/秒及电机方向。
4. 通过 `DjiSpeedGroupControl()` 适配调用 `DJIMotorControlGuarded()`，每只电机计算速度 PID（默认 Kd=0），统一 CAN 分组发送。
5. 根据最终控制状态更新就绪判定，发布带时间戳的完整快照。

在线 PID/斜坡事务由电机任务在周期开始应用；`FrictionGetTelemetry()` 提供同周期状态、
P/I/D、输出电流指令、已生效参数及事务序号。只有台架启用 `PID_Continuous`，
误差为零时保留积分维持力矩；其他应用原有 PID 选项保持不变。

只有被使能的电机需要在线；因此单轮测试时其他两轮可以断电。
三轮同时使能时任何一轮超时都会使整组停止。默认反馈有效期 50 ms。
控制间隔为 0 或大于 5 ms 时，运行中的电机组进入故障；不补算错过的周期。
看门狗只由电机任务刷新。

保护发送路径会跳过停止电机的 PI 并清理其动态状态、刷新时间基准，
拒绝非有限输入/输出，限制最终协议电流，并检查 Bus-Off、发送 FIFO 和入队结果。
不调用原驱动带等待及打印的 CAN 发送路径。原机器人仍调用原 `DJIMotorControl()`。

| 故障位 | 原因 |
| --- | --- |
| 1 | 电机/控制配置无效 |
| 2 | 命令超时、掩码错误、目标非有限或越界 |
| 4 | 使能电机无有效反馈或反馈超时 |
| 8 | 使能电机温度达到门槛 |
| 16 | 控制周期异常 |
| 32 | 驱动计算、CAN 状态或发送入队失败 |

故障会锁存。故障排除后，先提交关闭，再提交新的使能；恢复通信不会自动重启。
API 返回的停止状态表示软件已禁止后续非零输出；CAN 入队不等于电调已收到，
总线故障时无法保证零电流报文送达，且先前已入队的报文可能仍在硬件队列中。

## 验证

主机测试编译真实的台架控制、DJI 驱动和 PID，仅替换 HAL/CAN/RTOS 接口：

```powershell
./tests/friction_bench/run_tests.ps1 -Compiler "你的本机gcc路径"
```

覆盖未配置 ID 时禁止注册、跨总线/跨分组打包、旋向、斜坡、超时、故障锁存、
关闭优先、温度、Bus-Off/FIFO 满、停止后 PI 清理、单轮模式、非有限值和 tick 回绕。
主机测试不验证真实调度延迟、电气接线、机械旋向、带载 PI 或实际 CAN 丢帧。

尚未执行硬件上电/烧录验证。上板时先保持关闭检查三路反馈，再逐轮低速确认旋向、
超时停机和恢复流程，最后验证三轮共同运行。这里没有拨弹或发射状态机。

## 可移植边界（2026-09-29）

`friction.c` 负责业务、邮箱和快照，不再直接包含 HAL/FreeRTOS，也不直接访问 DJI/PID 内部成员。
`module/motor/dji_speed_group` 统一封装现有 M3508/C620 驱动、参数应用、遥测和单位换算；
`bsp/ports/control_platform` 提供时钟与短临界区接口。旧 DJI/PID 实现仍依赖原板级服务/DWT，并未变成完全跨平台。

`FrictionInit()` 在任务启动前将明确的零掩码 OFF 交给控制核心处理，不发送电机输出，
也不留下会丢弃首条 RUN 的待处理停机。默认来源优先已开启遥控，其次是 USB、RTT、NONE。
等待首条命令不会产生 COMMAND 故障；开始运行后仍要求有效反馈和未超时的完整命令。
初始化只执行一次，重复调用不能清除故障；运行中停机、故障恢复和来源切换保持原有规则。

构建文件清单位于本目录 `build.mk`，目标文件保留源路径。统一测试入口见
[tests/README.md](../../tests/README.md)，整机组合约束与后续路线见
[控制系统架构](../architecture/control-system.md)。
