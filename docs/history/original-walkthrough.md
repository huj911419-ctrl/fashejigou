> 历史记录：用于了解当时背景；当前目录、构建与功能状态以 [文档导航](../README.md) 为准。文中过去的验证数字不代表当前版本。

# 整个工程怎样组织、怎样运行

逐模块接实物时，先打开 [`application/bench_features.h`](../../application/bench_features.h)，按 [模块开关与验证顺序](../architecture/module-switches.md) 选择。1 开启、0 关闭，重新编译烧录；开关不等于让电机立即运行。

适合能读普通 C 函数、尚不熟悉多文件工程和 RTOS 的读者。先读第 1～3 节，后面的表格用于查找文件。

## 1. 文件夹是在分工，文件不会自己执行

一个普通 C 程序可以把所有函数写在 `main.c`。工程变大后，把不同职责的函数放进不同 `.c` 文件，依然遵循函数调用规则。
`.h` 提供其他文件需要知道的函数声明和数据类型。`#include` 让编译器看到这些声明，不会启动这个模块。
函数运行，需要有人调用，或把它注册为系统随后会调用的任务/回调。文件夹的位置本身不决定执行顺序。

| 目录 | 在整个项目里负责什么 | 本项目中的例子 |
| --- | --- | --- |
| `Core` | 启动应用、配置芯片时钟和外设、中断入口 | `main.c`、`fdcan.c`、`usart.c`、`freertos.c` |
| `Drivers` | 芯片厂商提供的基础库，帮助操作硬件寄存器 | STM32 HAL；我们的板级代码会调用它 |
| `Middlewares` | 已有的软件库 | FreeRTOS 调度任务，USB 协议栈处理 USB，RTT 支持调试 |
| `bsp` | 把软件接到这块板的具体接口 | 哪个 CAN、哪个串口、哪个 SPI 和引脚，怎样收发数据 |
| `module` | 可复用的算法、设备协议和功能组件 | PID、遥控解码、电机协议、姿态解算、VOFA |
| `application` | 组合上述能力，决定机器人如何工作 | 云台是否允许转、三轮目标转速、创建哪些任务 |
| `USB_DEVICE` | Cube 生成的 USB 设备配置和接口 | 让电脑识别这块板的虚拟串口，连接 USB 协议栈 |
| `tests` / `tools` | 在电脑上验证或辅助使用 | 离线测试、USB 文本接收脚本；当前固件不编入这些文件 |
| `docs` / `.scratch` | 说明、设计记录和工作产物 | 架构说明、验证报告、临时分析材料 |
| `build` | 编译产生的结果 | `.o`、`.elf`、`.hex`、`.bin`，不是需要手写的业务源码 |

拿“发一个电机命令”举例：应用决定期望转速，协议模块把要求编码成电机认识的数据，板级适配器调用 STM32 库把数据交给 CAN 硬件。
这样以后换板时，板级接法集中修改；换机器人功能时，主要调整应用组合。现有旧模块的边界并非都这么干净，新维护的模块在逐步按此组织。

## 2. 先选一套程序，再把它编译成固件

这个目录同时保存了旧哨兵、旧步兵，以及我们新增的独立云台和摩擦轮应用。它们不会因为放在同一目录里就全部一起运行。

| 编译入口 | 选择的应用 | 对应目录 |
| --- | --- | --- |
| `make -f Makefile.gimbal` | 独立云台 | `application/gimbal` |
| `make -f Makefile.friction` | 独立三摩擦轮 | `application/friction` |
| `make ROBOT=sentry` | 原有哨兵 | `application/legacy/sentry` |
| `make ROBOT=infantry` | 原有步兵 | `application/legacy/infantry` |

普通 `make` 默认选择旧哨兵。学习我们的云台/摩擦轮时，要认准专属编译入口。
`bench` 在这里表示独立调试应用，因此 `gimbal_bench` 是云台调试应用的名字。

Makefile 相当于编译清单：指定用哪些源文件、去哪里找头文件、选择什么编译选项。两个 bench 的详细清单分别在各自的 `build.mk` 中。
每套应用可以都有 `RobotInit()`，因为对应构建只选择其中一套应用入口，不会把四个同名实现全部链接到一起。

```text
选择应用 → 编译需要的 .c 文件 → 链接成一份固件 → 烧录进板子 → 板子运行
```

编译是在电脑上发生的，任务调度是在板子上运行固件时发生的。这是两件不同的事。

## 3. 板子上电后，怎样从 main 走到多个任务

本工程实际启动顺序如下：

```text
复位 → 启动汇编准备 C 运行环境 → Core/Src/main.c 的 main()
    ├─ 初始化 HAL、时钟、GPIO、CAN、UART、SPI 等外设
    ├─ RobotInit()：准备所选应用，并创建它的任务
    ├─ MX_FREERTOS_Init()：创建系统默认任务
    └─ osKernelStart()：启动 FreeRTOS 调度
            ├─ 控制任务：循环控制云台或摩擦轮
            ├─ 采样任务：云台构建中的 IMU 采集与解算
            ├─ 诊断任务：检查调试输入，输出日志
            └─ 默认任务：启动 USB，随后结束自己
```

**任务仍然是一个 C 函数。** 通常在里面写一个循环，再通过 `xTaskCreate()` 把这个函数交给 FreeRTOS，指定栈空间和优先级。
创建任务不会在当前位置把任务函数执行完；调度器启动后，会安排它们轮流获得 CPU。
此板的 CPU 是单核，每个时刻执行一个任务或中断处理程序。外设也能独立收发数据，CPU 随后通过中断等方式处理。

下面是云台任务的教学简化版，不是完整控制代码，不用于替换工程源码：

```c
void YawTask(void *argument)
{
    (void)argument;
    TickType_t next = xTaskGetTickCount();
    for (;;)
    {
        GimbalTask(); /* 处理这一轮云台控制，函数会返回 */
        vTaskDelayUntil(&next, pdMS_TO_TICKS(2)); /* 等下一轮，让出 CPU */
    }
}
```

源码用 `vTaskDelayUntil()` 按计划周期等待；`vTaskDelay()` 则从调用时刻开始等待一段时间。
等待期间该任务不会占着 CPU 空转。高优先级任务变为可运行时，可以抢先运行；一个高优先级任务如果一直忙循环、不等待，会挤占低优先级任务的运行机会。
任务周期是调度目标，处理超时、长中断或长临界区仍会影响实际时序。

所以 `main()` 里的最后一个 `while (1)` 不是当前应用的正常控制循环；正常运行时，调度器已经接管，控制循环在任务函数里面。

## 4. 当前两套应用各自运行哪些任务

下面两张表对应两份不同的固件。

| 云台固件任务 | 做什么 | 当前计划频率/周期 |
| --- | --- | --- |
| `YawTask` | 收遥控和电机反馈，推进云台速度控制；喂看门狗 | 2 ms 一轮，优先级高于 IMU/诊断 |
| `ImuTask` | 采集 BMI088，校准、解算并发布姿态 | 2 ms 一轮；启用 IMU 时创建 |
| `UsbDiagnosticTask` | 处理 USB 查询、发送状态 | 每轮之后等待 2 ms；默认每 1000 ms 生成报告 |
| `DiagnosticTask` | 输出 RTT 状态日志 | 每轮之后等待 200 ms |

| 摩擦轮固件任务 | 做什么 | 当前计划频率/周期 |
| --- | --- | --- |
| `MotorTask` | 处理三轮目标、反馈、PID 和输出；喂看门狗 | 1 ms 一轮，优先级高于诊断 |
| `DiagnosticTask` | 轮询 VOFA/RTT 输入和调试状态 | 每轮之后等待 1 ms；RTT 状态约每 200 ms 输出 |
| `UsbDiagnosticTask` | 处理 USB 查询、发送状态 | 每轮之后等待 2 ms；默认每 1000 ms 生成报告 |

两套还都有 FreeRTOS 自身的后台任务和启动 USB 的默认任务。上表突出应用任务。
“看门狗”是防止程序长期卡死的硬件：控制任务必须持续运行并定期刷新它，否则板子会复位。

## 5. 不同任务怎样交换信息

以云台为例：

```text
UART/CAN 硬件收到数据
    → 中断处理把数据放入队列
    → 云台任务取出数据并解析
    → 云台任务决定这一轮如何控制电机
    → 保存一份状态副本
    → USB 任务读取副本，在电脑上显示
```

队列用于暂存先后到来的数据；状态副本用于让其他任务读取一组一致的信息。它们是保存数据的结构，不是又多了几个任务。
中断通常用于及时处理硬件事件，处理时间应短；控制计算和详细打印放到任务中完成。
摩擦轮另有命令和调参“邮箱”：调试代码提交一份请求，由电机控制任务统一应用，避免两个地方同时改 PID 或电机输出。
USB 文本诊断只读状态；摩擦轮原有 VOFA/RTT 控制入口有单独的命令规则。

云台现在还分了“命令”和“执行”：`robot_cmd/gimbal_remote.c` 把摇杆/开关转换为角速度请求，
`gimbal_control.c` 检查请求是否过期、电机是否正常，再决定输出。两者在同一个任务里依次调用。
这样后面增加其他输入时，不用把它的协议解析塞进电机控制函数。

## 6. 认识全貌后，用这张表找代码

| 想理解什么 | 从哪里找 |
| --- | --- |
| 上电时先做什么 | [Core/Src/main.c](../../Core/Src/main.c)，先看调用顺序 |
| 这个应用准备了什么 | [云台 robot.c](../../application/gimbal/robot.c) 或 [摩擦轮 robot.c](../../application/friction/robot.c) |
| 创建哪些任务、谁在循环 | [云台 robot_task.c](../../application/gimbal/robot_task.c) 或 [摩擦轮 robot_task.c](../../application/friction/robot_task.c) |
| 一轮云台控制 | [gimbal.c](../../application/gimbal/gimbal.c) → [gimbal_control.c](../../application/gimbal/control/gimbal_control.c) |
| 一轮三摩擦轮控制 | [friction.c](../../application/friction/friction.c) 的 `FrictionTask()`，再按调用展开 |
| 遥控数据怎样变成通道值 | `module/protocol/dbus_input` |
| 通道、开关怎样变成带单位的控制请求 | [命令层说明](../../application/commands/README.md)、`application/commands/gimbal_remote.c` |
| 云台如何生成转速、怎样和电机通信 | `module/control/yaw_rate`、`module/protocol/dm_velocity` |
| 三摩擦轮怎样执行速度 PID | `module/motor/dji_speed_group`、`module/motor/dji`、`module/algorithm` |
| IMU 怎样变成姿态 | `bsp/ports/imu_port` → `module/sensor/bmi088_sensor` → `module/sensor/attitude`；由 `gimbal_imu.c` 组织 |
| USB 怎样显示状态 | `application/services/diagnostics`、各应用的 `*_diagnostics.c`、`module/protocol/text_console`、`bsp/ports/usb_console_port` |
| 查询以前发生过什么、任务周期有没有异常 | `application/services/runtime`、`module/system/event_history`、`module/system/loop_stats`；USB 用 `events` / `status` |
| 后续角度保持怎样计算 | `module/control/yaw_angle`；目前独立验证，尚未接入电机 |
| 改方向、限速、ID 等配置 | 各应用的 `*_config.h`；具体单位和合法范围见文件注释 |
| 结构参考了哪些开源工程、为什么这样分工 | [实际源码对照与修改](../history/open-source-review.md) |

旧应用目录 `application/legacy/gimbal`、`shoot` 等，以及旧的 BMI088/INS 模块保留在基座里；我们新增的 bench 使用自己的清单和入口。
目前云台是手动速度控制，姿态用于观测；摩擦轮是三路速度控制。角度外环核心已写好并独立验证，但云台角度保持的整机接入、视觉自瞄和完整自动供弹仍待完成。

## 7. 根目录其他文件是什么

| 文件 | 用途 |
| --- | --- |
| `Meta-Embedded-NG.ioc` | CubeMX 的引脚、时钟和外设配置工程 |
| `startup_stm32h723xx.s` | 上电复位后的启动汇编，最后进入 C 的 `main()` |
| `STM32H723XG_FLASH.ld` | 链接脚本，安排程序和数据放到哪段 Flash/RAM |
| `cmsis-dap-h7.cfg`、`烧录方式.txt` | 调试器/烧录配置与说明 |
| `.clang-format` | 代码排版规则 |
| `.vscode`、`.codeartsdoer`、`.mxproject` | 开发工具相关配置 |
| `README.md`、`docs` | 面向人的项目说明 |
| `AGENTS.md`、`CLAUDE.md` | 开发协作规则，不会在板子上执行 |
| `.gitignore`、`LICENSE` | 版本管理忽略规则、许可证 |

阅读时先确定三件事：当前编译哪套应用、这个函数由谁调用、它读写什么数据。确认后再进入下一层函数，不需要从目录第一个文件一直读到最后。
