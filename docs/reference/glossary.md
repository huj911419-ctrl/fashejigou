# 看到这些词，先知道它干什么

[文档目录](../README.md)

| 名字 | 含义 | 本工程中的例子 |
| --- | --- | --- |
| 目录 / 文件夹 | 同一种组织文件的东西；路径是它的位置 | `application/friction/` |
| `.c` | 函数的实现 | `friction.c` |
| `.h` | 对外声明、数据类型、配置 | `friction.h` |
| `#include` | 编译时引入声明，不会启动函数 | `#include "friction.h"` |
| `main()` | C 程序的入口函数 | `Core/Src/main.c` |
| 初始化 | 开机时准备状态、资源和接口 | `RobotInit()` |
| task / RTOS 任务 | 交给调度器运行的函数，常含周期循环 | `MotorTask` |
| callback / 回调 | 先登记一个函数，事件发生后再由别处调用 | CAN 接收回调 |
| ISR / 中断 | 硬件事件触发的处理入口 | UART 接收、定时器 |
| HAL | 厂商提供的硬件操作接口 | STM32 HAL |
| BSP / port | 把程序接到具体控制板的代码 | `gimbal_port.c` |
| protocol / 协议 | 约定每个数据字节代表什么 | DBUS、CAN 电机帧 |
| snapshot / 快照 | 某一时刻状态的副本，便于其他任务读取 | 诊断读取控制状态 |
| mailbox / 邮箱 | 在任务之间交接请求的状态/队列 | 输入提交，控制任务取出 |
| PID | 根据目标和反馈误差计算控制输出的方法 | 电机速度控制 |
| IMU | 测角速度、加速度的传感器 | BMI088 |
| Makefile / `.mk` | 编译规则与源码清单 | 根 Makefile、应用 build.mk |
| `.o` | 单个源文件编译后的目标文件 | 放在 `build/` |
| `.elf` | 链接后的固件，包含调试所需信息 | 调试器使用 |
| `.hex` / `.bin` | 常见的烧录文件格式 | 烧录到 Flash |
| host test | 在电脑上执行的测试 | `make test` |
| legacy | 保留的旧实现或旧机器人业务 | `application/legacy/` |
