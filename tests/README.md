[首页](../README.md) · [构建指南](../docs/guides/build.md)

# 控制代码主机测试

模块开关测试新增遥控/电机分离、IMU 原始采集、采集关闭、三种单轮、整组关闭、VOFA/到位监测关闭、USB/板级诊断/事件关闭及 BSP 接收发送隔离。配置与验证顺序见 [模块开关](../docs/architecture/module-switches.md)。

统一入口只需要主机 GCC/Clang、CMake 3.21+ 和 Python 3，不需要 STM32 SDK、串口或电机。
使用主机编译器，不是 `arm-none-eabi-gcc`。

## 一次验证，自动清理构建文件

推荐运行独立入口；在工程根目录执行：

```powershell
python tests/verify_offline.py `
  --compiler 'D:/CLion/CLion 2025.3.2/bin/mingw/bin/gcc.exe' `
  --make 'D:/CLion/CLion 2025.3.2/bin/mingw/bin/mingw32-make.exe' `
  --generator 'MinGW Makefiles' --jobs 6
```

已有默认主机工具链的环境可直接运行 `python tests/verify_offline.py`。
脚本自动把显式提供的编译器目录加入本次子进程 PATH，不修改系统设置。

- 在系统临时目录创建唯一构建目录；成功后校验路径和所有权标记，再删除本次目录。
- 失败时保留构建现场；加 `--keep-build` 可让成功运行也保留现场。
- 每次报告放在 `.scratch/offline-verification/<时间-唯一编号>/`：Markdown、JSON、CTest XML 和阶段日志。
- `--output <目录>` 可改报告位置。报告记录源码摘要、执行命令、随机种子和测试状态。
- 不运行任何刷写工具，不连接串口，不更改电机 ID、PID 参数或生产配置。

验证范围、缺陷记录和后续步骤见 [离线验证方案](../docs/verification/offline-validation.md)。

## 需要保留构建目录时

Linux/具备默认主机构建工具的环境：

```sh
cmake -S tests -B build/host-tests
cmake --build build/host-tests --parallel
ctest --test-dir build/host-tests --output-on-failure
```

当前 Windows 机器示例（工具路径仅在命令行提供，没有写死到 CMake）：

```powershell
$env:PATH = 'D:\CLion\CLion 2025.3.2\bin\mingw\bin;' + $env:PATH
cmake -S tests -B build/host-tests -G 'MinGW Makefiles' `
  '-DCMAKE_C_COMPILER=D:/CLion/CLion 2025.3.2/bin/mingw/bin/gcc.exe' `
  '-DCMAKE_MAKE_PROGRAM=D:/CLion/CLion 2025.3.2/bin/mingw/bin/mingw32-make.exe'
cmake --build build/host-tests --parallel 6
ctest --test-dir build/host-tests --output-on-failure
```

更换生成器或编译器时使用另一个构建目录。各模块原有 `run_tests.ps1 -Compiler <path>` 继续有效，现在统一转交完整验证入口，避免维护多份重复且容易过时的源码清单。需要主机编译器、CMake 和 Python。

| 测试 | 验证内容 |
| --- | --- |
| vofa | 分包/粘包、非法输入、超时、回执、小端波形编码 |
| friction_profile | 固定格式、跨语言 CRC、一整条记录逐字节损坏、有限数值与范围 |
| profile_service | 冻结快照、完整事务、错误 token/CRC、超时/回绕、排队与生效、修订冲突 |
| parameter_store | 双槽存储，1666 个字节边界中断位置、逐字节损坏、IO 错误、代次冲突 |
| profile_client | 电脑 JSON 文件和模拟串口：24 项 Python 测试，无 pyserial/硬件要求 |
| switch_parameters_off | 关闭完整参数包，原有逐项调参正常；真实摩擦轮应用另验证导入不会启动电机 |
| text_console | 有界行队列、发送堵塞、分包命令、溢出/非法输入、时间回绕和会话隔离 |
| runtime_history | 调用间隔、计数饱和、首次/变化事件、去重、覆盖、时间/序号回绕、多实例与同步快照 |
| yaw_angle | 角度外环的限速/限加速度、防积分饱和、多圈/参考变更、失联与故障锁定；另含非实测的简化动态模型 |
| usb_console_port | 真实 CDC 适配 + USB 替身：未枚举、异步缓冲寿命、RX 先复制再重装、断线和 DTR |
| diagnostics | 真实只读命令和两套报告：未配置/无数据/过期语义、整份报告/事件历史容量、周期跳过和重连横幅 |
| speed_ready | 最终目标到位、连续时间、迟滞、样本间隔、方向/目标变化 |
| gimbal_core | DBUS、斜坡/使能、DM 模式握手与故障锁定 |
| gimbal_instances | 两个子系统实例不串状态、配置复制、非法配置禁止输出 |
| gimbal_commands | 遥控方向/死区、同批停机保留、原始时间不续期；无需 DBUS 的执行链、非法角速度、空请求和时间回绕 |
| attitude | 静止零偏/倾斜初始化、轴映射、已知旋转、多圈 Yaw、奇异位置、时序和非有限数据 |
| bmi088_sensor | 真实传感器协议：配置回读、dummy 字节、SI/负温度、无新数据及错误锁定 |
| imu_port | 真实 SPI 适配 + HAL 替身：片选、有限超时、错误传播与缓冲区保护 |
| gimbal_imu | 完整采集→解算→快照链：校准授权、安装确认门槛、旧快照失效、采集故障 |
| gimbal_application | 真实编排 + 模拟传输：掉线恢复、队列丢失、任务延迟、中途 IRQ 时间边界 |
| gimbal_port / gimbal_port_init_failure | 真实 IRQ 队列、溢出丢弃、UART 重启、初始化失败、CAN 类型过滤及发送失败 |
| control_stress + 两个种子变体 | 每个种子 10 万步控制、1 万次 VOFA 轮询、50 万随机 DBUS 字节；数值边界和故障约束 |
| friction_configured | 真实 DJI 驱动/PID + 模拟 HAL：电流分组、方向、到位、调参、初次停机确认 |
| friction_unconfigured | 默认 ID 未配置时不注册、不驱动 |
| friction_usb_parser | 数值分隔符、无符号十进制、整数溢出、浮点溢出/下溢；非法输入不提交命令、调参或停止请求，合法小数/科学计数法仍可用 |
| layout_links | 本地 Markdown 文件/目录链接与显式相对 C include 路径；不访问网络，不校验外部 URL 或标题锚点 |
| architecture_boundaries | 可移植代码不引入 HAL/RTOS/驱动内部依赖；命令源不控制电机、执行核心不解析遥控 |

`portable_control` 构建时没有 HAL/RTOS/vendor include 路径，使用 C11 严格警告。
摩擦轮集成测试使用真实旧驱动，其已有未使用函数等告警单独处理，不降低纯 C 核心的检查强度。
架构检查只覆盖新维护的控制边界，不要求一次性重写整个旧工程；新增可移植模块时应更新检查范围。

固件仍使用根 Makefile，分别运行 `make -f Makefile.gimbal`、`make -f Makefile.friction`、
`make ROBOT=sentry`。这与主机测试是两个构建过程。

测试没有模拟电机物理响应、电气故障、真实 IRQ 延迟、串口 DMA 或 IMU 姿态误差。
通过测试不代表已经完成实机控制验收。

## 删除验证内容

正式固件 Makefile 不引用 `tests/` 或 `.scratch/offline-verification/`。
成功运行的临时构建已由脚本删除；报告目录可以单独删除，不影响固件。
此前手动创建的 `build/host-tests/` 也仅用于主机验证。
如确实不再需要验证工具，可以删除整个 `tests/`，正式固件仍独立构建；
以后修改控制代码还需要回归验证，因此建议保留测试源码，只清理构建产物和旧报告。
