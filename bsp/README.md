# bsp：程序怎样接到这块板

[首页](../README.md) · [项目地图](../docs/start/01-project-map.md)

| 分类 | 作用 |
| --- | --- |
| [ports](ports/README.md) | 把应用/纯 C 接口适配到此板：云台、遥控、IMU、USB、时钟和临界区 |
| [peripherals](peripherals/README.md) | CAN/UART/SPI/GPIO 等外设的通用封装 |
| [board](board/README.md) | 板卡与外设状态观测 |
| [bsp_tools.h](bsp_tools.h) | 旧通用辅助接口 |
| [bsp_init.h](bsp_init.h) | 原有板级初始化相关入口 |

具体引脚和外设基础初始化仍在 `Core/`。port 封装接法；协议组件解释数据；应用决定行为。
