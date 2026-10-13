# Meta-Embedded-NG：云台与摩擦轮电控

运行在 **达妙 DM-MC02 / STM32H723** 控制板上的 C 语言固件。当前学习入口是独立云台和三摩擦轮；两者分别编译成一份固件。

**第一次来，打开 [从这里开始](从这里开始.md)。** 先看你要用的功能，再跟着函数调用往下读。

## 我想做什么

| 你的目标 | 从这里进入 |
| --- | --- |
| 看懂文件夹、文件、函数各自做什么 | [项目地图](docs/start/01-project-map.md) |
| 看懂上电后谁先运行、任务如何循环 | [启动过程](docs/start/02-startup.md) |
| 读三摩擦轮代码 | [摩擦轮阅读路线](docs/start/03-read-friction.md) → [源码入口](application/friction/README.md) |
| 读独立云台代码 | [云台阅读路线](docs/start/04-read-gimbal.md) → [源码入口](application/gimbal/README.md) |
| 编译、运行电脑测试 | [构建指南](docs/guides/build.md) |
| 找开关、CAN ID、方向、PID 配置 | [配置在哪里](docs/guides/configure.md) |
| 查 USB 日志、遥控和故障 | [调试入口](docs/guides/debug.md) |
| 找全部说明或旧文件的新位置 | [文档目录](docs/README.md) · [路径迁移表](docs/reference/path-migration.md) |

## 先认四个位置

| 目录 | 一句话职责 | 先看哪里 |
| --- | --- | --- |
| [application](application/README.md) | 决定机器人做什么，并安排任务 | `friction/` 摩擦轮、`gimbal/` 云台 |
| [module](module/README.md) | 提供控制算法、协议、传感器和电机组件 | 按 `control/`、`protocol/`、`sensor/` 等分类找 |
| [bsp](bsp/README.md) | 把组件接到这块板的 CAN/UART/SPI 等接口 | `ports/` 应用适配、`peripherals/` 外设封装 |
| [docs](docs/README.md) | 学习路线、使用方法和设计说明 | `start/` 入门、`guides/` 操作、`reference/` 查表 |

`Core/` 是芯片启动和外设初始化，`Drivers/`、`Middlewares/`、`USB_DEVICE/` 是 HAL、RTOS、USB 等基础支持。保留 CubeMX 布局便于再生成；目录分类不决定函数执行顺序。

旧哨兵、旧步兵及它们的底盘/双云台/供弹业务集中在 [application/legacy](application/legacy/README.md)，有各自的构建入口。

## 常用命令

在项目根目录执行；工具要求见 [构建指南](docs/guides/build.md)。

```sh
make               # 显示帮助
make friction -j4  # 编译摩擦轮
make gimbal -j4    # 编译云台
make test          # 在电脑上运行测试
make check         # 检查层级与文档路径
```

旧的 `make -f Makefile.friction`、`make -f Makefile.gimbal`、`make ROBOT=sentry` 和 `make ROBOT=infantry` 继续可用。普通 `make` 现在显示帮助，不再默认编译旧哨兵。

## 当前功能边界

- 云台：水平轴速度控制、遥控输入、独立 IMU 采集/姿态观察。角度控制核心已有测试，尚未接入实际电机链。
- 摩擦轮：三轮速度控制、输入来源选择、调参和日志。完整供弹、弹速闭环和整机协同仍需另行实现和实测。
- 参数：完整参数包可应用到 RAM；真实 Flash 保存/启动加载尚未绑定。
- 主机测试与固件编译验证软件行为；实际接线、方向、机械和控制性能需要接板验证。

## 来源与维护

工程继承湖南大学跃鹿战队 [basic_framework](https://github.com/HNUYueLuRM/basic_framework) 的三层设计。此次目录调整也参考 [UICRM-Embedded](https://github.com/UIC-RoboMaster/UICRM-Embedded) 的分类方式，取舍记录在 [布局设计](docs/architecture/layout-design.md)。

[贡献与维护约束](AGENTS.md) · [MIT 许可证](LICENSE) · [历史资料](docs/history/README.md)
