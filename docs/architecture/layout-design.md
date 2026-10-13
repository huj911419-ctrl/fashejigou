# 目录重构的依据与边界

查阅日期：2026-10-13。[文档导航](../README.md) · [迁移表](../reference/path-migration.md)

## 学了哪些开源组织方式

| 参考 | 实际观察 | 本工程采用的部分 |
| --- | --- | --- |
| [跃鹿 basic_framework](https://github.com/HNUYueLuRM/basic_framework) | `application`、`modules`、`bsp` 三层，应用通过组件使用硬件能力 | 保留现有三层概念，明确当前功能和底层组件的入口 |
| [UICRM-Embedded](https://github.com/UIC-RoboMaster/UICRM-Embedded) | 板级支持、算法、驱动、组件、独立示例与机器人程序分别组织 | 对组件按职责分组，当前独立应用与旧整机业务分开 |

这是针对现有 C/H723 工程的组织调整，没有引入参考工程的 C++ 框架或复制第三方实现。厂商生成目录继续保留，以适应 CubeMX 再生成流程。

## 为什么采用现在的目录

1. `application/friction` 和 `application/gimbal` 成为当前功能入口；分别能看到初始化、任务、控制、配置和诊断。
2. 旧整机应用集中在 `application/legacy`。共享且仍被使用的电机/协议组件继续放在 `module`，不会因服务旧应用而被误删。
3. 遥控输入的含义由 `application/commands` 决定；执行控制消费内部命令，公共服务放在 `application/services`。
4. `module` 按控制、协议、传感器、电机、系统和设备分类；`bsp` 按外设封装、应用适配和板卡观测分类。
5. 文档按入门、操作、参考、设计、验证、历史分类。首页保持短入口，详细材料通过目录链接进入。

## 依赖与运行契约

- 纯控制/协议核心使用普通 C 数据和接口；主机测试不提供 HAL/RTOS 头文件。
- 应用通过邮箱/快照交接请求与状态。目录迁移没有改变控制任务所有权、命令时间戳、失联判断或重解锁流程。
- `bsp/ports/gimbal_port` 继续拥有独立云台 CAN1/UART5；IMU port 仍由独立 IMU 任务使用。两个独立应用继续分别构建。
- `module/motor/dji_speed_group` 仍统一提交旧 DJI 注册表的控制输出，不能因新目录就假设支持多个独立控制任务。
- 当前维护边界由 `tests/check_architecture.py` 验证；旧模块仍有历史耦合，重构未宣称整仓库都完全硬件无关。

## 构建兼容性

旧 `ROBOT` 名称和产物名称保留：`gimbal_bench`、`friction_bench`、`sentry`、`infantry`。源码目录不再直接由 `ROBOT` 字符串拼接，构建规则显式映射到应用位置。

根 Makefile 提供 `gimbal/friction/test/check` 等短命令；普通 `make` 改为显示帮助。详细编译规则移入 `build_scripts/firmware.mk`，每个应用继续维护显式源清单，避免把旁边的旧业务自动编进固件。

接口/目录引用、文档相对链接、测试脚本和架构检查同步迁移。验证结果单列在 [结构重构验证](../verification/layout-refactor.md)。
