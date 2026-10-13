# 目录重构验证

[文档目录](../README.md) · [布局依据](../architecture/layout-design.md) · [路径迁移表](../reference/path-migration.md)

验证日期：2026-10-13。重构基于提交 `7d87f5c6b58b7936f450b927b5370bb69b811f7d`，其中已包含 USB 数值校验修复。以下为迁移后的实际运行结果。

## 验证范围与结果

| 项目 | 结果 |
| --- | --- |
| 全量主机测试（`make test`） | **68/68 通过**，包括新增布局检查、协议/控制/集成/隔离配置测试 |
| 随机压力测试 | 种子 1、20260929、4294967295 均通过 |
| `make check` | 可移植边界与本地文档/include 路径检查通过；缺失链接和头文件的反例均正确报错 |
| 默认应用固件 | 云台、摩擦轮、旧哨兵、旧步兵均编译并链接成功 |
| 隔离配置固件 | 摩擦轮关闭 USB；云台关闭 Yaw 与遥控、保留 IMU：均编译成功 |
| 源码迁移核对 | 核对 214 份原有业务/组件/板级 C/H 文件，排除 include 路径、注释与空白后无实现变化 |
| 生成代码与厂商库 | `Core`、`Drivers`、`Middlewares`、`USB_DEVICE`、IOC 与链接脚本无差异 |
| 格式 | `git diff --check` 通过 |

主机验证使用 Linux、GCC 13.3.0、CMake 和 Python 3；ARM 固件使用 `arm-none-eabi-gcc` 13.2.1。全量主机验证的源码摘要（SHA-256）为：

```text
f15733161a052f38a38c96e926489028cf64d74f8c21ba91d44b2ded45a3ec0b
```

摘要由 [verify_offline.py](../../tests/verify_offline.py) 生成，覆盖业务、组件、BSP、测试和构建清单；不覆盖说明文档。详细测试覆盖在 [tests/README.md](../../tests/README.md)，本次记录不把测试替身当成真实硬件反馈。

## 固件链接结果

以下是 GNU `size` 报告的字节数；`text/data/bss` 用于核对构建，不是运行时性能测量。

| 配置 | text | data | bss |
| --- | ---: | ---: | ---: |
| 云台默认 | 95496 | 352 | 51008 |
| 摩擦轮默认 | 139604 | 1512 | 52032 |
| 旧哨兵 | 104472 | 1312 | 39000 |
| 旧步兵 | 77608 | 1156 | 37248 |
| 摩擦轮 USB=0 | 80784 | 888 | 35032 |
| 云台 Yaw=0、Remote=0 | 86912 | 352 | 50544 |

复现命令（工具链加入 PATH 后，在工程根目录运行）：

```sh
make test
make check
make gimbal -j4
make friction -j4
make sentry -j4
make infantry -j4
make -f Makefile.friction -j4 BUILD_DIR=build/friction-usb-off BENCH_DEFINES=-DBENCH_ENABLE_USB=0
make -f Makefile.gimbal -j4 BUILD_DIR=build/gimbal-imu-only 'BENCH_DEFINES=-DBENCH_ENABLE_YAW=0 -DBENCH_ENABLE_REMOTE=0'
```

切换编译宏必须使用独立输出目录或先清理，详见 [构建指南](../guides/build.md)。

## 本次没有验证的内容

- 没有烧录实板或测试电机、遥控、USB 枚举和 IMU 的真实时序/性能。
- 固件仍有旧代码已有的告警，包括未使用变量/回调、旧步兵函数隐式声明；成功链接不代表无告警。
- PowerShell 兼容入口已统一调用 Python 验证器；本次在 Linux 验证 Python 主入口，没有运行 Windows PowerShell。
- 布局检查验证本地文件/目录链接及显式相对 include；不验证外部网址、Markdown 标题锚点和参考资料的全部历史表述。
- 目录重构保持控制实现、硬件配置及功能边界；不能据此宣称已实现完整发射、视觉闭环、角度保持或掉电保存。
