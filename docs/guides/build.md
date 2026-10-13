# 构建与测试

[文档目录](../README.md)

所有命令都在工程根目录执行。需要 GNU Make；固件编译需要 `arm-none-eabi-gcc`，电脑测试需要主机 GCC/Clang、CMake 3.21+ 和 Python 3。

## 先知道选了哪套程序

| 命令 | 实际应用 | 输出目录 |
| --- | --- | --- |
| `make` | 显示帮助 | 不编译 |
| `make friction -j4` | 独立三摩擦轮 | `build/friction_bench/` |
| `make gimbal -j4` | 独立云台 | `build/gimbal_bench/` |
| `make sentry -j4` | 旧哨兵 | `build/sentry/` |
| `make infantry -j4` | 旧步兵 | `build/infantry/` |

`-j4` 表示最多同时做 4 个编译工作。构建会自动创建输出目录，无需先手动 `mkdir build`。

原有入口继续可用：

```sh
make -f Makefile.friction -j4
make -f Makefile.gimbal -j4
make ROBOT=sentry -j4
make ROBOT=infantry -j4
```

工具不在 PATH 时可以指定位置：

```sh
make friction -j4 GCC_PATH=/实际工具链路径/bin
```

摩擦轮产物是 `build/friction_bench/Meta-Embedded-NG-friction_bench.elf`、同名 `.hex` 和 `.bin`。`.elf` 还包含调试信息；`.hex/.bin` 是常见烧录输入。仓库没有自动烧录目标，编译命令不会连接控制板。

## 在电脑上验证

```sh
make test
make check
```

`make test` 通过 [verify_offline.py](../../tests/verify_offline.py) 运行实际源码的主机测试。测试里用替身提供硬件反馈，验证协议、状态、参数、故障和架构边界；不能代替实板测试。

需要指定工具时直接用脚本：

```sh
python3 tests/verify_offline.py --compiler /usr/bin/gcc --jobs 4
```

更多 Windows、CMake 和报告选项在 [测试说明](../../tests/README.md)。

## 配置隔离与清理

切换编译宏时用不同构建目录，避免沿用旧宏编译的对象：

```sh
make -f Makefile.friction -j4 BUILD_DIR=build/friction-usb-off BENCH_DEFINES=-DBENCH_ENABLE_USB=0
make -f Makefile.gimbal -j4 BUILD_DIR=build/gimbal-imu-only 'BENCH_DEFINES=-DBENCH_ENABLE_YAW=0 -DBENCH_ENABLE_REMOTE=0'
make -f Makefile.friction clean
make ROBOT=sentry clean
```

[根 Makefile](../../Makefile) 负责入口；[firmware.mk](../../build_scripts/firmware.mk) 负责芯片/编译规则；每个应用的 `build.mk` 明确列出自己的源码。不要把测试或文档目录加入固件源码列表。
