# 参数双槽存储测试

核心：`module/system/parameter_store/parameter_store.c`。
测试：`tests/parameter_store/test_parameter_store.c`，不依赖 HAL、RTOS 或外部库。

在工程根目录用主机 GCC 编译运行：

```powershell
$hostBin = 'D:/CLion/CLion 2025.3.2/bin/mingw/bin'
$env:PATH = "$hostBin;" + $env:PATH
& "$hostBin/gcc.exe" "-B$hostBin/" -std=c11 -Wall -Wextra -Werror -pedantic -UNDEBUG `
    -Imodule/system/parameter_store module/system/parameter_store/parameter_store.c `
    tests/parameter_store/test_parameter_store.c `
    -o tests/parameter_store/test_parameter_store.exe
if ($LASTEXITCODE -ne 0) { throw 'Host compilation failed' }
& ./tests/parameter_store/test_parameter_store.exe
if ($LASTEXITCODE -ne 0) { throw 'Host tests failed' }
```

`-B` 和 PATH 指向同一个主机工具链，防止装有 ARM 工具链的环境误选 ARM 汇编器。Windows 路径可以按实际主机 GCC 安装位置修改。

2026-10-03 已执行上述严格编译和测试，输出：

```text
parameter_store: 1666 byte-boundary power cuts verified
parameter_store: all tests passed
```

测试是电脑内存模拟，不接触板卡或真实 Flash。若统一 CMake 测试已接入，应优先通过工程的 `tests/verify_offline.py` 运行并生成报告；这里的直接命令用于独立移植该模块时验证。
