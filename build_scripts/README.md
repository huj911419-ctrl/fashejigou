# 固件构建规则

[构建指南](../docs/guides/build.md)

根目录 [Makefile](../Makefile) 提供短命令和帮助。[firmware.mk](firmware.mk) 保存芯片编译选项、公共依赖、机器人选择、链接和产物规则。

当前应用各自的源清单在 [friction/build.mk](../application/friction/build.mk)、[gimbal/build.mk](../application/gimbal/build.mk)；公共诊断清单在 [services/diagnostics/build.mk](../application/services/diagnostics/build.mk)。

从工程根目录执行命令。`ROBOT` 是稳定的构建标识，`APP_DIR` 显式指向现在的源码位置。
