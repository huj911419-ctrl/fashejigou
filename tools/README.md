# 电脑端辅助工具

[首页](../README.md) · [调试指南](../docs/guides/debug.md)

| 工具 | 做什么 | 入口 |
| --- | --- | --- |
| [usb_console.py](usb_console.py) | USB 文本控制台辅助 | `python3 tools/usb_console.py --help` |
| [friction_profiles.py](friction_profiles.py) | 参数 JSON 校验、导入和导出 | [详细说明](FRICTION_PROFILES.md) |

脚本运行在电脑上，不编入 STM32 固件。端口选择、依赖和实际命令先看对应帮助；参数工具使用 VOFA 协议，不能把它当作 USB 文本终端。
