# 第二轮开源对照：哪些值得用于本工程

查阅日期：2026-10-02。接续 [上一轮命令层与执行层拆分](open-source-review.md)。
本轮增加检查 5 个仓库，固定提交并下载 20 份源文件供对照。依据是具体实现和测试，不以仓库热度判断可靠性。
所查对象包含官方历史固件、队伍框架、算法示例和嵌入式基础库，不能统称为已验证适合本机构的完整方案。

## 1. RoboRTS-Firmware：让停止原因可以追溯

官方历史固件，本轮固定 `icra2021` 分支提交 `8cffd936b503eef819fa05265cf0162710c9cb3b`（2021-03-29）。
查看 [offline_service.c](https://github.com/RoboMaster/RoboRTS-Firmware/blob/8cffd936b503eef819fa05265cf0162710c9cb3b/application/offline_service.c)、对应头文件及云台任务/模块。

其离线服务集中登记设备事件、超时、级别、回调和告警优先级，以最后接收时间判断离线，并区分首次在线/离线与持续状态。
值得采用的是“设备观测有明确含义，系统响应另有负责人”。
本工程已有反馈年龄和状态历史，因此本轮补足 **Yaw 为什么没有获得运行授权**，接入 USB 报告与事件历史。
本地控制任务仍独占电机状态；没有搬入后台回调直接控制执行器的组织方式。

## 2. UICRM-Embedded：板卡适配和角度语义

固定 `main` 提交 `225427b004568e75069f0ed7a92180a548082295`（2026-09-11）。
仓库包含 DM-MC02 板卡支持，板级配置、驱动、组件、独立示例与机器人程序分开。
具体查看 [连接状态](https://github.com/UIC-RoboMaster/UICRM-Embedded/blob/225427b004568e75069f0ed7a92180a548082295/boards/drivers/src/connection_driver.cpp)、
[CAN 电机基类](https://github.com/UIC-RoboMaster/UICRM-Embedded/blob/225427b004568e75069f0ed7a92180a548082295/boards/drivers/src/Motor/CanMotorBase.cpp)、
[云台组件](https://github.com/UIC-RoboMaster/UICRM-Embedded/blob/225427b004568e75069f0ed7a92180a548082295/boards/components/src/gimbal.cpp)。

连接状态由有效接收更新时刻判断；CAN 接收按 ID 和实例上下文分发。云台区分圆周运动轴与有限范围轴，IMU 更新路径将角度误差约束到半圈以内。
对本工程最重要的启发是：**“可以转 360°”不足以定义角度命令。**

例如实际角度 +179°、目标 −179°，最短路径是 +2°，按连续角数值相减则是 −358°。
二者适合不同意图。当前 `module/yaw_angle` 明确消费连续角，不能直接替换成自动取最短路径。
未来命令层应明确目标是方向、连续多圈角度还是手动速度，并处理切换时的目标捕获和零点参考。
这一部分尚未实现；本轮没有改变角度核心或电机控制模式。

板卡与电机型号相近，有助于核对接口和独立调试例程；不能据此照搬 C++ 类型体系、驱动模式、机械方向或 PID 参数。
合并总线时仍须解决本工程 CAN1 独占与 DJI 统一发送的所有权，单纯有 ID 分发函数并不足够。

## 3. C-Board-INS-Example：判断加速度观测是否可信

固定 `master` 提交 `4361e22fc5c7fa26d2a136548c0f8c4e6f8ff1d2`（2022-08-13）。
查看 README、姿态任务与 [QuaternionEKF.c](https://github.com/WangHongxi2001/RoboMaster-C-Board-INS-Example/blob/4361e22fc5c7fa26d2a136548c0f8c4e6f8ff1d2/Components/Algorithm/QuaternionEKF.c)。

该实现包含加速度低通、静止条件判断、基于预测与观测差异的卡方检验、异常观测拒绝和增益调整。
它还明确处理了六轴航向可观测性的限制，不能因为采用 EKF 就宣称消除了 Yaw 漂移。

本地互补滤波已有静止校准和加速度模长门限，但模长接近重力不代表方向一定可信。
后续值得记录加速度模长、预测重力与测量的差异、观测被拒绝的次数，再用实际采样回放评估改进。
这是待验证的设计方向，不是已经证实的本地故障。现在缺少运动、振动和温漂数据，本轮没有替换姿态算法，也没有复制其滤波参数。

## 4. XRobot / LibXR：依赖声明与参数存储验证

XRobot 固定 `XRobot2.0` 提交 `15bff04ec4a21c2fdb31d4411e2802851e3de4c9`（2026-04-29）。
查看 README 和 [ModuleParser.py](https://github.com/xrobot-org/XRobot/blob/15bff04ec4a21c2fdb31d4411e2802851e3de4c9/src/xrobot/ModuleParser.py)。
当前版本承担模块管理与入口生成，解析构造参数、硬件需求和依赖声明。不要把它等同于早期完整机器人固件。
本工程已有独立构建清单，暂时足够；整机组合时可借鉴明确列出每个模块所需 CAN/UART/SPI、任务和唯一所有者的做法。

LibXR 固定 `master` 提交 `4e9670164541b6af6b600a6d544115a9b3e49d98`（2026-09-14）。
查看数据库接口、布局、初始化恢复、键读写及 [主机端数据库测试](https://github.com/xrobot-org/libxr/blob/4e9670164541b6af6b600a6d544115a9b3e49d98/test/automatic/middleware/database/raw/raw/test_linux_database_raw.cpp)。
它以主/备块和写入状态管理恢复；测试通过文件模拟 Flash，覆盖重复更新、重新打开、大小不符、不完整备份、写入失败和损坏数据恢复等情况。
这比先增加一个“保存 PID”命令更值得学习：保存功能需要证明中途中断后还能得到完整、可验证的参数。

本工程下一步可以提前做的具体范围（**尚未实现**）：

1. 参数配置包：明确版本、长度、校验、单位和范围，拒绝非有限值及不兼容版本。
2. 区分“请求已接收”“控制任务已应用到 RAM”“持久保存已完成”，让上位机得到真实结果。
3. 以 read/write/erase 回调隔离存储硬件，用电脑文件模拟写入失败、部分写入及重启恢复。
4. 实物 Flash 布局确认后再绑定驱动；使能状态、历史运行命令不作为参数自动恢复。

版本格式和校验方案是本工程的后续设计建议，并非声称 LibXR 与其完全相同。
当前 VOFA 在线参数仍只存 RAM，没有新增 Flash 写入，也没有引入 C++20 库或代码生成器。

## 本轮实际修改

在原有停止/使能判断处记录 `YAW_INHIBIT_*` 位，传递到执行状态快照：

```text
yaw_rate 的既有判断 → gimbal_control 状态快照
                           ├─ USB [yaw_inhibit] 解释字段
                           └─ events 的 yaw.inhibit 变化记录
```

可以区分配置错误、命令无效/过期、执行器未就绪、调用间隔异常、速度请求不合法、开关关闭、需要重新解锁、启动请求非零。
字段逐位说明见 [USB 诊断](../../application/diagnostics/README.md#云台为什么没有运行)。
这些字段解释最近一轮的判断，不改变使能、恢复、PID 或 CAN 输出规则。
`mask=0` 表示应用允许输出，不代表电机已确认 ON；旧快照会标记 STALE。
短暂原因可能只持续一个控制周期，可结合 `events` 查询；历史容量仍为 32 条，复位丢失。

## 取舍与验证

按对当前工程的价值排序：先完善调试可观测性和参数生命周期，再明确角度坐标/模式，最后用实测数据评估姿态与控制算法。
三摩擦轮仍使用逐轮速度闭环、目标斜坡和整组到位判断。现有资料不能证明应换成另一种控制算法，也不能直接给出适合本机构的参数。

本轮没有把第三方源码复制进生产代码。RoboRTS 所查文件含 GPL 声明、UICRM 为 GPL-3.0、XRobot/LibXR 为 Apache-2.0；INS 示例本轮未确认明确的项目级授权，不能把公开可读当作任意复制许可。
只读源文件及固定提交清单保存在 `.scratch/upstream-review-20261002/`，不参加固件构建。

验证结果：23 项离线测试通过；同场景修改前后 118 次 CAN 发送尝试的时间、ID、长度和载荷一致；两套 ARM bench 构建成功。
详细记录见 [本轮验证](../../.scratch/upstream-review-20261002/verification.md)。这些结果验证软件行为与构建，不证明电机响应、物理实时性、PID 效果或弹速一致性。
