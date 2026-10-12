#ifndef BENCH_FEATURES_H
#define BENCH_FEATURES_H

/* 独立云台/摩擦轮工程的模块开关：1 开启，0 关闭，修改后重新编译烧录。
 * 默认保持原有功能。开关不等于运行命令，打开电机模块仍需正常解锁。
 * 不影响旧哨兵/步兵。详见 docs/architecture/module-switches.md。
 * #ifndef 允许测试构建通过 -D 覆盖，不要在其他配置文件重复定义。 */

/* 两套 bench 共用的调试功能。 */
#ifndef BENCH_ENABLE_USB
#define BENCH_ENABLE_USB 1
#endif
#ifndef BENCH_ENABLE_BOARD_DIAGNOSTICS
#define BENCH_ENABLE_BOARD_DIAGNOSTICS 1
#endif
#ifndef BENCH_ENABLE_RTT_LOG
#define BENCH_ENABLE_RTT_LOG 1
#endif
#ifndef BENCH_ENABLE_EVENT_HISTORY
#define BENCH_ENABLE_EVENT_HISTORY 1
#endif

/* 仅独立云台：遥控可以单独测，电机可单独完成握手但不会自动转动。 */
#ifndef BENCH_ENABLE_REMOTE
#define BENCH_ENABLE_REMOTE 1
#endif
#ifndef BENCH_ENABLE_YAW
#define BENCH_ENABLE_YAW 1
#endif
#ifndef BENCH_ENABLE_IMU
#define BENCH_ENABLE_IMU 1
#endif
#ifndef BENCH_ENABLE_ATTITUDE
#define BENCH_ENABLE_ATTITUDE 1
#endif
/* 云台命令来源仲裁；关闭时仍只有原遥控路径，执行层保护始终保留。 */
#ifndef BENCH_ENABLE_COMMAND_AUTHORITY
#define BENCH_ENABLE_COMMAND_AUTHORITY 1
#endif

/* 仅独立摩擦轮：总开关 + 三个独立电机开关。关闭轮无需填写 ID。
 * F1/F2/F3 编号不随开关变化；请求启动被关闭轮会拒绝整条命令。 */
#ifndef BENCH_ENABLE_FRICTION
#define BENCH_ENABLE_FRICTION 1
#endif
#ifndef BENCH_ENABLE_F1
#define BENCH_ENABLE_F1 1
#endif
#ifndef BENCH_ENABLE_F2
#define BENCH_ENABLE_F2 1
#endif
#ifndef BENCH_ENABLE_F3
#define BENCH_ENABLE_F3 1
#endif
#ifndef BENCH_ENABLE_SPEED_READY
#define BENCH_ENABLE_SPEED_READY 1
#endif
#ifndef BENCH_ENABLE_VOFA
#define BENCH_ENABLE_VOFA 0
#endif
#ifndef BENCH_ENABLE_RTT_COMMAND
#define BENCH_ENABLE_RTT_COMMAND 1
#endif
/* DT7/DR16 六档摩擦轮遥控。独立于云台 BENCH_ENABLE_REMOTE；
 * 开启时上电默认遥控，必须先收到 S1 停止再选择运行组。
 * 关闭时 USB 开启则默认 USB。source usb/remote 显式停机切换来源。 */
#ifndef BENCH_ENABLE_FRICTION_REMOTE
#define BENCH_ENABLE_FRICTION_REMOTE 1
#endif
/* 完整配置包导入/导出；不影响原有逐项 VOFA 调参，不自动写 Flash。 */
#ifndef BENCH_ENABLE_PARAMETERS
#define BENCH_ENABLE_PARAMETERS 1
#endif

/* 摩擦轮 USB 输出：1=故障诊断，2=供 AI 分析的运行日志。
 * 可用 USB 命令 log 1 / log 2 临时切换；不改变电机控制或 UART VOFA。 */
#ifndef BENCH_FRICTION_USB_LOG_MODE
#define BENCH_FRICTION_USB_LOG_MODE 1
#endif
#if BENCH_FRICTION_USB_LOG_MODE != 1 && BENCH_FRICTION_USB_LOG_MODE != 2
#error "Friction USB log mode must be 1 or 2"
#endif

/* 派生结果：不要手改。算法的硬件来源关闭时，不再运行该算法。 */
#define BENCH_ATTITUDE_ACTIVE (BENCH_ENABLE_IMU && BENCH_ENABLE_ATTITUDE)
#define BENCH_FRICTION_MASK (BENCH_ENABLE_FRICTION * \
    (BENCH_ENABLE_F1 | (BENCH_ENABLE_F2 << 1) | (BENCH_ENABLE_F3 << 2)))
#define BENCH_FRICTION_ACTIVE (BENCH_FRICTION_MASK != 0)
#define BENCH_SPEED_READY_ACTIVE (BENCH_FRICTION_ACTIVE && BENCH_ENABLE_SPEED_READY)
#define BENCH_FRICTION_REMOTE_ACTIVE (BENCH_FRICTION_ACTIVE && BENCH_ENABLE_FRICTION_REMOTE)

#define BENCH_BOOLEAN(value) ((value) == 0 || (value) == 1)
#if !BENCH_BOOLEAN(BENCH_ENABLE_USB) || !BENCH_BOOLEAN(BENCH_ENABLE_BOARD_DIAGNOSTICS) || \
    !BENCH_BOOLEAN(BENCH_ENABLE_RTT_LOG) || !BENCH_BOOLEAN(BENCH_ENABLE_EVENT_HISTORY) || \
    !BENCH_BOOLEAN(BENCH_ENABLE_REMOTE) || !BENCH_BOOLEAN(BENCH_ENABLE_YAW) || \
    !BENCH_BOOLEAN(BENCH_ENABLE_IMU) || !BENCH_BOOLEAN(BENCH_ENABLE_ATTITUDE) || \
    !BENCH_BOOLEAN(BENCH_ENABLE_FRICTION) || !BENCH_BOOLEAN(BENCH_ENABLE_F1) || \
    !BENCH_BOOLEAN(BENCH_ENABLE_F2) || !BENCH_BOOLEAN(BENCH_ENABLE_F3) || \
    !BENCH_BOOLEAN(BENCH_ENABLE_SPEED_READY) || !BENCH_BOOLEAN(BENCH_ENABLE_VOFA) || \
    !BENCH_BOOLEAN(BENCH_ENABLE_RTT_COMMAND) || !BENCH_BOOLEAN(BENCH_ENABLE_PARAMETERS) || \
    !BENCH_BOOLEAN(BENCH_ENABLE_COMMAND_AUTHORITY) || !BENCH_BOOLEAN(BENCH_ENABLE_FRICTION_REMOTE)
#error "Bench module switches must be 0 or 1"
#endif
#undef BENCH_BOOLEAN

#endif
