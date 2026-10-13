# 可移植六轴姿态核心

复制本目录和 `module/sensor/imu_sample/imu_sample.h`，链接 C 数学库即可使用。
每个设备持有一个 `Attitude_s`，由同一个任务调用初始化、更新与过期检查；算法不读取硬件时钟。
传入 SI 单位、同源毫秒时间戳，以及当前是否允许静止校准的条件。

```c
Attitude_s attitude;
/* config 由目标应用提供；先检查初始化返回值。 */
if (!AttitudeInit(&attitude, &config))
{
    return;
}

/* 收到完整新样本，在唯一所有者任务中调用。 */
AttitudeUpdate(&attitude, &sample, now_ms, motors_disabled);

/* 没有新样本也需检查；跨任务发布 output 的一致快照。 */
AttitudeCheckFresh(&attitude, now_ms);
```

四元数为 Hamilton `w,x,y,z`、机体系到 Z 向上参考系；角速度属于机体系，Euler 角采用 ZYX 约定。
Yaw 没有绝对航向参考，初始方向为零。使用数据必须检查 `valid`；连续偏航还要检查
`heading_valid` 和 `yaw_continuous`。故障后显式初始化，并同步废弃依赖旧零点的控制目标。

字段单位见头文件，配置数值范围见 `AttitudeInit()`。不允许其他任务或中断直接改写零偏、四元数和配置。
标定要求实际静止；温漂补偿、磁航向、加速度标定及安装小角度补偿尚未实现。
本项目的配置、集成和测试范围见 [板载 IMU](../../../docs/gimbal/IMU.md)。
