# 可移植转速就绪判定

复制 `speed_ready.c`、`speed_ready.h` 即可移植。使用标准 C，无 HAL、RTOS、电机类型、
总线或内存分配依赖。时间、目标、斜坡给定、实际速度和反馈有效性全部由调用者提供。
每个实例判断一条速度通道，多电机业务自行组合各通道结果。

## 判断过程

1. 必须启用、反馈有效，且目标/给定/反馈为有限数。
2. 最终目标的绝对值至少达到 `minimum_target`，反馈方向与目标一致。
3. 给定与最终目标差不超过 `reference_tolerance`，避免斜坡途中因跟踪良好而误判。
4. 进入时，误差须连续满足 `max(enter_absolute_error, enter_relative_error * abs(target))`。
5. 从第一条合格观察开始计时，连续达到 `dwell_ms` 才置 ready。
6. 已就绪时使用较宽退出误差带；超过退出误差带立即撤销，恢复需重新满足完整进入条件。

两组误差带形成迟滞，减少边界噪声导致的状态反复。相对误差使用比例，0.02 表示 2%。
`stable_ms` 达到 dwell 后封顶，保持 ready 不依赖一个无限增长的计时器。

目标任意改变、重复时间戳、采样间距过大、停用或反馈无效都会撤销就绪。
`SpeedReadyReset()` 可在 PID 参数改变后主动重新判定，保留初始化配置。
毫秒计时器使用 uint32 无符号差值处理回绕；调用必须连续，不能隔完整的计时器周期才更新。

## 使用示例

```c
static SpeedReady_s detector;

uint8_t Init(void)
{
    const SpeedReady_Config_s config = {
        .minimum_target = 100.0f,
        .reference_tolerance = 1.0f,
        .enter_absolute_error = 10.0f,
        .enter_relative_error = 0.02f,
        .exit_absolute_error = 20.0f,
        .exit_relative_error = 0.04f,
        .dwell_ms = 200U,
        .maximum_sample_gap_ms = 5U,
    };
    /* Return value must be checked; invalid config never produces ready. */
    return SpeedReadyInit(&detector, &config);
}

SpeedReady_Status_s Update(float target_rpm, float ramp_rpm, float actual_rpm,
                          uint8_t enabled, uint8_t feedback_fresh, uint32_t now_ms)
{
    const SpeedReady_Input_s sample = {
        .target = target_rpm,
        .reference = ramp_rpm,
        .measured = actual_rpm,
        .enabled = enabled,
        .feedback_valid = feedback_fresh,
    };
    return SpeedReadyUpdate(&detector, &sample, now_ms);
}
```

示例数字仅为本项目低速台架初值，并非 DJI 指标或实际弹速验收标准。
所有速度必须采用相同单位；调用者负责时间戳新鲜度及驱动/温度等外部故障判断。
模块不控制电机、不检测真实出弹，也不提供完整发射许可。

## 测试

`tests/speed_ready/run_tests.ps1 -Compiler <本机gcc>` 编译真实模块及三轮状态组合逻辑，
覆盖连续稳定时间边界、迟滞、斜坡、低速/反向、目标切换、反馈中断、非法配置、
采样间隔异常、时钟回绕，以及三轮就绪/掉速恢复/单轮测试/故障。
