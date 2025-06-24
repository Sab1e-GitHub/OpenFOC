# ⚙OpenFOC

OpenFOC 是一个面向嵌入式系统的开源 FOC（Field Oriented Control，磁场定向控制）电机控制库，支持多种控制模式、灵活的 PID/低通滤波器参数配置、易于移植和扩展。适用于无刷直流电机（BLDC）、永磁同步电机（PMSM）等高性能伺服应用。

## 🔧主要特性
支持多种 FOC 控制模式：
 - [x] 电流环（Current Control）
 - [x] 速度闭环（Velocity Closedloop）
 - [x] 角度闭环（Angle Closedloop）
 - [x] 速度开环（Velocity Openloop）
 - [x] 角度开环（Angle Openloop）

每种模式可独立配置 PID 参数和低通滤波器时间常数
支持三相/两相信号采样
支持自动相序检测与零电角校准
结构体设计清晰，便于扩展和移植
代码注释详细，便于二次开发
## 📁目录结构
```
OpenFOC/
├── Inc/
│   ├── foc.h              // FOC主头文件，结构体与接口定义
│   ├── foc_utils.h        // FOC数学与工具函数
│   ├── lowpass_filter.h   // 低通滤波器模块
│   └── pid.h              // PID控制器模块
├── Src/
│   ├── foc.c              // FOC主实现
│   ├── foc_utils.c        // FOC数学与工具函数实现
│   ├── lowpass_filter.c   // 低通滤波器实现
│   └── pid.c              // PID控制器实现
```
## 👉快速上手
1. 配置初始化参数
根据你的应用需求，填写 FOC_InitConfig 结构体，包括模式、PID参数、滤波器时间常数、回调函数等。例如：
```
FOC_InitConfig config = {
    .mode = FOC_Mode_Velocity_Closedloop,
    .mode_params.vel_closed = {
        .target_speed_rad_per_sec = 10.0f,
        .speed_pid = {1.0f, 0.1f, 0.0f, 5.0f},
        .iq_pid = {2.0f, 0.2f, 0.0f, 5.0f},
        .id_pid = {2.0f, 0.2f, 0.0f, 5.0f},
        .lpf_speed_ts = 0.01f,
        .lpf_iq_ts = 0.005f,
        .lpf_id_ts = 0.005f,
    },
    .pole_pairs = 7,
    .pwm_period = 0.00002f,
    .v_dc = 24.0f,
    // 其它回调函数和参数...
};
```
1. 初始化 FOC 实例
```
FOC_Instance foc;
foc_init(&foc, &config);
```
1. 在主循环中调用运行函数
```
while (1) {
    foc_run(&foc);
    // 其它应用逻辑
}
```

### TimeBase
> [!CAUTION]
> OpenFOC 所有定时相关功能（如 PID、低通滤波器、速度计算等）都依赖于 get_micros 回调函数。
> 你必须确保 get_micros 返回的是单调递增的、单位为微秒（us）的 64 位无符号时间戳。
>
> 不正确的实现（如返回毫秒、非单调递增、溢出等）会导致控制算法异常甚至失控。
> 推荐直接调用 MCU 的高精度定时器或系统滴答定时器，并确保溢出处理正确。
> 务必在移植和使用前充分验证 get_micros 的准确性和稳定性！

示例：
```
/* ------基于STM32 HAL库------ */
void dwt_init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk; // 使能 DWT 模块
    DWT->CYCCNT = 0;                                // 复位计数器
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;            // 启用 CYCCNT
}
uint64_t micros64(void)
{
    static uint32_t last_cycle = 0;
    static uint64_t micros_accum = 0;

    uint32_t current_cycle = DWT->CYCCNT;
    uint32_t cycle_delta = current_cycle - last_cycle;
    last_cycle = current_cycle;

    micros_accum += (uint64_t)cycle_delta / (SystemCoreClock / 1000000UL);
    return micros_accum;
}

dwt_init();

config.get_micros = micros64;
```

### 角度传感器方向

请自行判断角度传感器方向。

 - 如果角度传感器顺时针增加，则设置为`FOC_Direction_CW`
 - 如果角度传感器逆时针增加，则设置为`FOC_Direction_CCW`

### 电机的相序

相序检测函数必须基于正确角度传感器方向；
如果相序不清楚，只需要设置某一相为：`FOC_PHASE_UNKNOWN`即可，上电后就会自动开始校准。

> [!NOTE]
> 此功能还在测试中，不确定是否能完美适配所有电机

示例：
```
config.phase_ch1=FOC_PHASE_UNKNOWN;
```

### 电角度的校准

如果电角度设置为0则会自动校准电角度。

### 模式配置

有多种模式可供配置，但是要注意使用`config.mode`配置完模式后，需要填入`config.mode_params`里面的必要参数。

## 主要接口说明
```
void foc_init(FOC_Instance *foc, const FOC_InitConfig *config);  // 初始化 FOC 控制器实例
```

```
void foc_run(FOC_Instance *foc);  // 执行一次 FOC 控制流程（建议周期性调用）
```

```
FOC_Status foc_auto_detect_phase_order(FOC_Instance *foc);  // 自动检测电机相序
```

```
void foc_calibrate_zero_electric_angle(FOC_Instance *foc);  // 零电角校准
```

```
void foc_set_phase(FOC_Instance *foc, Motor_Phase ch1, Motor_Phase ch2, Motor_Phase ch3);  // 手动设置相序
```

## 💿依赖与移植
仅依赖标准 C 库

所有硬件相关操作（PWM输出、电流采样、角度读取、时间戳等）均通过回调函数实现，便于适配不同平台

支持浮点运算
## ⚖贡献与许可
本项目遵循 MIT 开源协议，欢迎提交 Issue 和 PR 参与改进！

## 📨联系作者
作者：Sab1e

如有问题或建议，请通过 GitHub Issue 联系。

Enjoy OpenFOC!😆
