# OpenFOC

OpenFOC 是一个面向嵌入式系统的开源 FOC（Field Oriented Control，磁场定向控制）电机控制库，支持多种控制模式、灵活的 PID/低通滤波器参数配置、易于移植和扩展。适用于无刷直流电机（BLDC）、永磁同步电机（PMSM）等高性能伺服应用。

## 主要特性
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
## 目录结构
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
## 快速上手
1. 配置初始化参数
根据你的应用需求，填写 FOC_InitConfig 结构体，包括模式、PID参数、滤波器时间常数、回调函数等。例如：

1. 初始化 FOC 实例
1. 在主循环中调用运行函数

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

## 依赖与移植
仅依赖标准 C 库
所有硬件相关操作（PWM输出、电流采样、角度读取、时间戳等）均通过回调函数实现，便于适配不同平台
支持浮点运算
## 贡献与许可
本项目遵循 MIT 开源协议，欢迎提交 Issue 和 PR 参与改进！

## 联系作者
作者：Sab1e
如有问题或建议，请通过 GitHub Issue 联系。

Enjoy OpenFOC!
