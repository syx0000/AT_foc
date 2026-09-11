# AT_foc_hall 代码结构说明

## 1. 项目概览

本仓库是一个基于 **AT32F456CEU7** 的 Hall 传感器 FOC 电机控制项目，主要使用 C 语言开发，并采用 Keil MDK 工程组织。

项目包含主应用、独立 Bootloader、AT32 芯片库、上位机工具、算法测试以及一套参考 BLDC Hall 工程。

```text
AT_foc_hall/
├── project/                         主应用工程
│   ├── src/                         启动、外设配置及系统服务
│   ├── inc/                         对应头文件
│   ├── bsp/                         电机硬件抽象层
│   ├── config/                      板级、电机、OTA、版本配置
│   ├── foc/
│   │   ├── foc_app/                 电机控制业务层
│   │   └── foc_kernel/              无硬件依赖的算法内核
│   └── MDK_V5/                      Keil 工程与编译产物
├── bootloader/                      独立 IAP Bootloader
├── libraries/                       AT32 CMSIS 与标准外设驱动
├── tests/                           纯算法和存储单元测试
├── tools/motor_host/                Python 串口上位机及打包程序
├── sch截图/                          原理图截图
├── *.md                             设计、验证和错误码文档
└── *.pdf / *.ATWP / *.json          电机资料与工程配置
```

## 2. 主应用工程

### 2.1 `project/src`：系统入口和基础服务

- `main.c`：系统启动、外设初始化、电机模块初始化和主循环。
- `at32f45x_int.c`：系统异常及外设中断处理。
- `at32f45x_wk_config.c`：AT32 Work Bench 生成的公共配置。
- `wk_adc.c`、`wk_tmr.c`、`wk_dma.c`、`wk_gpio.c` 等：芯片外设初始化代码。
- `motor_cli.c`：串口命令行解析和命令调度。
- `motor_log.c`：运行日志和日志等级管理。
- `motor_performance_monitor.c`：控制任务执行时间及性能统计。
- `interrupt_monitor.c`：中断次数和频率监控。
- `flash_port.c`：Flash 操作适配层。
- `ota_app.c`：应用侧 OTA/IAP 支持。

### 2.2 `project/bsp`：电机硬件抽象层

- `motor_adc_port`：ADC 与电流采样硬件接口。
- `motor_pwm_port`：三相 PWM、栅极驱动、Break 和紧急停机接口。
- `motor_hall_port`：三路 Hall GPIO 采样及边沿时间戳。
- `motor_uart_port`：串口收发接口。
- `motor_timebase`：DWT 和系统时间基准。

这一层负责隔离 AT32 外设细节，使上层控制模块尽量不直接访问寄存器。

### 2.3 `project/config`：集中配置

- `motor_board_config.h`：板级引脚、外设和硬件参数。
- `motor_control_config.h`：电机本体、PI、保护、启动及 Hall 参数。
- `ota_config.h`：Flash 和 OTA 存储布局。
- `product_version.h`：产品及固件版本信息。

## 3. FOC 控制模块

### 3.1 `project/foc/foc_kernel`：纯算法内核

该目录尽量不依赖具体硬件，包含：

- `motor_foc_math`：Clarke/Park、反 Park、SVPWM 等定点运算。
- `motor_current_pi`：电流 PI 控制器。
- `motor_current_pi_design`：根据电机参数计算电流 PI 参数。
- `motor_speed_pi`：速度 PI 控制器。
- `motor_voltage_limit`：dq 电压矢量联合限幅。
- `motor_ramp`：有符号斜坡生成。
- `motor_hall_decoder`：Hall 状态合法性、方向和边沿解码。
- `motor_hall_angle_math`：Hall 角度计算与转换。
- `motor_hall_calibration`：Hall 六状态顺序和边界角校准算法。
- `motor_hall_offset_identification`：转子角度补偿辨识。
- `motor_direction`：逻辑方向和物理方向转换。

### 3.2 `project/foc/foc_app`：控制业务层

- `motor_control`：统一状态机、启动/停止、运行模式与故障管理。
- `motor_current_control`：正式电流闭环控制。
- `motor_speed_control`：速度外环控制。
- `motor_open_loop`：开环电压和频率控制。
- `motor_current_startup`：I/F 启动及 Hall 闭环接管。
- `motor_current_sample`：快速电流样本处理。
- `motor_current_transform`：相电流及 dq 电流转换。
- `motor_current_calibration`：电流 ADC 零偏校准。
- `motor_rotor_feedback`：转子位置反馈组织。
- `motor_speed_feedback`：Hall 转速计算和滤波。
- `motor_hall_angle_observer`：Hall 边沿角度观测。
- `motor_hall_angle_estimator`：边沿之间的连续角度估算。
- `motor_sensorless_observer`：10 kHz 影子反电势/PLL 观测，只产生诊断快照，不参与 FOC 接管、速度反馈或保护。
- `motor_slow_sensor`：母线电压和温度等慢速传感器处理。
- `motor_parameter`：默认、活动和候选电机参数管理。
- `motor_parameter_storage`：Flash 双槽参数存储。
- `motor_commissioning`：电机自动辨识流程编排。
- `motor_resistance_identification`：相电阻辨识。
- `motor_inductance_identification`：d/q 轴电感辨识。
- `motor_current_loop_test`：电流环测试。
- `motor_torque_loop_test`：转矩环测试。

## 4. 程序运行链路

主应用从 `project/src/main.c` 启动，主要流程如下：

```text
系统时钟和基础时间初始化
          ↓
电机参数及 Flash 参数加载
          ↓
FOC、Hall、采样和控制模块初始化
          ↓
GPIO / PWM / ADC / DMA / UART / CAN 初始化
          ↓
Hall 解码器初始化
          ↓
栅极驱动和硬件故障检查
          ↓
电流 ADC 零偏校准
          ↓
motor_control 进入 READY 状态
          ↓
主循环执行：
  motor_slow_sensor_process()
  motor_cli_poll()
  motor_control_poll()
```

实时控制主要由中断驱动：

- TMR1：产生三相 PWM，控制周期约为 20 kHz。
- ADC：快速电流采样和电流环计算。
- Hall EXINT：捕获 Hall 边沿。
- DMA：慢速 ADC 和串口数据搬运。
- USART：CLI 与通信数据处理。
- CAN：CAN 总线接收和错误处理。
- TMR1 Break：硬件故障紧急关断功率输出。

## 5. Bootloader

`bootloader` 是独立的 AT32F456 IAP 工程：

- `src/main.c`：Bootloader 启动和升级判断。
- `src/iap.c`：升级协议和镜像处理。
- `src/flash.c`：Flash 擦写操作。
- `src/boot_usart.c`：升级串口通信。
- `inc/ota_config.h`：Bootloader 侧内存布局。
- `system/`：启动文件、系统初始化和中断入口。
- `bsp/`：Bootloader 使用的精简外设驱动。
- `mdk_v5/`：Keil Bootloader 工程。

## 6. 芯片支持库

`libraries` 包含：

- `cmsis/cm4`：Cortex-M4 CMSIS 内核、设备定义、启动文件和链接脚本。
- `drivers/inc`：AT32F45x 标准外设驱动头文件。
- `drivers/src`：ADC、DMA、TMR、GPIO、USART、CAN、Flash 等驱动实现。

## 7. 测试代码

`tests` 当前包含以下测试：

- `motor_current_pi_design_test.c`：电流 PI 参数设计测试。
- `motor_direction_angle_test.c`：方向和角度转换测试。
- `motor_hall_calibration_test.c`：Hall 自动校准测试。
- `motor_hall_decoder_test.c`：Hall 解码测试。
- `motor_hall_offset_identification_test.c`：Hall 转子偏移辨识测试。
- `motor_parameter_storage_test.c`：参数双槽存储及异常恢复测试。

## 8. 上位机工具

`tools/motor_host` 是串口调试上位机：

- `motor_host.py`：Python GUI 主程序。
- `requirements.txt`：Python 依赖。
- `AT_foc_hall_MotorHost.spec`：PyInstaller 打包配置。
- `dist/AT_foc_hall_MotorHost.exe`：已打包的 Windows 程序。
- `README.md`：使用说明。

## 9. 参考工程和文档

- `FOC_REFACTOR_DESIGN.md`：FOC 重构设计说明。
- `FOC_DEVELOPMENT_VERIFICATION_PLAN.md`：开发和验证计划。
- `MOTOR_AUTO_COMMISSIONING_DESIGN.md`：电机自动辨识设计。
- `ERROR_CODE_REFERENCE.md`：错误码对照说明。
- `60BLDC140-44030-14J.pdf`：电机相关资料。

## 10. 总体架构

```text
CLI / CAN / 自动辨识
          ↓
    motor_control 状态机
          ↓
开环 / 电流环 / 速度环 / 启动接管
          ↓
      FOC 纯算法内核
          ↓
ADC / Hall / PWM 硬件抽象层
          ↓
       AT32F456 外设
```

核心入口文件：

- `project/src/main.c`：系统入口。
- `project/foc/foc_app/motor_control.c`：电机控制状态机。
- `project/config/motor_control_config.h`：电机控制参数。
- `project/src/at32f45x_int.c`：实时中断入口。
- `project/MDK_V5/AT32F456CEU7_foc.uvprojx`：主应用 Keil 工程。

## 11. 当前实现快照（2026-09-11）

- `project/foc/foc_app/motor_sensorless_observer.c/.h` 在 10 kHz 电流环中运行影子反电势/PLL 观测。调用点位于电压矢量联合限幅、反 Park 及采样电流换算之后，输入是实际限幅 `Vα/Vβ` 和采样 `Iα/Iβ`。
- 观测器输出电角度、电频率、`bemf_alpha/beta`、相位误差、0..1000 置信度、样本计数和 `valid`；它不写入 FOC 角度、速度反馈、指令或保护状态。
- 当前运行参数为 4 对极、`Rs=268 mΩ`、`Ld=222 µH`、`Lq=206 µH`、磁链 `19494 µWb`，电流 PI Q15 为 `40588/37663/4900`。`flux_linkage_uwb` 已纳入默认/活动/候选参数和 Flash 记录。
- Flash 参数记录格式为 v2，双槽地址为 `0x0807F000` / `0x0807F800`；旧 v1 记录会被判为无效并回退编译期默认值。
- CLI 诊断命令为 `observer status`、`observer debug 1`、`observer debug 0`。主应用最近一次 Keil 全量构建为 `0 Error(s), 0 Warning(s)`，产物规模 `Code=64926, RO-data=7010, RW-data=544, ZI-data=8688`。
- 观测器尚未完成目标电机上的 Hall 对比、低速有效性和长时间噪声验证，暂不参与接管或任何保护判定。
