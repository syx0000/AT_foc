# iflytek — 项目信息记录

> 本文档汇总本仓库 FOC 电机控制项目的基础信息与最新进展，作为快速入口。内容以 **代码实际状态为准**（2026-09-01 核对）。

## 核心原则 ⚠️

**【验证原则】不允许假设、猜测的结果做依据，一切要有明确依据**

- ❌ **禁止**：基于文档描述、理论推测、经验判断做决策
- ✅ **必须**：以代码实际实现、硬件手册规格、实测数据为准
- 📋 **验证顺序**：代码 > 数据手册 > 参考文档 > 经验
- 🔍 **存疑时**：实际运行测试，用示波器/逻辑分析仪验证，或查阅芯片勘误表

## 项目定位与概述

基于 **AT32F456CEU7** 微控制器的 **FOC（Field-Oriented Control）伺服电机控制**工程。

- **MCU**: AT32F456CEU7（雅特力 Cortex-M4，实测 192MHz，**支持 CAN-FD**，库头文件 `SUPPORT_CAN_FD`）
- **开发工具**: Artery WorkBench（代码生成）+ Keil MDK5
- **固件库**: AT32F45x_Firmware_Library_V2.0.1
- **构建产物**: `project/MDK_V5/objects/AT32F456CEU7_foc.axf/.bin`（2026-09-01 11:51 编译）
- **App 地址**：0x08004000 起（VTOR 重定位，常驻 Bootloader IAP）

## 工程结构（重构中）


## 硬件与平台（代码现状）

- **栅极驱动**: DRV8353 三相驱动（NFAULT 保护反馈）
- **功率级**: 三相全桥 MOS（桥臂为 SP010N04BGNK 100V SGT N 沟道 MOS，两管并联）
- **电源母线**: **当前宏配置为 48V**（`DC_BUS_VOL_48=1`，`DC_BUS_VOL_24=0`，`DC_BUS_REF=480`即48.0V）；24V 分支保留可切换（阈值 276=27.6V / 192=19.2V 已注释）
- **编码器**: **三路霍尔开关（新板启用，PB5/6/7 = EXINT5/6/7 双沿，`ENCODER_SRC_HALL=1`）**，电角度为 **6 扇区查表模式**（`FlashData.hall_sec_angle[6]`，`Cali` 自动建表，霍尔线序任意）
- **CAN**: **CAN-FD（AT32F456 硬件支持）**，FDCAN 协议 V1.7，32B payload，`can_fd_iso_mode_enable(TRUE)`


## GPIO 引脚映射（代码实际）

| 信号 | 引脚 | 功能 |
|------|------|------|
| **PWM输出（TMR1）** ||
| TMR1_CH1/2/3 | PA8/PA9/PA10 | A/B/C 相上桥 |
| TMR1_CH1C/2C/3C | PA7/PB0/PB1 | A/B/C 相下桥（互补） |
| TMR1_BRK | PB12 | 刹车输入 |
| **ADC采样** ||
| SOA (ADC1_IN0) | PA0 | A相电流采样 |
| SOB (ADC2_IN1) | PA1 | B相电流采样 |
| SOC (ADC2_IN2) | PA2 | C相电流采样（运放偏置异常遗留，无碍双电阻FOC） |
| VDC (ADC2_IN3) | PA3 | 母线电压采样 |
| TEMP_MOTOR (ADC1_IN4) | PA4 | 电机温度 |
| TEMP_MOS (ADC1_IN5) | PA5 | MOS温度 |
| **霍尔编码器（新板，ENCODER_SRC_HALL=1）** | PB5/PB6/PB7 | EXINT5/6/7 双沿，60° 电角度锁相，3us 去抖、400ms 静止判零 |
| **其他** ||
| LED_RUN / LED_ERR | PC13 / PC14 | 运行/错误指示灯 |
| EN_GATE | PC15 | 栅极驱动使能 |
| EN_485 | PB2 | RS485使能 |
| TP_TEST | PB11 | 测试点（示波器） |
| CAN_RES / CAN_STB | PH3 / PH2 | CAN 终端电阻 / STB |

## 应用层模块（重构中）

## 关键外设配置（代码实际）

### 时钟与外设
- 系统时钟 192MHz；NVIC 组4（全抢占）；SysTick 优先级 15

### 编码器速度测量（新板）
- 霍尔双沿 → EXINT9_5_IRQ（优先级0）→ `Encoder_Hall_EdgeISR()`
- 每 FOC tick 估算速度（counts/100us），速度上限 40000，400ms 无边沿清零

### ADC（双 ADC 同步，与 CLAUDE 阶段日志一致）
- **Combine**: `ADC_ORDINARY_SMLT_PREEMPT_SMLT_ONESLAVE_MODE`
- **注入组 10kHz**（TMR1 TRGO）：A/B 相电流（ADC1 CH0 / ADC2 CH1），采样 2.5C，PCCE 中断软件读
- **普通组 1kHz**（TMR6 TRGO）：TEMP_MOTOR(4)/TEMP_MOS(5)/SOC(2)/VDC(3)，DMA1_CH3 → `adc_ordinary_buffer[4]`
- **实际采样时间**：普通组 6.5 cycles（代码 `ADC_SAMPLETIME_6_5`，CLAUDE 记 2.5 已过时）
- 转换公式（12bit）：VDC：`raw*693/4095`（0.1V 单位）；电流：`(raw-offset)*528/8`（Q10）

### TMR
- **TMR1**：中心对齐，Period 9599（10kHz），死区 80，重复计数 1，PWM 模式 B，CH4=9099 预触发
- **TMR6**：UP 计数，Period 999 / Prescaler 191（~1kHz 普通组触发）

### DMA
| 通道 | 用途 |
|------|------|
| DMA1_CH1/CH2 | USART3 TX/RX（DPT，当前禁用） |
| DMA1_CH3 | ADC 普通组（温度/VDC）⚠️ FDT 中断 + recovery |
| DMA1_CH4/CH5 | USART1 RX/TX（调试串口） |

### NVIC（代码实际）
```c
ADC1_2_IRQn(0)   TMR1_CH_IRQn(2)   USART3_IRQn(2)   DMA1_Channel3(4)
CAN1_RX(6)       CAN1_ERR(6)       USART1_IRQn(8)
EXINT9_5_IRQn(0)   // 新增：霍尔编码器双沿
SysTick(15)
```

## 通信（重构中）

## OTA / Bootloader（代码现状：**已实现**）

- App 常驻 **0x08004000**（主工程 VTOR 重定位），Bootloader 0x08000000~0x08004000
- **Flash 布局**（ota_config.h / bootloader README）：
  - 0x08000000 Bootloader(14KB) → 0x08003800 App Header(2KB, app_header_t+回滚计数) → 0x08004000 App 执行区(238KB) → 0x0803F800 Staging Header(2KB) → 0x08040000 Staging 暂存区(254KB) → 0x0807F800 User Data(FOC 参数,2KB)
- **升级协议**：上位机 UART 0x5A 0x01 启动 / 0x31+addr+data+checksum 数据帧 / 0x5A 0x02 结束 / 0x5A 0xA5 跳转；应答 0xCC 0xDD 成功 / 0xEE 0xFF 失败；升级标志 'BWSC' 写入 0x08003800
- App 侧 `ota_init()/ota_process()/ota_mark_self_stable()` 支持回滚保护
- **状态**：OTA 代码已就绪（`ota_app.c` 完整，含 4B 对齐写入）；CAN OTA 命令已接入

## 编译与烧录（代码现状）

**Keil 路径**：`C:\Keil_v5\UV4\UV4.exe`（App 工程 `project/MDK_V5/AT32F456CEU7_foc.uvprojx`，Bootloader 在 `bootloader/mdk_v5/bootloader.uvprojx`）

```bash
# 全量 / 增量 / 烧录（在对应 MDK_V5 目录下）
"C:/Keil_v5/UV4/UV4.exe" -r <工程>.uvprojx -o build_log.txt -j0
"C:/Keil_v5/UV4/UV4.exe" -b <工程>.uvprojx -o build_log.txt -j0
"C:/Keil_v5/UV4/UV4.exe" -f <工程>.uvprojx -o flash_log.txt
```

- App 产物：`objects/AT32F456CEU7_foc.axf` + `.bin`（最近编译 2026-09-01，bin ≈104KB）
- 阶段9 编译规模参考：Code=101502 RO-data=13510 RW-data=264 ZI-data=14144


