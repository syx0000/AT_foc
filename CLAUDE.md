# AT32F456CEU7 FOC 电机控制项目

## 核心原则 ⚠️

**【验证原则】不允许假设、猜测的结果做依据，一切要有明确依据**

- ❌ **禁止**：基于文档描述、理论推测、经验判断做决策
- ✅ **必须**：以代码实际实现、硬件手册规格、实测数据为准
- 📋 **验证顺序**：代码 > 数据手册 > 参考文档 > 经验
- 🔍 **存疑时**：实际运行测试，用示波器/逻辑分析仪验证，或查阅芯片勘误表

**示例**：
- ❌ 错误：文档说支持CAN-FD → 假设能用32字节
- ✅ 正确：查看芯片头文件 `SUPPORT_CAN_FD` 宏定义 → 确认 `CAN_DLC_BYTES_32` 存在 → 验证通过

## 项目概述

基于AT32F456CEU7微控制器的FOC（Field-Oriented Control）电机控制项目。

- **MCU**: AT32F456CEU7
- **开发工具**: Artery WorkBench (代码生成工具)
- **项目路径**: `AT32F456CEU7_foc\project\`
- **固件库**: AT32F45x_Firmware_Library_V2.0.1
- **官方demo参考**: `AT32F45x_Firmware_Library_V2.0.1\project\at_start_f456\examples\`

## 移植参考工程

**参考工程路径**: `C:\Users\syx19\Desktop\X\cubemx_yxsui\`

这是一个基于STM32H7的CubeMX工程（STM32H7xx HAL），是当前AT32F456工程的**移植源头**。后续将逐步把参考工程的全部FOC控制逻辑、协议、保护代码移植到本工程。

### 参考工程关键目录

```
cubemx_yxsui/
├── cubemx_yxsui.ioc           # CubeMX配置文件（参考外设配置）
├── Core/
│   ├── Inc/                   # HAL外设驱动（adc/can_debug/can_wly/dma/encoder/fdcan/
│   │                          #  flash_port/gpio/main/ota_app/tim/usart）
│   └── Src/                   # 对应实现
├── foc/
│   ├── foc_app/               # 应用层
│   │   ├── ifly_fault.c       # 故障保护
│   │   ├── ifly_fault_api.c
│   │   ├── ifly_flux_ident.c  # 磁链辨识
│   │   ├── ifly_inertia_ident.c # 惯量辨识
│   │   ├── ifly_led.c
│   │   ├── ifly_test.c
│   │   └── phase_resistance_ident.c # 相电阻辨识
│   └── foc_fast/              # 快速循环（FOC核心）
│       ├── encoder_calc.c     # 编码器计算
│       ├── foc_api.c
│       ├── foc_bsp.c          # 板级支持
│       ├── foc_controller.c   # FOC控制器
│       ├── foc_current_loop.c # 电流环
│       ├── foc_data.c
│       └── foc_kernel.c       # FOC内核
├── Bootloader/                # IAP升级
├── MDK-ARM/                   # Keil工程
└── 文档：
    ├── CAN_0x100_FRAME_SPEC.md      # CAN协议
    ├── CAN_DEBUG_VERIFICATION.md
    ├── CAN_WLY_PROTOCOL_VERIFY.md
    ├── DRV8353_PROTECTION.md         # DRV8353驱动保护
    ├── FAULT_PROTECTION.md
    ├── FOC_ISR_OPTIMIZATION.md       # FOC中断优化
    ├── HIGH_SPEED_100RPM_ANALYSIS.md
    ├── PHU_FEATURE_GAP.md
    ├── SERIAL_CMD_GUIDE.md
    ├── TORQUE_DEFICIT_VERIFY.md      # 扭矩下跌验证
    ├── WEAK_FLUX_VERIFY.md           # 弱磁验证
    └── 中断配置清单.md / 死代码清单.md
```

### 移植策略

**移植内容（待规划）：**
1. **foc/foc_fast/** - FOC核心算法（电流环、控制器、编码器计算）
2. **foc/foc_app/** - 应用层（故障保护、参数辨识、LED）
3. **Core/Src/** 中与外设无关的协议层（can_wly、ota_app、flash_port）
4. **CAN协议** - 参照 `CAN_0x100_FRAME_SPEC.md` / `CAN_WLY_PROTOCOL_VERIFY.md`
5. **故障保护** - 参照 `FAULT_PROTECTION.md` / `DRV8353_PROTECTION.md`

**不移植内容：**
- HAL外设驱动（已用AT32 BSP替代：wk_adc/wk_dma/wk_tmr/wk_usart/wk_can/wk_gpio）
- STM32 specific代码（stm32h7xx_*.c/h）

**关键差异：**
- STM32H7 → AT32F456：库不同（HAL → AT32 BSP），外设寄存器结构相似但API不同
- ADC：STM32 HAL_ADC vs AT32 adc_xxx
- DMA：STM32 DMA + DMAMUX vs AT32 DMA + DMAMUX（结构相似）
- TIM：STM32 TIM vs AT32 TMR（API名称改 tim → tmr）
- CAN：STM32 FDCAN vs AT32 CAN（注意是bxCAN，不是FDCAN）

## GPIO引脚映射

| 信号 | 引脚 | 功能 |
|------|------|------|
| **PWM输出（TMR1）** |||
| TMR1_CH1 | PA8 | A相上桥 |
| TMR1_CH2 | PA9 | B相上桥 |
| TMR1_CH3 | PA10 | C相上桥 |
| TMR1_CH1C | PA7 | A相下桥（互补） |
| TMR1_CH2C | PB0 | B相下桥（互补） |
| TMR1_CH3C | PB1 | C相下桥（互补） |
| TMR1_BRK | PB12 | 刹车输入 |
| **ADC采样** |||
| SOA (ADC1_IN0) | PA0 | A相电流采样 |
| SOB (ADC2_IN1) | PA1 | B相电流采样 |
| SOC (ADC2_IN2) | PA2 | C相电流采样 |
| SO3 (ADC2_IN3) | PA3 | 第4路电流采样 |
| TEMP_MOTOR (ADC1_IN4) | PA4 | 电机温度 |
| TEMP_MOS (ADC1_IN5) | PA5 | MOS温度 |
| **其他** |||
| LED_RUN | PC13 | 运行指示灯 |
| LED_ERR | PC14 | 错误指示灯 |
| EN_GATE | PC15 | 栅极驱动使能 |
| EN_485 | PB2 | RS485使能 |
| TP_TEST | PB11 | 测试点（示波器） |
| CAN_RES | PH3 | CAN终端电阻 |
| CAN_STB | PH2 | CAN STB |

## 关键外设配置

### ADC配置（双ADC同步采样）

**Combine模式：** `ADC_ORDINARY_SMLT_PREEMPT_SMLT_ONESLAVE_MODE`（普通组+注入组同时同步双ADC模式）

**注入组（Preempt，10kHz）- 电流采样**
- ADC1: CH0 (PA0) - A相电流
- ADC2: CH1 (PA1) - B相电流
- 触发源: TMR1 TRGO (10kHz)，上升沿
- ADC1 采样时间: 2.5 cycles
- 读取方式: **软件读取**（在ADC1_2_IRQHandler的PCCE中断里）

**普通组（Ordinary，500Hz）- 温度等慢速数据**
- ADC1: CH4 (PA4 TEMP_MOTOR), CH5 (PA5 TEMP_MOS)
- ADC2: CH2 (PA2 SOC), CH3 (PA3 SO3)
- 触发源: TMR6 TRGO，上升沿（ADC2 NONE，从机自动同步）
- 采样时间: 2.5 cycles
- 序列长度: 2
- sequence_mode: TRUE
- repeat_mode: FALSE
- 读取方式: **DMA1_CHANNEL3**（4个halfword: ADC1_CH4, ADC2_CH2, ADC1_CH5, ADC2_CH3）

**Common DMA配置：**
- common_dma_mode: `ADC_COMMON_DMAMODE_1`
- common_dma_request_repeat_state: `TRUE`
- div: `ADC_HCLK_DIV_3`
- sampling_interval: `ADC_SAMPLING_INTERVAL_5CYCLES`

### TMR配置

**TMR1 (PWM + 注入组触发，10kHz)**
- 频率: 10kHz
- 计数模式: `TMR_COUNT_TWO_WAY_3` (中心对齐)
- Period: 9599
- Prescaler: 0
- 重复计数: 1
- PWM模式: `TMR_OUTPUT_CONTROL_PWM_MODE_B`
- 死区时间: 80
- TRGO: `TMR_PRIMARY_SEL_OVERFLOW`
- CH4中断使能（用于触发ADC同步采样）

**TMR6 (普通组触发，500Hz)**
- 计数模式: `TMR_COUNT_UP`
- Period: 999
- Prescaler: 191
- TRGO: `TMR_PRIMARY_SEL_OVERFLOW`
- 192MHz / 192 / 1000 = 1kHz（实际500Hz观察值，可能与中心对齐有关）

### DMA配置

**DMA1_CHANNEL1 (USART3_TX)**
- direction: M2P, halfword, low priority

**DMA1_CHANNEL2 (USART3_RX)**
- direction: P2M, halfword, low priority

**DMA1_CHANNEL3 (ADC1 - 普通组Common DMA)** ⚠️ 关键
- direction: P2M
- 数据宽度: HALFWORD (peripheral) / HALFWORD (memory)
- 优先级: LOW（建议改成HIGH）
- loop_mode_enable: FALSE（建议改成TRUE）
- DMAMUX: `DMAMUX_DMAREQ_ID_ADC1`
- 中断: `DMA_FDT_INT` 已使能
- 外设地址: `&ADCCOM->codt`
- 内存地址: `adc_ordinary_buffer[4]`
- 传输大小: 4

**DMA1_CHANNEL4 (USART1_RX)** / **DMA1_CHANNEL5 (USART1_TX)**

### NVIC中断配置

```c
nvic_irq_enable(DMA1_Channel3_IRQn, 0, 0);  // ADC DMA
nvic_irq_enable(ADC1_2_IRQn, 0, 0);         // ADC转换完成
nvic_irq_enable(CAN1_RX_IRQn, 0, 0);        // CAN接收
nvic_irq_enable(TMR1_CH_IRQn, 0, 0);        // TMR1 CH4
nvic_irq_enable(USART1_IRQn, 0, 0);
nvic_irq_enable(USART3_IRQn, 0, 0);
```

## 中断处理函数（at32f45x_int.c）

### 全局变量（计数器）
```c
uint32_t tmr1_ch4_int_count = 0;     // TMR1 CH4 中断
volatile uint32_t systick_ms = 0;    // SysTick 1ms
uint32_t adc_occe_count = 0;         // ADC普通组转换完成
uint32_t adc_pcce_count = 0;         // ADC注入组转换完成
uint32_t dma_fdt3_count = 0;         // DMA传输完成
uint32_t adc_occo_count = 0;         // ADC普通组溢出（声明但未使用）
uint32_t adc_tcf_count = 0;          // ADC触发转换失败
extern uint16_t adc_ordinary_buffer[4];  // DMA缓冲区
```

### 中断处理逻辑

| 中断 | 处理动作 |
|------|---------|
| `SysTick_Handler` | `systick_ms++` |
| `DMA1_Channel3_IRQHandler` (FDT3) | 清标志 + `dma_fdt3_count++` + 翻转TP_TEST(PB11) |
| `ADC1_2_IRQHandler` (OCCE) | 清标志 + `adc_occe_count++` + 调用 `adc_ordinary_convert_recovery()` |
| `ADC1_2_IRQHandler` (PCCE) | 清标志 + `adc_pcce_count++` |
| `ADC1_2_IRQHandler` (TCF) | 清标志 + `adc_tcf_count++` + 调用 `adc_ordinary_convert_recovery()` |
| `TMR1_CH_IRQHandler` (CH4) | 清标志 + `tmr1_ch4_int_count++` |
| `CAN1_RX_IRQHandler` | CAN接收处理 |

## 关键函数（wk_adc.c）

### `adc_ordinary_convert_recovery()`
```c
void adc_ordinary_convert_recovery(void)
{
    // 1. 禁用ADC
    adc_enable(ADC1, FALSE);
    adc_enable(ADC2, FALSE);

    // 2. 保存combine模式，临时切换到独立模式
    uint32_t recovery_index = adc_combine_mode_get();
    adc_combine_mode_set(ADC_INDEPENDENT_MODE);

    // 3. 重新初始化DMA
    dma_channel_enable(DMA1_CHANNEL3, FALSE);
    dma_flag_clear(DMA1_FDT3_FLAG);
    dma_data_number_set(DMA1_CHANNEL3, 4);
    dma_channel_enable(DMA1_CHANNEL3, TRUE);

    // 4. 恢复combine模式
    adc_combine_mode_set((adc_combine_mode_type)recovery_index);

    // 5. 重新使能ADC
    adc_enable(ADC1, TRUE);
    adc_enable(ADC2, TRUE);
}
```

每次ADC普通组转换完成（OCCE）或触发失败（TCF）时调用，重置DMA计数器，让DMA继续工作。

## main.c 主程序

### 初始化顺序（当前WorkBench默认顺序）

```c
1. wk_system_clock_config()
2. wk_periph_clock_config()
3. systick_interrupt_config(1000)  // 1kHz tick
4. wk_nvic_config()
5. wk_timebase_init()
6. wk_gpio_config()
7. wk_tmr6_init()                  // TMR6 (会自动启动计数器)
8. wk_dma1_channel3_init() + config + enable  // ADC DMA
9. wk_adc_common_init()
10. wk_adc1_init() + adc_enable
11. wk_adc2_init() + adc_enable
12. wk_usart1_init() / wk_usart3_init() / wk_can1_init()
13. wk_tmr1_init()                 // TMR1 (会自动启动计数器)
14. 其他DMA channels (1/2/4/5)
```

### 主循环（user code begin 3）

```c
while(1) {
    static uint32_t last_tick = 0;
    if(systick_ms - last_tick >= 1000) {
        last_tick = systick_ms;
        printf("PCCE=%u OCCE=%u DMA=%u TCF=%u\r\n",
               adc_pcce_count, adc_occe_count,
               dma_fdt3_count, adc_tcf_count);

        // PWM占空比扫描 0-4800（0-50%）
        if(pwm_dir) { pwm_duty += 480; if(pwm_duty >= 4800) pwm_dir = 0; }
        else        { pwm_duty -= 480; if(pwm_duty == 0) pwm_dir = 1; }

        tmr_channel_value_set(TMR1, TMR_SELECT_CHANNEL_1, pwm_duty);
        tmr_channel_value_set(TMR1, TMR_SELECT_CHANNEL_2, pwm_duty);
        tmr_channel_value_set(TMR1, TMR_SELECT_CHANNEL_3, pwm_duty);
    }
}
```

## 已知问题

### DMA持续工作问题
- **现象**: DMA有时只触发一次或不触发
- **原因**: WorkBench默认初始化顺序与官方demo不一致
- **官方demo顺序**: TMR init → DMA init → ADC init+enable → DMA enable → TMR enable
- **解决方案**: 在ADC的OCCE/TCF中断里调用`adc_ordinary_convert_recovery()`手动重置DMA

### 调试用引脚
- **TP_TEST (PB11)**: 在DMA FDT3中断里翻转，可用示波器观察DMA触发频率

## 代码文件结构

```
project/
├── inc/
│   ├── at32f45x_wk_config.h    # 引脚定义、DMA buffer宏
│   ├── at32f45x_int.h
│   ├── wk_adc.h                # ADC初始化 + 业务层（FOC采样、校准 - 待实现）
│   ├── wk_dma.h
│   ├── wk_tmr.h                # TMR初始化 + DWT性能测量
│   ├── wk_usart.h              # USART初始化 + isr_print
│   ├── wk_can.h, wk_gpio.h, wk_system.h
│   └── ...
├── src/
│   ├── main.c                   # 主程序，初始化顺序
│   ├── at32f45x_int.c          # 中断处理 + 计数器变量
│   ├── at32f45x_wk_config.c    # 时钟、NVIC、外设时钟
│   ├── wk_adc.c                # ADC初始化 + recovery函数 + 业务层（待实现）
│   ├── wk_dma.c
│   ├── wk_tmr.c                # TMR初始化 + DWT实现（user code区）
│   ├── wk_usart.c              # USART初始化 + isr_print实现（user code区）
│   ├── wk_can.c, wk_gpio.c, wk_system.c
│   └── ...
└── MDK_V5/
    └── AT32F456CEU7_foc.uvprojx  # Keil工程文件

注：业务层代码（DWT、isr_print、ADC采样等）放在wk_*.c的user code区，
    对齐参考工程 cubemx_yxsui/Core/{Inc,Src}/*.{h,c} 的组织方式，
    方便后续移植foc_app/foc_fast代码。
```

## DMA缓冲区定义

```c
// main.c
uint16_t adc_ordinary_buffer[4]; // HALFWORD: 2x ADC1 + 2x ADC2
// 实际数据排列: [ADC1_CH4, ADC2_CH2, ADC1_CH5, ADC2_CH3]
```

## 调试输出格式

```
FOC start
PCCE=10000 OCCE=500 DMA=10 TCF=0
PCCE=20000 OCCE=1000 DMA=20 TCF=0
...
```

**期望频率：**
- PCCE: ~10kHz
- OCCE: ~500Hz（或1kHz，看TMR6实际频率）
- DMA: 与OCCE接近（每次OCCE后recovery重置DMA）
- TCF: 应为0（无错误）

## 编译方法

### 使用Keil IDE
1. 打开 `project/MDK_V5/AT32F456CEU7_foc.uvprojx`
2. 点击 Rebuild 按钮或按 Shift+F7

### 命令行编译

**Keil路径**：`C:\Keil_v5\UV4\UV4.exe`

**全量编译（rebuild）**：
```bash
cd project/MDK_V5
"C:/Keil_v5/UV4/UV4.exe" -r AT32F456CEU7_foc.uvprojx -o build_log.txt -j0
cat build_log.txt | tail -10
```

**增量编译（build）**：
```bash
cd project/MDK_V5
"C:/Keil_v5/UV4/UV4.exe" -b AT32F456CEU7_foc.uvprojx -o build_log.txt -j0
cat build_log.txt | tail -10
```

**参数说明：**
- `-r` rebuild全量重编译，`-b` build增量编译
- `-o` 指定日志输出文件（必须在工程目录下，不支持绝对路径）
- `-j0` 使用所有CPU核心并行编译

**编译产物：**
- `objects/AT32F456CEU7_foc.axf` - ELF可执行文件
- `objects/AT32F456CEU7_foc.hex` - HEX烧录文件
- `build_log.txt` - 编译日志（纯文本）

**编译规模：**
```
Program Size: Code=60322 RO-data=5102 RW-data=252 ZI-data=14044
".\objects\AT32F456CEU7_foc.axf" - 0 Error(s), 0 Warning(s).
```

## 烧录方法

### 使用Keil IDE
1. 连接DAP调试器（CMSIS-DAP / J-Link / ST-Link）
2. 点击 Download 按钮或按 F8

### 命令行烧录
```bash
cd project/MDK_V5
"C:/Keil_v5/UV4/UV4.exe" -f AT32F456CEU7_foc.uvprojx -o flash_log.txt
cat flash_log.txt | tail -5
```

**参数说明：**
- `-f` flash（下载到芯片）

**预期输出：**
```
Load "C:\\...\\AT32F456CEU7_foc.axf"
Erase Done.Programming Done.Verify OK.Application running ...
Flash Load finished at HH:MM:SS
```

## 参考资料

- AT32F456 数据手册
- AT32F45x固件库用户手册
- 官方ADC+DMA示例：`combine_ordinary_smlt_tmr1trgout2_dma1`
- iflytek参考项目: `C:\Users\syx19\Desktop\X\iflytek_joint_module_angle\`
- [ADC_DMA_INIT_ORDER.md](./ADC_DMA_INIT_ORDER.md) - 详细初始化顺序说明

## 开发日志

### 阶段0：基础设施（已完成 ✅）

**目标**：建立DWT性能测量 + isr_print非阻塞打印

**完成内容：**
1. ✅ DWT cycle counter（`wk_tmr.c/h` user code区）
   - `DWT_Init()` - 使能CYCCNT，CPU频率lazy-load（无初始化顺序依赖）
   - `DWT_GetCycles()` - 读取cycle count
   - `DWT_GetMicros()` - 读取微秒数
   - `DWT_CyclesToUs()` - cycles转us
2. ✅ isr_print环形缓冲区（`wk_usart.c/h` user code区）
   - `isr_print_init()` - 初始化ring buffer
   - `isr_print(str)` - ISR里调用，非阻塞写入
   - `isr_print_poll()` - 主循环轮询drain
3. ✅ main.c验证代码（1000 NOPs测试 + 100us busy-wait）
4. ✅ TMR1 CH4 ISR测试（每秒打印一次）

**验证结果：**
```
FOC start
DWT: 1000-NOP cycles=1261
DWT: 100us busy-wait, actual=100 us
[ISR] TMR1 CH4 tick
PCCE=10003 OCCE=1000 DMA=1000 TCF=0
```

**关键发现：**
- DWT必须在系统时钟配置后读取频率，解决方案：lazy-load CPU频率
- AT32F456 实测运行 192MHz
- 代码组织对齐参考工程（DWT在wk_tmr、isr_print在wk_usart），方便后续移植

**编译规模：** Code=14902 RO-data=566 RW-data=16 ZI-data=8080

### 阶段1.1：ADC业务层（已完成 ✅）

**目标**：ADC采样业务封装 + 零点校准

**完成内容：**
1. ✅ `FOC_CurrentSample_t` 数据结构（`wk_adc.h` user code区）
2. ✅ `adc_foc_on_injected_done()` - 注入组PCCE ISR读取Phase A/B电流
3. ✅ `adc_foc_on_regular_done()` - 规则组OCCE ISR读取温度/so_c/so_3
4. ✅ `adc_calibrate_offsets(1024)` - 阻塞式零点校准（1024样本平均）
5. ✅ 启动时自动校准，校准后电流值归零（i_a/i_b ≈ 0±10）

**验证结果：**
```
ADC: calibrating offsets (1024 samples)...
ADC: offset_a=967 offset_b=12
ADC: i_a=-2 i_b=-3 (raw_a=965 raw_b=9 offs_a=967 offs_b=12)
     temp_m=4092 temp_mos=8 so_c=1368 so_3=2815
```

**关键发现：**
- AT32注入组数据用 `adc_preempt_conversion_data_get(ADC, ADC_PREEMPT_CHANNEL_1)` 读取
- `pdt`是data寄存器，`pcdto`是offset寄存器（不可混淆）
- 硬件零点（raw_a≈970, raw_b≈12）不在标准2048，但校准可自动补偿
- ⚠️ raw_b≈12异常偏低，待后续确认硬件（不影响当前开发）

### 阶段1.2：TMR业务层（已完成 ✅）

**目标**：PWM启停 + 占空比控制

**完成内容（wk_tmr.c/h user code区）：**
1. ✅ `TIM1_PWM_Start()` - 启动PWM输出
2. ✅ `TIM1_PWM_Stop()` - 安全关断
3. ✅ `TIM1_SetDuty(ccr1, ccr2, ccr3)` - 设置三相占空比
4. ✅ `pwm_ccr_set()` - 直接寄存器写入（FOC ISR热路径）
5. ✅ ADC ISR性能计数器变量（`g_adc_isr_t_*`）

### 阶段2：DPT编码器（已完成 ✅）

**目标**：DPT双磁24位编码器UART读取

**硬件配置：**
- USART3 @ 2.5Mbps
- TX=PB10, RX=PA6, DE=PB14（硬件RS485模式）
- 不需要软件方向控制

**完成内容（wk_usart.c/h user code区）：**
1. ✅ `DPT_CalcCRC8()` - CRC-8查表算法（256字节表）
2. ✅ `DPT_ReadDualAngle()` - 阻塞读取双角度（0x33命令，7字节响应）
3. ✅ `DPT_ReadDualAngleWithStatus()` - 带状态字（0x43命令，8字节响应）
4. ✅ `DPT_ReadTemperature()` - 读温度（0x74命令，3字节响应）
5. ⏸️ `DPT_GetLatestAngles()` - 异步API（stub，待DMA实现）

**验证结果：**
```
DPT: inner_raw=8564847 outer_raw=14933413 inner=183.78 outer=320.44
```
- ✅ CRC校验通过率100%
- ✅ 24-bit角度解析正确
- ✅ 转动测试角度连续变化

### 阶段3：FOC核心移植（已完成 ✅）

**目标**：将参考工程FOC算法移植到AT32

**移植规模：**
- 30个C文件（foc_fast 16 + foc_app 7 + 工具函数）
- 8400+行算法代码
- 编译产物：Code=20372 + RO-data=1356 = ~21KB Flash

**移植策略：**
1. 直接复制全部FOC源码到 `project/foc/{foc_fast,foc_app}/`
2. 批量替换 `#include` 路径：`tim.h→wk_tmr.h`, `adc.h→wk_adc.h` 等
3. STM32 HAL兼容宏（在foc_bsp.h）：
   - `HAL_Delay → wk_delay_ms`
   - `HAL_GPIO_WritePin → gpio_bits_set/reset`
   - `HAL_GetTick → systick_ms`
4. STM32 TIM寄存器映射：
   - `TIM1->CCR1/2/3 → TMR1->c1dt/c2dt/c3dt`
   - `TIM1->BDTR → TMR1->brk`, `TIM1->SR → TMR1->ists`, `TIM1->CCER → TMR1->cctrl`
5. 全局变量分散到合适位置：
   - `g_theta_offset_pos/neg, g_theta_comp_pos/neg` → encoder_calc.c
   - `NPP` → foc_data.c
   - `g_vdc_raw` → wk_adc.c
   - `pwm_ccr_set, g_adc_isr_t_*` → wk_tmr.c
6. 未移植组件用stub头文件：
   - `can_wly.h`, `can_debug.h`, `flash_port.h` 在 project/inc/
7. **foc_bsp.c 重写**为AT32最小适配版（HPMicro hardware init全部空stub）
   - dbg_cmd_set/dbg_log_print 暂未移植（依赖HAL UART + CAN协议）

**已就绪的算法模块：**
- foc_kernel - SVPWM + Park/Clarke变换
- foc_current_loop - 电流环PI
- foc_speed_loop - 速度环PI
- foc_position_loop - 位置环PI
- foc_controller - 上层调度器
- encoder_calc - 编码器解算
- foc_api - 应用接口
- ifly_fault - 故障保护
- ifly_flux_ident, inertia_ident - 参数辨识
- ifly_led - LED指示
- func_pid, func_filter, func_trigonometric - 工具函数

**待后续工作：**
- 异步DPT编码器（DMA + ISR触发）
- CAN协议层移植（can_wly.c / can_debug.c）
- OTA固件升级

**编译规模：** Code=20372 RO-data=1356 RW-data=16 ZI-data=8112
**烧录后：** 原有ADC/PWM/编码器功能保持正常工作

### 阶段4：FOC闭环（部分完成 ⏸️）

**目标**：FOC初始化 + ADC ISR调度 + 调试打印（待硬件验证开环/闭环）

**完成内容：**

**Step 1 - main.c FOC初始化（user code begin 2）：**
```c
/* 1. 设置电机参数 */
set_ver_par(90);  /* id=90 motor_h7_0426 配套，NPP=8 */

/* 2. 启动PWM输出 */
TIM1_PWM_Start();
TMR1->cctrl |= 0x0555u;  /* enable CH1/CH2/CH3 */

/* 3. FOC核心初始化 */
Init_foc(&controller_eyou);

/* 4. 同步ADC偏置到FOC */
controller_eyou.FlashData.Ia_offset = (uint16_t)g_adc_offset_a;
controller_eyou.FlashData.Ib_offset = (uint16_t)g_adc_offset_b;

/* 5. 编码器零位初始化 */
Encoder_out_data_Reset(MaxPositionLimit, MinPositionLimit);

/* 6. 复位控制数据（清积分器） */
ResetControlData(&controller_eyou);

/* 7. 安全初始状态 */
controller_eyou.foc_run = 0;             /* 闭环未启动 */
controller_eyou.controller_mode = PROFILE_TORQUE_MODE;
controller_eyou.I_q_ref = 0;
```

**Step 2 - ADC PCCE ISR FOC调度（at32f45x_int.c）：**
```c
if(adc_interrupt_flag_get(ADC1, ADC_PCCE_FLAG) != RESET) {
    adc_flag_clear(ADC1, ADC_PCCE_FLAG);
    adc_pcce_count++;
    adc_foc_on_injected_done();

    /* FOC scheduling */
    uint16_t raw_a = adc_preempt_conversion_data_get(ADC1, ADC_PREEMPT_CHANNEL_1);
    uint16_t raw_b = adc_preempt_conversion_data_get(ADC2, ADC_PREEMPT_CHANNEL_1);
    controller_eyou.Ia_raw = raw_a;
    controller_eyou.Ib_raw = raw_b;

    if (controller_eyou.foc_run >= 1) {
        MC_Loop_Schedule(&controller_eyou);  /* 闭环：电流/速度/位置 */
    } else if (g_foc_openloop_enable) {
        phase_current_sample(&controller_eyou);
        FocOpenTest(&controller_eyou, open_loop_mode, v_d_test, v_q_test, raw_a, raw_b);
    }
}
```

**Step 3 - main循环FOC状态打印（每秒）：**
- INT计数（PCCE/OCCE/DMA/TCF）
- ADC原始/校准值 + 温度 + so_c/so_3 + vdc
- DPT编码器角度
- FOC状态：run/mode/openloop
- FOC变量：I_d/I_q/I_q_ref/theta_e/pos_out/dtheta

**全局变量定义（运行时控制）：**
- `g_foc_openloop_enable` (wk_adc.c) - 开环测试使能 [0/1]
- `open_loop_mode` (main.c) - 开环模式 [0=自动旋转, 1=编码器跟随]
- `v_d_test, v_q_test` (main.c) - 开环DQ轴电压 (Q10格式)
- `controller_eyou.foc_run` - 闭环使能 [0=禁用, 1=电流环, 2=速度环, 3=位置环]
- `controller_eyou.controller_mode` - 控制模式 [PROFILE_TORQUE/VELOCITY/POSITION]

**待硬件验证（下次有硬件时）：**

⚠️ **当前是安全状态：foc_run=0, openloop=0, PWM占空比=0，电机不会动**

**测试步骤：**

1. **基础验证**（无需电机）：
   - 烧录运行，串口输出"FOC init done: NPP=8 foc_run=0 mode=2"
   - vdc原始值有数据（接电源后）
   - 电流/角度都为0

2. **开环测试**（电机能转）：
   - 修改main.c初始化最后：`g_foc_openloop_enable = 1; v_q_test = 256;`
   - 烧录后电机应该慢慢自转
   - 看 `theta_e` 在电角度0~65535周期变化

3. **闭环验证**（FOC闭环工作）：
   - 修改：`controller_eyou.foc_run = 1; controller_eyou.I_q_ref = 1024;` (1A)
   - I_d→0, I_q→1024 (跟踪给定)
   - 电机产生扭矩

**编译规模：** Code约21KB（FOC初始化+调度+调试打印）

**未完成（阶段5+）：**
- 电流环参数辨识（需要电机 + 硬件验证）
- 速度环/位置环调试
- 性能测量（DWT记录ISR耗时）
- 异步DPT编码器（DMA + ISR触发）

### 阶段5：USART串口指令移植（已完成 ✅）

**目标**：移植参考工程`dbg_cmd_set()`，实现串口命令交互

**移植内容：**

**1. USART1 DMA + IDLE中断接收（wk_usart.c）：**
```c
void USART1_DebugRx_Start(void);     // 启动DMA1_CH4接收
void USART1_IDLE_Handler(void);       // IDLE中断里调用，拷贝数据到dbgRecvBuf
```
- 缓冲区：`dbgRecvBuf[1024]` + `usart_rx_len`
- 接收机制：DMA单次接收 + IDLE中断触发 → 拷贝buffer → 重启DMA
- 集成到 at32f45x_int.c 的 USART1_IRQHandler

**2. dbg_cmd_set() 命令解析（foc_bsp.c）：**

精简版（~150行，对比参考工程的949行）。保留核心FOC控制命令，CAN/OTA命令暂未移植。

**已实现命令：**

| 命令 | 功能 |
|------|------|
| `version` | 打印固件版本 + 编译时间 |
| `reset` | 系统复位 |
| `CurrentPID Kp100Ki50Kd0` | 设置/查询电流环PID |
| `SpeedPID Kp100Ki50Kd0` | 设置/查询速度环PID |
| `PositionPID Kp100Ki50Kd0` | 设置/查询位置环PID |
| `Run cmd1 M2 tar0.5` | 启动FOC（cmd=0停机, M2=扭矩/M3=速度/M4=位置） |
| `enable1` / `enable0` | PWM输出使能/禁止 |
| `openloop1` / `openloop0` | 开环测试启/停（含模式选择） |
| `vd<value>` / `vq<value>` | 设置开环DQ轴电压（Q10格式） |
| `getparams` | 打印所有电机参数（NPP/offset/PID/MaxCur/MaxSpd） |

**未移植命令（依赖CAN/OTA）：**
- `bwtest` - 带宽测试（需要foc_test函数链路）
- `injectV` - 电压注入测试（需要完整辨识链路）
- `Cali` - 电角度校准（需要Flash写入）
- `canstat` / `cantest` / `canrxdbg` - CAN状态/测试
- `testfreq` / `testampl` / `teststart` / `teststop` - 单频注入测试
- `otabegin` / `otaend` / `otaabort` / `otaswap` - OTA升级
- `offsetpos` / `offsetneg` / `comppos` / `compneg` - 相位补偿
- `mit` - MIT阻抗模式

**关键改造：**
- HAL_Delay → wk_delay_ms（已有宏映射）
- TIM1->CCR/CCER/BDTR → TMR1->c1dt/cctrl/brk
- can_wly_Nm_to_iA(tq_nm) → 简化为 `tq_nm * 10.0f` 占位
- motorProValue.Udc → g_vdc_raw

**3. main.c 集成：**
```c
// 初始化阶段
USART1_DebugRx_Start();

// 主循环
while(1) {
    isr_print_poll();
    dbg_cmd_set();    // 串口命令解析
    // ...
}
```

**编译规模：** Code=48642 RO-data=3894 RW-data=196 ZI-data=13396
（增加约28KB是printf/atof浮点库链接进来）

**使用示例（串口@921600）：**
```
> version↵
AT32F456 FOC v1.0 built Jun  7 2026 14:30:45

> getparams↵
NPP=8 elec_offset=0
Ia_off=970 Ib_off=12
CurPID: 100 50 0
SpdPID: 200 100 0
PosPID: 50 0 0
MaxCur=10240 MaxSpd=102400

> openloop1↵
openloop ON, mode=0

> vq256↵
v_q_test=256

> enable0↵
PWM disabled
```

### 阶段6：dbg_log_print + Flash + CAN协议层（已完成 ✅）

**目标**：完善调试日志、参数保存、CAN通信

#### 6A. dbg_log_print 周期日志（foc_bsp.c）

由`logid<N>`命令选择打印模式，由`logfreq<ms>`设置周期。

**已实现log id：**

| logid | 内容 |
|-------|------|
| 1 | 测试（清velocity_ref） |
| 10 | 电角度/机械角/位置/速度 |
| 11 | 编码器raw + 位置 |
| 30 | DQ轴电压 |
| 40 | 电流PI（I_dq, V_dq, I_ref） |
| 50 | 速度环（ref/filt/act/err rpm） |
| 90 | 三相电流raw |
| 100 | 位置环（ref/act/err deg） |
| 110 | ISR分段耗时（us @192MHz） |
| 120 | 开环测试状态（1Hz） |
| 150 | ADC采样率 + raw电流 + offset |
| 151 | VDC + 温度监控 |
| 165 | 故障标志查询 |

**调度方式（main loop）：**
```c
static uint32_t log_tick = 0;
if (systick_ms - log_tick >= logPriodMs) {
    log_tick = systick_ms;
    dbg_log_print();
}
```

#### 6B. Flash参数保存（flash_port.c/h）

**AT32 Flash规格（经iflytek同平台参考验证）：**
- Sector size: **2KB**（`FLASH_PAGE_SIZE_OTA = 0x800`）
- 用户参数区: `0x0807F800 ~ 0x08080000`（最后2KB）
- 写入粒度: 4字节（`flash_word_program`）
- 关键：写入时需要 `__set_PRIMASK(1)` 屏蔽中断

**API：**
```c
HAL_StatusTypeDef Flash_EraseSector(void);    // 擦除参数区（2KB）
HAL_StatusTypeDef Flash_WriteData(addr, data, len);  // 写入（4B对齐）
void Flash_ReadData(addr, buf, len);          // 读取（memcpy）
uint32_t Flash_Crc32(data, len);             // CRC32校验
```

#### 6C. CAN协议层移植

**wk_can.c 业务层 - AT32 CAN-FD 基础发送/接收：**
```c
HAL_StatusTypeDef fdcan_send(uint32_t std_id, const uint8_t *data, uint32_t len);
void fdcan_rx_user(uint32_t id, const uint8_t *data, uint32_t len);  // weak回调
void wk_can1_rx_dispatch(void);  // CAN1_RX_IRQ内调用
```

- TX: `can_txbuf_write(CAN1, CAN_TXBUF_STB, &tx)` + `can_txbuf_transmit(STB_ONE)`
- RX: `can_rxbuf_read` 循环读取 → `fdcan_rx_user(id, data, len)`
- CAN-FD + BRS (8Mbps data phase)
- DLC编码与STM32 FDCAN完全一致（0x00~0x0F）

**can_wly.c 万里扬协议（963行）：**
- 帧ID: 0x100(状态) / 0x200(命令) / 0x300(SDO) / 0x600(扩展)
- `fdcan_rx_user()` 覆盖弱符号，接管所有RX帧
- 支持: 力矩/速度/位置控制、参数读写、状态上报
- `can_wly_tick_1ms()` 周期状态发送
- `can_wly_Nm_to_iA()` 扭矩→电流转换（含Kt LUT）

**can_debug.c 调试通道（687行）：**
- 帧ID: 0x7E0~0x7EF（32B CAN-FD payload）
- `can_debug_send_log()` 双通道日志（CAN+UART）
- `can_debug_poll()` 主循环处理
- SDO读写、电角度校准、故障管理
- OTA命令 → stub（待后续bootloader移植）

**CAN IRQ集成（at32f45x_int.c）：**
```c
void CAN1_RX_IRQHandler(void) {
    if (can_interrupt_flag_get(CAN1, CAN_RIF_FLAG)) {
        can_flag_clear(CAN1, CAN_RIF_FLAG);
        wk_can1_rx_dispatch();
    }
}
```

**待main.c集成（下次有硬件时）：**
```c
can_wly_init();         // 初始化阶段
can_debug_init();
// 主循环:
can_wly_tick_1ms();     // 1ms周期调用
can_debug_poll();       // 处理OTA数据帧
```

**编译规模：** Code=60322 RO-data=5102 RW-data=252 ZI-data=14044
（CAN协议层+调试日志，总Flash≈63.7KB，RAM≈14.3KB）
