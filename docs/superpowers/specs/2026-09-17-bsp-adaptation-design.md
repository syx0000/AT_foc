# AT32 PMSM FOC Hall Demo BSP 适配设计

日期：2026-09-17
分支：foc_hall_demo
目标：把 `motor_evb_2v0/at32f45x/pmsm_foc_hall_sensor` demo 的 BSP 迁移到目标板（AT32F456CEU7 LQFP48，DRV8353 三相，60BLDC140-44030-14J 电机），保留 demo 原有的 `mc_hwio` 架构，只替换硬件事实。

参考目标板：`E:\src\AT_foc_hall`（已在硬件上验证过的自研 BSP，供事实提取）。

---

## 1. 硬件事实对照表

### 1.1 芯片与时钟

| 项 | demo | 目标板 |
|---|---|---|
| MCU | AT32F455RET7 (LQFP64) | AT32F456CEU7 (LQFP48) |
| 晶振 HEXT | 8 MHz | 8 MHz |
| 系统主频 | 192 MHz | 192 MHz |
| PLL | NS=96, MS=1, FP=4 | NS=144, MS=1, FP=6（等价） |
| APB1/APB2 | /1 | /1 |
| APB3 | 未设置（默认） | /4 = 48 MHz |
| Flash wait cycle | 5 | 5 |

**决定**：保留 demo 的 PLL 参数（96/1/FP4）。新增一句 `crm_apb3_div_set(CRM_APB3_DIV_4)` 避免复位默认值。`HEXT_VALUE=8000000` 不变。

### 1.2 PWM (TMR1)

| 项 | demo | 目标板事实 | 采用值 |
|---|---|---|---|
| 定时器 | TMR1 | TMR1 | TMR1 |
| 载波 | 16 kHz | 10 kHz | **16 kHz** (抬 demo) |
| 计数模式 | 中心对齐 3 | 中心对齐 3 | 中心对齐 3 |
| clock_source_div | DIV4 (Tdts=20.83 ns，只影响死区/滤波采样时钟，不影响计数器) | DIV1 | **DIV4** (抬 demo) |
| ARR | 5999 | 9599 | **5999** (192MHz / 6000 / 2 = 16 kHz) |
| 重复计数 | 1 | 1 | 1 |
| 死区 | 300 ns (14 counts) | 80 counts@DIV1 (~417 ns) | **300 ns** (抬 demo) |
| CH4 用途 | ADC 触发 | ADC 触发 | 保留 |
| 高边 A/B/C | PA8/PA9/PA10 MUX1 | PA8/PA9/PA10 MUX1 | PA8/PA9/PA10 MUX1 |
| 低边 A/B/C | PB13/PB14/PB15 MUX1 | **PA7/PB0/PB1 MUX1** | **PA7/PB0/PB1 MUX1** |
| BRK | PB12 MUX1 低有效 | PB12 MUX1 低有效 | PB12 MUX1 低有效 |
| EN_GATE | 无 | **PC15** | **PC15** (新增) |

### 1.3 Hall (TMR4)

| 项 | demo | 目标板事实 | 采用值 |
|---|---|---|---|
| 定时器 | TMR3 | (EXINT9_5) | **TMR4** |
| Hall A | PB4 MUX2 | PB5 (EXINT) | **PB6 MUX2** (改硬件) |
| Hall B | PB5 MUX2 | PB6 (EXINT) | **PB7 MUX2** (改硬件) |
| Hall C | PB0 MUX2 | PB7 (EXINT) | **PB8 MUX2** (改硬件，待上板核对) |
| 输入滤波 | 0x6 | — | 0x6 |

**风险点**：AT32F456 上 PB8 的 MUX2 是否为 TMR4_CH3 未从 datasheet 100% 确认。按 STM32F4 家族惯例强推断，上板验证前不改硬件。

### 1.4 电流/电压/温度 ADC

| 项 | demo | 目标板事实 | 采用值 |
|---|---|---|---|
| ADC 拓扑 | ADC1+ADC2 双转换器同步注入 3 相 | ADC1+ADC2 分离，A/B 注入 C 走 DMA | **抬 demo 三相同步注入** |
| Phase A | ADC1_CH0 / PA0 | PA0 | ADC1_CH0 / PA0 |
| Phase B | ADC1_CH1 / PA1 | PA1 | ADC1_CH1 / PA1 |
| Phase C | ADC1_CH2 / PA2 | PA2 | ADC1_CH2 / PA2 |
| VBUS | ADC1_CH4 / PA4 | **PA3** (SO3) | **ADC1_CH3 / PA3** |
| MOS 温度 | ADC1_CH9 / PB1 | **PA5** (TEMP_MOS) | **ADC1_CH5 / PA5** |
| 电位器 | ADC1_CH10 / PC0 | 无 | **删除** |
| 母线电流 | ADC1_CH3 / PA3 | 无（PA3 已给 VBUS） | **删除** |
| IBUS 平均 | ADC1_CH13 / PC3 | 无 | **删除** |
| BEMF A/B/C | ADC1_CH5/6/7 | 无 | **删除**（保留宏，编译宏关闭） |
| 电机温度 | 无 | PA4 (TEMP_MOTOR) | **不启用** |
| ADC 分频 | HCLK/4 | — | HCLK/4 |
| 注入采样时间 | 6.5 cyc | 6.5 cyc | 6.5 cyc |
| 注入触发源 | TMR1_CH4 | TMR1_TRGOUT | TMR1_CH4 (抬 demo) |

### 1.5 UART1

| 项 | demo | 目标板事实 | 采用值 |
|---|---|---|---|
| 外设 | USART1 | USART1 | USART1 |
| TX | PB6 MUX7 | **PA15 MUX7** | **PA15 MUX7** |
| RX | PB7 MUX7 | **PB3 MUX7** | **PB3 MUX7** |
| 波特率 | 1.5 Mbps | 921600 | **1.5 Mbps** (抬 demo) |
| DMA TX | DMA1_CH2 | (未用) | DMA1_CH2 |
| DMA RX | DMA1_CH3 | DMA1_CH3 | DMA1_CH3 |

### 1.6 LED

| demo | 目标板 | 映射 |
|---|---|---|
| `ERROR_LED_PIN=PC13` | LED_ERR=PC14 | 改 PC13→PC14 |
| `STATUS1_LED_PIN=PC14` | LED_RUN=PC13 | 改 PC14→PC13 |
| `STATUS2_LED_PIN=PC15` | 无 | 删除 |
| `STATUS3_LED_PIN=PB9` | 无 | 删除 |
| `ADC_TRIG_LED_PIN=PA11` | 无 | 删除 |

### 1.7 删除项

- 按键：USER_BUTTON、HALL_LEARN_BUTTON、MODE1_BUTTON、MODE2_BUTTON、REVERSE_SW、BRAKE_SW、LOCK_MOTOR_SW、PARKING_LOCK_SW；`button_exint_init()` 清空
- 编码器：ABZ、增量、磁编码器相关宏与初始化
- 刹车电阻 PWM (TMR10 PB8)
- 一 shunt / 两 shunt 相关分支

### 1.8 保留但不启用的编译宏（`motor_control_drive_param.h`）

删除 `#define GATE_DRIVER_LOW_SIDE_INVERT`（DRV8353 非反相）；不定义 `FIELD_WEAKENING`、`E_BIKE_SCOOTER`、`BRAKING_RESISTOR`、`CURRENT_DECOUPLE_CTRL`、`TWO_ADC_CONVERTERS`、`LOW_SPEED_VOLT_CTRL`、`VOLT_SENSE`、`WIND_SENSE`、`MOS_RDS_SHUNT`、`ABZ`、`MAGNET_ENCODER_W_ABZ`、`MAGNETIC_ENCODER`。保留 `AT_MOTOR_EVB_V2`、`FOC_CONTROL`、`THREE_SHUNT`、`HALL_SENSORS`、`SPMSM`、`MOTOR_PARAM_IDENTIFY`、`USE_MOTOR_MONITOR`。

---

## 2. 电气与电机参数（motor_control_drive_param.h）

来源：目标板 `project/config/motor_control_config.h` + 电机手册 `60BLDC140-44030-14J.pdf`

```c
/* 电机（60BLDC140-44030-14J） */
#define POLE_PAIRS         (8/2)     /* 8 极 */
#define RS_LL              (0.536f)  /* line-to-line = 2 × 0.268 Ω */
#define LS_LL              (0.000428f) /* line-to-line = 2 × (222+206)/2 μH */
#define LD_LQ_RATIO        (1.078f)  /* 222/206 */
#define KE                 (0.00849f) /* V/rpm，与手册 8.4 V/kRPM 一致 */
#define NOMINAL_CURRENT    (11.5f)   /* A，手册额定电流 */

/* 硬件采样电路 */
#define VDC_RATED          (48.0f)
#define V_SENSE_GAIN       (1.0f/21.0f)      /* 分压比 1/21 */
#define ADC_REFERENCE_VOLT (3.3f)
#define R_SHUNT            (0.0025f)         /* 2.5 mΩ */
#define OP_GAIN            (10.0f)           /* DRV8353 内建 CSA */
#define CURR_OFFSET_VOLT   (1.65f)           /* VREF/2 */

/* 保护 */
#define OVER_VOLT_THRESHOLD  (60)
#define UNDER_VOLT_THRESHOLD (24)
#define OVER_CURRENT_SW      (55.0f)
#define MAX_CURRENT          (50.0f)
#define MIN_CURRENT          (-MAX_CURRENT)
#define DC_MAX_CURRENT       (60.0f)

/* 控制 */
#define PWM_FREQ         (16000)
#define UI_UART_BAUDRATE (1500000UL)
#define MAX_SPEED_RPM    (4000)
#define MIN_SPEED_RPM    (200)
#define STABLE_SPEED_RPM (50)
```

**Hall 学习表**：保留 demo 默认值（1,5,4,6,2,3），上板后用 mclib 自带的 hall learn 流程重新标定，通过 motor monitor 上位机触发。

---

## 3. 文件改动清单

| 文件 | 动作 |
|---|---|
| `inc/mc_hwio_v2.h` | 改引脚宏、Hall 定时器宏、UART 引脚宏、ADC 通道宏、删除按键/编码器/多余 LED 宏、新增 EN_GATE 宏 |
| `inc/motor_control_drive_param.h` | 改电气参数、删除不用的编译宏、保留 Hall 学习表默认值 |
| `src/mc_hwio.c` | 改 GPIO 初始化引脚、NVIC 精简、EN_GATE 初始化为低、`led_config()` 精简、`button_exint_init()` 清空、`brake_pwm_init()` 删除 |
| `src/at32f45x_clock.c` | 新增 `crm_apb3_div_set(CRM_APB3_DIV_4)` |
| `src/at32f45x_int.c` | 删除按键/编码器/SYNC_TIMER 中断处理函数 |
| `src/main.c` | 在 `motor_control_foc_init()` 之后、`while(1)` 之前拉高 EN_GATE 并延时 3 ms |
| `mdk_v5/pmsm_foc_hall_sensor.uvprojx` | device 型号从 AT32F455RET7 改为 AT32F456CEU7；Flash/RAM 大小按 datasheet 调整（实施阶段先读工程文件确认现状再改） |
| `at32_ide - proj/ldscripts/*.ld` | 若容量变化需要改链接脚本；实施阶段视 MDK 情况决定是否同步改 IDE 工程 |

**不改**：`mclib/`、`libraries/AT32F45x_Firmware_Library/`、`src/mc_isr.c`（宏透过跟随）、`src/main.c` 主循环逻辑（除 EN_GATE 一句）

---

## 4. 上板验证顺序（供后续实施计划参考，不是本 spec 的一部分）

1. 编译通过（0 error, 0 warning）
2. 时钟：观察 MCO 或用调试观测 `system_core_clock` = 192000000
3. LED 心跳：`STATUS1_LED` (PC13) blink
4. UART：motor monitor 连通
5. PWM：低压母线（12 V）观察三相互补波形，死区正常
6. ADC 零偏：VBUS 读值、shunt 零偏
7. Hall：手动转轴，观察 Hall 组合与 mclib 内部状态
8. Hall learn：跑一次，记录学习表
9. 电机启动：先开环，再闭环

---

## 5. 已知风险

1. **PB8 = TMR4_CH3 未从 datasheet 确认**：上板后如果 Hall 无捕获，回退到 EXINT + 时间戳方案。
2. **Hall 学习表方向未知**：demo 默认值大概率与目标板不同，必须重新学习。
3. **PWM 低边极性**：DRV8353 低边输入是非反相，demo `GATE_DRIVER_LOW_SIDE_INVERT` 已删除，需在上板前用示波器确认。
4. **MDK 工程 device 目标**：若 pack 里没有 AT32F456CEU7 变体，需要先安装最新 Artery pack 再编译。
5. **Flash 容量差异**：AT32F455RET7 是 512 KB / AT32F456CEU7 是 256 KB。当前 demo 编出来的 map 显示 32 KB 左右，容量足够，但链接脚本要正确。

---

## 6. 不做的事（明确边界）

- 不移植 CAN 通信（目标板 BSP 有 CAN1，本次不引入 demo）
- 不移植 RS485 (USART3)
- 不移植 bootloader / OTA
- 不引入目标板的 `motor_*_port` / `wk_*` 抽象层
- 不改 mclib 内部代码
- 不做参数辨识运行（等上板后由 mclib `MOTOR_PARAM_IDENTIFY` 流程自动跑）
