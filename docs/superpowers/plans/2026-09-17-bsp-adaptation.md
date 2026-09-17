# AT32 PMSM FOC Hall Demo BSP 适配 - 实施计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 把 `motor_evb_2v0/at32f45x/pmsm_foc_hall_sensor` demo 的 BSP 适配到目标板（AT32F456CEU7 LQFP48，DRV8353，60BLDC140-44030-14J），编译通过、结构就绪，等硬件到位后上板验证。

**Architecture:** 保留 demo 原有的 `mc_hwio_v2.h + mc_hwio.c + mc_isr.c + main.c` 架构。只替换硬件事实：引脚宏、Hall 定时器、UART 引脚、ADC 通道、电气参数；删除目标板不存在的外设（编码器、按键、电位器、BEMF、刹车电阻）；新增 EN_GATE 使能引脚。**mclib、AT32 固件库、mc_isr.c 里的中断处理逻辑完全不动**。

**Tech Stack:** AT32F45x Firmware Library V2.0.0, Keil MDK v5, C99

## Global Constraints

- 主频 192 MHz，HEXT 8 MHz，PLL 96/1/FP4（保留 demo 值）
- PWM 载波 16 kHz，中心对齐 3，clock_div=DIV4，死区 300 ns（保留 demo 值）
- UART1 波特率 1.5 Mbps（保留 demo 值）
- 全部 GPIO_MUX 值参考 STM32F4 家族惯例；PB8=TMR4_CH3 为强推断，未从 AT32F456 datasheet 100% 确认
- 事实来源：spec `docs/superpowers/specs/2026-09-17-bsp-adaptation-design.md`（commit bea1098）
- 不改：`mclib/`、`libraries/`、`main.c` 主循环逻辑（除 EN_GATE 一句）、`mc_isr.c` 除 BUTTON_EXINT_IRQ 外的所有函数
- 提交粒度：每个 Task 一个 commit，commit message 用中文短描述
- 每个 Task 最后必须运行 `git status` 检查未预期改动

## Verification Model

BSP 移植无法通过单元测试证明正确性（无硬件模拟环境）。本计划的每个 Task 使用以下三层验证：

1. **grep 交叉检查**：修改宏后 grep 该宏所有引用点，确认没有孤立引用
2. **MDK 编译**：Keil `armcc` 全量编译，0 error 0 warning
3. **上板波形验证**：**不在本计划范围**，见 spec 第 4 节

因为主开发环境是 Windows + MDK GUI，命令行编译需要 MDK 的 `UV4.exe -b` 或 armclang 独立工具链。若这些工具在环境中不可用，Task 的"编译验证"步改为**由用户在 Keil GUI 中手动 Build 一次，把编译输出粘贴给 review 者**。

---

## File Structure

**修改**（5 个文件）：
- `motor_evb_2v0/at32f45x/pmsm_foc_hall_sensor/inc/motor_control_drive_param.h` - 电气参数、编译宏
- `motor_evb_2v0/at32f45x/pmsm_foc_hall_sensor/inc/mc_hwio_v2.h` - 引脚宏、Hall/UART/ADC 定义、新增 EN_GATE
- `motor_evb_2v0/at32f45x/pmsm_foc_hall_sensor/src/mc_hwio.c` - GPIO 初始化、NVIC、精简 LED/按键/编码器代码
- `motor_evb_2v0/at32f45x/pmsm_foc_hall_sensor/src/mc_isr.c` - 清空 BUTTON_EXINT_IRQ 函数体
- `motor_evb_2v0/at32f45x/pmsm_foc_hall_sensor/src/main.c` - 删除 button_exint_init 调用，插入 EN_GATE 使能序列

**不改**（明确列出）：
- `motor_evb_2v0/at32f45x/pmsm_foc_hall_sensor/src/at32f45x_clock.c`：APB3 分频在 `:114` 已存在
- `motor_evb_2v0/at32f45x/pmsm_foc_hall_sensor/src/at32f45x_int.c`：只有内核异常，无外设 IRQ
- `mclib/`、`libraries/`

**可能需要改**（Task 6 决定）：
- `motor_evb_2v0/at32f45x/pmsm_foc_hall_sensor/mdk_v5/pmsm_foc_hall_sensor.uvprojx` - device 型号
- `motor_evb_2v0/at32f45x/pmsm_foc_hall_sensor/at32_ide - proj/ldscripts/AT32F455xE_FLASH.ld` - 若容量变化

---

## Task 1: 电气参数与编译宏

**Files:**
- Modify: `motor_evb_2v0/at32f45x/pmsm_foc_hall_sensor/inc/motor_control_drive_param.h`

**Interfaces:**
- Consumes: 无
- Produces: `POLE_PAIRS`、`RS_LL`、`LS_LL`、`KE`、`NOMINAL_CURRENT`、`VDC_RATED`、`V_SENSE_GAIN`、`R_SHUNT`、`OP_GAIN`、`CURR_OFFSET_VOLT`、`OVER_VOLT_THRESHOLD`、`UNDER_VOLT_THRESHOLD`、`OVER_CURRENT_SW`、`MAX_CURRENT`、`DC_MAX_CURRENT` 供 mclib 和 mc_hwio.c 使用

**Rationale:** 电气参数完全由硬件决定，与引脚宏解耦，先改这里以便 Task 2/3 需要电气参数时可用。

- [ ] **Step 1: 定位现有值并确认无其他文件依赖**

Run:
```bash
cd /e/src/AT_foc_hall_demo/motor_evb_2v0/at32f45x/pmsm_foc_hall_sensor
grep -n "VDC_RATED\|R_SHUNT\|OP_GAIN\|CURR_OFFSET_VOLT\|OVER_VOLT\|UNDER_VOLT\|OVER_CURRENT_SW\|MAX_CURRENT\|POLE_PAIRS\|RS_LL\|LS_LL\|KE\b\|NOMINAL_CURRENT\|V_SENSE_GAIN\|GATE_DRIVER_LOW_SIDE_INVERT" inc/motor_control_drive_param.h
```
Expected: 上述宏都在此文件中定义，行号见 spec 第 2 节。

- [ ] **Step 2: 修改电机参数（第 99-104 行段）**

用 Edit 工具修改：

`old_string`:
```c
#define POLE_PAIRS                      (8/2)
#define RS_LL                           (1.89f)      /* Stator resistance(line-to-line), ohm */
#define LS_LL                           (0.002387f)  /* Stator inductance(line-to-line), H */
#define LD_LQ_RATIO                     (1.0f)       /* Ratio of Ld to Lq (Ld/Lq) */
#define KE                              (0.003437f)  /* Back EMF constant(line-to-line, peak voltage), V/rpm */
#define NOMINAL_CURRENT                 (1.7f)       /* Nominal current of motor, Ampere */
```

`new_string`:
```c
#define POLE_PAIRS                      (8/2)        /* 60BLDC140: 8 poles */
#define RS_LL                           (0.536f)     /* line-to-line, 2 × 0.268 Ω */
#define LS_LL                           (0.000428f)  /* line-to-line, 2 × 214 μH */
#define LD_LQ_RATIO                     (1.078f)     /* Ld/Lq = 222/206 */
#define KE                              (0.00849f)   /* V/rpm，手册 8.4 V/kRPM */
#define NOMINAL_CURRENT                 (11.5f)      /* A，手册额定电流 */
```

- [ ] **Step 3: 修改 VDC_RATED、V_SENSE_GAIN（第 122-124 行段）**

`old_string`:
```c
#define VDC_RATED                       (24.0f)
#define BAT_LOW_VOLT                    (12.0f)              /*!< minimum allowable battery voltage For E_BIKE_SCOOTER use only*/
#define V_SENSE_GAIN                    (10/(3.9f+180+10))  // 0.05157
```

`new_string`:
```c
#define VDC_RATED                       (48.0f)
#define BAT_LOW_VOLT                    (12.0f)              /*!< minimum allowable battery voltage For E_BIKE_SCOOTER use only*/
#define V_SENSE_GAIN                    (1.0f/21.0f)         /* 分压比 1/21 */
```

- [ ] **Step 4: 修改 MAX_CURRENT / DC_MAX_CURRENT（第 168-170 行段）**

`old_string`:
```c
#define MAX_CURRENT                     (5.0f)
#define MIN_CURRENT                     (-MAX_CURRENT)
#define DC_MAX_CURRENT                  (10.0f)
```

`new_string`:
```c
#define MAX_CURRENT                     (50.0f)
#define MIN_CURRENT                     (-MAX_CURRENT)
#define DC_MAX_CURRENT                  (60.0f)
```

- [ ] **Step 5: 修改三 shunt 分支 R_SHUNT / OP_GAIN / CURR_OFFSET_VOLT（第 177-179 行段）**

`old_string`:
```c
#define R_SHUNT                         (0.002f)
#define OP_GAIN                         (33.0f/(1+33)*(1+16/1))                         // 16.5
#define CURR_OFFSET_VOLT                (ADC_REFERENCE_VOLT*(1.0f/(1+33)*(1+16/1)))     // 1.65 V
```

`new_string`:
```c
#define R_SHUNT                         (0.0025f)     /* 2.5 mΩ */
#define OP_GAIN                         (10.0f)       /* DRV8353 内建 CSA */
#define CURR_OFFSET_VOLT                (1.65f)       /* VREF/2 */
```

- [ ] **Step 6: 修改保护阈值（第 195、205、206 行）**

`old_string`:
```c
#define OVER_CURRENT_SW                 (43.0)  /* A */
```
`new_string`:
```c
#define OVER_CURRENT_SW                 (55.0f)  /* A */
```

`old_string`:
```c
#define OVER_VOLT_THRESHOLD             (55)
#define UNDER_VOLT_THRESHOLD            (10)
```
`new_string`:
```c
#define OVER_VOLT_THRESHOLD             (60)
#define UNDER_VOLT_THRESHOLD            (24)
```

- [ ] **Step 7: 删除 GATE_DRIVER_LOW_SIDE_INVERT（第 42-43 行）**

`old_string`:
```c
/* gate driver low side inverting logic input or non-inverting logic input*/
#define GATE_DRIVER_LOW_SIDE_INVERT
```
`new_string`:
```c
/* gate driver low side inverting logic input or non-inverting logic input*/
/* DRV8353 uses non-inverting inputs, so GATE_DRIVER_LOW_SIDE_INVERT is NOT defined */
```

- [ ] **Step 8: 修改 MAX_SPEED_RPM（第 235 行）**

Run:
```bash
grep -n "MAX_SPEED_RPM\s\|MIN_SPEED_RPM\s" inc/motor_control_drive_param.h
```
Expected: MAX_SPEED_RPM 在 235 行左右，MIN_SPEED_RPM 在 234 行左右。

用 Edit：

`old_string`:
```c
#define MAX_SPEED_RPM                   (6000)
```
`new_string`:
```c
#define MAX_SPEED_RPM                   (4000)
```

MIN_SPEED_RPM=200 保留不改。

- [ ] **Step 9: 验证 - grep 反向检查已改宏的所有引用**

Run:
```bash
cd /e/src/AT_foc_hall_demo
grep -rn "GATE_DRIVER_LOW_SIDE_INVERT" motor_evb_2v0/ mclib/
```
Expected: 只在 `motor_control_drive_param.h` 里出现的注释行，其余全部无引用（因为已删除 `#define`）。若 mclib 里有 `#ifdef GATE_DRIVER_LOW_SIDE_INVERT`，那么该分支现在会走 non-invert 路径，符合设计意图。

Run:
```bash
grep -n "OVER_CURRENT_SW\|MAX_CURRENT\|VDC_RATED\|POLE_PAIRS" mclib/src/*.c mclib/inc/*.h | head -20
```
Expected: 引用点存在，无孤立宏。

- [ ] **Step 10: Commit**

```bash
cd /e/src/AT_foc_hall_demo
git add motor_evb_2v0/at32f45x/pmsm_foc_hall_sensor/inc/motor_control_drive_param.h
git commit -m "$(cat <<'EOF'
adapt drive param to 60BLDC140 motor and 48V DRV8353 board

VDC=48V, R_SHUNT=2.5mΩ, OP_GAIN=10, KE=0.00849, RS=0.536Ω,
LS=0.428mH. Remove GATE_DRIVER_LOW_SIDE_INVERT (DRV8353 non-inv).

Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>
EOF
)"
git status
```
Expected: working tree clean.

---

## Task 2: 引脚宏（mc_hwio_v2.h）

**Files:**
- Modify: `motor_evb_2v0/at32f45x/pmsm_foc_hall_sensor/inc/mc_hwio_v2.h`

**Interfaces:**
- Consumes: 无
- Produces: 所有 GPIO/外设宏供 `mc_hwio.c`、`mc_isr.c`、`main.c` 使用；新增 `EN_GATE_*` 三个宏

**Rationale:** 头文件宏是"事实层"，改完后 mc_hwio.c/mc_isr.c 函数体不用动，宏透明地映射到新引脚。

- [ ] **Step 1: 修改 PWM 低边引脚（第 92-106 行段）**

`old_string`:
```c
#define PWM_PHASE_A_LOW_GPIO_CRM_CLK        CRM_GPIOB_PERIPH_CLOCK
#define PWM_PHASE_A_LOW_PORT                GPIOB
#define PWM_PHASE_A_LOW_GPIO_PIN            GPIO_PINS_13
#define PWM_PHASE_A_LOW_PIN_SOURCE          GPIO_PINS_SOURCE13
#define PWM_PHASE_A_LOW_IOMUX               GPIO_MUX_1
#define PWM_PHASE_B_LOW_GPIO_CRM_CLK        CRM_GPIOB_PERIPH_CLOCK
#define PWM_PHASE_B_LOW_PORT                GPIOB
#define PWM_PHASE_B_LOW_GPIO_PIN            GPIO_PINS_14
#define PWM_PHASE_B_LOW_PIN_SOURCE          GPIO_PINS_SOURCE14
#define PWM_PHASE_B_LOW_IOMUX               GPIO_MUX_1
#define PWM_PHASE_C_LOW_GPIO_CRM_CLK        CRM_GPIOB_PERIPH_CLOCK
#define PWM_PHASE_C_LOW_PORT                GPIOB
#define PWM_PHASE_C_LOW_GPIO_PIN            GPIO_PINS_15
#define PWM_PHASE_C_LOW_PIN_SOURCE          GPIO_PINS_SOURCE15
#define PWM_PHASE_C_LOW_IOMUX               GPIO_MUX_1
```

`new_string`:
```c
#define PWM_PHASE_A_LOW_GPIO_CRM_CLK        CRM_GPIOA_PERIPH_CLOCK
#define PWM_PHASE_A_LOW_PORT                GPIOA
#define PWM_PHASE_A_LOW_GPIO_PIN            GPIO_PINS_7
#define PWM_PHASE_A_LOW_PIN_SOURCE          GPIO_PINS_SOURCE7
#define PWM_PHASE_A_LOW_IOMUX               GPIO_MUX_1
#define PWM_PHASE_B_LOW_GPIO_CRM_CLK        CRM_GPIOB_PERIPH_CLOCK
#define PWM_PHASE_B_LOW_PORT                GPIOB
#define PWM_PHASE_B_LOW_GPIO_PIN            GPIO_PINS_0
#define PWM_PHASE_B_LOW_PIN_SOURCE          GPIO_PINS_SOURCE0
#define PWM_PHASE_B_LOW_IOMUX               GPIO_MUX_1
#define PWM_PHASE_C_LOW_GPIO_CRM_CLK        CRM_GPIOB_PERIPH_CLOCK
#define PWM_PHASE_C_LOW_PORT                GPIOB
#define PWM_PHASE_C_LOW_GPIO_PIN            GPIO_PINS_1
#define PWM_PHASE_C_LOW_PIN_SOURCE          GPIO_PINS_SOURCE1
#define PWM_PHASE_C_LOW_IOMUX               GPIO_MUX_1
```

- [ ] **Step 2: 修改 Hall 定时器与引脚（第 114-138 行段）**

`old_string`:
```c
/**************** define Timer for Hall ******************/
/* hall sensor pin definition */
#define HALL_CAPTURE_TIMER                  TMR3
#define HALL_CAPTURE_CRM_CLK                CRM_TMR3_PERIPH_CLOCK
#define HALL_CAPTURE_IRQ                    TMR3_GLOBAL_IRQHandler
#define HALL_CAPTURE_IRQn                   TMR3_GLOBAL_IRQn
#define HALL_CAPTURE_FILTER_CLK_DIV         TMR_CLOCK_DIV2
#define TMR_HALL_IN_FILTER                  0x6                        /* 0x0 ~ 0xF */

/**************** define GPIO for Hall *******************/
#define HALL_A_GPIO_CRM_CLK                 CRM_GPIOB_PERIPH_CLOCK
#define HALL_A_PORT                         GPIOB
#define HALL_A_GPIO_PIN                     GPIO_PINS_4
#define HALL_A_GPIO_PIN_SOURCE              GPIO_PINS_SOURCE4
#define HALL_A_IOMUX                        GPIO_MUX_2
#define HALL_B_GPIO_CRM_CLK                 CRM_GPIOB_PERIPH_CLOCK
#define HALL_B_PORT                         GPIOB
#define HALL_B_GPIO_PIN                     GPIO_PINS_5
#define HALL_B_GPIO_PIN_SOURCE              GPIO_PINS_SOURCE5
#define HALL_B_IOMUX                        GPIO_MUX_2
#define HALL_C_GPIO_CRM_CLK                 CRM_GPIOB_PERIPH_CLOCK
#define HALL_C_PORT                         GPIOB
#define HALL_C_GPIO_PIN                     GPIO_PINS_0
#define HALL_C_GPIO_PIN_SOURCE              GPIO_PINS_SOURCE0
#define HALL_C_IOMUX                        GPIO_MUX_2
```

`new_string`:
```c
/**************** define Timer for Hall ******************/
/* hall sensor pin definition - target board: PB6/PB7/PB8 on TMR4 */
#define HALL_CAPTURE_TIMER                  TMR4
#define HALL_CAPTURE_CRM_CLK                CRM_TMR4_PERIPH_CLOCK
#define HALL_CAPTURE_IRQ                    TMR4_GLOBAL_IRQHandler
#define HALL_CAPTURE_IRQn                   TMR4_GLOBAL_IRQn
#define HALL_CAPTURE_FILTER_CLK_DIV         TMR_CLOCK_DIV2
#define TMR_HALL_IN_FILTER                  0x6                        /* 0x0 ~ 0xF */

/**************** define GPIO for Hall *******************/
#define HALL_A_GPIO_CRM_CLK                 CRM_GPIOB_PERIPH_CLOCK
#define HALL_A_PORT                         GPIOB
#define HALL_A_GPIO_PIN                     GPIO_PINS_6
#define HALL_A_GPIO_PIN_SOURCE              GPIO_PINS_SOURCE6
#define HALL_A_IOMUX                        GPIO_MUX_2
#define HALL_B_GPIO_CRM_CLK                 CRM_GPIOB_PERIPH_CLOCK
#define HALL_B_PORT                         GPIOB
#define HALL_B_GPIO_PIN                     GPIO_PINS_7
#define HALL_B_GPIO_PIN_SOURCE              GPIO_PINS_SOURCE7
#define HALL_B_IOMUX                        GPIO_MUX_2
#define HALL_C_GPIO_CRM_CLK                 CRM_GPIOB_PERIPH_CLOCK
#define HALL_C_PORT                         GPIOB
#define HALL_C_GPIO_PIN                     GPIO_PINS_8
#define HALL_C_GPIO_PIN_SOURCE              GPIO_PINS_SOURCE8
#define HALL_C_IOMUX                        GPIO_MUX_2
```

- [ ] **Step 3: 修改 VBUS ADC 通道（PA4→PA3）**

Run:
```bash
grep -n "VOLT_BUS_ADC" inc/mc_hwio_v2.h
```
Expected: 找到 4 行连续宏定义。

用 Edit：

`old_string`:
```c
#define VOLT_BUS_ADC_CH                     ADC_CHANNEL_4
#define VOLT_BUS_ADC_GPIO_CRM_CLK           CRM_GPIOA_PERIPH_CLOCK
#define VOLT_BUS_ADC_PORT                   GPIOA
#define VOLT_BUS_ADC_GPIO_PIN               GPIO_PINS_4
```

`new_string`:
```c
#define VOLT_BUS_ADC_CH                     ADC_CHANNEL_3
#define VOLT_BUS_ADC_GPIO_CRM_CLK           CRM_GPIOA_PERIPH_CLOCK
#define VOLT_BUS_ADC_PORT                   GPIOA
#define VOLT_BUS_ADC_GPIO_PIN               GPIO_PINS_3
```

- [ ] **Step 4: 修改 MOS 温度 ADC 通道（PB1→PA5）**

Run:
```bash
grep -n "MOS_TEMP_ADC" inc/mc_hwio_v2.h
```

`old_string`:
```c
#define MOS_TEMP_ADC_CH                     ADC_CHANNEL_9
#define MOS_TEMP_ADC_GPIO_CRM_CLK           CRM_GPIOB_PERIPH_CLOCK
#define MOS_TEMP_ADC_PORT                   GPIOB
#define MOS_TEMP_ADC_GPIO_PIN               GPIO_PINS_1
```

`new_string`:
```c
#define MOS_TEMP_ADC_CH                     ADC_CHANNEL_5
#define MOS_TEMP_ADC_GPIO_CRM_CLK           CRM_GPIOA_PERIPH_CLOCK
#define MOS_TEMP_ADC_PORT                   GPIOA
#define MOS_TEMP_ADC_GPIO_PIN               GPIO_PINS_5
```

- [ ] **Step 5: 修改 UART1 引脚（PB6/PB7 → PA15/PB3）**

Run:
```bash
grep -n "COMM_UART_TX\|COMM_UART_RX" inc/mc_hwio_v2.h
```

`old_string`:
```c
#define COMM_UART_TX_GPIO_CRM_CLK           CRM_GPIOB_PERIPH_CLOCK
#define COMM_UART_TX_PORT                   GPIOB
#define COMM_UART_TX_GPIO_PIN_SOURCE        GPIO_PINS_SOURCE6
#define COMM_UART_TX_PIN                    GPIO_PINS_6
#define COMM_UART_TX_IOMUX                  GPIO_MUX_7
#define COMM_UART_RX_GPIO_CRM_CLK           CRM_GPIOB_PERIPH_CLOCK
#define COMM_UART_RX_PORT                   GPIOB
#define COMM_UART_RX_GPIO_PIN_SOURCE        GPIO_PINS_SOURCE7
#define COMM_UART_RX_PIN                    GPIO_PINS_7
#define COMM_UART_RX_IOMUX                  GPIO_MUX_7
```

`new_string`:
```c
#define COMM_UART_TX_GPIO_CRM_CLK           CRM_GPIOA_PERIPH_CLOCK
#define COMM_UART_TX_PORT                   GPIOA
#define COMM_UART_TX_GPIO_PIN_SOURCE        GPIO_PINS_SOURCE15
#define COMM_UART_TX_PIN                    GPIO_PINS_15
#define COMM_UART_TX_IOMUX                  GPIO_MUX_7
#define COMM_UART_RX_GPIO_CRM_CLK           CRM_GPIOB_PERIPH_CLOCK
#define COMM_UART_RX_PORT                   GPIOB
#define COMM_UART_RX_GPIO_PIN_SOURCE        GPIO_PINS_SOURCE3
#define COMM_UART_RX_PIN                    GPIO_PINS_3
#define COMM_UART_RX_IOMUX                  GPIO_MUX_7
```

- [ ] **Step 6: 修改 LED 引脚（PC13 ↔ PC14 交换）**

Run:
```bash
grep -n "ERROR_LED\|STATUS1_LED" inc/mc_hwio_v2.h
```

`old_string`:
```c
/* error led state */
#define  ERROR_LED_GPIO_CRM_CLK              CRM_GPIOC_PERIPH_CLOCK
#define  ERROR_LED_PORT                      GPIOC
#define  ERROR_LED_GPIO_PIN                  GPIO_PINS_13
```

`new_string`:
```c
/* error led state (target board: LED_ERR = PC14) */
#define  ERROR_LED_GPIO_CRM_CLK              CRM_GPIOC_PERIPH_CLOCK
#define  ERROR_LED_PORT                      GPIOC
#define  ERROR_LED_GPIO_PIN                  GPIO_PINS_14
```

`old_string`:
```c
/* operating status LEDs */
#define  STATUS1_LED_GPIO_CRM_CLK            CRM_GPIOC_PERIPH_CLOCK
#define  STATUS1_LED_PORT                    GPIOC
#define  STATUS1_LED_GPIO_PIN                GPIO_PINS_14
```

`new_string`:
```c
/* operating status LEDs (target board: LED_RUN = PC13) */
#define  STATUS1_LED_GPIO_CRM_CLK            CRM_GPIOC_PERIPH_CLOCK
#define  STATUS1_LED_PORT                    GPIOC
#define  STATUS1_LED_GPIO_PIN                GPIO_PINS_13
```

- [ ] **Step 7: 新增 EN_GATE 宏（在 LED 段末尾之后、button 段之前插入）**

Run:
```bash
grep -n "STATUS3_LED_GPIO_PIN\|/\\*\\*\\* define button" inc/mc_hwio_v2.h
```
Expected: 定位 STATUS3_LED_GPIO_PIN（约第 327 行）与 button 段起始位置。

用 Edit（在 STATUS3_LED 定义之后插入 EN_GATE 宏，保持 STATUS2/STATUS3 现存但 Task 3 里会在源文件中不引用它们）：

`old_string`:
```c
#define  STATUS3_LED_GPIO_CRM_CLK            CRM_GPIOB_PERIPH_CLOCK
#define  STATUS3_LED_PORT                    GPIOB
#define  STATUS3_LED_GPIO_PIN                GPIO_PINS_9

/******************* define button *******************/
```

`new_string`:
```c
#define  STATUS3_LED_GPIO_CRM_CLK            CRM_GPIOB_PERIPH_CLOCK
#define  STATUS3_LED_PORT                    GPIOB
#define  STATUS3_LED_GPIO_PIN                GPIO_PINS_9

/******************* define EN_GATE for DRV8353 *******************/
#define  EN_GATE_GPIO_CRM_CLK                CRM_GPIOC_PERIPH_CLOCK
#define  EN_GATE_PORT                        GPIOC
#define  EN_GATE_GPIO_PIN                    GPIO_PINS_15

/******************* define button *******************/
```

- [ ] **Step 8: 验证 - grep 反向检查所有已改宏**

Run:
```bash
cd /e/src/AT_foc_hall_demo
# 确认 demo 里不再有 PB13/14/15 作为 PWM 输出（除头文件本身的注释）
grep -rn "GPIO_PINS_13\|GPIO_PINS_14\|GPIO_PINS_15" motor_evb_2v0/at32f45x/pmsm_foc_hall_sensor/src/mc_hwio.c motor_evb_2v0/at32f45x/pmsm_foc_hall_sensor/src/mc_isr.c
```
Expected: 无输出，或只在与本次改动无关的位置（例如 GPIOA_PINS_15 是 USART TX，属于新引脚）。

Run:
```bash
grep -n "TMR3\|TMR4" motor_evb_2v0/at32f45x/pmsm_foc_hall_sensor/inc/mc_hwio_v2.h motor_evb_2v0/at32f45x/pmsm_foc_hall_sensor/src/mc_hwio.c
```
Expected: Hall 用 TMR4，编码器分支引用的 TMR3/TMR2 可以保留（Task 3 里编码器分支不再被 nvic 启用，但代码保留不删）。

- [ ] **Step 9: Commit**

```bash
cd /e/src/AT_foc_hall_demo
git add motor_evb_2v0/at32f45x/pmsm_foc_hall_sensor/inc/mc_hwio_v2.h
git commit -m "$(cat <<'EOF'
map hardware pins to target board (LQFP48, DRV8353)

PWM low: PA7/PB0/PB1. Hall: PB6/PB7/PB8 on TMR4.
UART1: PA15/PB3. VBUS: PA3. MOS temp: PA5.
LED_ERR/LED_RUN swapped to PC14/PC13. Add EN_GATE = PC15.

Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>
EOF
)"
git status
```
Expected: clean working tree.

---

## Task 3: mc_hwio.c 精简与 EN_GATE

**Files:**
- Modify: `motor_evb_2v0/at32f45x/pmsm_foc_hall_sensor/src/mc_hwio.c`

**Interfaces:**
- Consumes: 所有 Task 2 里的宏
- Produces: `nvic_config()` 只启用目标板需要的中断；`led_config()` 只初始化 ERROR_LED 和 STATUS1_LED；`gpio_pins_init()` 新增 EN_GATE 输出低；`button_exint_init()` 清空函数体

**Rationale:** 目标板不存在的外设（编码器、按键、多余 LED）在源文件层面必须移除引用，不然编译报错（因为对应宏 Task 2 里没删还留着，但函数调用了不存在的引脚初始化会用到空的 CRM_CLK；同时 nvic 使能不存在的中断会挂）。

- [ ] **Step 1: 精简 nvic_config() - 删除编码器、SYNC、按键的中断使能**

Run:
```bash
grep -n "nvic_irq_enable" motor_evb_2v0/at32f45x/pmsm_foc_hall_sensor/src/mc_hwio.c
```
Expected: 11 行（52-84 行段），BRK/CYCLE/ADC/SPEED/HALL/ENC_CAP/EXINT_ENC_IDX/SYNC/SysTick/BUTTON/UART。

用 Edit：

`old_string`:
```c
  nvic_irq_enable(ENCODER_CAPTURE_IRQn, 6, 0);
```
`new_string`:
```c
  /* nvic_irq_enable(ENCODER_CAPTURE_IRQn, 6, 0); -- 目标板无编码器 */
```

`old_string`:
```c
  nvic_irq_enable(EXINT_ENCODER_IDX_IRQn, 0, 0);
```
`new_string`:
```c
  /* nvic_irq_enable(EXINT_ENCODER_IDX_IRQn, 0, 0); -- 目标板无编码器 */
```

`old_string`:
```c
  nvic_irq_enable(SYNC_TIMER_CH_IRQn, 0, 0);
```
`new_string`:
```c
  /* nvic_irq_enable(SYNC_TIMER_CH_IRQn, 0, 0); -- 目标板无磁编码器同步 */
```

`old_string`:
```c
  nvic_irq_enable(BUTTON_EXINT_IRQn, 7, 0);
```
`new_string`:
```c
  /* nvic_irq_enable(BUTTON_EXINT_IRQn, 7, 0); -- 目标板无按键 */
```

**注意**：注释掉而不是直接删除，是为了保留代码上下文，便于将来核对是否漏删。

- [ ] **Step 2: 清空 button_exint_init() 函数体**

Run:
```bash
sed -n '1123,1173p' motor_evb_2v0/at32f45x/pmsm_foc_hall_sensor/src/mc_hwio.c
```
Expected: 看到完整函数体，含 USER_BUTTON / HALL_LEARN_BUTTON / MODE1_BUTTON / MODE2_BUTTON 相关的 GPIO/exint 初始化。

用 Read 工具先读完整函数（如果超 50 行分两次读），再用 Edit 把整个函数体替换为空实现：

用 Edit：

`old_string`: `void button_exint_init(void)`（连整个函数体到匹配的 `}`）
`new_string`:
```c
void button_exint_init(void)
{
  /* 目标板无按键与 Hall learn button，函数保留为空以维持 main.c 调用兼容 */
}
```

**注意**：如果原函数体过长导致 Edit 的 old_string 太大，可以先用 Read 精确获取行号后，改用逐块修改：先把函数内所有实体代码用 `#if 0 / #endif` 包起来，然后再删除。二选一：**推荐第一种（整段替换）**。

- [ ] **Step 3: 清空 mode_switch_init() 函数体**

Run:
```bash
sed -n '1287,1314p' motor_evb_2v0/at32f45x/pmsm_foc_hall_sensor/src/mc_hwio.c
```

用 Edit 把整个 `mode_switch_init()` 替换为空实现：
```c
void mode_switch_init(void)
{
  /* 目标板无 MODE1/MODE2/REVERSE/BRAKE/LOCK/PARKING 开关 */
}
```

- [ ] **Step 4: 替换 gpio_pins_init() - 目标板无 LOCK_MOTOR / PARKING_LOCK 按键，改为只初始化 EN_GATE**

现有函数体（`mc_hwio.c:1315-1336`）初始化的是 `LOCK_MOTOR_SW`（PB11）和 `PARKING_LOCK_SW`（PB3）两个开关，前者硬件不存在、后者 PB3 已在 Task 2 分配给 UART1 RX。因此整个函数体要重写。

用 Edit：

`old_string`:
```c
void gpio_pins_init(void)
{
  gpio_init_type gpio_init_struct;

  /* enable the led clock */
  crm_periph_clock_enable(LOCK_MOTOR_SW_CRM_CLK, TRUE);
  crm_periph_clock_enable(PARKING_LOCK_SW_CRM_CLK, TRUE);

  /* set default parameter */
  gpio_default_para_init(&gpio_init_struct);

  /* configure the led gpio */
  gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
  gpio_init_struct.gpio_out_type  = GPIO_OUTPUT_OPEN_DRAIN;
  gpio_init_struct.gpio_mode = GPIO_MODE_INPUT;
  gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
  gpio_init_struct.gpio_pins = LOCK_MOTOR_SW_PIN;
  gpio_init(LOCK_MOTOR_SW_PORT, &gpio_init_struct);

  gpio_init_struct.gpio_pins = PARKING_LOCK_SW_PIN;
  gpio_init(PARKING_LOCK_SW_PORT, &gpio_init_struct);
}
```

`new_string`:
```c
void gpio_pins_init(void)
{
  gpio_init_type gpio_init_struct;

  gpio_default_para_init(&gpio_init_struct);

  /* EN_GATE = PC15 for DRV8353, init low (gate driver disabled) */
  crm_periph_clock_enable(EN_GATE_GPIO_CRM_CLK, TRUE);
  gpio_bits_reset(EN_GATE_PORT, EN_GATE_GPIO_PIN);
  gpio_init_struct.gpio_pins = EN_GATE_GPIO_PIN;
  gpio_init_struct.gpio_mode = GPIO_MODE_OUTPUT;
  gpio_init_struct.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
  gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
  gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_MODERATE;
  gpio_init(EN_GATE_PORT, &gpio_init_struct);
}
```

**注意**：`LOCK_MOTOR_SW_*` 和 `PARKING_LOCK_SW_*` 宏在 `mc_hwio_v2.h` 里保留不删（Task 2 决定），只是这里不再调用。这样宏值即使不匹配硬件也不会造成运行时问题。

- [ ] **Step 5: 精简 led_config() - 只保留 ERROR_LED 和 STATUS1_LED**

Run:
```bash
sed -n '1256,1286p' motor_evb_2v0/at32f45x/pmsm_foc_hall_sensor/src/mc_hwio.c
```
Expected: 完整 led_config() 和 led_blink()。

用 Read 精读后，把 STATUS2_LED、STATUS3_LED、ADC_TRIG_LED 相关的 `crm_periph_clock_enable` + `gpio_init` + `led_off` 调用行注释掉。**保留宏定义**，只不初始化。

同理 `led_blink()` 里 toggle STATUS2/STATUS3/ADC_TRIG 的行注释掉。

- [ ] **Step 6: 精简 brake_pwm_init()**

Run:
```bash
grep -n "brake_pwm_init\|BRAKING_RESISTOR" motor_evb_2v0/at32f45x/pmsm_foc_hall_sensor/src/mc_hwio.c motor_evb_2v0/at32f45x/pmsm_foc_hall_sensor/src/main.c
```
Expected: `brake_pwm_init()` 在 mc_hwio.c:976 定义，main.c 里在 `#ifdef BRAKING_RESISTOR` 保护下调用。

**决定**：Task 1 里没定义 `BRAKING_RESISTOR`，函数不会被调用，函数体保留不动即可。**本步无操作**。

- [ ] **Step 7: 验证 - 编译**

如果 MDK GUI 环境可用：请用户手动 Build 一次，粘贴 Build Output。
如果有命令行 UV4：
```bash
/c/Keil_v5/UV4/UV4.exe -b motor_evb_2v0/at32f45x/pmsm_foc_hall_sensor/mdk_v5/pmsm_foc_hall_sensor.uvprojx -j0 -o build.log
cat build.log
```
Expected: 0 error, 0 warning。若有 unused variable warning，允许保留（因为按键代码被注释后残留局部变量）。

**若编译报错**：99% 是某处引用了删掉的 IDX 或 STATUS 宏。回去补注释。

- [ ] **Step 8: Commit**

```bash
cd /e/src/AT_foc_hall_demo
git add motor_evb_2v0/at32f45x/pmsm_foc_hall_sensor/src/mc_hwio.c
git commit -m "$(cat <<'EOF'
strip mc_hwio.c to target board peripherals

Disable encoder/sync/button interrupts in nvic_config. Empty out
button_exint_init and mode_switch_init. Add EN_GATE init (PC15,
push-pull low) in gpio_pins_init. Skip STATUS2/3/ADC_TRIG LEDs.

Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>
EOF
)"
git status
```
Expected: clean.

---

## Task 4: mc_isr.c 清空 BUTTON_EXINT_IRQ

**Files:**
- Modify: `motor_evb_2v0/at32f45x/pmsm_foc_hall_sensor/src/mc_isr.c`

**Interfaces:**
- Consumes: `BUTTON_EXINT_LINE`、`HALL_LEARN_BUTTON_EXINT_LINE`（这些宏 Task 2 里保留了）
- Produces: 无

**Rationale:** BUTTON_EXINT_IRQ 中断已经在 nvic 里注释掉，实际不会被触发。但函数体如果保留原样引用 `hall_learn.check_flag` 等 mclib 全局变量赋值，虽然不会执行，但保留也无害。**决定：函数体保留不动**。本 Task 实际是"确认无操作后跳过"，方便 review 时明确"我看了没漏"。

- [ ] **Step 1: 阅读 BUTTON_EXINT_IRQ 函数体，确认无副作用**

Run:
```bash
sed -n '512,570p' motor_evb_2v0/at32f45x/pmsm_foc_hall_sensor/src/mc_isr.c
```
Expected: 完整 BUTTON_EXINT_IRQ()，检查 exint_flag、hall_learn 等。

- [ ] **Step 2: 决策**

因为 nvic 里已注释，中断永不触发。函数体可以保留。**不做修改**。

如果 review 者认为应该清空以避免混淆，那么用 Edit 把整个函数体替换为：
```c
void BUTTON_EXINT_IRQ(void)
{
  /* 目标板无按键中断，本函数保留占位 */
}
```

- [ ] **Step 3: Commit（如果没改就跳过）**

如果 Step 2 决定不改，本 Task 无 commit。跳到 Task 5。

---

## Task 5: main.c - 删除 button_exint_init 调用（可选），插入 EN_GATE 使能序列

**Files:**
- Modify: `motor_evb_2v0/at32f45x/pmsm_foc_hall_sensor/src/main.c`

**Interfaces:**
- Consumes: `EN_GATE_PORT`、`EN_GATE_GPIO_PIN`、`mc_delay_ms()`
- Produces: 无

**Rationale:** button_exint_init 在 Task 3 已清空为空函数，可以保留调用。EN_GATE 拉高必须在 tmr_pwm_init 之后（因为 PB12 BRK 引脚要先配置好否则 EN_GATE 使能时可能触发误保护），且在启动电机流程之前。

- [ ] **Step 1: 定位插入点**

Run:
```bash
grep -n "tmr_pwm_init\|enable_pwm_timer\|mc_delay_ms" motor_evb_2v0/at32f45x/pmsm_foc_hall_sensor/src/main.c | head -10
```
Expected: `tmr_pwm_init()` 在 main.c:68，`mc_delay_ms(500)` 在 main.c:96 附近。

**插入点**：在 `mc_delay_ms(500);` 这行之后、`enable_pwm_timer_channel_buffer` 之前插入 EN_GATE 使能。理由：`mc_delay_ms(500)` 后硬件已稳定，此时把 EN_GATE 拉高唤醒 DRV8353，再等 3 ms 让驱动器初始化完成，然后再打开 PWM。

- [ ] **Step 2: 插入 EN_GATE 使能**

Read main.c 第 94-100 行段确认精确文本，然后用 Edit：

`old_string`:
```c
  /*delay for hardware stable */
  mc_delay_ms(500);

  /* enable tmr channel mode buffer */
  enable_pwm_timer_channel_buffer(&pwm_duty);
```

`new_string`:
```c
  /*delay for hardware stable */
  mc_delay_ms(500);

  /* wake up DRV8353 gate driver: pull EN_GATE high, wait for wake time */
  gpio_bits_set(EN_GATE_PORT, EN_GATE_GPIO_PIN);
  mc_delay_ms(3);

  /* enable tmr channel mode buffer */
  enable_pwm_timer_channel_buffer(&pwm_duty);
```

- [ ] **Step 3: 验证 - grep 检查 EN_GATE 使用点**

Run:
```bash
grep -rn "EN_GATE" motor_evb_2v0/at32f45x/pmsm_foc_hall_sensor/
```
Expected: 头文件 3 处宏定义，mc_hwio.c 一处 gpio 初始化，main.c 一处使能。共 5 处。

- [ ] **Step 4: 编译验证**

同 Task 3 Step 7 方式。0 error 0 warning。

- [ ] **Step 5: Commit**

```bash
cd /e/src/AT_foc_hall_demo
git add motor_evb_2v0/at32f45x/pmsm_foc_hall_sensor/src/main.c
git commit -m "$(cat <<'EOF'
enable DRV8353 gate driver at startup

Pull EN_GATE high 3 ms after hardware stable delay, before enabling
PWM. Fault protection continues via BRK/PB12 hardware path.

Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>
EOF
)"
git status
```
Expected: clean.

---

## Task 6: MDK 工程 device 型号确认与编译验证

**Files:**
- Modify (conditional): `motor_evb_2v0/at32f45x/pmsm_foc_hall_sensor/mdk_v5/pmsm_foc_hall_sensor.uvprojx`

**Interfaces:**
- Consumes: 前 5 个 Task 的所有改动
- Produces: 可交付的 Keil 工程

**Rationale:** demo 原工程 device 型号是 AT32F455RET7（512KB Flash / 144KB SRAM，LQFP64）。目标板是 AT32F456CEU7（LQFP48，容量需 datasheet 确认）。这一步要么确认 device 目标不需要改（同族 IP 兼容），要么改成 AT32F456CEU7。

- [ ] **Step 1: 读工程文件中 Device 字段**

Run:
```bash
grep -n "Device\|Dvendor\|DeviceId\|FlashSize\|RAMSize" motor_evb_2v0/at32f45x/pmsm_foc_hall_sensor/mdk_v5/pmsm_foc_hall_sensor.uvprojx | head -20
```
Expected: 看到 `<Device>AT32F455RET7</Device>` 或类似。

- [ ] **Step 2: 决策与操作**

**若 device 是 AT32F455RET7**：
- 用户在 Keil GUI 里 Options for Target → Device → 选 AT32F456CEU7（需要已经安装 Artery AT32F45x pack）
- 或者手动在 xml 里改：

  Edit：
  `old_string`: `<Device>AT32F455RET7</Device>`
  `new_string`: `<Device>AT32F456CEU7</Device>`

  再改 `<DeviceId>` 和 `<SFDFile>`（这些字段名称由 Keil 版本决定，遵循 xml 中已有格式）。

**注意**：uvprojx 是 XML 但含二进制路径引用，建议**用户在 GUI 里改**再用 git diff 观察。

- [ ] **Step 3: 全量编译**

- 如 GUI 可用：用户在 Keil 里 F7 全量 Build，把 Build Output 粘贴。
- 如命令行可用：`UV4.exe -r -b ...uvprojx -j0 -o build.log`

Expected: 0 error 0 warning。**Warning 允许的情况**：
- unused variable / unused function（因删除按键代码残留）
- 若出现，Task 6 后追加一个"清理 unused"小 commit

**Error 必查项**：
- `undefined reference to ENCODER_CAPTURE_IRQn` → 有代码路径没被 `#ifdef` 保护而引用了编码器宏。检查 mc_hwio.c 和 mc_isr.c 是否还有硬编码。
- `PB13/PB14/PB15` 相关的 clock enable 缺失 → 有 GPIO 初始化路径没被 Task 3 精简到

- [ ] **Step 4: 生成的 .hex/.axf 文件容量报告**

Run:
```bash
ls -lh motor_evb_2v0/at32f45x/pmsm_foc_hall_sensor/mdk_v5/Objects/*.axf motor_evb_2v0/at32f45x/pmsm_foc_hall_sensor/mdk_v5/*.hex 2>/dev/null
```
Expected: 输出文件存在。demo 原产物 Code/RO/RW/ZI 报告能看到 Code 段小于 AT32F456CEU7 的 Flash 容量。

- [ ] **Step 5: Commit（若有工程文件改动）**

```bash
cd /e/src/AT_foc_hall_demo
git add motor_evb_2v0/at32f45x/pmsm_foc_hall_sensor/mdk_v5/pmsm_foc_hall_sensor.uvprojx
git commit -m "$(cat <<'EOF'
switch MDK device target to AT32F456CEU7

Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>
EOF
)"
git status
```
Expected: clean.

- [ ] **Step 6: 最终 diff 汇总**

```bash
cd /e/src/AT_foc_hall_demo
git log --oneline foc_hall_demo | head -10
git diff --stat $(git log --format=%H | tail -1) HEAD | tail -20
```
把输出粘贴给 review 者，作为"BSP 适配完成"的证据。

---

## 完成标准

- 前 6 个 Task 都完成 commit
- MDK 编译 0 error（warning 尽量 0，unused 允许）
- `git log foc_hall_demo` 至少 6 个 commit（对应 6 个 Task）
- 已知未验证项（列在 spec 第 5 节，本计划外）：
  - PB8=TMR4_CH3 上板验证
  - Hall learn 表实测标定
  - PWM 低边极性用示波器确认
