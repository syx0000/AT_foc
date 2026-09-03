# FOC 重构设计

> 状态：架构草案，评审通过后再开始实现。
>
> 详细实施与验收步骤见 `FOC_DEVELOPMENT_VERIFICATION_PLAN.md`。

## 1. 设计目标

1. 保留 AT32 WorkBench 重新生成工程的能力。
2. FOC 算法不直接访问 ADC、TMR、GPIO、CAN 等外设寄存器。
3. 中断中只保留采样、硬实时计算和安全输出，不执行通信、Flash、打印等业务。
4. 拆分旧版 `ControllerStruct`，明确配置、指令、反馈、运行状态和故障的数据所有权。
5. 每个迁移阶段均可独立编译和上板验证，关键节点单独提交。
6. 迁移时修正旧代码中不合理的文件、函数、变量和宏命名，不延续历史命名债务。
7. 只迁移当前电机闭环必需的功能；旧工程中的冗余、测试和历史兼容代码不迁移。
8. 当前阶段不迁移 CAN 协议，只保留与通信方式无关的电机命令接口。

## 2. 已确认的旧工程行为

以下内容来自只读参考工程 `E:\src\AT_foc_hall - 副本`，不是推测：

- `project/src/at32f45x_int.c`：ADC 注入组完成中断以 10 kHz 调用位置计算和 `MC_Loop_Schedule()`。
- `project/src/at32f45x_int.c`：ADC 普通组约 1 kHz 更新温度、母线电压等慢速采样。
- `project/src/at32f45x_int.c`：Hall 边沿中断调用 `Encoder_Hall_EdgeISR()`。
- `project/src/main.c`：启动顺序包括 DRV8353 唤醒、ADC 零偏校准、位置传感器初始化、FOC 初始化，最后才允许 PWM 输出。
- `project/foc/foc_fast/foc_kernel.c`：现有 Clarke、Park、反 Park、SVPWM 和矢量限幅使用整数接口。
- `project/foc/foc_fast/foc_controller.h`：旧 `ControllerStruct` 同时包含控制状态、Flash 参数、滤波器、辨识和带宽测试数据。
- `project/foc/foc_app/can_debug.c`：旧通信层直接修改控制器和 Flash 数据，模块边界较弱。

## 3. 总体分层

```text
WorkBench 生成层
  main.c / at32f45x_int.c / wk_*.c
              │ 仅在 USER CODE 区调用稳定入口
              ▼
电机硬件端口层（新增文件）
  motor_adc_port / motor_pwm_port / motor_hall_port / motor_timebase
              │ 原始采样与最终PWM命令
              ▼
FOC 内核层（foc_kernel）
  数学变换 / PI / SVPWM / 电流环 / 速度估计
              │ 状态快照与受控命令
              ▼
应用层（foc_app）
  状态机 / 必要故障 / 校准 / 参数 / 本地调试
```

依赖只能自上而下。`foc_kernel` 不包含 `at32f45x*.h`，不调用 `printf`、Flash 或通信接口。

## 4. 建议目录

```text
project/foc/
├── foc_kernel/
│   ├── foc_types.h             公共数值类型和输入输出结构
│   ├── foc_math.c/.h           限幅、角度和基础数学
│   ├── foc_transform.c/.h      Clarke、Park、反 Park
│   ├── foc_pi.c/.h             PI及抗积分饱和
│   ├── foc_svpwm.c/.h          电压矢量到三相占空比
│   ├── foc_current_loop.c/.h   电流采样到PWM命令
│   └── hall_estimator.c/.h     Hall状态、方向、速度及扇区内插值
│
└── foc_app/
    ├── motor_control.c/.h      单电机上下文及多速率调度入口
    ├── motor_state.c/.h        启停和运行状态机
    ├── motor_fault.c/.h        故障检测、锁存和清除
    ├── motor_calibration.c/.h  ADC零偏及Hall角度校准
    ├── motor_config.c/.h       电机与控制参数默认值、校验和装载
    └── motor_command.c/.h      与通信协议无关的控制命令接口

project/bsp/
├── motor_board_config.h
├── motor_adc_port.c
├── motor_adc_port.h
├── motor_pwm_port.c
├── motor_pwm_port.h
├── motor_hall_port.c
├── motor_hall_port.h
├── motor_timebase.c
└── motor_timebase.h
```

`project/src` 只保存 WorkBench 生成代码和现有系统入口；所有人工维护的电机硬件适配代码放在独立的 `project/bsp`。Keil 工程为 `bsp` 建立独立分组和头文件包含路径。

调用和编译依赖固定为：

```text
project/src（生成代码 USER CODE 区） → foc_app/motor_control
                                      ├→ foc_kernel
                                      └→ project/bsp → AT32驱动
```

- `project/src` 的 USER CODE 区只调用 `foc_app` 的稳定入口；初始化阶段可调用必要的 `bsp` 初始化接口。
- `project/bsp` 可以包含 AT32 驱动头文件和 `at32f45x_wk_config.h`。
- `project/foc` 不允许包含 `wk_*.h`、AT32 驱动头文件或访问外设寄存器。
- `foc_app/motor_control` 是唯一编排者，负责从 `bsp` 取得输入、调用 `foc_kernel`，再将输出交给 `bsp`。

### 4.1 参数放置

参数按所有权分成三类：

1. `project/bsp/motor_board_config.h`：只保存板级硬件常量，例如电流采样极性、ADC比例、分流电阻和门极有效电平。PWM周期、ADC通道、GPIO等已由 WorkBench 生成的配置不重复定义，由端口层读取现有配置。
2. `project/foc/foc_app/motor_config.c/.h`：保存本型号电机的默认参数和控制参数，例如极对数、相电阻、电感、磁链、最大电流、最大转速、PI参数及Hall映射表。每个数值必须注明单位和来源。
3. `motor_control_t`：初始化时取得一份经过校验的运行时配置。FOC中断只读取运行时副本，不直接读取Flash，也不在运行中修改默认配置。

```c
typedef struct {
    float phase_resistance_ohm;
    float d_inductance_h;
    float q_inductance_h;
    float flux_weber;
    uint8_t pole_pairs;
} motor_physical_config_t;

typedef struct {
    foc_pi_config_t current_d_pi;
    foc_pi_config_t current_q_pi;
    foc_pi_config_t speed_pi;
    foc_pi_config_t position_pi;
    float current_limit_a;
    float speed_limit_rpm;
} motor_control_config_t;

typedef struct {
    motor_physical_config_t physical;
    motor_control_config_t control;
    hall_estimator_config_t hall;
} motor_config_t;

const motor_config_t *motor_config_get_default(void);
bool motor_config_validate(const motor_config_t *config);
```

参数持久化由独立的 `motor_config_storage.c/.h` 负责；存储层负责地址、版本、长度、CRC和掉电安全，不把Flash读写放进 `motor_config.c`。具体布局见第 8 节。

## 5. 数据模型

不再建立一个无边界的大结构体，拆为以下对象：

```c
typedef struct {
    float ia_a;
    float ib_a;
    float ic_a;
    float bus_voltage_v;
    uint16_t electrical_angle;
    int32_t mechanical_position;
    float speed_rpm;
} motor_feedback_t;

typedef struct {
    float id_ref_a;
    float iq_ref_a;
    float speed_ref_rpm;
    int32_t position_ref;
    uint8_t mode;
} motor_command_t;

typedef struct {
    motor_config_t config;
    motor_feedback_t feedback;
    motor_command_t command;
    foc_current_loop_t current_loop;
    motor_runtime_t runtime;
    motor_fault_state_t fault;
} motor_control_t;
```

约束：

- Flash 只保存独立的持久参数结构，不直接保存 `motor_control_t`。
- 外部调用者只能通过命令接口更新目标值，不能获得控制器内部结构的可写指针。
- ISR 与主循环共享的数据使用单写者原则；多字段快照采用短临界区或序列号机制。
- 物理量在接口名中标注单位，ADC/PWM 原始值只存在于硬件端口边界。

## 6. 实时调度

### 10 kHz：ADC 注入组中断

```text
读取两相电流原始值
→ 转换为物理电流
→ 获取当前电角度
→ Clarke/Park
→ Id/Iq PI
→ 反Park/SVPWM
→ 写PWM影子寄存器
→ 更新快速运行快照
```

WorkBench 中断文件中只保留稳定入口：

```c
motor_control_fast_isr();
```

快速入口禁止阻塞、动态内存、Flash、通信发送和格式化打印。

### Hall 边沿中断

读取三路 Hall 状态和 DWT 时间戳，然后调用：

```c
motor_control_hall_edge_isr(hall_state, timestamp);
```

Hall 必须捕获上升沿和下降沿，否则六步状态会漏采。非法状态和非法跳变只记录并上报，不在 EXINT 内执行复杂故障流程。

### 1 kHz 慢速控制

- 速度环。
- 指令斜坡。
- 快速故障检查。
- 状态机时间条件。

### 主循环或 100 Hz 后台任务

- 位置环（最终频率待实测确定）。
- 本地调试和状态观测。
- Flash 保存。
- 必要的校准流程。
- 非实时日志。

## 7. 状态机与安全

```text
BOOT → SELF_TEST → CALIBRATING → READY → RUNNING
                                  ▲          │
                                  └─ STOPPING┘

任意状态 ──严重故障──> FAULT
FAULT ──清除条件满足──> READY
```

只有状态机可以正常使能门极和 PWM。另保留不依赖状态机的紧急关闭接口：

```c
void motor_pwm_emergency_stop(motor_fault_code_t reason);
```

启动顺序继续遵循旧工程已经验证过的安全约束：先完成控制器和反馈初始化，再允许 PWM 输出。

## 8. Flash 参数存储规划

### 8.1 当前布局依据

当前 AT32F456CEU7 Flash 总空间为 512 KB，扇区大小为 2 KB。现有 `project/inc/ota_config.h` 和 Bootloader 使用以下尾部布局：

```text
0x08040000 ─────────────── OTA Staging数据
0x0807F800 ─────────────── 当前User Data，2 KB
0x08080000 ─────────────── Flash结束
```

单个2 KB扇区在更新时必须先整扇区擦除。擦除后、新记录提交前若掉电，会同时失去旧参数和新参数，因此新架构不继续使用单槽覆盖方案。

### 8.2 固定参数区域

规划 Flash 最后两个扇区为固定参数区域，共4 KB：

```text
MOTOR_CONFIG_REGION_START     0x0807F000U
MOTOR_CONFIG_REGION_LENGTH    0x00001000U  /* 4 KB */

Slot A: 0x0807F000 - 0x0807F7FF  /* 2 KB */
Slot B: 0x0807F800 - 0x0807FFFF  /* 2 KB */
```

因此 OTA Staging 数据上限同步调整为 `0x0807F000`。Staging 可用长度从254 KB减少为252 KB，仍大于当前238 KB的 `APP_MAX_SIZE`。该地址变更必须同时更新 App 与 Bootloader 共享的 Flash 布局定义，不能只在 FOC 模块内部定义。

### 8.3 A/B 双槽记录

每个槽包含一个固定头部和可扩展负载：

```c
typedef struct {
    uint32_t magic;
    uint16_t format_version;
    uint16_t header_length;
    uint32_t sequence;
    uint32_t payload_length;
    uint32_t payload_crc32;
    uint32_t header_crc32;
    uint32_t commit_marker;
} motor_config_record_header_t;
```

- `magic`：固定标识，排除空Flash和其他数据。
- `format_version`：存储格式版本，不等同于固件版本。
- `sequence`：每次成功保存递增；启动时选择有效且序号更新的槽。
- `payload_length`：实际有效负载长度，不能超过单槽容量。
- `payload_crc32`：校验完整参数负载。
- `header_crc32`：校验除提交标记外的记录头。
- `commit_marker`：最后一步单独写入；只有提交标记、头部和负载校验全部有效，记录才可加载。

保存流程：

```text
确认电机停止且PWM关闭
→ 选择非当前槽
→ 擦除目标槽
→ 写入记录头（未提交状态）
→ 写入参数负载
→ 回读并校验CRC
→ 最后写入commit_marker
→ 将新槽作为当前槽
```

写新槽过程中旧槽始终保持有效。掉电重启后扫描两个槽，忽略未提交或CRC错误的记录。

### 8.4 可扩展负载

负载不直接保存 `motor_control_t`，使用带ID、版本和长度的参数块：

```c
typedef struct {
    uint16_t section_id;
    uint16_t section_version;
    uint32_t section_length;
} motor_config_section_header_t;
```

初始规划的参数块：

```text
MOTOR_CONFIG_SECTION_PHYSICAL       电阻、电感、磁链、极对数
MOTOR_CONFIG_SECTION_CONTROL        PI参数及控制限制
MOTOR_CONFIG_SECTION_HALL           Hall映射和电角度校准
MOTOR_CONFIG_SECTION_ADC            电流零偏和采样校准
MOTOR_CONFIG_SECTION_LIMITS         电流、速度和位置限制
```

新固件可以追加新的参数块；读取时根据 `section_id + section_version + section_length` 解析，未知参数块跳过，缺少的参数块使用当前固件默认值。禁止把C结构体内存直接整体写入Flash，以避免编译器填充、类型变化和字段顺序破坏兼容性。

### 8.5 软件职责

```text
project/bsp/flash_storage_port.c/.h
    擦除、编程、读取、地址边界检查

project/foc/foc_app/motor_config_storage.c/.h
    A/B槽选择、记录校验、参数块编解码、默认值回退

project/foc/foc_app/motor_config.c/.h
    参数定义、默认值和业务有效性校验
```

Flash写入只能在电机停止、PWM和门极输出关闭时执行。10 kHz ISR、Hall ISR和1 kHz任务均不得擦写Flash。

### 8.6 异常策略

- 两槽均无效：加载编译期默认值，设置配置异常状态，但不自动反复擦写Flash。
- 仅一个槽有效：加载有效槽；是否后台修复另一个槽由后续需求决定，启动阶段不自动写入。
- 参数CRC有效但业务校验失败：拒绝该参数块并使用对应默认值，同时记录具体错误原因。
- 保存失败：继续使用RAM中的当前参数并保留旧有效槽，不切换当前序号。
- 序号回绕：使用无符号差值比较，不简单使用数值大小判断新旧。

## 9. WorkBench 再生成边界

允许修改：

- WorkBench 的 USER CODE 区。
- 新增且由我们维护的 `.c/.h` 文件。
- Keil 工程中新增文件、分组和包含路径。

避免修改：

- WorkBench 自动生成区中的业务逻辑。
- CMSIS 和厂商驱动库。

自动生成文件只调用以下少量稳定入口：

```c
motor_app_init();
motor_app_poll();
motor_control_fast_isr();
motor_control_hall_edge_isr(state, timestamp);
motor_control_slow_adc_isr();
```

## 10. 命名规范

所有新增或迁移代码使用英文命名；同一概念只能保留一个名称。文件名和函数名统一采用 `motor_control` 风格，即全小写蛇形命名法。

### 10.1 文件和目录

```text
motor_control.c
motor_control.h
foc_current_loop.c
hall_estimator.c
```

- 文件名、目录名：`lower_snake_case`。
- `.c` 与 `.h` 使用相同基础名称。
- 文件名表达单一职责，不使用 `data`、`api`、`func`、`common` 等含义模糊的名称。

旧文件迁移示例：

```text
foc_api.c          → motor_control.c / motor_command.c
foc_bsp.c          → motor_pwm_port.c / motor_adc_port.c
encoder_calc.c     → hall_estimator.c
func_pid.c         → foc_pi.c
func_filter.c      → signal_filter.c
func_errMes.c      → motor_fault.c
```

### 10.2 函数和变量

```c
void motor_control_init(motor_control_t *motor);
void motor_control_fast_isr(void);
void hall_estimator_on_edge(hall_estimator_t *hall,
                            uint8_t hall_state,
                            uint32_t timestamp);

float speed_reference_rpm;
uint16_t electrical_angle;
bool flash_dirty;
```

- 函数、局部变量、结构体成员：`lower_snake_case`。
- 函数名采用“模块 + 动作”，例如 `motor_control_start()`，避免 `motorRun()`、`Init_foc()`、`ServoStateFlagGudge()`。
- 布尔变量使用 `is_`、`has_`、`enable_`、`*_valid`、`*_pending` 等可判断语义。
- 单位写入变量名或接口注释；跨模块接口优先写入名称，例如 `speed_rpm`、`bus_voltage_v`、`period_us`。
- 禁止无意义缩写和拼写错误；`position`、`current`、`reference` 等词在整个工程中保持一致。
- 全局变量原则上不直接暴露；确有必要时使用模块前缀，不使用无语义的 `g_` 作为唯一边界说明。

### 10.3 宏、常量、类型和枚举

```c
#define MOTOR_CONTROL_FREQUENCY_HZ       10000U
#define HALL_ESTIMATOR_STOP_TIMEOUT_US  400000U

typedef struct {
    /* ... */
} motor_control_t;

typedef enum {
    MOTOR_STATE_BOOT = 0,
    MOTOR_STATE_READY,
    MOTOR_STATE_RUNNING,
    MOTOR_STATE_FAULT
} motor_state_t;
```

- 宏和枚举成员：`UPPER_SNAKE_CASE`，必须带所属模块前缀。
- 类型：`lower_snake_case_t`。
- 常量优先使用有类型的 `static const`；只有编译期条件或必须由预处理器使用时才定义宏。
- 头文件保护宏统一为 `文件名大写_H`，例如 `MOTOR_CONTROL_H`。

### 10.4 迁移和兼容策略

- 从旧工程迁移功能时，先确定新职责和新名称，再迁移实现；禁止先复制旧文件后长期保留旧命名。
- 内部接口不提供旧名称兼容宏，例如不保留 `#define Init_foc motor_control_init`。
- 需要保留的 Flash 布局等外部标识可兼容，但内部变量和函数必须使用新命名；CAN 当前不迁移。
- 每次重命名同时更新声明、定义、调用、注释和设计文档，仓库中不得残留失效旧名称。
- 文件头统一包含 `@file`、`@brief`、`@author`、`@date`、`@version`，格式遵循 `重构.md`。

## 11. 分阶段实施与验证

### M0：架构基线

- 评审本文档和关键决策。
- 建立空模块、公共类型和 Keil 分组。
- 验证裸工程编译通过。
- **提交节点：架构骨架可编译。**

### M1：硬件安全链路

- DWT 时间基准。
- PWM 写入、使能和紧急关闭。
- ADC 快慢采样接口与零偏校准。
- 验证 PWM 默认关闭、ADC 原始值和中断频率。
- **提交节点：BSP 接口上板验证通过。**

### M2：Hall 反馈

- 双边沿 EXINT 接入。
- 状态表、方向、非法跳变、速度和角度插值。
- 手转电机对照旧工程输出验证。
- **提交节点：Hall 反馈验证通过。**

### M3：FOC 数学内核

- 迁移 Clarke/Park、PI、SVPWM。
- 使用旧工程输入输出向量做离线对照测试。
- **提交节点：算法回归一致。**

### M4：受限开环与电流闭环

- 低母线电压/低占空比开环验证相序。
- 接入 Id/Iq 电流环、限流和紧急关闭。
- **提交节点：电流环验证通过。**

### M5：速度、位置和必要应用功能

- 依次接入速度环、位置环、状态机、必要故障和必要参数存储。
- CAN、OTA、带宽测试和自动参数辨识等功能不在本阶段迁移范围内。
- 每项功能单独评估、迁移、验证和提交，不整包恢复旧 `foc_app`。

## 12. 待评审决策

1. **核心数值制式**：沿用旧整数/Q格式，还是第一阶段改为浮点物理单位？建议先沿用经过验证的旧整数数学结果，再逐模块评估浮点化，避免算法与架构同时变化。
2. **位置反馈范围**：新版本只支持 Hall，还是保留 DPT 作为校准/对照源？遵循最小迁移原则，若当前硬件验证不需要 DPT，则本阶段不迁移。
3. **位置环频率**：旧工程与目标性能需求需进一步确认，先不在接口中写死。
4. **旧参数兼容性**：新区域继续覆盖旧地址 `0x0807F800`，需要确认首次升级时是否读取一次旧 `FlashSavedData` 并转换；CAN对象字典和协议当前不迁移。

## 13. 第一条实施原则

在上述决策确认前，不迁移旧 FOC 实现。任何旧模块迁移前必须回答“当前闭环是否必需、是否有明确验证方法”；不能同时满足则不迁移。确认后从 M0 开始，每完成一个可验证节点先更新 `README.md` 修改记录并提醒提交。

## 14. 软硬件版本管控

### 14.1 版本类型

四类版本分别管理，不允许混用：

```text
固件版本          firmware_version       对外发布和README修改记录
硬件版本          hardware_version       PCB/PCBA硬件修订
参数存储格式版本  config_format_version  Flash参数解析兼容性
Bootloader版本    bootloader_version     Bootloader自身发布版本
```

CAN协议当前不迁移，因此暂不建立新协议版本；以后恢复通信时单独增加 `protocol_version`。

### 14.2 固件版本

固件采用日历版本：

```text
YYYY.MM.DD.REVISION
```

示例：`2026.09.03.0`。同一天第一次可发布构建为 `.0`，后续依次递增。README每条修改记录使用相同格式。

版本唯一来源放在人工维护目录：

```text
project/config/product_version.h
```

建议定义：

```c
#define FIRMWARE_VERSION_YEAR       2026U
#define FIRMWARE_VERSION_MONTH         9U
#define FIRMWARE_VERSION_DAY           3U
#define FIRMWARE_VERSION_REVISION      0U
#define FIRMWARE_VERSION_STRING       "2026.09.03.0"
```

OTA `app_header_t.version` 继续使用 `uint32_t`，由上述四段确定性编码产生，禁止在多个文件手写十六进制版本值。具体位编码在实现前与Bootloader共同确认，并用编译期断言检查年月日和修订号范围。

开发过程中的普通提交不必每次升级版本；只有形成可下载、可回归的发布构建时升级版本并新增README记录。同一版本号不得对应两个不同发布固件。

### 14.3 硬件版本

硬件版本采用：

```text
MAJOR.MINOR
```

- `MAJOR`：引脚、电源、采样拓扑、驱动器或接口发生不兼容变化。
- `MINOR`：不改变固件基础适配关系的PCB/物料修订。

```c
#define HARDWARE_VERSION_MAJOR        1U
#define HARDWARE_VERSION_MINOR        0U
#define HARDWARE_VERSION_STRING      "1.0"
```

硬件版本必须来源于原理图/PCB正式版本，不从旧代码猜测。若PCB没有版本识别电阻、EEPROM或OTP，则它是编译期目标版本，固件无法自动判断实际接入的板卡；发布记录必须注明适配硬件版本。后续若增加硬件识别机制，再由BSP读取实际版本。

### 14.4 兼容性声明

每个发布固件明确声明：

```c
#define HARDWARE_VERSION_MIN_SUPPORTED ...
#define HARDWARE_VERSION_MAX_SUPPORTED ...
#define MOTOR_CONFIG_FORMAT_VERSION     ...
#define BOOTLOADER_VERSION_MIN_REQUIRED ...
```

构建信息中至少保留固件版本、目标硬件版本和Git短提交号。Git工作区不干净时生成的固件只能用于开发验证，不能作为正式发布件。

### 14.5 发布规则

正式发布必须同时具备：

1. 对应的Git提交且工作区干净。
2. `product_version.h`中的唯一固件版本。
3. README中同版本修改记录。
4. 验证记录中注明固件版本、Git提交和硬件版本。
5. 固件产物文件名包含产品名、固件版本和硬件版本。
6. 创建与固件版本一致的Git标签，建议格式为 `fw-2026.09.03.0`。

硬件资料正式发布时使用标签 `hw-1.0`；硬件文件未纳入同一发布流程时，至少在固件验证记录中引用明确的原理图和PCB版本。

## 15. 日志等级与输出约束

日志模块由 `project/inc/motor_log.h` 和 `project/src/motor_log.c` 组成，底层继续使用USART1重定向的 `printf`。等级语义参考 `E:\src\cw_joint_project`，运行时等级越高，允许输出的信息越详细：

```text
0  MOTOR_LOG_LEVEL_OFF    关闭日志
1  MOTOR_LOG_LEVEL_ERROR  错误
2  MOTOR_LOG_LEVEL_WARN   错误、警告
3  MOTOR_LOG_LEVEL_DEBUG  错误、警告、调试
4  MOTOR_LOG_LEVEL_INFO   全部日志，当前开发默认值
```

统一使用 `LOGE`、`LOGW`、`LOGD`、`LOGI`，格式为 `[LEVEL] function line: message`。通过 `motor_log_level_set()` 修改运行时等级；超出0~4的值不得生效。正式版本的默认等级在发布评审时确定。

启动日志至少包含产品名、固件版本、目标硬件版本和App启动成功标记。版本字符串只能来自 `project/config/product_version.h`。

日志禁止用于10 kHz FOC ISR、Hall边沿ISR和紧急关断路径。实时路径只更新状态、计数器或事件码，由主循环或低频后台任务限频输出，避免阻塞式 `printf` 改变控制周期。Bootloader日志保持独立，不依赖App的 `motor_log` 模块。

### 15.1 中断频率诊断

开发验证阶段使用 `project/inc/interrupt_monitor.h` 和 `project/src/interrupt_monitor.c` 统计中断频率。ISR只递增32位计数器，主循环以1 kHz SysTick为时间基准，每1000个tick生成快照并输出，禁止在ISR内直接打印。

监控名称使用业务含义而非DMA通道号：`adc_fast`表示10 kHz预注入电流采样，`adc_slow_dma`表示1 kHz普通组慢速采样搬运。当前实测结果为SysTick 1 kHz、TMR1通道4 20 kHz、ADC快速采样10 kHz、ADC慢速DMA 1 kHz，ADC触发失败为0。

ADC普通组转换完成中断和DMA完整传输中断对应同一批1 kHz数据，不同时保留；当前方案关闭 `ADC_OCCE_INT`，保留DMA1 Channel3完整传输中断。DMA1 Channel3使用4个半字循环缓冲区，保存ADC1/ADC2普通组同步采样结果。

### 15.2 DWT时间基准与性能统计

`project/bsp/motor_timebase.c/.h`在系统时钟配置完成后初始化Cortex-M4 DWT自由运行周期计数器。DWT只初始化一次，各调用点通过独立起始时间戳并行测量，不得在测量点重新清零全局计数器。

`project/inc/motor_performance_monitor.h`和`project/src/motor_performance_monitor.c`提供通用性能统计器，记录最近执行周期、统计区间峰值和调用次数。每个被测任务或中断必须使用独立统计器；高频实时路径只执行开始和结束打点，统计快照及日志打印放在主循环低频执行。

低优先级代码被高优先级中断抢占时，测量值包含抢占时间，这一结果用于评估真实最坏响应时间。192 MHz下DWT 32位计数器约22.37秒回绕，短时间间隔通过无符号减法兼容单次回绕。

## 16. 当前实现与实测基线（2026.09.03.4）

- `motor_adc_port`统一快速两相电流和慢速四半字DMA采样，实测频率分别为10 kHz和1 kHz。
- 电流采样使用2.5 mΩ分流电阻、DRV8353硬件10 V/V增益，实测确认ADC低于零偏为正电流。
- `motor_foc_math`提供Q15正余弦、Clarke/Park、反Park和SVPWM，开环10 Hz试转稳定。
- Hall正向顺序为`1→5→4→6→2→3`，六状态边沿角度已标定并使用完整周期频率进行边沿间插值。
- 阻塞辨识实测`Rs=273 mΩ、Ld=225 µH、Lq=206 µH`；辨识结果当前只打印，尚未写入Flash。

## 17. FOC模块分层与依赖规范

### 17.1 `foc_kernel`纯算法层

- `motor_foc_math`：Q15三角函数、Clarke/Park、反Park和SVPWM。
- `motor_current_pi`：单轴电流PI、输出预置和外部限幅抗饱和回算。
- `motor_voltage_limit`：dq合成电压联合限幅，不读取母线电压或硬件状态。
- `motor_ramp`：通用有符号线性斜坡，支持跨零和运行中更新目标。
- 本层禁止包含ADC、Hall、PWM、AT32寄存器及WorkBench生成接口。

### 17.2 `foc_app`控制与流程层

- `motor_open_loop`：管理对齐、运行指令、频率斜坡、PWM所有权和故障状态。
- `motor_current_control`：正式Hall角度电流闭环，是闭环运行时唯一PWM写入者。
- `motor_*_identification`和`motor_*_test`：只管理辨识/验证时序和结果统计。
- 测试模块允许计算交接坐标和采集统计，但不得复制正式PI、限幅或PWM控制链。

### 17.3 `bsp`硬件端口层

- `motor_adc_port`只负责原始ADC采样快照。
- `motor_pwm_port`只负责比较值、MOE、门极使能和硬件故障安全操作。
- `motor_hall_port`只负责Hall边沿、状态、方向及周期测量。
- `motor_timebase`只提供统一周期计数和时间换算。

依赖方向固定为：`foc_app -> foc_kernel + bsp`。`foc_kernel`不得反向依赖
`foc_app`或`bsp`，BSP也不得包含FOC业务策略。

### 17.4 控制权规范

- 任一时刻只能有一个运行模块写入三相PWM比较值。
- 开环切换闭环时，先停止开环更新但保持MOE，再由
  `motor_current_control_handover()`预置PI并接管。
- 普通停止关闭MOE；采样、Hall、过流或刹车异常必须调用紧急停机。
- 正式控制参数与一次性测试参数分别分组，均集中在
  `motor_control_config.h`，禁止在控制源文件内散落可调常量。
- 200 Hz电流PI采用毫安输入、毫伏输出和条件积分抗饱和；固定角度`Id=2 A`测试平均值为1991 mA，Iq平均值为-12 mA。
- 当前启动代码用于开发验证，会依次执行Rs、Ld/Lq和电流PI测试，完成后关闭PWM；正式状态机接入时必须移除自动辨识流程。
