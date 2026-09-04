# 电机自动辨识、接线适配与标定设计

## 1. 目标

建立一套完整的新电机适配流程，在用户提供额定能力和必要机械信息后，自动完成可安全辨识的电气参数、Hall接线关系和角度参数，并分阶段验证电流环与速度环。允许用户在满足电气安全的前提下任意排列连接三根电机相线U/V/W和三根Hall信号HA/HB/HC，软件通过一次低功率调试流程学习当前接线关系。

完整范围包括：

- ADC电流零偏校准；
- 相电阻Rs辨识；
- D/Q轴电感Ld/Lq辨识；
- 任意相线与Hall信号排列适配；
- Hall六步顺序与六个物理边界角度标定；
- Hall转子坐标补偿标定；
- 电流PI参数计算及低电流验证；
- 开环、电流环和速度环分阶段接管；
- 速度PI调试入口；
- 电流、电压、堵转、超速和温度保护验证；
- 参数校验、候选配置应用和后续Flash保存。

其中Hall自动适配支持：

- 正反转Hall状态识别；
- 正反转电角度插值；
- 有符号电频率和机械转速；
- 设备逻辑正方向反转设置；
- 标定结果校验、应用和后续Flash保存。

Hall电源和地线不属于可任意排列范围，必须严格按硬件定义连接。相线、Hall信号线也禁止带电插拔。

## 2. 方向定义

系统使用三层方向定义，避免把接线顺序、FOC坐标和设备机械方向混为一谈。

1. 电气默认正方向：PWM电角度递增时形成的旋转方向，由自动标定过程建立。
2. 设备逻辑正方向：用户和上层协议看到的正方向，由`direction_inverted`配置决定。
3. 本次运行方向：由速度或转矩指令的正负号决定。

换算关系：

```text
direction_sign = direction_inverted ? -1 : +1
物理目标转速 = 逻辑目标转速 × direction_sign
逻辑反馈转速 = 物理反馈转速 × direction_sign
```

自动标定不需要识别机构的“前进”方向。若默认正方向不符合整机定义，只修改方向反转配置，不交换相线和Hall线。

## 3. 模块职责

### 3.1 motor_hall_port

只负责硬件输入和原始事件：

- 同时读取HA/HB/HC并生成3位原始状态；
- 捕获DWT边沿时间戳；
- 统计原始边沿、重复中断和非法电平；
- 不固化特定电机的六步顺序。

### 3.2 motor_hall_commissioning

负责一次性的自动识别和标定：

- 在受限电压、电流和频率下驱动开环电角度；
- 收集原始Hall状态跳转序列；
- 识别六个有效状态及正向邻接关系；
- 从正向关系自动推导反向邻接关系；
- 记录每个物理Hall边界的电角度；
- 计算扇区宽度和离散程度；
- 校验标定结果并生成`motor_hall_config_t`；
- 只在用户明确应用后替换运行配置；
- 后续通过参数存储模块保存，不直接操作Flash。

### 3.3 motor_hall_angle_estimator

负责正常运行时使用标定结果：

- 根据跳转表判断正转、反转和非法跳转；
- 正转边沿使用“进入当前状态”的物理边界角度；
- 反转边沿使用“离开前一状态”的同一物理边界角度；
- 根据有符号电频率在相邻边沿之间插值；
- 正转相位累加，反转相位递减；
- Hall超时后输出无效，不继续盲目外推。

### 3.4 motor_speed_feedback

负责将有符号电频率转换为机械转速：

```text
机械转速milli-rpm = 有符号电频率mHz × 60 ÷ 极对数
```

滤波仅在新的Hall测速结果到达时更新，周期任务只维护状态与超时。

## 4. Hall运行配置

建议将当前编译期六个角度宏逐步替换为运行时配置：

```c
typedef struct
{
    uint32_t magic;
    uint16_t format_version;
    uint16_t length;

    uint8_t positive_next[8];
    uint16_t positive_entry_angle_u16[8];

    uint8_t pole_pairs;
    uint8_t direction_inverted;
    uint8_t valid;
    uint8_t reserved;

    uint32_t crc32;
} motor_hall_config_t;
```

说明：

- 状态0和7始终非法；
- `positive_next[state]`保存电气默认正方向的下一个状态；
- 反向关系由正向表反推，不重复存储；
- `positive_entry_angle_u16[state]`保存正向进入该状态时的物理边界角度；
- `direction_inverted`只改变上层逻辑方向，不改变物理边界标定结果；
- `magic/version/length/crc32`用于后续Flash兼容性与完整性检查。

## 5. 自动标定状态机

建议状态：

```text
IDLE
  → PRECHECK
  → ALIGN
  → FORWARD_SCAN
  → FORWARD_VALIDATE
  → REVERSE_SCAN
  → REVERSE_VALIDATE
  → BUILD_CONFIG
  → WAIT_ACCEPT
  → COMPLETE

任意异常 → FAULT → PWM关闭
```

### 5.1 PRECHECK

- 电机处于READY，PWM和速度/电流闭环均未运行；
- ADC零偏校准有效；
- 母线电压处于安全范围；
- DRV的nFAULT为高，TMR1 BIF已清除；
- Hall静态状态不是0或7；
- 最近没有过流、过压或过温故障；
- 标定电压和电流限制低于正式运行限值。

### 5.2 ALIGN

- 以受限电压施加固定电角度；
- 等待转子稳定；
- 记录初始Hall状态；
- 超时、电流异常或Hall非法立即退出。

### 5.3 FORWARD_SCAN

- 以低电频率递增开环角度；
- 每个合法Hall状态变化时记录：前状态、当前状态、开环电角度、DWT时间戳；
- 运行多个完整电周期，不使用单圈数据直接生成配置；
- 对同一物理边界使用环形平均，正确处理0/65535回绕；
- 收集期间继续执行过流、DRV和母线保护。

### 5.4 FORWARD_VALIDATE

必须满足：

- 只出现6个有效状态；
- 每个状态只有一个正向后继和一个正向前驱；
- 六步关系能够闭合回到起始状态；
- 不存在状态0、7或跨状态跳变；
- 每个边界达到最小样本数；
- 相邻物理边界角度接近60°，允许配置化安装误差；
- 各电周期顺序一致。

### 5.5 REVERSE_SCAN与REVERSE_VALIDATE

- 停止正向输出并等待转速降至安全范围；
- 重新对齐后以负电频率低速旋转；
- 验证反向序列严格等于正向序列的逆序；
- 验证正反扫描观察到的是同一组物理边界；
- 反向结果只用于验证，不单独保存重复角度表。

### 5.6 BUILD_CONFIG与WAIT_ACCEPT

- 根据正向扫描生成跳转表和边界角度；
- 填写极对数、格式版本和CRC；
- 生成候选配置，不立即覆盖当前有效配置；
- 输出完整结果、质量指标和失败原因；
- 用户执行accept后应用到RAM；执行save后再交由Flash参数模块持久化。

## 6. 正反转运行时角度处理

假设上一次状态为`previous_state`，本次状态为`current_state`：

### 正向合法跳转

```text
positive_next[previous_state] == current_state
边界角度 = positive_entry_angle_u16[current_state]
角度步长 = +frequency_step
```

### 反向合法跳转

```text
positive_next[current_state] == previous_state
边界角度 = positive_entry_angle_u16[previous_state]
角度步长 = -frequency_step
```

反向进入当前状态时经过的是正向进入前一状态的同一个物理边界，因此不需要第二套角度表。

## 7. 串口接口规划

### 7.1 统一流程控制

```text
motor commissioning start
motor commissioning status
motor commissioning abort
motor commissioning diff
motor commissioning accept
motor commissioning discard
motor commissioning save
```

- `start`启动完整自动流程；
- `status`只查询当前运行阶段和进度；
- 每个阶段完成或失败时主动打印结果，不要求用户再次查询result；
- `diff`统一查看活动参数与候选参数差异；
- `accept`统一应用完整流程或任意单项产生的候选参数；
- `discard`统一丢弃尚未生效的候选参数；
- `save`只保存已经accept的活动参数，不直接保存未确认的候选值。

### 7.2 单项触发

每个单项只提供一个启动命令，不为每一项重复设计`result`、`repeat`、`apply`和`discard`子命令：

```text
calibrate current_offset
identify resistance
identify inductance
calibrate hall sequence
calibrate hall angle
calibrate hall offset
calculate current_pi
test current_d
test current_q
test current_handover
```

统一行为：

1. 命令成功启动后返回`started`；
2. 执行期间可通过`motor commissioning status`查询进度；
3. 执行结束自动打印状态、测量条件、结果、样本数和离散度；
4. 成功结果只更新对应的候选字段；
5. 用户重复发送同一启动命令即可检查多次一致性；
6. 单项执行失败不清除其他已经存在的候选字段；
7. 所有单项共用`motor commissioning diff/accept/discard/save`。

示例：

```text
> identify inductance
OK identify inductance started

INFO identify inductance PASS ld=225 uH lq=206 uH frequency=600 Hz samples=10000

> identify inductance
OK identify inductance started

INFO identify inductance PASS ld=224 uH lq=207 uH frequency=600 Hz samples=10000

> motor commissioning diff
ld_uh: active=225 candidate=224
lq_uh: active=206 candidate=207

> motor commissioning accept
OK candidate parameters applied
```

多次一致性统计由模块内部自动累计。每次完成时同时打印本次值和本次上电以来的统计值，不增加`repeat <count>`命令：

```text
run=3 current=224 average=225 min=224 max=226 deviation=2 valid=1
```

### 7.3 参数纠正

自动结果异常时，允许通用候选参数接口单项纠正，不为辨识模块增加专用apply：

```text
motor param set phase_resistance_mohm 273
motor param set direct_inductance_uh 225
motor param set quadrature_inductance_uh 206
motor param set pole_pairs 4
motor param set hall_offset_u16 7482
```

Hall数组使用专用候选参数命令：

```text
motor hall sequence set 1 5 4 6 2 3
motor hall angle set 1 51938
```

修改后仍通过统一的`motor commissioning diff`检查，通过`motor commissioning accept`生效。

### 7.4 方向与运行命令

```text

motor direction
motor direction normal
motor direction reverse
```

运行指令使用逻辑方向：

```text
speed start 500
speed start -500
speed set 1000
speed set -1000
```

方向配置和标定结果只允许在READY且PWM关闭时修改。所有会驱动电机的单项任务共用同一个互斥状态机；已有任务运行时再次启动返回`commissioning_busy`。标定过程中除status和abort外，拒绝其他电机控制命令。

## 8. 安全约束

- 标定必须明确由用户命令启动，上电不得自动旋转；
- 标定前提示电机会正反两个方向转动；
- 要求机构可自由旋转、解除制动并移除危险负载；
- 使用独立的低电压、低电流、低频率和超时限制；
- 任意nFAULT、BIF、过流、非法Hall、母线异常或用户abort立即关闭PWM；
- 标定失败不得覆盖原有效配置；
- 未通过反向验证时不得开放正式反向闭环；
- 不允许通过软件适配掩盖Hall电源、地线或相线短路等接线错误。

## 9. 失败码规划

```c
typedef enum
{
    MOTOR_HALL_COMMISSIONING_OK = 0,
    MOTOR_HALL_COMMISSIONING_INVALID_STATE,
    MOTOR_HALL_COMMISSIONING_DRIVER_FAULT,
    MOTOR_HALL_COMMISSIONING_BUS_VOLTAGE_INVALID,
    MOTOR_HALL_COMMISSIONING_OVERCURRENT,
    MOTOR_HALL_COMMISSIONING_ALIGNMENT_TIMEOUT,
    MOTOR_HALL_COMMISSIONING_ROTATION_TIMEOUT,
    MOTOR_HALL_COMMISSIONING_ILLEGAL_HALL_STATE,
    MOTOR_HALL_COMMISSIONING_ILLEGAL_TRANSITION,
    MOTOR_HALL_COMMISSIONING_INCOMPLETE_SEQUENCE,
    MOTOR_HALL_COMMISSIONING_SECTOR_ANGLE_INVALID,
    MOTOR_HALL_COMMISSIONING_REVERSE_MISMATCH,
    MOTOR_HALL_COMMISSIONING_ABORTED
} motor_hall_commissioning_result_t;
```

## 10. 开发顺序

1. 将Hall合法跳转表和边界角度改为可注入运行配置；
2. 让现有Hall端口测速不再依赖固定编译期顺序；
3. 改造Hall角度估算器支持正负方向步长和反向边界选择；
4. 增加`direction_inverted`逻辑方向映射；
5. 增加自动标定采集器和候选配置校验；
6. 增加串口调试命令；
7. 接入参数校验和Flash双槽保存；
8. 最后进行低功率正反向实物验证。

## 11. 验证标准

### 不依赖实物的验证

- 使用六种相线排列、六种Hall排列和方向反转组合生成模拟状态序列；
- 正向序列均能生成闭合跳转表；
- 反向序列均能被识别为正向逆序；
- 正反边沿选择指向同一物理边界；
- 角度跨0/65535时环形平均正确；
- 非法状态、重复状态、缺失状态和跨状态跳变均被拒绝；
- 旧配置在候选标定失败后保持不变；
- CRC或版本错误的Flash配置不会被加载。

### 需要实物的验证

- 任意相线和Hall排列下完成低功率正反扫描；
- 六个扇区角度、宽度和重复性满足阈值；
- 正反转Hall估算角度连续且方向正确；
- `direction normal/reverse`只改变逻辑方向定义；
- 标定中断、DRV故障、Hall断线和过流均能立即关闭PWM；
- 应用并重启后能够从Flash恢复相同标定结果。

## 12. 当前状态

本文档为方案设计，尚未实现`motor_hall_commissioning`。现有工程仍使用固定Hall正向跳转表、固定六边沿角度和仅正向Hall角度估算器；实施时必须保持默认不开PWM，并在完成实物验证前继续禁止正式反向闭环。

## 13. 完整新电机适配流程

### 13.1 自动化边界

参数分为三类：

1. 用户输入：额定电压、电流、峰值电流、额定/最高转速、极对数、NTC型号和机械安全方向等不能可靠通过短时电气试验获得的参数；
2. 自动辨识：ADC零偏、Rs、Ld/Lq、Hall顺序、Hall边界角度和机械转速换算等具备确定测量依据的参数；
3. 半自动调试：Hall转子角度补偿、电流环验证、速度PI和保护阈值验证需要软件执行流程、输出结果，并由用户确认是否接受。

所有自动辨识结果先进入候选配置。单项失败或用户未执行accept时，不允许覆盖当前有效配置。

### 13.2 阶段一：额定参数和极对数

用户必须先提供：

```text
额定电压
额定连续电流
峰值电流及允许持续时间
额定转速
最高机械转速
极对数
电机NTC型号
是否允许反转
```

极对数可以通过人工输入，或在具有机械一圈参考的条件下辅助统计Hall电周期数。仅凭Hall信号而没有机械一圈参考，软件不能可靠区分极对数。

### 13.3 阶段二：U/V/W和HA/HB/HC接线预检

- Hall供电与地线必须按定义连接，不允许自动适配错误的电源接线；
- 三根相线可以任意排列接入逆变器三个输出；
- 三根Hall信号可以任意排列接入三个数字输入；
- 静止状态检查Hall不得长期为0或7；
- 检查三相输出无短路、DRV无故障、母线电压处于调试范围；
- 软件暂不判断机构正方向，标定所得电角度递增方向定义为默认电气正方向。

### 13.4 阶段三：ADC零偏校准

在EN_GATE关闭、PWM关闭且确认相电流为零时，阻塞采集ADC样本：

```text
输入：样本数、超时时间
输出：A/B相零偏、样本数、有效标志
```

检查内容：

- 两路零偏在标称值容差范围内；
- 两路零偏差值不超过配置上限；
- 采集期间不得出现DRV或ADC故障；
- 校准结果只更新运行时电流采样配置，不改变分流电阻和放大倍数。

该步骤当前已经在每次启动时自动执行。

### 13.5 阶段四：小电流相电阻Rs辨识

采用锁转子直流注入方式，逐步提高受限电压，使目标相电流达到辨识值：

```text
Rs = 稳态施加电压 / 稳态相电流
```

辨识要求：

- 使用较小目标电流和独立的辨识过流阈值；
- 达到目标后等待电流稳定，再对电压和电流取平均；
- 超过最大注入电压仍达不到最小电流时失败；
- 结果单位统一为单相等效电阻mΩ；
- 星形电机万用表线间电阻通常约为两倍相电阻，比较时必须统一定义；
- 多次辨识离散度超限时拒绝接受结果。

当前工程已有阻塞式Rs辨识接口，但曾出现自动结果与万用表线间测量差异较大的情况。在重新确认电压定义、电流路径及线间/相电阻关系前，自动结果只能作为候选值，不能自动写入正式配置。

### 13.6 阶段五：锁转子Ld/Lq辨识

在转子保持静止且电流零偏有效时，对d轴、q轴分别施加受限交流电压，依据电流响应估算阻抗并扣除Rs分量：

```text
|Z| = 电压幅值 / 电流幅值
X = sqrt(|Z|² - Rs²)
L = X / (2πf)
```

辨识要求：

- 使用已确认的Rs候选值；
- 注入频率、电压、稳定样本数和测量样本数均配置化；
- D/Q轴分别统计电流幅值；
- 过流、PWM关闭、转子明显转动或信号幅值过低时失败；
- 可在多个电角度重复辨识，判断凸极电机的Ld/Lq差异；
- 结果单位统一为µH并记录测试频率、母线电压和样本数。

当前工程已有阻塞式Ld/Lq辨识接口，结果仍需人工确认并写入配置。

### 13.7 阶段六至八：低电压开环与Hall标定

执行顺序：

1. 固定角度低电压对齐；
2. 低电频率正向开环；
3. 自动识别Hall六步正向关系；
4. 记录并环形平均六个物理边界角度；
5. 低电频率反向扫描，验证正向序列逆序；
6. 生成Hall运行配置；
7. 标定转子坐标补偿，使Id、Vd、母线电流和振动处于合理范围。

Hall顺序和边界角度可以自动采集。转子角度补偿初版采用半自动方式：软件按受限范围给出候选角度和测量结果，用户确认后应用；后续再考虑自动搜索最小Id或最小母线功率点。

### 13.8 阶段九：低电流Id闭环验证

根据候选Rs、Ld和Lq自动计算电流PI初值：

```text
Kp_d = 2π × 电流环带宽 × Ld
Kp_q = 2π × 电流环带宽 × Lq
Ki   = 2π × 电流环带宽 × Rs
```

再转换为当前10 kHz离散PI使用的Q15参数。验证时固定安全电角度，给出较小Id指令并检查：

- Id平均值跟随目标；
- Iq平均值接近0；
- 峰值电流不超过独立测试阈值；
- PI输出未持续饱和；
- 测试结束自动关闭PWM。

PI参数可以由辨识值自动计算，但未经该步骤验证不得直接用于高电流运行。

### 13.9 阶段十：低Iq正向转矩验证

- 从低电压开环建立稳定Hall角度；
- 无扰切换到电流环；
- Id目标保持0，Iq从0缓慢增加到低电流测试值；
- 检查机械方向、Hall方向、Iq符号、Id偏差、电流峰值和电压饱和；
- 如果方向与设备定义不一致，只修改`direction_inverted`，不要求交换相线。

### 13.10 阶段十一：开环到电流环切换验证

验证切换前后的：

- 电角度连续性；
- Vd/Vq预置连续性；
- 三相占空比无异常跳变；
- 电流峰值不超过阈值；
- Hall估算器在正反方向均保持有效。

切换失败立即关闭PWM，不允许自动重复尝试。

### 13.11 阶段十二：速度PI调试

速度PI输出单位为Iq mA，输入和反馈单位为milli-rpm，执行频率固定为1 kHz。调试需要真实电机和负载惯量，不能仅依据Rs/Ld/Lq可靠自动完成。

建议流程：

- 从低Iq限幅、低目标速度和缓慢加速度开始；
- 先增加Kp获得基本跟随，再增加Ki消除稳态误差；
- 观察速度超调、Iq饱和、机械振动和Hall测速波动；
- 将最终Kp/Ki作为对应电机及负载的候选参数；
- 更换负载惯量后允许重新调试速度PI，而不重复Rs/Ld/Lq辨识。

### 13.12 阶段十三：逐级提高电流和电压限制

该步骤必须实物验证，不允许自动一次性放开：

```text
电流：低测试值 → 额定范围 → 经批准的峰值范围
电压：15% → 30% → 50%
```

48 V母线下当前线性SVPWM正常调制度限制最大为50%，dq硬电压理论边界为27700 mV。每一级均检查电压饱和、电流波形、母线电流、MOS温度、电机温度、噪声和振动。

### 13.13 阶段十四：保护验证

必须分别验证：

- 堵转：低速、高Iq持续达到配置时间后停机；
- Hall丢失：超过边沿超时立即退出闭环；
- 反向异常：只允许正向的模式检测到反向时停机；
- 超速：超过机械阈值立即停机；
- 软件过流：连续样本达到阈值后停机；
- DRV nFAULT和TMR1 BIF：硬件与软件均关闭PWM；
- 欠压、过压、MOS过温和绕组过温；
- 故障锁存、查询、清除条件及重新启动行为。

温升、额定连续电流和峰值持续时间不能通过短时自动辨识得出，必须依据电机手册和整机散热实测确定。

## 14. 完整调试状态机建议

Hall标定状态机作为完整调试状态机的子流程：

```text
IDLE
  → USER_PARAMETER_CHECK
  → ADC_OFFSET_CALIBRATION
  → RESISTANCE_IDENTIFICATION
  → INDUCTANCE_IDENTIFICATION
  → HALL_COMMISSIONING
       → PRECHECK
       → ALIGN
       → FORWARD_SCAN
       → REVERSE_SCAN
       → BUILD_CONFIG
  → CURRENT_D_AXIS_TEST
  → CURRENT_Q_AXIS_TEST
  → CURRENT_HANDOVER_TEST
  → WAIT_USER_ACCEPT
  → COMPLETE

任意异常 → FAULT → PWM关闭 → 保留原有效配置
```

速度PI调试、电流电压放开、最高转速和温升验证不纳入无人值守自动流程，必须由用户分阶段明确启动。

## 15. 参数输出与保存

完整候选参数至少包括：

```c
typedef struct
{
    uint8_t pole_pairs;
    uint8_t direction_inverted;
    uint16_t reserved;

    uint32_t phase_resistance_mohm;
    uint32_t direct_inductance_uh;
    uint32_t quadrature_inductance_uh;

    int32_t current_d_kp_q15;
    int32_t current_q_kp_q15;
    int32_t current_ki_q15;

    int32_t speed_kp_q20;
    int32_t speed_ki_q20;

    motor_hall_config_t hall;
} motor_identified_config_t;
```

额定电流、峰值电流、最大转速、NTC参数和保护温度属于用户/产品安全配置，不应被自动辨识结果覆盖。保存前必须分别校验“自动辨识参数”和“产品安全参数”。

## 16. 分提交实施规划

实施保持当前默认参数、正向控制和上电不开PWM行为兼容。每个提交独立编译、可审查、可回退；需要实物的能力只实现代码并标记待验证。

### 16.1 提交1：新增运行时电机参数与候选配置

新增：

```text
project/foc/foc_app/motor_parameter.c
project/foc/foc_app/motor_parameter.h
```

建立三层参数：

```text
默认参数：来自motor_control_config.h
活动参数：当前控制器实际使用
候选参数：辨识或人工修改、尚未生效
```

首批运行参数包含：

```text
pole_pairs、direction_inverted
Rs、Ld、Lq
电流PI Kp_d/Kp_q/Ki
速度PI Kp/Ki
Hall正向跳转表和边界角度表
```

核心接口：

```c
motor_parameter_init()
motor_parameter_active_read()
motor_parameter_candidate_read()
motor_parameter_candidate_field_set()
motor_parameter_candidate_accept()
motor_parameter_candidate_discard()
motor_parameter_diff_read()
motor_parameter_validate()
```

约束：

- 单项辨识只修改自己负责的候选字段；
- `accept`统一应用候选变更，`discard`统一丢弃；
- 活动参数运行中禁止修改；
- 额定能力和产品安全参数不能被自动辨识覆盖；
- 初始化后的活动值与当前编译宏完全一致；
- 本提交不包含Flash。

基础串口：

```text
motor params active
motor params candidate
motor commissioning diff
motor commissioning accept
motor commissioning discard
motor param set <name> <value>
```

验收：默认值一致、候选值不影响活动值、accept/discard正确、非法参数被拒绝、现有控制行为不变。

建议提交标题：`新增运行时电机参数与候选配置`。

实施状态（2026.09.04）：已完成。

- 已新增`motor_parameter.c/.h`，建立默认、活动、候选三层参数；
- 已实现单字段修改、整组候选替换、完整校验、差异查询、accept和discard；
- Hall跳转表会校验1至6状态唯一、单一前驱及六步闭环，非法候选值不会写入；
- `accept`仅允许在控制状态为READY且PWM关闭时执行；
- 已增加本节规划的基础串口命令，候选变更不会隐式影响活动参数；
- 已将当前Hall正向序列统一定义到`motor_control_config.h`；
- 已加入Keil工程并通过43个工程源文件ARMCC逐文件编译；
- 本阶段尚未迁移现有控制模块的宏参数读取。活动参数只是后续Hall解码器、双向估算器及辨识状态机的统一数据源，因此本提交保持现有上板运行行为不变。

### 16.2 提交2：解耦Hall端口并新增运行时解码器

新增：

```text
project/foc/foc_kernel/motor_hall_decoder.c
project/foc/foc_kernel/motor_hall_decoder.h
```

职责调整：

```text
motor_hall_port：只采集原始状态、时间戳和边沿计数
motor_hall_decoder：按活动Hall表判断合法跳转、方向和速度
```

从`motor_hall_port.c`移除固定六步表。解码器继续使用最近6个合法边沿间隔滑动测速，并正确处理重复中断、非法电平、跨状态跳变、方向切换和DWT回绕。

兼容策略：默认加载当前电机Hall表；迁移期间保留旧Hall快照接口作为适配层，所有调用迁移后再删除。

模拟验收：当前正向序列、逆向序列、六种Hall排列、状态0/7、重复状态、非法跳变、方向切换和计数器回绕。

建议提交标题：`解耦Hall端口并新增运行时解码器`。

### 16.3 提交3：支持双向Hall角度估算与逻辑方向配置

改造：

```text
motor_hall_angle_estimator
motor_speed_feedback
motor_speed_control
motor_control
motor_cli
```

正向边沿：

```text
positive_next[previous] == current
边界角度 = entry_angle[current]
角度步长为正
```

反向边沿：

```text
positive_next[current] == previous
边界角度 = entry_angle[previous]
角度步长为负
```

增加逻辑方向：

```text
physical_command = logical_command × direction_sign
logical_feedback = physical_feedback × direction_sign
```

串口：

```text
motor direction
motor direction normal
motor direction reverse
speed start/set <signed_rpm>
```

方向配置只允许在READY且PWM关闭时修改。实物反转未验证前保留`reverse_control_verified=false`安全门，不允许正式反向PWM启动。

软件验收：正向角度递增、反向角度递减、正反边界一致、角度回绕正确、方向逻辑换算正确、运行中修改方向被拒绝。

建议提交标题：`支持双向Hall估算与逻辑方向配置`。

### 16.4 提交4：新增Hall自动标定纯算法及模拟测试

新增：

```text
project/foc/foc_kernel/motor_hall_calibration.c
project/foc/foc_kernel/motor_hall_calibration.h
tests/motor_hall_calibration_test.c
```

纯算法接收前后Hall状态、开环电角度、时间戳和扫描方向，不访问GPIO、PWM或AT32寄存器。输出正向跳转表、六个边界角度、样本数、扇区宽度、最大偏差和正反验证结果。

能力：

- 不预设Hall顺序；
- 自动发现6个有效状态并生成唯一后继；
- 检查六状态闭环；
- 环形平均跨0°边界；
- 反向扫描验证正向逆序；
- 异常时不产生有效候选配置。

模拟覆盖所有相线/Hall排列对应的序列组合，以及缺状态、0/7、重复、跨状态、多后继、角度跨零、扇区异常和事件丢失。

建议提交标题：`新增Hall自动标定算法与模拟测试`。

### 16.5 提交5：新增完整电机辨识状态机和串口接口

新增：

```text
project/foc/foc_app/motor_commissioning.c/.h
project/foc/foc_kernel/motor_current_pi_design.c/.h
```

复用现有模块：

```text
motor_current_calibration
motor_resistance_identification
motor_inductance_identification
motor_open_loop
motor_current_loop_test
motor_torque_loop_test
```

完整状态机：

```text
IDLE → USER_PARAMETER_CHECK → ADC_OFFSET_CALIBRATION
→ RESISTANCE_IDENTIFICATION → INDUCTANCE_IDENTIFICATION
→ HALL_PRECHECK → HALL_ALIGN → HALL_FORWARD_SCAN
→ HALL_REVERSE_SCAN → BUILD_PARAMETER
→ CURRENT_D_AXIS_TEST → CURRENT_Q_AXIS_TEST
→ CURRENT_HANDOVER_TEST → WAIT_ACCEPT → COMPLETE

任意异常 → FAULT → PWM关闭 → 保留原活动参数
```

完整流程命令：

```text
motor commissioning start/status/abort/diff/accept/discard
```

单项只保留一个启动命令：

```text
calibrate current_offset
identify resistance
identify inductance
calibrate hall sequence
calibrate hall angle
calibrate hall offset
calculate current_pi
test current_d
test current_q
test current_handover
```

统一行为：启动返回`started`，完成或失败自动打印结果；重复发送同一命令用于一致性检查；模块自动累计本次值、平均值、最小值、最大值和离散度；成功时只更新对应候选字段。所有单项共用`diff/accept/discard`，不增加每项的`result/repeat/apply/discard`命令。

人工纠正通过通用候选参数命令：

```text
motor param set <name> <value>
motor hall sequence set ...
motor hall angle set <state> <angle_u16>
```

Rs特殊门槛：因现有结果与万用表线间测量曾有明显差异，第一版只产生候选值，必须打印相电阻定义、注入电压、电流、样本数和离散度，禁止自动accept。

电流PI设计器根据候选Rs/Ld/Lq、带宽和10 kHz采样频率计算Q15参数，只写候选区，必须经过低电流测试才能接受。

执行模型优先采用非阻塞状态机：主循环可持续响应`status`和`abort`，10 kHz采样留在ADC中断，每个阶段独立超时，同一时刻只允许运行一个驱动电机的任务。

建议提交标题：`新增电机辨识状态机与单项调试命令`。

### 16.6 提交6：新增参数Flash双槽保存

固定参数区域使用A/B双槽。每槽包含：

```text
magic、format_version、length、sequence
parameter_data、crc32、commit_marker
```

写入流程：

```text
选择非当前槽 → 擦除 → 写头和参数 → 写CRC
→ 读回校验 → 最后写commit_marker
```

启动时检查两个槽，选择CRC有效且序号最新的参数；无有效槽或参数范围检查失败时使用编译默认参数。

串口：

```text
motor commissioning save
motor params load
motor params defaults
```

规则：`save`只保存已accept的活动参数；候选参数不能直接保存；`load/defaults`只允许在READY且PWM关闭时执行；Flash失败不影响当前RAM参数。

软件验证覆盖空Flash、单槽有效、双槽序号、CRC错误、未提交槽、写入中途掉电、序号回绕、旧版本、长度变化和参数超范围。

建议提交标题：`新增电机参数Flash双槽保存`。

### 16.7 每个提交的共同门槛

每个提交前必须完成：

```text
ARMCC全部工程源文件编译
Keil工程XML检查
git diff --check
README修改记录
版本号递增
中文提交标题和修改点
```

纯算法提交必须执行模拟测试。涉及实物但本阶段未验证的能力，在README中明确标记“代码已实现，待上板验证”，不得记录为PASS。

### 16.8 实施顺序

```text
提交1：参数基础
提交2：Hall解码基础
提交3：双向运行基础
提交4：自动标定算法
提交5：完整辨识编排和单项命令
提交6：Flash持久化
```

第一轮只实施提交1，完成后先审查运行参数结构、候选参数语义和串口输出，再进入Hall端口解耦。
