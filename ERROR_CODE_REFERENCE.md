# AT_foc_hall 错误码与故障诊断表

更新日期：2026-09-04

## 1. 使用约定

- 本文覆盖应用层统一电机故障、故障恢复、自动辨识、底层控制/辨识状态、串口命令、参数存储、OTA及CPU异常。
- `fault`查询统一电机故障及最近5次RAM历史；历史记录不掉电保存。
- `motor commissioning status`查询自动辨识的`error + detail`两级错误。
- 枚举数字由源码顺序确定。已经通过串口输出的数字应保持兼容；新增错误应追加，禁止在中间插入或重排。
- ISR中禁止`printf`。软件过流和TMR1 Break先关断PWM并投递故障，由主循环锁存和打印。

## 2. 统一电机故障 `motor_fault_code_t`

来源：`project/foc/foc_app/motor_control.h`。这是运行控制对外使用的主故障码。

| 值 | 名称 | 串口名称 | 含义 | 恢复要求 |
|---:|---|---|---|---|
| 0 | `MOTOR_FAULT_NONE` | `none` | 无故障 | 无 |
| 1 | `MOTOR_FAULT_UNKNOWN` | `unknown` | 非法或无法识别的故障码 | 排除原因后可尝试清除 |
| 2 | `MOTOR_FAULT_CONTROL_SUBSYSTEM` | `control_subsystem` | 控制子系统未提供更具体原因 | 排查底层状态后清除 |
| 3 | `MOTOR_FAULT_READY_TRANSITION` | `ready_transition` | 启动阶段无法进入READY | 必须复位 |
| 4 | `MOTOR_FAULT_CURRENT_OFFSET_INVALID` | `current_offset_invalid` | ADC电流零偏结果无效 | 必须复位并重新校准 |
| 5 | `MOTOR_FAULT_CURRENT_CALIBRATION_TIMEOUT` | `current_calibration_timeout` | ADC零偏校准超时或请求无效 | 必须复位 |
| 6 | `MOTOR_FAULT_DRIVER_NOT_READY` | `driver_not_ready` | DRV未就绪、nFAULT有效或BIF无法清除 | 必须复位并检查硬件 |
| 7 | `MOTOR_FAULT_CURRENT_HANDOVER` | `current_handover` | 开环切换电流环失败 | 停机、排查Hall/电流反馈后清除 |
| 8 | `MOTOR_FAULT_SPEED_START` | `speed_start` | 速度环启动失败 | 检查开环、Hall和目标方向 |
| 9 | `MOTOR_FAULT_OPEN_LOOP` | `open_loop` | 开环控制器进入FAULT | 检查PWM和指令 |
| 10 | `MOTOR_FAULT_CURRENT_SAMPLE` | `current_sample` | 电流采样不可用 | 恢复采样并确认电流安全 |
| 11 | `MOTOR_FAULT_HALL` | `hall` | Hall状态或估算无效 | 恢复Hall反馈后清除 |
| 12 | `MOTOR_FAULT_OVERCURRENT` | `overcurrent` | 软件过流达到连续样本阈值 | PWM立即关闭；确认电流安全后清除 |
| 13 | `MOTOR_FAULT_PWM` | `pwm` | PWM输出或MOE状态异常 | 检查nFAULT/BIF后清除 |
| 14 | `MOTOR_FAULT_SPEED_FEEDBACK` | `speed_feedback` | 速度反馈失效 | 恢复Hall测速后清除 |
| 15 | `MOTOR_FAULT_SPEED_CURRENT_CONTROL` | `speed_current_control` | 速度环所依赖的电流环失效 | 先处理电流环故障 |
| 16 | `MOTOR_FAULT_SPEED_COMMAND` | `speed_command` | 速度指令非法 | 修正目标后清除 |
| 17 | `MOTOR_FAULT_REVERSE_DIRECTION` | `reverse_direction` | 请求了尚未验证的物理反向控制 | 完成反向验证或修正方向 |
| 18 | `MOTOR_FAULT_STALL` | `stall` | 堵转判定 | 去除负载/机械卡滞后清除 |
| 19 | `MOTOR_FAULT_OVERSPEED` | `overspeed` | 超速判定 | 转速降至安全范围后清除 |
| 20 | `MOTOR_FAULT_HARDWARE_BREAK` | `hardware_break` | TMR1 Break/nFAULT硬件保护触发 | 确认nFAULT为高并成功清除BIF |

故障记录字段：`first_code`锁存首故障，`last_code`记录同一事件最后新增故障，`code_mask`合并该事件的不同故障，`occurrence_count`统计不同故障种类，`sequence`为上电后的事件序号。

## 3. 故障清除结果 `motor_fault_clear_result_t`

| 值 | 名称 | 串口名称 | 含义/处理 |
|---:|---|---|---|
| 0 | `MOTOR_FAULT_CLEAR_OK` | `ok` | 已清除并进入READY |
| 1 | `MOTOR_FAULT_CLEAR_NOT_FAULTED` | `not_faulted` | 当前不在FAULT，无需清除 |
| 2 | `MOTOR_FAULT_CLEAR_PWM_ENABLED` | `pwm_enabled` | PWM仍开启，拒绝清除 |
| 3 | `MOTOR_FAULT_CLEAR_RESET_REQUIRED` | `reset_required` | 启动自检类故障，必须复位 |
| 4 | `MOTOR_FAULT_CLEAR_CURRENT_UNSAFE` | `current_unsafe` | 电流采样仍不安全或过流锁存未满足清除条件 |
| 5 | `MOTOR_FAULT_CLEAR_DRIVER_ACTIVE` | `driver_fault_active` | nFAULT仍有效或BIF重新置位 |

## 4. 自动辨识阶段错误 `motor_commissioning_error_t`

`error`指出失败阶段，`detail`保留该模块原始状态。`0=无错误、1=任务中止`保持原有含义。

| error | 名称 | 串口名称 | detail来源 |
|---:|---|---|---|
| 0 | `MOTOR_COMMISSIONING_ERROR_NONE` | `none` | `0=none` |
| 1 | `MOTOR_COMMISSIONING_ERROR_ABORTED` | `aborted` | `1=requested` |
| 2 | `MOTOR_COMMISSIONING_ERROR_CURRENT_OFFSET` | `current_offset` | 见4.1 |
| 3 | `MOTOR_COMMISSIONING_ERROR_RESISTANCE` | `resistance` | `motor_resistance_identification_status_t` |
| 4 | `MOTOR_COMMISSIONING_ERROR_INDUCTANCE` | `inductance` | `motor_inductance_identification_status_t` |
| 5 | `MOTOR_COMMISSIONING_ERROR_HALL_SCAN` | `hall_scan` | `motor_hall_calibration_error_t`及扫描扩展码 |
| 6 | `MOTOR_COMMISSIONING_ERROR_HALL_OFFSET` | `hall_offset` | 见4.5 |
| 7 | `MOTOR_COMMISSIONING_ERROR_CURRENT_PI` | `current_pi` | `1=pi_design_failed` |
| 8 | `MOTOR_COMMISSIONING_ERROR_CURRENT_D` | `current_d` | `1=test_failed` |
| 9 | `MOTOR_COMMISSIONING_ERROR_CURRENT_Q` | `current_q` | `motor_torque_loop_test_status_t` |
| 10 | `MOTOR_COMMISSIONING_ERROR_PARAMETER` | `parameter` | `1=candidate_update_failed`，`2=trial_or_restore_failed` |

### 4.1 ADC零偏 detail

| detail | 名称 | 含义 |
|---:|---|---|
| 1 | `calibration_failed` | 采样超时或校准失败 |
| 2 | `offset_apply_failed` | 零偏结果超出允许范围，无法应用 |

### 4.2 Rs辨识 detail

| detail | 名称 | 含义 |
|---:|---|---|
| 0 | `ok` | 成功 |
| 1 | `invalid_state` | 电流状态无效或PWM已占用 |
| 2 | `bus_voltage_invalid` | 母线电压无效或越界 |
| 3 | `pwm_enable_failed` | PWM无法安全开启 |
| 4 | `overcurrent` | 辨识电流超过安全限制 |
| 5 | `driver_fault` | DRV/nFAULT/Break触发 |
| 6 | `current_too_low` | 注入后有效电流过低，无法计算Rs |

### 4.3 Ld/Lq辨识 detail

| detail | 名称 | 含义 |
|---:|---|---|
| 0 | `ok` | 成功 |
| 1 | `invalid_argument` | 结果指针或相电阻参数无效 |
| 2 | `invalid_state` | PWM已占用等状态不允许 |
| 3 | `bus_voltage_invalid` | 母线反馈无效或为零 |
| 4 | `pwm_enable_failed` | PWM无法安全开启 |
| 5 | `overcurrent` | 注入期间电流超限 |
| 6 | `driver_fault` | MOE丢失或驱动保护触发 |
| 7 | `current_too_low` | 同步检波电流幅值为零/过低 |
| 8 | `impedance_invalid` | 阻抗不大于Rs或计算得到的Ld/Lq为零 |

### 4.4 Hall双向扫描 detail

`0..11`直接对应`motor_hall_calibration_error_t`。

| detail | 名称 | 含义 |
|---:|---|---|
| 0 | `ok` | 成功 |
| 1 | `invalid_argument` | 上下文或配置参数无效 |
| 2 | `invalid_state` | 调用顺序或扫描方向与状态不匹配 |
| 3 | `duplicate_state` | Hall状态未变化却被作为边沿提交 |
| 4 | `non_adjacent_state` | 相邻状态变化超过一个Hall位 |
| 5 | `multiple_successor` | 同一状态出现多个正向后继 |
| 6 | `timestamp_invalid` | 边沿时间戳/间隔无效 |
| 7 | `incomplete_sequence` | 六个合法状态或样本数不完整 |
| 8 | `sector_angle_invalid` | 扇区宽度超出配置范围 |
| 9 | `edge_deviation_exceeded` | 同一边沿角度重复性超差 |
| 10 | `reverse_mismatch` | 正反向边沿角不一致 |
| 11 | `sequence_error` | 六步跳转表不能形成闭环序列 |
| 100 | `port_sample_failed` | 无法读取当前Hall端口状态 |
| 101 | `open_loop_start_failed` | Hall扫描开环无法启动 |
| 102 | `scan_timeout` | 规定时间内未采满六状态样本 |
| 103 | `result_invalid` | 结果读取或候选参数准备失败 |

### 4.5 Hall转子补偿 detail

| detail | 名称 | 含义 |
|---:|---|---|
| 1 | `candidate_read_failed` | 无法读取候选电机参数 |
| 2 | `measurement_failed` | 低Iq测量或临时参数恢复失败 |
| 3 | `invalid_argument` | 补偿算法输入/门限无效 |
| 4 | `quadrature_voltage_too_low` | Vq幅值不足，结果不可信 |
| 5 | `correction_exceeded` | 计算修正角超过允许上限 |
| 6 | `candidate_update_failed` | 补偿结果无法写入候选参数 |

## 5. 底层运行控制故障

### 5.1 电流环 `motor_current_control_fault_t`

| 值 | 名称 | 统一映射 |
|---:|---|---|
| 0 | `MOTOR_CURRENT_CONTROL_FAULT_NONE` | 无 |
| 1 | `MOTOR_CURRENT_CONTROL_FAULT_SAMPLE` | `MOTOR_FAULT_CURRENT_SAMPLE` |
| 2 | `MOTOR_CURRENT_CONTROL_FAULT_HALL` | `MOTOR_FAULT_HALL` |
| 3 | `MOTOR_CURRENT_CONTROL_FAULT_OVERCURRENT` | `MOTOR_FAULT_OVERCURRENT` |
| 4 | `MOTOR_CURRENT_CONTROL_FAULT_PWM` | `MOTOR_FAULT_PWM` |

### 5.2 速度环 `motor_speed_control_fault_t`

| 值 | 名称 | 统一映射 |
|---:|---|---|
| 0 | `MOTOR_SPEED_CONTROL_FAULT_NONE` | 无 |
| 1 | `MOTOR_SPEED_CONTROL_FAULT_FEEDBACK` | `MOTOR_FAULT_SPEED_FEEDBACK` |
| 2 | `MOTOR_SPEED_CONTROL_FAULT_CURRENT_CONTROL` | `MOTOR_FAULT_SPEED_CURRENT_CONTROL` |
| 3 | `MOTOR_SPEED_CONTROL_FAULT_COMMAND` | `MOTOR_FAULT_SPEED_COMMAND` |
| 4 | `MOTOR_SPEED_CONTROL_FAULT_REVERSE_DIRECTION` | `MOTOR_FAULT_REVERSE_DIRECTION` |
| 5 | `MOTOR_SPEED_CONTROL_FAULT_STALL` | `MOTOR_FAULT_STALL` |
| 6 | `MOTOR_SPEED_CONTROL_FAULT_OVERSPEED` | `MOTOR_FAULT_OVERSPEED` |

### 5.3 Iq/接管测试 `motor_torque_loop_test_status_t`

| 值 | 名称 | 含义 |
|---:|---|---|
| 0 | `MOTOR_TORQUE_TEST_STATUS_OK` | 测试通过 |
| 1 | `INVALID_ARGUMENT` | 结果或PI参数无效 |
| 2 | `PWM_BUSY` | PWM已被其他模块占用 |
| 3 | `OPEN_LOOP_START_FAILED` | 开环起转失败 |
| 4 | `BOOTSTRAP_FAILED` | Hall接管条件失败 |
| 5 | `BOOTSTRAP_TIMEOUT` | 开环建立速度超时 |
| 6 | `SENSOR_INVALID` | 电流或慢速传感器状态无效 |
| 7 | `HALL_LOST` | Hall反馈丢失 |
| 8 | `OVERCURRENT` | 测试电流超限 |
| 9 | `PWM_FAULT` | PWM/MOE/DRV故障 |

说明：固定角度Id测试当前仅返回`bool`，因此自动辨识`current_d detail=1`仍是汇总错误，尚未细分采样、过流和PWM原因。

## 6. 串口命令错误

### 6.1 带数字的CLI错误

| 编号 | 文本 | 触发条件 |
|---:|---|---|
| 1 | `unknown_command` | 命令无法识别 |
| 2 | `line_too_long` | 接收行超过CLI缓冲区 |
| 3 | `invalid_parameter` | 参数缺失、格式错误或越界 |
| 4 | 保留未使用 | 禁止复用，避免后续兼容冲突 |
| 5 | `motor_not_ready_or_invalid_command` | 非READY或开环启动参数无效 |
| 6 | `open_not_running_or_invalid_command` | 开环未运行或更新值无效 |
| 7 | 保留未使用 | 禁止复用，避免后续兼容冲突 |
| 8 | `open_not_running_hall_invalid_or_bad_command` | 电流环接管前提不满足 |
| 9 | `current_not_running_or_invalid_command` | 电流环未运行或Id/Iq无效 |
| 10 | `open_not_running_or_speed_feedback_invalid` | 速度闭环启动条件不满足 |
| 11 | `speed_not_running_or_invalid_command` | 速度环未运行或目标无效 |
| 12 | `accept_requires_ready_pwm_off_and_valid_candidate` | 候选参数不可接受 |
| 13 | `direction_requires_ready_pwm_off_and_no_pending_diff` | 方向修改安全条件不满足 |

### 6.2 当前未编号的CLI错误

| 文本 | 含义 |
|---|---|
| `parameter_load_requires_ready_pwm_off_and_valid_slot` | 加载Flash参数的状态或槽有效性不满足 |
| `parameter_defaults_requires_ready_pwm_off` | 恢复默认参数的安全状态不满足 |
| `commissioning_not_running` | 无正在运行的辨识任务可中止 |
| `candidate_applied_but_hall_decoder_reload_failed` | 参数已应用，但Hall运行时解码器重载失败 |
| `parameter_save_requires_ready_pwm_off_or_flash_failed` | 保存条件不满足或Flash擦写/校验失败 |
| `fault_clear_denied reason=...` | 统一故障清除被拒绝，原因见第3节 |

未编号文本目前不属于稳定协议码；后续协议化时应分配独立编号，不能仅靠自然语言解析。

## 7. 参数存储状态

`motor params storage`中的`source`不是故障码，但`3`表示扫描/存储异常。

| source | 名称 | 含义 |
|---:|---|---|
| 0 | `MOTOR_PARAMETER_STORAGE_DEFAULTS` | 未找到有效Flash记录，使用默认参数 |
| 1 | `MOTOR_PARAMETER_STORAGE_SLOT_A` | 从A槽加载 |
| 2 | `MOTOR_PARAMETER_STORAGE_SLOT_B` | 从B槽加载 |
| 3 | `MOTOR_PARAMETER_STORAGE_ERROR` | Flash读取、记录校验或参数应用异常 |

`slot_a=0 slot_b=0`本身不一定是故障，空Flash首次启动时属于正常默认状态。

## 8. OTA可观察错误

OTA当前使用文本结果，没有独立数字错误枚举。

| 输出 | 含义 |
|---|---|
| `OTA_ERR already_active` | 已存在OTA会话 |
| `OTA_ERR size_invalid` | 固件长度为零或超过Staging区 |
| `OTA_ERR erase_header_failed` | Staging头擦除失败 |
| `OTA_ERR erase_staging_failed` | Staging数据区擦除失败 |
| `OTA_NAK seq_err` | 数据包序号与期望不符，可由发送端重传 |
| `OTA_NAK flash_err` | 数据块写Flash失败，进入`OTA_ERROR` |
| `OTA_FAIL flush_err` | 尾部不足4字节的数据刷新失败 |
| `OTA_FAIL crc32` | 整包CRC32不一致 |
| `OTA_FAIL hdr_write_err` | Staging有效头写入失败 |
| `BOOT: mark_stable erase_err` | App稳定标记所在扇区擦除失败 |
| `BOOT: mark_stable write_err` | App稳定标记回写失败 |

OTA状态：`0=OTA_IDLE`、`1=OTA_ERASING`、`2=OTA_RECEIVING`、`3=OTA_DONE`、`4=OTA_ERROR`。

## 9. CPU异常与诊断边界

| 异常 | 当前动作 | 当前限制 |
|---|---|---|
| `HardFault` | 关闭中断并紧急关闭门极/PWM，然后停留死循环 | 不进入RAM电机故障历史，无异常寄存器快照 |
| `MemManage` | 同上 | 同上 |
| `BusFault` | 同上 | 同上 |
| `UsageFault` | 同上 | 同上 |

这些异常不能在异常上下文直接调用`printf`。后续若需要定位，应增加`.noinit`崩溃快照，保存CFSR、HFSR、MMFAR、BFAR、PC、LR和SP，并在下次启动时打印。

## 10. 快速查询

```text
fault
fault clear
motor commissioning status
motor params storage
status
```

故障排查顺序：先看即时`LOGE`，再执行`fault`确认统一首故障和同事件位图；辨识失败再执行`motor commissioning status`读取失败步骤、`error`及`detail`。
