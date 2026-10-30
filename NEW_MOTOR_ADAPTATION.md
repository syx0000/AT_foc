# AT_foc_hall 新电机适配指导

适用基线：`a9c2e19` 后的 24 V 电机编译默认配置，目标电机 `60BLDC140-24030-14J30`。本文描述现有软件的配置与上板顺序；实机参数辨识与带载验证尚未完成。

## 1. 先弄清哪些配置已经生效

固件编译默认已采用此款 24 V 电机参数，上位机已移除单独的候选预设按钮。启动仍优先读取合法 Flash 参数；已有设备需停机后恢复默认、检查并保存，恢复前记录原参数和 Hall 标定。

参数分三层：候选用于编辑和辨识；接受后成为 RAM 活动参数；保存后才会在下次上电加载。加载配置会从活动值重新生成候选，覆盖尚未接受的候选修改，因此应在辨识之前执行。

相关入口：

- 编译默认与保护：[motor_control_config.h](project/config/motor_control_config.h)
- 新电机候选初值：[motor_profile_60bldc140_24v.h](project/config/motor_profile_60bldc140_24v.h)
- 参数结构：[motor_parameter.h](project/foc/foc_app/motor_parameter.h)
- 上位机：[使用说明](tools/motor_host/README.md)，可执行文件 `tools/motor_host/dist/AT_foc_hall_MotorHost.exe`
- 电机原始资料：[新电机手册](60BLDC140-24030-14J30.pdf)

## 2. 电气参数口径

手册线间电阻为 0.16 Ω、线间电感为 0.1 mH。本工程输入的是 FOC 等效星形相参数，初值取 `Rs=80 mΩ`、`Ld=Lq=50 µH`。这是将端子测量换算为等效模型的初始假设，实际 Ld/Lq 仍需辨识，不能把线间值原样填入相参数。

电机为 8 极，即 4 对极；额定电压 24 V、额定转速 3000 rpm。手册额定电流 25.6 A 的测量口径尚未明确，不能直接等同于本工程 Iq 或母线电流。35 A 是目前的软件速度环限流配置，不代表确认可长期输出的电流。

## 3. 本工程参数配置表

表内电气和速度初值现已作为编译默认值，不代表所有设备当前活动值。表内“继承活动值”描述兼容 CLI profile 的行为；恢复默认会同时恢复编译默认的方向、Hall、磁链和速度 PI。以 `motor params active` 实际回读为准。

### 电机模型、电流环与速度环

| CLI 参数名 | 单位/格式 | 新电机候选初值 | 含义与调整说明 |
|---|---|---:|---|
| `pole_pairs` | 对极数 | 4 | 影响机械转速与电频率换算；8 极填 4 |
| `direction_inverted` | 0/1 | 继承活动值 | 逻辑正方向；先核对接线、Hall 顺序，再确定整机正方向 |
| `phase_resistance_mohm` | mΩ，相电阻 | 80 | 来自线电阻除以 2；以 Rs 辨识结果更新 |
| `direct_inductance_uh` | µH，d 轴 | 50 | 临时取线间电感的一半；辨识后更新 |
| `quadrature_inductance_uh` | µH，q 轴 | 50 | 暂按 SPMSM、Ld=Lq；以实际辨识为准 |
| `current_loop_bandwidth_hz` | Hz | 888 | 是频率 Hz，非 rad/s；更新 R/L 后重新计算 PI |
| `current_d_kp_q15` | Q15 整数 | 9141 | d 轴比例增益；优先用 `calculate current_pi` 生成 |
| `current_q_kp_q15` | Q15 整数 | 9141 | q 轴比例增益；Ld/Lq 不同则可能与 d 轴不同 |
| `current_ki_q15` | Q15 整数/周期 | 914 | 当前 16 kHz 采样下的离散积分增益 |
| `flux_linkage_uwb` | µWb | 继承活动值 | 作为新电机当前可用初值，后续重新辨识；参与滑行启动反电势前馈 |
| `speed_kp_q20` | Q20 整数 | 继承活动值 | 速度比例增益；编译默认 4800，按负载重新整定 |
| `speed_ki_q20` | Q20 整数/速度周期 | 继承活动值 | 速度积分增益；编译默认 64，先稳住比例再逐步增加积分 |
| `speed_maximum_rpm` | rpm | 3000 | 速度指令绝对值上限；编译默认 3000 |
| `speed_acceleration_rpm_per_s` | rpm/s | 30000 | 加速参考斜率；限流且速度落后时会暂停推进 |
| `speed_deceleration_rpm_per_s` | rpm/s | 30000 | 独立减速参考斜率；需确认电源能吸收再生能量 |
| `speed_startup_iq_slew_ma_s` | mA/s | 200000 | 即 200 A/s；启动和提高限流时平滑增加 Iq，降低限流立即生效 |
| `speed_current_limit_ma` | mA，Iq 限幅 | 35000 | 速度 PI 输出限幅；首次试转临时改为 2000，验证后逐级提高 |

PI 设计使用 `Kp=2πfL`、`Ki_discrete=2πfR/Fs`，`Fs=16000 Hz`，再转换为本工程 Q15。不能将这里的整数直接复制到 demo 的 PI 宏。

### Hall、辨识与通信

| 参数/数据 | 单位或候选来源 | 说明 |
|---|---|---|
| `hall_positive_next[8]` | 继承活动表；通过 sequence 标定 | 正向六状态闭环关系，0/7 非法；换电机后重新确认 |
| `hall_entry_angle_u16[8]` | 继承活动表；通过 angle 标定 | 六个边沿的电角度，65536 对应 360°电角；正反向查询表由同一套边界生成 |
| `hall_rotor_offset_u16` | 继承活动值；编译默认 448 | Hall 角与转子磁场的整体电角偏移，不能只靠电机极对数推算 |
| `hall_delay_us` | µs；继承，编译默认 75 | Hall 动态延迟补偿，先完成静态标定，再按高速电流/角度波形微调 |
| `hall_scan_mv` | mV；默认 1200，范围 100～4800 | Hall 开环扫描电压；从能平顺转动的低值开始，不具备专用闭环限流 |
| `hall_scan_mhz` | mHz 电频率；默认 5000，范围 1000～20000 | 5000 表示 5 Hz 电频率；不是 5000 Hz，也不是机械 rpm |
| `offset_iq_ma` / `flux_iq_ma` | mA；默认 10000，范围 1～20000 | 分别用于 Hall offset / 磁链辨识；首次按负载降低，不能以速度环 2 A 限流代替辨识限流 |
| `offset_voltage_pct` / `flux_voltage_pct` | 母线百分比；默认 30，范围 1～57 | 相应辨识电压上限之一 |
| `offset_voltage_mv` / `flux_voltage_mv` | mV；默认 8400，范围 100～27700 | 与百分比上限取较小者，24 V 下默认实际上限为 7.2 V |
| `offset_settle_ms` / `flux_settle_ms` | ms；默认 2000，范围 100～10000 | 相应辨识等待时间；等待结束不等于实际已经稳速 |
| `offset_measure_ms` / `flux_measure_ms` | ms；默认 4000，范围 100～10000 | 相应辨识采集窗口 |
| `startup_timeout_ms` | ms；默认 3000，范围 500～15000 | 电流启动路径超时；不是速度准备阶段的超时设置 |
| `can_node_id` | 十进制 1～127；默认 1 | ZXQ 左右电机通常分别为 1、2；修改并接受后对端立即改用新 ID |

`startup_align_*`、`startup_if_max_ma`、`startup_ramp_mhz_s` 是兼容保留的启动字段，正常 Hall 速度启动不靠这些参数调节。`stall_speed_rpm` / `stall_time_ms` 已不执行速度堵转停机，不作为保护配置。

### 固定硬件和保护参数（不在上述运行时参数页面修改）

| 配置 | 当前值 | 适配时的解释 |
|---|---:|---|
| 电流采样 | 2.5 mΩ 分流、CSA ×10、3.3 V ADC | 必须匹配实际板卡；换电机不应改变采样倍率 |
| PWM / 速度环 | 16 kHz / 1 kHz | 改频率需要同步重新计算 PI 和相关时间参数 |
| 母线欠压 | 18 V，连续 300 ms | 当前运行欠压保护，24 V 电源仍需确认压降 |
| 母线过压 | 60 V | 全局硬件平台阈值，不能作为 24 V 电机的允许供电电压 |
| MOS 温度 | 90°C，恢复门限 80°C | 检测的是板上 MOS 温度，不是电机绕组温度 |
| 相电流软件保护 | ±55 A，连续 30 次越窗 | 16 kHz 下约 1.875 ms；与 35 A Iq 运行限流不同，硬件 Break 另行保留 |
| 超速 | 4400 rpm | 独立于 3000 rpm 指令限幅，不能依靠它限制正常工作转速 |

## 4. 实际适配顺序

### 第一步：记录原配置，检查电源与接线

使用 24 V 可限流电源，确认母线分压、电流采样零点、三相及 Hall 接线。电机脱离负载并固定，减速时确认供电系统有再生能量处理能力。串口为 `921600, 8N1`。

先停止电机、暂停 ZXQ 周期运行命令，记录 `version`、`status`、`motor params active`、`motor params storage`。不能只断开串口代替停机。

### 第二步：装入初值并降低首次试转电流

新设备没有有效 Flash 时已使用新编译默认。已有设备若要保留活动参数中的 Hall、磁链和速度 PI，可使用下面保留兼容的 CLI profile 命令；上位机已移除该预设按钮，可从原始终端发送。若选择 `motor params defaults`，则会同时重置这些标定项，须事先备份，重新核对后再保存。

在 READY、PWM 关闭时逐条执行，等待每条成功回显：

```text
motor stop
motor params profile 60bldc140-24v
motor param set speed_current_limit_ma 2000
motor params candidate
motor commissioning diff
```

这一步仍未使候选生效。上位机“参数差异”“接受候选”“保存参数”分别对应后续不同操作；不要连续盲点。

### 第三步：静态辨识及 Hall 标定

逐项执行，每项结束后检查状态、结果和候选差异，失败先排查，不继续后面的运动辨识：

```text
calibrate current_offset
identify resistance
identify inductance
motor param set current_loop_bandwidth_hz 888
calculate current_pi
calibrate hall sequence
calibrate hall angle
motor commissioning status
motor commissioning diff
```

Rs/Ld/Lq 辨识结果已经是本工程相参数，不要再次除以 2。Hall 标定会驱动电机，扫描参数需要在开始前设置；不能用旧 Hall 表带载试转。中止使用 `motor commissioning abort`，随后检查故障和 PWM 状态。

### 第四步：辨识 Hall offset 与磁链

先核对候选中的 Hall 顺序、角度和电流 PI；按空载转动需要设置 `offset_*`、`flux_*`。依次运行：

```text
calibrate hall offset
identify flux
motor commissioning status
motor commissioning diff
```

两项运动辨识会临时使用候选参数，结束后恢复原活动参数，因此辨识成功仍需接受。磁链继承值可作为当前初值使用，最终用实机辨识结果更新；不要混用线间反电势 RMS、峰值与相磁链口径。

### 第五步：接受、空载验证、保存

```text
motor commissioning accept
motor params active
speed start 100
speed stop
```

先以低电流低速验证，再逐级正向/反向试转，检查实测方向、速度、电流和温升。每次修改候选都需要重新检查并接受。低电流不能转动时先看 Hall、采样极性和电流环，不能直接升到 35 A。

验证静止启动、同向滑行启动、反向滑行启动、零限流及恢复、减速与换向。速度启动先进入 `SPEED_PREPARING=7`；合法 Hall 连续 1 秒无边沿才确认静止并预充，滑行则跳过低边预充。收到启动成功回显不等于已经进入闭环。

低电流验证通过后逐级提高 `speed_current_limit_ma`，最终是否使用 35000 由实机温升、负载及电机允许电流决定。加减速正式配置均为 30000 rpm/s、Iq 上升速率为 200000 mA/s；首轮若机械或母线承受不了，可临时降低，验证后记录最终值。

确认整套参数后执行并检查：

```text
motor stop
motor commissioning save
motor params storage
```

重启后再次读取活动参数，确认 Flash 回读一致。`motor params defaults` 现在会恢复新电机的编译默认，并重置原 Hall 等标定值；它直接更新 RAM 活动和候选参数，不会自动保存。Flash v7 兼容加载 v2～v6，但兼容加载不会自动把旧参数转换为新电机配置。

## 5. ZXQ 联调与交付记录

CAN 为 1 Mbps，确认两台节点 ID 分别匹配 ZXQ。先验证停止和零限流，再验证速度、电流限幅及反馈。ZXQ 可能请求 40～52 A，而本工程速度环默认限流 35 A，联调必须明确实际限幅口径；接收端故障字段解析也需单独核对。

堵转不再由速度误差超时自动停机：低速带载运行仍受相电流、驱动器、温度、欠压等保护约束。板上温度不能替代电机绕组温度验证。故障恢复成功只回 READY，不自动重启，应重新确认运行命令。

交付记录应包含电机型号、接线相序、Hall 表与 offset、实测 Rs/Ld/Lq/磁链、888 Hz 对应 PI、速度 PI、实际限流、斜率、节点 ID、固件版本和重启回读结果，并附启动/换向/负载电流波形。尚无实机验证环境时，这些项目标记“待验证”，不能以软件回归通过代替验收。
