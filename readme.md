# 修改记录

2026.09.04.6  双向Hall估算与逻辑方向：Hall角度估算器正向按当前状态进入角校正，反向按当前状态正向后继的进入角校正，并使用有符号相位步长双向插值；
              新增无硬件依赖的方向变换与Hall边界角/相位步长算法，速度反馈按活动极对数和direction_inverted转换为逻辑有符号rpm；
              速度环统一接收有符号逻辑rpm，PI输出逻辑Iq后再转换为物理Iq，超速、堵转及方向检查改为双向对称，禁止运行中跨零反转；
              串口新增motor direction查询及normal/reverse设置，方向修改仅允许READY、PWM关闭且无其他候选差异时执行；
              MOTOR_REVERSE_CONTROL_VERIFIED默认保持0，未完成反向实物验证前禁止物理反向速度闭环；版本更新并完成46个工程源文件和2个测试源文件ARMCC编译。
2026.09.04.5  Hall端口与解码解耦：motor_hall_port仅采集三路GPIO组合、边沿计数和DWT时间戳，不再包含固定顺序、方向及测速业务；
              foc_kernel新增motor_hall_decoder，使用活动参数中的Hall正向表判断合法跳转和物理方向，并保留最近6段合法边沿滑动测速；
              解码器统一处理重复中断、0/7非法电平、跨状态跳变、方向切换及DWT计数回绕，快照使用序列锁避免主循环读取撕裂；
              Hall角度观察器、估算器、速度反馈及诊断日志统一改读解码结果，默认参数下保持原有正向序列与运行行为；
              新增纯算法测试用例，覆盖正反方向、六种Hall接线排列和异常路径；Keil工程加入解码器并完成44个工程源文件及测试源文件ARMCC编译。
2026.09.04.4  运行时电机参数管理：新增默认、活动和候选三层参数，覆盖极对数、逻辑方向、Rs/Ld/Lq、电流/速度PI及Hall跳转和边界角度；
              新增参数完整性与Hall六状态闭环校验，候选参数只有在电机READY且PWM关闭时才能accept，支持discard恢复活动值；
              串口新增motor params active/candidate、motor param set以及motor commissioning diff/accept/discard命令；
              Hall默认正向序列统一移入motor_control_config.h，消除参数模块与Hall端口的重复常量；
              Keil工程加入motor_parameter.c并完成43个工程源文件ARMCC逐文件编译。本阶段仅建立参数管理层，现有控制器继续使用编译配置，运行行为不变。
2026.09.04.3  电机自动辨识与标定方案：新增完整设计文档，规划任意U/V/W与HA/HB/HC接线适配、双向Hall角度估算和逻辑方向配置；
              明确ADC零偏、Rs、Ld/Lq、Hall顺序/边界、转子角度补偿、电流PI计算以及闭环测试的自动化边界和安全条件；
              串口同时支持完整流程和单项触发，单项完成后自动打印结果并统一复用候选参数diff/accept/discard/save；
              将实施拆分为运行参数、Hall解码、双向估算、纯算法标定、辨识状态机和Flash双槽保存6个独立提交。仅完成设计，功能待实施。

2026.09.04.2  Hall测速与速度闭环：Hall测速改为每个合法边沿更新，使用最近6段边沿间隔滑动平均并支持超时归零；
              新增4对极机械转速反馈、仅在新Hall测量到达时更新的IIR滤波，以及1 kHz Q20速度PI内核；
              新增正式motor_speed_control，将速度斜坡和PI输出的Iq mA指令接入现有10 kHz电流环，串口使用speed start/set <rpm>和speed stop；
              增加正向限定、Hall失效、反向、堵转和3300 rpm超速保护，初期Iq限制±2 A、加速度300 rpm/s；
              电流环硬电压上限按48 V线性SVPWM理论边界统一命名为27700 mV，当前正常调制度仍保持15%；
              自维护的板级、控制、版本及OTA配置统一至project/config，并增加关键参数编译期一致性检查。速度闭环待上板验证。

2026.09.04.1  串口命令与正式启动流程：新增USART1 RX DMA空闲帧接收端口和主循环文本CLI，关闭未配置的USART1 TX DMA；
              新增统一motor_control状态机，上电仅执行DRV检查及ADC零偏校准，成功后进入READY，不再自动辨识或试转；
              增加help、version、status、日志等级、停止、故障查询/清除以及开环启动/更新、电流闭环接管/更新命令；
              串口中断仅提交接收帧，命令解析和状态校验均在主循环执行，调试CLI电流指令暂限±2 A；
              新增4对极正式电机配置，后续机械转速统一由Hall电频率按极对数换算。待验证串口接收及命令控制链路。

2026.09.04.0  FOC控制架构规范化：将通用有符号斜坡和dq电压联合限幅下沉至foc_kernel，保持内核无硬件依赖；
              将固定开环重构为可配置控制器，支持Vd/Vq、正反目标电频率、加速度、对齐参数及运行中指令更新；
              新增正式motor_current_control，统一Hall角度电流闭环、双PI、联合限幅、抗饱和、SVPWM及故障停机；
              转矩测试改为调用正式控制器，仅保留开环起转、无扰交接、指令斜坡和结果统计，独立保留10 A测试保护；
              实测角度补偿后10%限压Vd平均由-2910 mV降至-235 mV，15%限压达到55.818 Hz电频率；
              统一模块内部静态符号前缀，补充架构与验证文档，并修复ARMCC5下OTA头结构编译期断言兼容性；
              已完成36个工程源文件ARMCC逐文件编译检查，结构调整后的上板回归待验证。

2026.09.03.4  FOC采样与控制内核：新增双ADC快慢采样端口、电流零偏校准、三相电流换算、慢速母线及温度换算；
              新增Q15正余弦、Clarke/Park、反Park、SVPWM和200 Hz定点电流PI，电流标幺固定为128 A对应1.0；
              新增Hall端口、六步方向及频率统计、边沿角度标定和边沿间插值，实测10 Hz下三路边沿20/20/20、非法跳变为0；
              完成低压开环试转，48 V空载母线电流约0.246 A，Hall估算角度与开环角度误差约0.5°；
              新增阻塞式Rs/Ld/Lq辨识，实测Rs=273 mΩ、Ld=225 µH、Lq=206 µH；
              新增固定角度Id=2 A电流PI测试，实测Id平均1991 mA、Iq平均-12 mA，测试结束自动关闭PWM。

2026.09.03.3  PWM安全端口与性能监控：新增motor_pwm_port和motor_board_config，统一三相比较值写入、门极控制、
              nFAULT/BIF检查、PWM使能关闭及紧急关断，初始化默认保持EN_GATE和PWM输出关闭；
              新增基于DWT的motor_timebase和通用motor_performance_monitor，支持多统计点独立记录最近值、峰值和次数；
              ADC快速中断接入执行耗时监控，实测常态104 cycles、区间峰值134 cycles，10 kHz周期峰值占用约0.69%；
              启动阶段DRV检查实测nFAULT=1、BIF=0、ready=1、MOE=0，DWT 10 ms时间基准自检通过；
              三相50% PWM上板测试暂由if (0)关闭，待低压限流和示波器条件具备后继续验证。

2026.09.03.2  中断频率验证：新增interrupt_monitor诊断模块，在中断内计数并由主循环每秒输出频率；
              实测SysTick 1 kHz、TMR1通道4 20 kHz、ADC快速采样10 kHz，均符合理论配置；
              DMA1通道3配置4个半字循环缓冲区，ADC慢速采样及DMA完成频率实测均为1 kHz；
              关闭冗余的ADC普通组完成中断，由DMA完成中断统一处理慢速采样事件；
              监控日志使用adc_fast、adc_slow_dma等业务命名，避免依赖DMA硬件通道编号。

2026.09.03.1  启动日志与版本管理：新增motor_log运行时日志等级，支持OFF、ERROR、WARN、DEBUG和INFO五级过滤；
              启动日志通过USART1 printf输出产品名、固件版本、目标硬件版本及App启动状态；
              Keil工程加入project/config头文件路径及motor_log.c，完善日志架构和对应验证计划。

2026.09.03.0  FOC重构准备：重新生成AT32F456外设代码，移除旧版foc_fast及foc_app实现；
              新建foc_kernel与foc_app目录，增加HA/HB/HC霍尔输入及外部中断配置；
              Keil工程头文件路径由foc_fast调整为foc_kernel；新增产品版本配置和USART1启动日志。
              Bootloader串口发送增加帧发送完成等待，避免跳转App关闭外设时截断启动日志。
              App在外设中断使能前恢复0x08004000向量表，避免Bootloader跳转后误用Bootloader中断向量。
              App关闭LTO并使用USART1重定向printf；补充ARMClang无半主机标记，避免sys_io重复定义及启动前BKPT 0xAB。
              App初始化期间全局关闭中断，完成启动日志后统一开启，避免未完成初始化的外设中断抢占。
              新增motor_log日志模块，参考cw_joint_project保留0~4运行时日志等级；启动时打印固件版本、硬件版本及App状态。
              Keil工程加入project/config头文件目录和motor_log.c源文件，日志底层继续使用USART1 printf。
              新FOC控制功能待重构及验证。
