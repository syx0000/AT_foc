# 修改记录

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
