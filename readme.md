# 修改记录

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
