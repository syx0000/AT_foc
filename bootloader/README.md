# AT32F456CEU7 Bootloader

## 内存布局

```
Flash (512KB total):
0x08000000 - 0x08003800   Bootloader Code      (0-14KB)
0x08003800 - 0x08004000   Upgrade Flag         (14-16KB, 2KB)
0x08004000 - 0x0803F800   App Execute Area     (16-254KB, max 238KB)
0x0803F800 - 0x08040000   Staging Header       (254-256KB, 2KB)
0x08040000 - 0x0807F000   OTA Staging Data     (256-508KB, max 252KB)
0x0807F000 - 0x0807F800   User Data Slot A     (508-510KB, 2KB)
0x0807F800 - 0x08080000   User Data Slot B     (510-512KB, 2KB)

SRAM (128KB):
0x20000000 - 0x20020000   128KB
```

## 升级流程

### 正常启动（无升级）
1. Bootloader检查升级标志(0x08003800) == 0xFFFFFFFF
2. 验证App复位向量有效性 (0x08004004 & 0xFF000000 == 0x08000000)
3. 跳转到App (0x08004000)

### IAP升级模式
1. 上位机通过UART发送升级命令 0x5A 0x01
2. Bootloader接收固件数据到备份区(0x08070000)
3. 写入升级标志 0x43575342 ('BWSC') 到 0x08003800
4. 重启设备
5. Bootloader检测升级标志，复制备份区→执行区
6. 清除升级标志
7. 跳转到新App

### 升级协议
- **命令帧**: 0x5A 0x01 (启动升级) / 0x5A 0x02 (结束升级) / 0x5A 0xA5 (跳转App)
- **数据帧**: 0x31 + addr[4] + data[2KB] + checksum
- **应答**: 0xCC 0xDD (成功) / 0xEE 0xFF (失败)

## 编译说明

### Keil MDK工程配置（手动步骤）

1. 打开 `bootloader/mdk_v5/bootloader.uvprojx`
2. 添加源文件分组：
   - **Bootloader**: src/main.c, src/iap.c, src/flash.c
   - **System**: system/system_at32f45x.c, system/at32f45x_int.c
   - **BSP**: bsp/at32f45x_crm.c, bsp/at32f45x_flash.c, bsp/at32f45x_gpio.c, bsp/at32f45x_usart.c, bsp/at32f45x_misc.c
   - **Startup**: system/startup_at32f45x.s
3. 配置Include路径:
   - `../inc`
   - `../system`
   - `../bsp`
   - `../../libraries/cmsis/cm4/core_support`
   - `../../libraries/cmsis/cm4/device_support`
   - `../../libraries/drivers/inc`
4. 配置Linker: 
   - 使用Scatter文件 `bootloader.sct`
   - 或手动设置ROM: 0x08000000 Size 0x4000
5. 定义宏: `AT32F456CEU7, USE_STDPERIPH_DRIVER`
6. 编译优化: Level 1 (-O1)

### 预期编译结果
- Code Size: <14KB
- Output: bootloader.hex, bootloader.bin

## App工程现状

主应用已使用 `0x08004000` 作为执行基址，并在启动阶段完成向量表重定位；App 与 Bootloader 共享 `ota_config.h` 的 238KB/252KB/4KB 边界。参数双槽由 App 负责读写，Bootloader 不解析参数内容，升级擦除和复制不得覆盖 `0x0807F000-0x08080000`。

当前 App 参数记录格式为 v2，包含可持久化的 `flux_linkage_uwb`；旧 v1 记录由 App 判为无效并回退默认参数。

## 测试步骤

1. 编译Bootloader → 烧录到0x08000000
2. 编译 App 工程（链接基址 `0x08004000`）
3. 烧录App到0x08004000 (或通过IAP升级)
4. 上电测试跳转

## 参考工程
- iflytek_joint_module (AT32F456平台IAP实现)
- cubemx_yxsui (STM32H743 OTA架构)

## 当前联调状态（2026-09-11）

- 主应用已接入 10 kHz 影子无感观测器和 `observer status/debug` 诊断，但这些运行时输出不改变 Bootloader 升级协议。
- 主应用最近一次 Keil 全量构建为 `0 Error(s), 0 Warning(s)`；Bootloader 仍需在目标板上单独验证升级、掉电和参数区不覆盖。
