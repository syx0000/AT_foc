# AT_foc_hall 简易上位机

用于代替手工输入常用串口命令。界面包含运行控制、自动辨识、参数管理、故障诊断及原始串口终端。

当前固件基线（2026-09-11）为 4 对极、`Rs=268 mΩ`、`Ld=222 µH`、`Lq=206 µH`、`flux_linkage_uwb=19494`，电流 PI Q15 为 `40588/37663/4900`。

## 环境

- Python 3.10 或更高版本
- PySide6
- pyserial 3.5

安装依赖并启动：

```powershell
cd tools\motor_host
python -m pip install -r requirements.txt
python motor_host.py
```

工程机也可直接复用已有环境：

```powershell
E:\src\foc_tuner\tools\python3\python.exe motor_host.py
```

串口固定使用固件当前配置：`921600, 8N1, 无流控`。

## 使用说明

1. 选择调试串口并连接。
2. 通过页签按钮发送常用指令；底部输入框可发送任意原始CLI命令，使用上下方向键浏览已发送的历史命令。
3. 顶部“复位设备”仅发送安全复位命令；固件会拒绝电机运行或PWM开启时复位。
4. “打开调试/关闭调试”使用同一个按钮切换。
5. 开环、电流环和速度环各使用一个“启动/更新”按钮：首次点击启动，后续点击更新；停止电机或断开连接后恢复为启动状态。
6. 单项辨识启动后，软件会暂时禁用其他辨识操作，收到成功或失败回显后自动解锁。
7. 辨识成功后依次查看“参数差异”，再执行“接受候选”和“保存参数”。
8. 电机运行中不要断开串口代替停机；应先点击“停止电机”。

磁链辨识使用“自动辨识”页的“辨识磁链”按钮（`identify flux`），应先确认Rs、Ld/Lq与Hall标定参数。成功后查看参数差异，再接受候选并保存；也可在参数页手动设置 `flux_linkage_uwb`。

参数设置区域的参数名下拉框与固件当前支持的 `motor param set` 字段保持一致。

新增调试参数支持“设置候选参数 → 参数差异 → 接受候选 → 保存参数”。接受要求READY且PWM关闭，接受后RAM生效；保存后下次上电加载。Flash为v5，兼容v2/v3/v4：原字段保留，新字段补固件默认值。修改CAN ID后对端应立即改用新ID。

正常Hall启动用不到的对齐电流、对齐时间、I/F最大电流和频率爬升率已从上位机参数列表隐藏；下位机和Flash字段保留兼容，启动超时仍可设置。

| 参数 | 单位 | 默认值 | 可设范围 |
|---|---|---:|---:|
| `startup_timeout_ms` | ms | 3000 | 500..15000，且大于对齐时间 |
| `offset_iq_ma` / `flux_iq_ma` | mA | 10000 | 1..20000 |
| `offset_voltage_pct` / `flux_voltage_pct` | %母线电压 | 30 | 1..57 |
| `offset_voltage_mv` / `flux_voltage_mv` | mV | 8400 | 100..27700 |
| `offset_settle_ms` / `flux_settle_ms` | ms | 2000 | 100..10000 |
| `offset_measure_ms` / `flux_measure_ms` | ms | 4000 | 100..10000 |
| `hall_scan_mv` | mV | 1200 | 100..4800 |
| `hall_scan_mhz` | 电频率mHz | 5000 | 1000..20000 |
| `stall_speed_rpm` | rpm | 30 | 1..500 |
| `stall_time_ms` | ms | 1000 | 100..10000 |
| `can_node_id` | 标准帧ID，十进制 | 1 | 1..127 |

`offset_*`只影响Hall补偿，`flux_*`只影响磁链辨识；辨识会临时使用候选参数，结束后恢复原活动值，再由用户接受和保存。两种电压限制取较小值。对齐和I/F启动电流还必须低于固件的辨识中止阈值（当前25000 mA）。对齐参数属于电流启动路径，不改变速度模式的直接启动。固定等待时间不等于已通过稳速检查，Hall开环扫描仍不具备专用闭环限流。硬件保护阈值仍由固件约束。

本工具只封装现有文本协议，不绕过固件中的状态检查与安全保护。

## 打包exe

使用工程机已有Python环境（已装PySide6、pyserial、PyInstaller）：

```powershell
cd tools\motor_host
E:\src\foc_tuner\tools\python3\python.exe -m PyInstaller --noconfirm AT_foc_hall_MotorHost.spec
```

输出为 `dist\AT_foc_hall_MotorHost.exe`（单文件、无控制台）；`build\` 为中间文件，可删除。新环境需先执行 `python -m pip install pyinstaller`。
