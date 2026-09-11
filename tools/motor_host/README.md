# AT_foc_hall 简易上位机

用于代替手工输入常用串口命令。界面包含运行控制、自动辨识、参数管理、故障诊断及原始串口终端。

当前固件基线（2026-09-11）为 4 对极、`Rs=268 mΩ`、`Ld=222 µH`、`Lq=206 µH`、`flux_linkage_uwb=19494`，电流 PI Q15 为 `40588/37663/4900`。影子无感观测器只做诊断，不接管 FOC、速度反馈或保护。

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
6. 完整自动辨识启动后，软件会暂时禁用其他辨识操作，收到成功或失败回显后自动解锁。
7. 辨识成功后依次查看“参数差异”，再执行“接受候选”和“保存参数”。
8. 电机运行中不要断开串口代替停机；应先点击“停止电机”。

### 影子无感观测器

上位机当前没有独立的观测器图表页，可在底部原始串口终端发送：

```text
observer status
observer debug 1
observer debug 0
```

`observer status` 返回电角度、电频率、`bemf_alpha/beta`、相位误差、`confidence`、`valid` 和样本数。`observer debug 1` 以约 500 Hz 输出；下列为 `OBS` 行的字段顺序（固件不重复打印表头）：

```text
OBS,electrical_angle_u16,electrical_frequency_millihz,bemf_alpha_mv,bemf_beta_mv,phase_error_u16,confidence,valid
```

如需修改磁链，使用参数页对应字段 `flux_linkage_uwb`，再在电机 READY 且 PWM 关闭时执行候选参数接受和保存；Flash 记录格式当前为 v2。

参数设置区域的参数名下拉框与固件当前支持的 `motor param set` 字段保持一致。

本工具只封装现有文本协议，不绕过固件中的状态检查与安全保护。
