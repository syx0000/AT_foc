# CAN万里扬协议测试报文手册

**基于**：万里扬FDCAN通信协议V1.7 + AT32F456CEU7实现  
**节点ID**：默认 0x01  
**波特率**：CAN-FD 500kbps/8Mbps (BRS)  
**日期**：2026/06/17

---

## 测试环境配置

```bash
# 连接CAN调试工具（CANalyst-II / USBCAN / PCAN）
# 设置: CAN-FD模式，仲裁段500kbps，数据段8Mbps
# 本文所有帧均为标准帧，数据段使用CAN-FD扩展长度
```

---

## 一、基础通信测试（5个命令）

### 1. **广播状态查询** (0x080)

**功能**：让所有节点上报状态  
**测试帧**：
```
ID: 0x080
DLC: 0 (无数据)
```

**预期响应**：节点ID=1返回 `0x101` 帧（32字节状态）

---

### 2. **使能PWM输出** (0x701, ID=1)

**功能**：启动FOC，复位PID积分器  
**测试帧**：
```
ID: 0x701
DLC: 8
Data: FF FF FF FF FF FF FF FA
      └─ D[7]=0xFA (Enable命令)
```

**预期结果**：
- 串口打印 "PWM enabled"
- 0x101状态帧 D[11].Bit0=1 (STA使能标志)

---

### 3. **停止PWM输出** (0x701, 禁止)

**测试帧**：
```
ID: 0x701
DLC: 8
Data: FF FF FF FF FF FF FF FB
      └─ D[7]=0xFB (Disable命令)
```

**预期结果**：电机立即安全关断，0x101状态帧 STA.Bit0=0

---

### 4. **清除故障标志** (0x701, 清错)

**测试帧**：
```
ID: 0x701
DLC: 8
Data: FF FF FF FF FF FF FF FD
      └─ D[7]=0xFD (Clear Error)
```

**预期结果**：ServoErrFlag归零，0x101状态帧 D[7-9]=0x00

---

### 5. **设置当前位置为零点** (0x701, 置零)

**测试帧**：
```
ID: 0x701
DLC: 8
Data: FF FF FF FF FF FF FF FC
      └─ D[7]=0xFC (Set Zero)
```

**预期结果**：串口打印 "Clear encoder position_out=0"

---

## 二、控制模式测试（4个命令）

### 6. **扭矩模式：给定1.0N·m** (0x300)

**量化计算**：
```python
T_des = 1.0  # N·m
T_min, T_max = -500.0, 500.0
quantized = int((T_des - T_min) / (T_max - T_min) * 65535)
          = int(501.0 / 1000.0 * 65535) = 32833 = 0x8041
```

**测试帧**：
```
ID: 0x300
DLC: 3
Data: 41 80 01
      ││   └─ D[2] CANID=1
      └┴─ D[0..1] T_des=0x8041 (LE)
```

**预期结果**：
- I_q_ref ≈ 0.1A（通过Kt LUT反查）
- 0x101状态帧 D[17-18] 回显 T_cmd=1.0N·m

---

### 7. **速度模式：给定5rad/s** (0x200)

**量化计算**：
```python
V_des = 5.0  # rad/s
V_min, V_max = -20.0, 20.0
quantized = int((5.0 - (-20.0)) / 40.0 * 65535) = 40959 = 0x9FFF
```

**测试帧**：
```
ID: 0x200
DLC: 3
Data: FF 9F 01
      ││   └─ CANID=1
      └┴─ V_des=0x9FFF (LE)
```

**预期结果**：
- controller_mode = PROFILE_VELOCITY_MODE
- velocity_ref = 5rad/s
- 电机平稳旋转，串口 Vcmd≈5rad/s

---

### 8. **位置模式：移动到180°(3.14rad)** (0x400)

**量化计算**：
```python
POS_des = 3.14  # rad
POS_min, POS_max = -7.0, 7.0
quantized = int((3.14 - (-7.0)) / 14.0 * 16777215) = 12175851 = 0xB9D9FB

V_lim = 10.0  # rad/s
V_quantized = int((10.0 - (-20.0)) / 40.0 * 65535) = 49151 = 0xBFFF
```

**测试帧**：
```
ID: 0x400
DLC: 6
Data: FB D9 B9 FF BF 01
      │  │  │  ││  └─ CANID=1
      │  │  │  └┴─ V_lim=0xBFFF (LE)
      └──┴──┴─ POS=0xB9D9FB (LE 24bit)
```

**预期结果**：
- position_ref = 3.14rad (180°)
- 电机转到目标位置并保持，到达后 STA.Bit3=1

---

### 9. **MIT阻抗模式：柔顺控制** (0x500)

**参数**：P_des=0rad, V_des=0, T_ff=0, Kp=50, Kd=1.0

**量化计算**：
```python
POS = int((0 - (-7)) / 14 * 16777215) = 8388607 = 0x7FFFFF
VEL = int((0 - (-20)) / 40 * 65535) = 32767 = 0x7FFF
T   = int((0 - (-500)) / 1000 * 65535) = 32767 = 0x7FFF
Kp  = int((50 - 0) / 500 * 65535) = 6553 = 0x1999
Kd  = int((1.0 - 0) / 20 * 65535) = 3276 = 0x0CCC
```

**测试帧**：
```
ID: 0x500
DLC: 12
Data: FF FF 7F FF 7F FF 7F 99 19 CC 0C 01
      │  │  │  ││  ││  ││  │  │  │  └─ CANID=1
      │  │  │  ││  ││  ││  │  └──┴─ Kd=0x0CCC (LE)
      │  │  │  ││  ││  └┴──┴─ Kp=0x1999 (LE)
      │  │  │  ││  └┴─ T_ff=0x7FFF (LE)
      │  │  │  └┴─ VEL=0x7FFF (LE)
      └──┴──┴─ POS=0x7FFFFF (LE 24bit)
```

**预期结果**：
- 外力推动电机时感受柔顺阻力
- 0x101状态帧 D[25-26] MIT_T 实时反馈扭矩

---

## 三、SDO参数读写（10个命令）

### 10. **读取节点ID** (SDO 0x2F00)

**测试帧**：
```
ID: 0x601
DLC: 8
Data: 40 00 2F 00 00 00 00 00
      │  ││  │  │  └──────┴─ 忽略
      │  ││  │  └─ Sub=0
      │  └┴──┴─ Index=0x2F00 (LE)
      └─ 0x40 (读命令)
```

**预期响应**：
```
ID: 0x581
DLC: 8
Data: 60 00 2F 00 01 00 00 00
      │  ││  │  │  └──────┴─ Value=1 (uint32 LE)
      │  ││  │  └─ Sub=0
      │  └┴──┴─ Index=0x2F00
      └─ 0x60 (成功)
```

---

### 11. **写入节点ID=2** (SDO 0x2F00)

**测试帧**：
```
ID: 0x601
DLC: 8
Data: 23 00 2F 00 02 00 00 00
      │  ││  │  │  └──────┴─ Value=2 (uint32 LE)
      │  ││  │  └─ Sub=0
      │  └┴──┴─ Index=0x2F00 (LE)
      └─ 0x23 (写命令)
```

**预期响应**：
```
ID: 0x581 (原ID=1)
Data: 60 00 2F 00 02 00 00 00  (成功确认)
```

**注意**：写入后立即生效，后续帧应发送到 0x602/0x702

---

### 12. **读取位置量程** (SDO 0x2000/0x2001)

**测试帧（读pos_min）**：
```
ID: 0x601
DLC: 8
Data: 40 00 20 00 00 00 00 00
      │  └┴──┴─ Index=0x2000 (pos_min)
      └─ 0x40 (读)
```

**预期响应**：
```
ID: 0x581
Data: 60 00 20 00 00 00 E0 C0
                    └──────┴─ float32 LE = -7.0
```

**测试帧（读pos_max）**：
```
ID: 0x601
Data: 40 01 20 00 00 00 00 00
         └┴──┴─ Index=0x2001 (pos_max)
```

**预期响应**：float32 = +7.0

---

### 13. **写入位置量程：-10~+10rad** (SDO 0x2000/0x2001)

**测试帧（写pos_min=-10.0）**：
```
ID: 0x601
DLC: 8
Data: 23 00 20 00 00 00 20 C1
      │  └┴──┴─ Index=0x2000
      │           └──────┴─ float32(-10.0) LE = 0xC1200000
      └─ 0x23 (写)
```

**测试帧（写pos_max=+10.0）**：
```
ID: 0x601
Data: 23 01 20 00 00 00 20 41
         └┴──┴─ Index=0x2001
                    └──────┴─ float32(10.0) LE = 0x41200000
```

**预期结果**：后续0x400位置命令量程扩展到±10rad

---

### 14. **读取速度环PID** (SDO 0x200C/0x200D)

**测试帧（读spd_kp）**：
```
ID: 0x601
Data: 40 0C 20 00 00 00 00 00
         └┴──┴─ Index=0x200C (drv_spd_kp)
```

**预期响应**：uint32 = 1200 (默认Kp)

**测试帧（读spd_ki）**：
```
ID: 0x601
Data: 40 0D 20 00 00 00 00 00
         └┴──┴─ Index=0x200D (drv_spd_ki)
```

**预期响应**：uint32 = 8 (默认Ki)

---

### 15. **写入电流环PID** (SDO 0x200B)

**测试帧（写cur_kp=100）**：
```
ID: 0x601
Data: 23 0B 20 00 64 00 00 00
         └┴──┴─ Index=0x200B (drv_pos_kp，复用cur_kp)
                    └──────┴─ uint32(100) LE
```

**预期结果**：串口 `getparams` 查询 CurKp=100

---

### 16. **触发电角度校准** (SDO 0x2F01 sub=1)

**测试帧**：
```
ID: 0x601
Data: 23 01 2F 01 01 00 00 00
         └┴──┴─ Index=0x2F01 Sub=1 (calibration)
                    └─ 写入1触发
```

**预期结果**：
- 电机锁定5秒（2V d轴电压）
- 串口打印 "elec_offset=xxxxx"
- 自动写入Flash

---

### 17. **设置单频注入频率** (SDO 0x2F06)

**测试帧（设置100Hz）**：
```
ID: 0x601
Data: 23 06 2F 00 64 00 00 00
         └┴──┴─ Index=0x2F06 (test_freq)
                    └──────┴─ uint32(100) Hz
```

---

### 18. **设置注入幅值** (SDO 0x2F07)

**测试帧（设置5A，Q10=5120）**：
```
ID: 0x601
Data: 23 07 2F 00 00 14 00 00
         └┴──┴─ Index=0x2F07 (test_ampl)
                    └──────┴─ uint32(5120) = 5A * 1024
```

---

### 19. **启动带宽测试** (SDO 0x2F05=1)

**测试帧**：
```
ID: 0x601
Data: 23 05 2F 00 01 00 00 00
         └┴──┴─ Index=0x2F05 (auto_report)
                    └─ 1=启动0x7FD数据流
```

**预期结果**：10kHz发送 0x7FD帧（I_q_ref + I_q_fdb）

**停止测试**：
```
ID: 0x601
Data: 23 05 2F 00 00 00 00 00
                    └─ 0=停止
```

---

## 四、调试通道命令（12个命令）

### 20. **PING测试** (0x7E0 CMD=0x00)

**测试帧**：
```
ID: 0x7E0
DLC: 1
Data: 00
      └─ CMD=0x00 (PING)
```

**预期响应**：
```
ID: 0x7E1
DLC: 2
Data: 00 00
      │  └─ Status=0 (OK)
      └─ CMD=0x00 (PONG)
```

---

### 21. **查询版本** (0x7E0 CMD=0x01)

**测试帧**：
```
ID: 0x7E0
DLC: 1
Data: 01
```

**预期响应**：
```
ID: 0x7E1
DLC: 32 (多帧)
Data: 01 00 "AT32F456 FOC v1.0 built Jun 17 2026"
      │  └─ Status=0
      └─ CMD=0x01
```

---

### 22. **系统复位** (0x7E0 CMD=0x02)

**测试帧**：
```
ID: 0x7E0
DLC: 1
Data: 02
```

**预期结果**：3秒后自动重启

---

### 23. **设置日志通道** (0x7E0 CMD=0x10)

**测试帧（logid=40，电流环）**：
```
ID: 0x7E0
DLC: 2
Data: 10 28
      │  └─ logid=40 (0x28)
      └─ CMD=0x10 (LOGID_SET)
```

**预期响应**：0x7E1确认 + 周期性0x7E2日志帧

---

### 24. **设置日志频率** (0x7E0 CMD=0x11)

**测试帧（100ms周期）**：
```
ID: 0x7E0
DLC: 3
Data: 11 64 00
      │  └┴─ uint16(100) ms (LE)
      └─ CMD=0x11 (LOGFREQ_SET)
```

---

### 25. **设置电流环PID** (0x7E0 CMD=0x20)

**测试帧（Kp=55, Ki=20, Kd=0）**：
```
ID: 0x7E0
DLC: 13
Data: 20 37 00 00 00 14 00 00 00 00 00 00 00
      │  └──────┴─ Kp=55 (float32 LE)
      │           └──────┴─ Ki=20
      │                    └──────┴─ Kd=0
      └─ CMD=0x20 (CUR_PID_SET)
```

---

### 26. **写入Flash** (0x7E0 CMD=0x40)

**测试帧**：
```
ID: 0x7E0
DLC: 1
Data: 40
```

**预期结果**：串口 "Flash write: OK" + 写入耗时

---

### 27. **擦除Flash** (0x7E0 CMD=0x41)

**测试帧**：
```
ID: 0x7E0
DLC: 1
Data: 41
```

**预期结果**：参数区清零，重启后恢复默认值

---

### 28. **清除故障** (0x7E0 CMD=0x43)

**测试帧**：
```
ID: 0x7E0
DLC: 1
Data: 43
```

**预期结果**：ServoErrFlag=0，主动刹车解除

---

### 29. **使能开关** (0x7E0 CMD=0x50)

**测试帧（使能）**：
```
ID: 0x7E0
DLC: 2
Data: 50 01
      │  └─ 1=Enable
      └─ CMD=0x50
```

**测试帧（禁止）**：
```
Data: 50 00
         └─ 0=Disable
```

---

### 30. **电角度校准** (0x7E0 CMD=0x5F)

**测试帧**：
```
ID: 0x7E0
DLC: 1
Data: 5F
```

**预期结果**：与SDO 0x2F01相同，锁定5秒标定

---

### 31. **带宽测试** (0x7E0 CMD=0x60)

**测试帧（测试项2：电流环阶跃）**：
```
ID: 0x7E0
DLC: 2
Data: 60 02
      │  └─ 测试ID=2 (电流环阶跃响应)
      └─ CMD=0x60 (BWTEST)
```

**可选测试ID**：
```
1  = 位置阶跃
2  = 电流阶跃
3  = 速度阶跃
4  = 相电阻辨识
5  = 磁链辨识
6  = 惯量辨识
7  = 自整定PID
8  = 死区标定
9  = 相位补偿
10 = BEMF反电动势
11 = 扭矩校准
```

**预期结果**：串口打印测试进度 + 结果

---

## 五、状态监控帧解码示例

### **0x101状态帧实例**（32字节）

```
ID: 0x101
DLC: 32
Data: 7F FF FF 7F FF 7F FF 00 00 00 00 09 ...
      │  │  │  ││  ││  ││  │  │  │  └─ STA=0x09 (使能+到达)
      │  │  │  ││  ││  ││  │  └─ warn=0x00
      │  │  │  ││  ││  └┴─ Err2=0x00
      │  │  │  ││  └┴─ Err1=0x0000 (无故障)
      │  │  │  └┴─ Tact=0xFF7F (中点)
      │  │  └─ Vact=0xFF7F (中点)
      └──┴──┴─ Pact=0xFFFF7F (中点≈0rad)
```

**解码**：
```python
Pact = (0xFFFF7F / 16777215) * 14 - 7 = 0.00 rad
Vact = (0xFF7F / 65535) * 40 - 20 = 0.00 rad/s
Tact = (0xFF7F / 65535) * 1000 - 500 = 0.00 N·m
```

---

## 六、故障注入测试

### **触发过压保护** (0x300扭矩过载)

**测试帧**（100N·m超额扭矩）：
```
ID: 0x300
DLC: 3
Data: 99 CC 01
      └┴─ T=0xCC99 ≈ 100N·m (量化)
```

**预期结果**：
- 电流超限 → ServoErrFlag.Bit3=1 (过流)
- 自动触发fault_safe_shutdown()
- 0x101状态帧 STA.Bit1=1 (故障)

---

## 七、Python解析脚本示例

```python
import struct

def decode_0x101(data):
    """解码32字节0x101状态帧"""
    assert len(data) == 32
    
    # 位置/速度/扭矩（量化）
    pact_raw = int.from_bytes(data[0:3], 'little')
    vact_raw = int.from_bytes(data[3:5], 'little')
    tact_raw = int.from_bytes(data[5:7], 'little')
    
    # 反量化（默认量程）
    pact = (pact_raw / 16777215) * 14 - 7  # rad
    vact = (vact_raw / 65535) * 40 - 20    # rad/s
    tact = (tact_raw / 65535) * 1000 - 500 # N·m
    
    # 故障/状态标志
    err1 = int.from_bytes(data[7:9], 'little')
    err2 = data[9]
    warn = data[10]
    sta  = data[11]
    
    # 电流
    iqref_raw = int.from_bytes(data[19:21], 'little')
    iqfdb_raw = int.from_bytes(data[21:23], 'little')
    iqref = (iqref_raw - 10000) / 100  # A
    iqfdb = (iqfdb_raw - 10000) / 100  # A
    
    # 温度/电压
    vdc = data[27]
    temp_d = struct.unpack('<h', data[28:30])[0] / 10  # °C
    temp_m = struct.unpack('<h', data[30:32])[0] / 10  # °C
    
    return {
        'Pact': pact, 'Vact': vact, 'Tact': tact,
        'Err': (err2 << 16) | err1, 'Warn': warn, 'STA': sta,
        'Iq_ref': iqref, 'Iq_fdb': iqfdb,
        'Vdc': vdc, 'T_driver': temp_d, 'T_motor': temp_m
    }

# 示例
raw = bytes.fromhex('7FFFFF7FFF7FFF000000000900...')
status = decode_0x101(raw)
print(f"位置={status['Pact']:.2f}rad 速度={status['Vact']:.1f}rad/s")
```

---

## 八、测试用例总结表

| 序号 | 功能 | 帧ID | 测试点 |
|------|------|------|--------|
| 1 | 广播查询 | 0x080 | 多节点响应 |
| 2-5 | 基础控制 | 0x701 | Enable/Disable/Zero/Clear |
| 6-9 | 4种控制模式 | 0x200/0x300/0x400/0x500 | 扭矩/速度/位置/MIT |
| 10-19 | SDO读写 | 0x600+ID | 10个参数对象 |
| 20-31 | 调试通道 | 0x7E0 | 12个管理命令 |

**总计**：31个核心测试用例，覆盖协议全部功能。

---

## 附录A：常用量化转换公式

```c
// 位置（24bit，±7rad）
uint32_t quantize_pos(float rad) {
    return (uint32_t)((rad + 7.0f) / 14.0f * 16777215);
}

// 速度（16bit，±20rad/s）
uint16_t quantize_vel(float rad_s) {
    return (uint16_t)((rad_s + 20.0f) / 40.0f * 65535);
}

// 扭矩（16bit，±500N·m）
uint16_t quantize_torque(float Nm) {
    return (uint16_t)((Nm + 500.0f) / 1000.0f * 65535);
}

// MIT增益Kp（16bit，0~500）
uint16_t quantize_kp(float kp) {
    return (uint16_t)(kp / 500.0f * 65535);
}

// MIT增益Kd（16bit，0~20）
uint16_t quantize_kd(float kd) {
    return (uint16_t)(kd / 20.0f * 65535);
}
```

---

## 附录B：故障码速查表

| Bit | ServoErrFlag | 说明 |
|-----|--------------|------|
| 0 | DRV_FAULT | DRV8353硬件故障 |
| 1 | DCUNDER_VOLT | 母线欠压(<25V) |
| 2 | DCUPPER_VOLT | 母线过压(>60V) |
| 3 | UPPER_CURRENT | 电流过载 |
| 4 | BOADR_TEMP | 板温过高(>85°C) |
| 5 | MOTOR_TEMP | 电机过温(>120°C) |
| 6 | ENCODER_FAULT | 编码器通信失败 |
| 7 | CANOPEN_FAULT | CAN超时(200ms) |
| 8 | PHASE_LOSS | 缺相（预留） |
| 9 | OVERLOAD_ALARM | 过载报警 |
| 10 | OVERLOAD_STOP | 过载停机 |
| 11 | POS_LIMIT | 位置软限位 |

---

**文档版本**：v1.0  
**最后更新**：2026/06/17  
**对应固件**：阶段10 (Bootloader + OTA)
