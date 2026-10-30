"""AT_foc_hall 简易串口上位机（PySide6）。"""

from __future__ import annotations

import sys
from datetime import datetime

import serial
from serial.tools import list_ports
from PySide6.QtCore import Qt, QTimer
from PySide6.QtGui import QTextCursor
from PySide6.QtWidgets import (
    QApplication, QComboBox, QFormLayout, QGridLayout, QGroupBox,
    QHBoxLayout, QLabel, QLineEdit, QMainWindow, QMessageBox, QPushButton,
    QPlainTextEdit, QTabWidget, QVBoxLayout, QWidget,
)

from trace_view import TraceView

BAUDRATE = 921600
PARAMETER_NAMES = [
    "pole_pairs",
    "direction_inverted",
    "phase_resistance_mohm",
    "direct_inductance_uh",
    "quadrature_inductance_uh",
    "current_loop_bandwidth_hz",
    "current_d_kp_q15",
    "current_q_kp_q15",
    "current_ki_q15",
    "speed_kp_q20",
    "speed_ki_q20",
    "hall_rotor_offset_u16",
    "hall_delay_us",
    "flux_linkage_uwb",
    "speed_maximum_rpm",
    "speed_acceleration_rpm_per_s",
    "speed_deceleration_rpm_per_s",
    "speed_startup_iq_slew_ma_s",
    "speed_current_limit_ma",
    "startup_timeout_ms",
    "offset_iq_ma",
    "offset_voltage_pct",
    "offset_voltage_mv",
    "offset_settle_ms",
    "offset_measure_ms",
    "flux_iq_ma",
    "flux_voltage_pct",
    "flux_voltage_mv",
    "flux_settle_ms",
    "flux_measure_ms",
    "hall_scan_mv",
    "hall_scan_mhz",
    "can_node_id",

]
# motor params active 回显字段 -> 参数名（多值字段按顺序拆分）。
ACTIVE_FIELD_MAP = {
    "pole_pairs": ["pole_pairs"],
    "direction_inverted": ["direction_inverted"],
    "rs_mohm": ["phase_resistance_mohm"],
    "ld_uh": ["direct_inductance_uh"],
    "lq_uh": ["quadrature_inductance_uh"],
    "bandwidth_hz": ["current_loop_bandwidth_hz"],
    "current_pi_q15": ["current_d_kp_q15", "current_q_kp_q15",
                       "current_ki_q15"],
    "speed_pi_q20": ["speed_kp_q20", "speed_ki_q20"],
    "hall_offset_u16": ["hall_rotor_offset_u16"],
    "hall_delay_us": ["hall_delay_us"],
    "flux_linkage_uwb": ["flux_linkage_uwb"],
    "speed_maximum_rpm": ["speed_maximum_rpm"],
    "speed_acceleration_rpm_per_s": ["speed_acceleration_rpm_per_s"],
    "speed_current_limit_ma": ["speed_current_limit_ma"],
}
ACTIVE_FIELD_MAP.update({name: [name] for name in PARAMETER_NAMES if name not in ACTIVE_FIELD_MAP})
PARAMETER_RANGES = {'startup_align_ma': (1, 24999), 'startup_align_ms': (10, 2000), 'startup_if_max_ma': (1600, 24999), 'startup_ramp_mhz_s': (1000, 100000), 'startup_timeout_ms': (500, 15000), 'offset_iq_ma': (1, 20000), 'offset_voltage_pct': (1, 57), 'offset_voltage_mv': (100, 27700), 'offset_settle_ms': (100, 10000), 'offset_measure_ms': (100, 10000), 'flux_iq_ma': (1, 20000), 'flux_voltage_pct': (1, 57), 'flux_voltage_mv': (100, 27700), 'flux_settle_ms': (100, 10000), 'flux_measure_ms': (100, 10000), 'hall_scan_mv': (100, 4800), 'hall_scan_mhz': (1000, 20000), 'stall_speed_rpm': (1, 500), 'stall_time_ms': (100, 10000), 'can_node_id': (1, 127)}
SILENT_QUERY = "motor params active"
PARAMETER_RANGES.update({
    "speed_deceleration_rpm_per_s": (1, 1000000),
    "speed_startup_iq_slew_ma_s": (1000, 1000000),
})
PARAMETER_HELP = {
    "speed_deceleration_rpm_per_s": "速度减速斜率，rpm/s，默认30000；换向先降至零，再等待实测零速确认",
    "speed_startup_iq_slew_ma_s": "启动Iq幅值上升速率，mA/s，默认200000（200 A/s）；降低限流及零限流立即生效",
    "startup_timeout_ms": "电流启动超时，ms；必须大于对齐时间",
    "offset_iq_ma": "Hall补偿测量Iq，mA",
    "offset_voltage_pct": "Hall补偿电压上限占母线百分比",
    "offset_voltage_mv": "Hall补偿电压硬上限，mV，与百分比限制取较小值",
    "offset_settle_ms": "Hall补偿采样前等待，ms；不是自动稳速检测",
    "offset_measure_ms": "Hall补偿采样时间，ms",
    "flux_iq_ma": "磁链辨识测量Iq，mA",
    "flux_voltage_pct": "磁链辨识电压上限占母线百分比",
    "flux_voltage_mv": "磁链辨识电压硬上限，mV，与百分比限制取较小值",
    "flux_settle_ms": "磁链采样前等待，ms；不是自动稳速检测",
    "flux_measure_ms": "磁链采样时间，ms",
    "hall_scan_mv": "Hall角度标定开环扫描电压，mV；不是闭环限流",
    "hall_scan_mhz": "Hall角度标定电频率，mHz；4对极时5000对应75 rpm",
    "stall_speed_rpm": "堵转低速门槛，rpm；Iq指令同时达到当前限幅才开始计时",
    "stall_time_ms": "堵转条件连续成立时间，ms",
    "can_node_id": "标准CAN节点ID；接受候选后立即切换，需同步修改对端ID",
}
SILENT_TIMEOUT_MS = 1500

# demo uses the vendor gains divided by 2**shift, not Hall's Q15/Q20 gains.
DEMO_PARAMETER_NAMES = [
    "demo_rs_mohm", "demo_ls_uh", "demo_bandwidth_rad_s", "demo_tune_bus_mv",
    "demo_current_limit_ma", "demo_acc_rpm_ms", "demo_dec_rpm_ms",
    *[f"demo_{axis}_{part}" for axis in ("id", "iq", "speed")
      for part in ("kp", "ki", "kp_shift", "ki_shift")],
    "demo_hall_scan_mv", "demo_hall_angle_inc",
    "demo_hall_align_ms",
]
for _name in DEMO_PARAMETER_NAMES:
    if _name.endswith("_shift"):
        PARAMETER_RANGES[_name] = (0, 15)
    elif _name.endswith(("_kp", "_ki")):
        PARAMETER_RANGES[_name] = (0, 32767)
    PARAMETER_HELP[_name] = "demo原厂参数；停机修改立即生效，保存沿用原厂Flash参数表"
PARAMETER_RANGES.update({
    "demo_rs_mohm": (1, 10000), "demo_ls_uh": (1, 100000),
    "demo_bandwidth_rad_s": (1, 65535), "demo_tune_bus_mv": (18000, 60000),
    "demo_current_limit_ma": (0, 50000), "demo_acc_rpm_ms": (1, 1000),
    "demo_dec_rpm_ms": (1, 1000), "demo_hall_scan_mv": (100, 4800),
    "demo_hall_angle_inc": (1, 100),
    "demo_hall_align_ms": (10, 10000),
})
PARAMETER_HELP.update({
    "demo_rs_mohm": "原厂Rs相参数，mΩ；线间测量需除以2",
    "demo_ls_uh": "原厂Ls相参数，µH；线间测量需除以2，不是独立Ld/Lq",
    "demo_bandwidth_rad_s": "原厂自动整定带宽rad/s，888 Hz约5579；算法可能调整实际带宽，本字段不写Flash",
    "demo_tune_bus_mv": "原厂电流PI整定额定母线，mV；不改变硬件采样定标",
    "demo_current_limit_ma": "运行电流限幅，mA；不代替学习/辨识电流约束",
    "demo_acc_rpm_ms": "原厂加速斜率rpm/ms，30相当于30000 rpm/s",
    "demo_dec_rpm_ms": "原厂减速斜率rpm/ms，30相当于30000 rpm/s",
    "demo_id_ki": "原厂d轴Ki；现有Flash保存槽取q轴Ki，独立值仅在本次RAM运行有效",
    "demo_hall_angle_inc": "原厂学习角度步长；沿用库角度单位，仅RAM生效",
    "demo_hall_align_ms": "原厂Hall学习对齐时间ms，仅RAM生效",
})


class HistoryLineEdit(QLineEdit):
    """支持上下方向键浏览已发送命令的输入框。"""

    def __init__(self) -> None:
        super().__init__()
        self.history: list[str] = []
        self.history_index = 0
        self.draft = ""

    def add_history(self, command: str) -> None:
        if command and (not self.history or self.history[-1] != command):
            self.history.append(command)
        self.history_index = len(self.history)
        self.draft = ""

    def keyPressEvent(self, event) -> None:  # noqa: N802
        if event.key() == Qt.Key_Up and self.history:
            if self.history_index == len(self.history):
                self.draft = self.text()
            if self.history_index > 0:
                self.history_index -= 1
            self.setText(self.history[self.history_index])
            self.end(False)
            return
        if event.key() == Qt.Key_Down and self.history:
            if self.history_index < len(self.history) - 1:
                self.history_index += 1
                self.setText(self.history[self.history_index])
            else:
                self.history_index = len(self.history)
                self.setText(self.draft)
            self.end(False)
            return
        super().keyPressEvent(event)


class MotorHost(QMainWindow):
    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle("AT_foc_hall 电机调试工具")
        self.resize(1120, 760)
        self.serial_port: serial.Serial | None = None
        self.firmware_type = "unknown"
        self.reply_pending = ""
        self.command_buttons: list[tuple[QPushButton, str, str]] = []
        self.commissioning_buttons: list[QPushButton] = []
        self.debug_enabled = False
        self.mode_started = {"open": False, "current": False, "speed": False}
        self.pending_mode_request: str | None = None
        self.mode_buttons: dict[str, QPushButton] = {}
        self.active_values: dict[str, str] = {}
        self.silent_active = False
        self.silent_buffer = ""
        self.silent_timer = QTimer(self)
        self.silent_timer.setSingleShot(True)
        self.silent_timer.timeout.connect(self.silent_query_timeout)
        self._build_ui()
        self.apply_firmware_profile("unknown")
        self.refresh_ports()
        self.rx_timer = QTimer(self)
        self.rx_timer.timeout.connect(self.read_serial)
        self.rx_timer.start(30)

    def _build_ui(self) -> None:
        root = QWidget()
        layout = QVBoxLayout(root)
        self.main_layout = layout
        self.setCentralWidget(root)

        connection = QHBoxLayout()
        connection.addWidget(QLabel("串口"))
        self.port_combo = QComboBox()
        self.port_combo.setMinimumWidth(160)
        connection.addWidget(self.port_combo)
        refresh = QPushButton("刷新")
        refresh.clicked.connect(self.refresh_ports)
        connection.addWidget(refresh)
        self.connect_button = QPushButton("连接")
        self.connect_button.clicked.connect(self.toggle_connection)
        connection.addWidget(self.connect_button)
        connection.addWidget(QLabel(f"{BAUDRATE} 8N1"))
        self.connection_state = QLabel("未连接")
        connection.addWidget(self.connection_state)
        self.firmware_label = QLabel("固件：未识别")
        connection.addWidget(self.firmware_label)
        reset_button = QPushButton("复位设备")
        reset_button.clicked.connect(self.reset_device)
        connection.addWidget(reset_button)
        stop_button = QPushButton("停止电机")
        stop_button.clicked.connect(self.stop_motor)
        connection.addWidget(stop_button)
        connection.addStretch()
        layout.addLayout(connection)

        tabs = QTabWidget()
        self.tabs = tabs
        tabs.addTab(self._control_tab(), "运行控制")
        tabs.addTab(self._commissioning_tab(), "自动辨识")
        self.parameter_tab_index = tabs.addTab(self._parameter_tab(), "参数与诊断")
        self.trace_view = TraceView(self.send_command)
        self.trace_tab_index = tabs.addTab(self.trace_view, "调试曲线")
        tabs.currentChanged.connect(self.on_tab_changed)
        layout.addWidget(tabs)

        console_group = QGroupBox("串口日志")
        self.console_group = console_group
        console_layout = QVBoxLayout(console_group)
        self.console = QPlainTextEdit()
        self.console.setReadOnly(True)
        self.console.setStyleSheet(
            "QPlainTextEdit {background:#101418;color:#d8dee9;font-family:Consolas;}"
        )
        console_layout.addWidget(self.console)
        layout.addWidget(console_group, 1)

        self.command_bar = QWidget()
        command_layout = QHBoxLayout(self.command_bar)
        command_layout.setContentsMargins(0, 0, 0, 0)
        self.command_entry = HistoryLineEdit()
        self.command_entry.setPlaceholderText("输入原始CLI命令，按回车发送")
        self.command_entry.returnPressed.connect(self.send_entry)
        command_layout.addWidget(self.command_entry, 1)
        send = QPushButton("发送")
        send.clicked.connect(self.send_entry)
        command_layout.addWidget(send)
        clear = QPushButton("清屏")
        clear.clicked.connect(self.console.clear)
        command_layout.addWidget(clear)
        layout.addWidget(self.command_bar)

    def _command_button(self, text: str, command: str,
                        commissioning: bool = False) -> QPushButton:
        button = QPushButton(text)
        button.setProperty("command", command)
        button.clicked.connect(lambda _checked=False, target=button:
                               self.send_command(target.property("command")))
        self.command_buttons.append((button, text, command))
        if commissioning:
            self.commissioning_buttons.append(button)
        return button

    def _control_tab(self) -> QWidget:
        tab = QWidget()
        layout = QVBoxLayout(tab)
        quick = QGridLayout()
        quick.addWidget(self._command_button("状态", "status"), 0, 0)
        self.debug_button = QPushButton("打开调试")
        self.debug_button.clicked.connect(self.toggle_debug)
        quick.addWidget(self.debug_button, 0, 1)
        for column, item in enumerate([
            ("故障", "fault"), ("清故障", "fault clear")], 2):
            quick.addWidget(self._command_button(*item), 0, column)
        quick.addWidget(self._command_button("帮助", "help"), 0, 4)
        layout.addLayout(quick)

        open_group = QGroupBox("开环控制")
        open_layout = QGridLayout(open_group)
        self.open_vd = QLineEdit("0")
        self.open_vq = QLineEdit("1440")
        self.open_freq = QLineEdit("10000")
        self.open_accel = QLineEdit("20000")
        for col, (name, edit) in enumerate([
            ("Vd (mV)", self.open_vd), ("Vq (mV)", self.open_vq),
            ("电频率 (mHz)", self.open_freq),
            ("加速度 (mHz/s)", self.open_accel),
        ]):
            open_layout.addWidget(QLabel(name), 0, col)
            open_layout.addWidget(edit, 1, col)
        self.mode_buttons["open"] = QPushButton("开环启动")
        self.mode_buttons["open"].clicked.connect(
            lambda: self.send_mode("open"))
        open_layout.addWidget(self.mode_buttons["open"], 1, 4, 1, 2)
        layout.addWidget(open_group)

        closed_group = QGroupBox("闭环控制")
        closed = QGridLayout(closed_group)
        self.current_id = QLineEdit("0")
        self.current_iq = QLineEdit("1000")
        self.speed_rpm = QLineEdit("150")
        closed.addWidget(QLabel("Id (mA)"), 0, 0)
        closed.addWidget(self.current_id, 1, 0)
        closed.addWidget(QLabel("Iq (mA)"), 0, 1)
        closed.addWidget(self.current_iq, 1, 1)
        self.mode_buttons["current"] = QPushButton("电流环启动")
        self.mode_buttons["current"].clicked.connect(
            lambda: self.send_mode("current"))
        closed.addWidget(self.mode_buttons["current"], 1, 2, 1, 2)
        closed.addWidget(QLabel("速度 (rpm)"), 0, 4)
        closed.addWidget(self.speed_rpm, 1, 4)
        self.mode_buttons["speed"] = QPushButton("速度启动")
        self.mode_buttons["speed"].clicked.connect(
            lambda: self.send_mode("speed"))
        closed.addWidget(self.mode_buttons["speed"], 1, 5)
        layout.addWidget(closed_group)
        return tab

    def _commissioning_tab(self) -> QWidget:
        tab = QWidget()
        layout = QVBoxLayout(tab)
        identify_items = [
            ("ADC零偏", "calibrate current_offset"),
            ("辨识Rs", "identify resistance"),
            ("辨识Ld/Lq", "identify inductance"),
            ("Hall角度", "calibrate hall angle"),
            ("Hall补偿", "calibrate hall offset"),
            ("计算电流PI", "calculate current_pi"),
            ("辨识磁链", "identify flux"),
        ]
        group = QGroupBox("辨识步骤")
        grid = QGridLayout(group)
        for index, item in enumerate(identify_items):
            grid.addWidget(self._command_button(*item, commissioning=True),
                           index // 5, index % 5)
        layout.addWidget(group)
        layout.addStretch(1)
        return tab

    def _parameter_tab(self) -> QWidget:
        tab = QWidget()
        layout = QVBoxLayout(tab)
        grid = QGridLayout()
        items = [
            ("正常方向", "motor direction normal"),
            ("反向方向", "motor direction reverse"),
            ("查询方向", "motor direction"),
            ("INFO日志", "log level 4"),
            ("版本", "version"),
        ]
        for index, item in enumerate(items):
            grid.addWidget(self._command_button(*item), index // 5, index % 5)
        layout.addLayout(grid)
        self.parameter_note = QLabel("hall：编辑候选参数，检查差异后接受，再保存。")
        self.parameter_note.setWordWrap(True)
        layout.addWidget(self.parameter_note)
        form = QFormLayout()
        self.parameter_name = QComboBox()
        self.parameter_name.addItems(PARAMETER_NAMES)
        self.parameter_value = QLineEdit()
        self.parameter_name.currentTextChanged.connect(
            self.on_parameter_name_changed)
        form.addRow("参数名", self.parameter_name)
        form.addRow("数值", self.parameter_value)
        set_button = QPushButton("设置候选参数")
        self.parameter_set_button = set_button
        set_button.clicked.connect(self.send_parameter)
        form.addRow(set_button)
        layout.addLayout(form)

        parameter_items = [
            ("活动参数", "motor params active"),
            ("存储状态", "motor params storage"),
            ("加载Flash", "motor params load"),
            ("恢复默认", "motor params defaults"),
            ("丢弃候选", "motor commissioning discard"),
            ("参数差异", "motor commissioning diff"),
            ("接受候选", "motor commissioning accept"),
            ("保存参数", "motor commissioning save"),
        ]
        passive = {"motor commissioning diff", "motor params active",
                   "motor params storage"}
        group = QGroupBox("参数管理")
        manage = QGridLayout(group)
        for index, item in enumerate(parameter_items):
            manage.addWidget(self._command_button(
                *item, commissioning=item[1] not in passive),
                index // 5, index % 5)
        layout.addWidget(group)
        layout.addStretch(1)
        return tab

    def refresh_ports(self) -> None:
        selected = self.port_combo.currentText()
        ports = [item.device for item in list_ports.comports()]
        self.port_combo.clear()
        self.port_combo.addItems(ports)
        if selected in ports:
            self.port_combo.setCurrentText(selected)

    def toggle_connection(self) -> None:
        if self.serial_port and self.serial_port.is_open:
            self.disconnect()
            return
        port = self.port_combo.currentText()
        if not port:
            QMessageBox.warning(self, "提示", "没有选择串口")
            return
        try:
            self.serial_port = serial.Serial(port, BAUDRATE, timeout=0,
                                             write_timeout=1)
        except serial.SerialException as exc:
            QMessageBox.critical(self, "连接失败", str(exc))
            return
        self.connect_button.setText("断开")
        self.connection_state.setText(f"已连接 {port}")
        self.append_console(f"[{datetime.now():%H:%M:%S}] 已连接 {port}\n")
        self.active_values.clear()
        self.reply_pending = ""
        self.apply_firmware_profile("unknown")
        self.send_command("version")

    def disconnect(self) -> None:
        if self.serial_port:
            try:
                self.serial_port.close()
            except serial.SerialException:
                pass
        self.serial_port = None
        self.finish_silent_query()
        self.reply_pending = ""
        self.active_values.clear()
        self.apply_firmware_profile("unknown")
        self.debug_enabled = False
        self.debug_button.setText("打开调试")
        self.trace_view.reset_connection()
        self.connect_button.setText("连接")
        self.connection_state.setText("未连接")
        self.set_commissioning_busy(False)
        self.reset_mode_buttons()

    def read_serial(self) -> None:
        if not self.serial_port or not self.serial_port.is_open:
            return
        try:
            waiting = self.serial_port.in_waiting
            if not waiting:
                return
            text = self.serial_port.read(waiting).decode("utf-8", errors="replace")
        except serial.SerialException as exc:
            self.append_console(f"\n[串口错误] {exc}\n")
            self.disconnect()
            return
        self.process_replies(text)
        if self.silent_active:
            text = self.consume_silent_reply(text)
            if not text:
                return
        text = self.trace_view.consume(text)
        for line in text.splitlines():
            if line.startswith("OK logid="):
                self.debug_enabled = "enabled=1" in line
            elif line == "OK debug 0" or line.startswith("OK capture armed"):
                self.debug_enabled = False
            self.debug_button.setText("关闭调试" if self.debug_enabled else "打开调试")
        self.append_console(text)
        if ("OK commissioning complete" in text or
                "ERR commissioning rejected_or_failed" in text):
            self.set_commissioning_busy(False)

    def send_command(self, command: str) -> bool:
        if not self.command_supported(command):
            self.append_console(f"[当前固件不支持或尚未识别] {command}\n")
            return False
        if not self.serial_port or not self.serial_port.is_open:
            QMessageBox.warning(self, "提示", "请先连接串口")
            return False
        parts = command.split()
        run_mode = (parts[0] if len(parts) >= 2 and parts[0] in self.mode_buttons
                    and parts[1] in ("start", "set") else None)
        if run_mode and self.pending_mode_request is not None:
            self.append_console("[等待运行命令回显，请勿重复发送]\n")
            return False
        try:
            self.serial_port.write((command.strip() + "\r\n").encode("ascii"))
        except (serial.SerialException, UnicodeEncodeError) as exc:
            QMessageBox.critical(self, "发送失败", str(exc))
            return False
        if run_mode:
            self.pending_mode_request = run_mode
        self.append_console(f"> {command.strip()}\n")
        if command.startswith(("calibrate ",
                               "identify ", "calculate current_pi")):
            self.set_commissioning_busy(True)

        return True

    def command_supported(self, command: str) -> bool:
        if command in ("version", "help", "motor stop", "debug 0"):
            return True
        if self.firmware_type == "unknown":
            return False
        if self.firmware_type == "hall":
            return not command.startswith("identify winding")
        if command.startswith("motor param set "):
            parts = command.split()
            return len(parts) == 5 and parts[3] in DEMO_PARAMETER_NAMES
        if command.startswith(("identify ", "calibrate ", "calculate ")):
            return command in ("identify winding", "calibrate current_offset",
                               "calibrate hall angle", "calculate current_pi")
        if command.startswith("motor "):
            return command in ("motor stop", "motor params active", "motor params storage",
                               "motor commissioning save", "motor commissioning abort")
        if command in ("status", "fault", "fault clear", "reset"):
            return True
        return command.startswith(("speed start ", "speed set ", "speed stop",
                                   "current start ", "current set ", "current stop",
                                   "open start ", "open set ", "open stop",
                                   "debug ", "logid ", "logdetail ", "lograte ", "logcapture "))

    def apply_firmware_profile(self, firmware: str) -> None:
        self.firmware_type = firmware
        self.firmware_label.setText({"unknown": "固件：未识别", "hall": "固件：hall",
                                     "demo": "固件：demo（停机直改）"}[firmware])
        demo = firmware == "demo"
        for button, label, command in self.command_buttons:
            mapped, title = command, label
            if demo and command == "identify resistance":
                mapped, title = "identify winding", "绕组参数辨识Rs/Ls"
            elif demo and command == "calibrate hall angle":
                title = "原厂Hall学习（自动保存）"
            elif demo and command == "calculate current_pi":
                title = "原厂电流PI自动整定"
            button.setProperty("command", mapped)
            button.setText(title)
            button.setVisible(not demo or self.command_supported(mapped))
            button.setEnabled(firmware != "unknown" or mapped in ("version", "help"))
        self.parameter_name.blockSignals(True)
        self.parameter_name.clear()
        self.parameter_name.addItems(DEMO_PARAMETER_NAMES if demo else PARAMETER_NAMES)
        self.parameter_name.blockSignals(False)
        self.parameter_set_button.setText("停机修改（立即生效）" if demo else "设置候选参数")
        self.parameter_set_button.setEnabled(firmware != "unknown")
        self.parameter_note.setText(
            "demo：停机修改直接生效；保存沿用原厂Flash，Hall学习成功仍自动保存。"
            "极对数、CAN ID、最高转速等编译项请修改宏后烧录；无候选、独立Hall补偿或磁链辨识。"
            if demo else "hall：编辑候选参数，检查差异后接受，再保存。")
        self.open_accel.setText("0" if demo else "20000")
        self.open_accel.setEnabled(not demo)
        self.trace_view.set_firmware(firmware)
        self.reset_mode_buttons()
        self.debug_enabled = False
        self.debug_button.setText("打开调试")
        for button in self.mode_buttons.values():
            button.setEnabled(firmware != "unknown")
        self.show_current_parameter_value()

    def process_replies(self, text: str) -> None:
        self.reply_pending += text
        while "\n" in self.reply_pending:
            line, self.reply_pending = self.reply_pending.split("\n", 1)
            line = line.strip()
            if line.startswith("OK fw="):
                target = "demo" if "firmware=demo" in line else "hall"
                if target != self.firmware_type:
                    self.active_values.clear()
                    self.apply_firmware_profile(target)
                    self.request_active_parameters()
            if (line.startswith("OK commissioning complete") or
                    line.startswith("ERR commissioning") or line.startswith("ERR ")):
                self.set_commissioning_busy(False)
            acknowledged_mode = None
            for mode in self.mode_buttons:
                if line.startswith((f"OK {mode} started", f"OK {mode} updated",
                                    f"OK {mode} requested")):
                    acknowledged_mode = mode
            if line.startswith("OK run requested"):
                fields = dict(token.split("=", 1) for token in line.split() if "=" in token)
                acknowledged_mode = fields.get("mode", self.pending_mode_request)
            if acknowledged_mode in self.mode_buttons:
                self.reset_mode_buttons()
                self.mode_started[acknowledged_mode] = True
                self.mode_buttons[acknowledged_mode].setText(
                    {"open": "开环更新", "current": "电流更新", "speed": "速度更新"}[acknowledged_mode])
            if line.startswith("ERR "):
                # A rejected edit/update says nothing about whether the motor stopped.
                self.pending_mode_request = None
            if line in ("OK stopped", "OK motor stopped", "OK open stopped",
                        "OK current stopped", "OK speed stopped", "OK resetting",
                        "OK fault clear stopped"):
                self.reset_mode_buttons()
                self.set_commissioning_busy(False)
        if len(self.reply_pending) > 8192:
            self.reply_pending = ""

    def send_entry(self) -> None:
        command = self.command_entry.text().strip()
        if command:
            self.send_command(command)
            self.command_entry.add_history(command)
            self.command_entry.clear()

    def send_open(self, action: str) -> None:
        self.send_command(f"open {action} {self.open_vd.text()} {self.open_vq.text()} "
                          f"{self.open_freq.text()} {self.open_accel.text()}")

    def send_current(self, action: str) -> None:
        self.send_command(f"current {action} {self.current_id.text()} {self.current_iq.text()}")

    def send_speed(self, action: str) -> None:
        self.send_command(f"speed {action} {self.speed_rpm.text()}")

    def send_mode(self, mode: str) -> None:
        if self.firmware_type == "unknown":
            return
        action = "set" if self.mode_started[mode] else "start"
        if mode == "open":
            self.send_open(action)
        elif mode == "current":
            self.send_current(action)
        else:
            self.send_speed(action)

    def stop_motor(self) -> None:
        self.send_command("motor stop")

    def reset_device(self) -> None:
        self.send_command("reset")

    def reset_mode_buttons(self) -> None:
        self.pending_mode_request = None
        self.mode_started = {"open": False, "current": False, "speed": False}
        labels = {"open": "开环启动", "current": "电流环启动",
                  "speed": "速度启动"}
        for mode, button in self.mode_buttons.items():
            button.setText(labels[mode])

    def send_parameter(self) -> None:
        name = self.parameter_name.currentText().strip()
        value = self.parameter_value.text().strip()
        if not name or not value:
            QMessageBox.warning(self, "提示", "请输入参数名和数值")
            return
        if name in PARAMETER_RANGES:
            try:
                number = int(value)
                low, high = PARAMETER_RANGES[name]
                if not low <= number <= high:
                    raise ValueError
            except ValueError:
                QMessageBox.warning(self, "参数范围", f"{name}: {PARAMETER_RANGES[name]}")
                return
        self.send_command(f"motor param set {name} {value}")

    # ---- 参数当前值静默读取：不在串口日志中回显命令与应答 ----
    def on_tab_changed(self, index: int) -> None:
        is_trace = index == self.trace_tab_index
        self.console_group.setVisible(not is_trace)
        self.command_bar.setVisible(not is_trace)
        self.main_layout.setStretchFactor(self.tabs, 1 if is_trace else 0)
        if index == self.parameter_tab_index:
            self.request_active_parameters()

    def on_parameter_name_changed(self, _name: str) -> None:
        self.show_current_parameter_value()
        self.request_active_parameters()

    def show_current_parameter_value(self) -> None:
        name = self.parameter_name.currentText().strip()
        hint = PARAMETER_HELP.get(name, name)
        if name in PARAMETER_RANGES:
            low, high = PARAMETER_RANGES[name]
            hint += f"；范围 {low}..{high}"
        self.parameter_value.setToolTip(hint)
        self.parameter_name.setToolTip(hint)
        if name in self.active_values:
            self.parameter_value.setText(self.active_values[name])

    def request_active_parameters(self) -> None:
        if (self.firmware_type == "unknown" or self.silent_active or self.debug_enabled or
                not self.serial_port or not self.serial_port.is_open):
            return
        try:
            self.serial_port.write((SILENT_QUERY + "\r\n").encode("ascii"))
        except serial.SerialException:
            return
        self.silent_active = True
        self.silent_buffer = ""
        self.silent_timer.start(SILENT_TIMEOUT_MS)

    def consume_silent_reply(self, text: str) -> str:
        """吞掉params active应答块，返回需要正常显示的其余文本。"""
        self.silent_buffer += text
        buffer = self.silent_buffer
        start = buffer.find("OK params active")
        if start < 0:
            return ""
        end_key = buffer.find("hall_angle=", start)
        end = buffer.find("\n", end_key) if end_key >= 0 else -1
        if end < 0:
            return ""
        self.parse_active_parameters(buffer[start:end])
        self.finish_silent_query()
        return buffer[:start] + buffer[end + 1:]

    def parse_active_parameters(self, block: str) -> None:
        for token in block.split():
            if "=" not in token:
                continue
            key, value = token.split("=", 1)
            names = [key] if key in DEMO_PARAMETER_NAMES else ACTIVE_FIELD_MAP.get(key)
            if not names:
                continue
            for name, part in zip(names, value.split("/")):
                self.active_values[name] = part
        self.show_current_parameter_value()

    def silent_query_timeout(self) -> None:
        # 无应答或格式不符：把缓存内容按原样补显示，避免丢日志。
        rest = self.silent_buffer
        self.finish_silent_query()
        if rest:
            self.append_console(rest)

    def finish_silent_query(self) -> None:
        self.silent_timer.stop()
        self.silent_active = False
        self.silent_buffer = ""

    def toggle_debug(self) -> None:
        if self.debug_enabled:
            self.trace_view.stop()
        else:
            self.send_command("debug 1")

    def set_commissioning_busy(self, busy: bool) -> None:
        for button in self.commissioning_buttons:
            button.setEnabled(not busy and self.firmware_type != "unknown")

    def append_console(self, text: str) -> None:
        self.console.moveCursor(QTextCursor.End)
        self.console.insertPlainText(text)
        self.console.moveCursor(QTextCursor.End)

    def closeEvent(self, event) -> None:  # noqa: N802
        self.disconnect()
        event.accept()


if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MotorHost()
    window.show()
    sys.exit(app.exec())
