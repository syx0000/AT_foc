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
]


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
        self.commissioning_buttons: list[QPushButton] = []
        self.debug_enabled = False
        self.mode_started = {"open": False, "current": False, "speed": False}
        self.mode_buttons: dict[str, QPushButton] = {}
        self._build_ui()
        self.refresh_ports()
        self.rx_timer = QTimer(self)
        self.rx_timer.timeout.connect(self.read_serial)
        self.rx_timer.start(30)

    def _build_ui(self) -> None:
        root = QWidget()
        layout = QVBoxLayout(root)
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
        reset_button = QPushButton("复位设备")
        reset_button.clicked.connect(self.reset_device)
        connection.addWidget(reset_button)
        connection.addStretch()
        layout.addLayout(connection)

        tabs = QTabWidget()
        tabs.addTab(self._control_tab(), "运行控制")
        tabs.addTab(self._commissioning_tab(), "自动辨识")
        tabs.addTab(self._parameter_tab(), "参数与诊断")
        layout.addWidget(tabs)

        console_group = QGroupBox("串口日志")
        console_layout = QVBoxLayout(console_group)
        self.console = QPlainTextEdit()
        self.console.setReadOnly(True)
        self.console.setStyleSheet(
            "QPlainTextEdit {background:#101418;color:#d8dee9;font-family:Consolas;}"
        )
        console_layout.addWidget(self.console)
        layout.addWidget(console_group, 1)

        command_layout = QHBoxLayout()
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
        layout.addLayout(command_layout)

    def _command_button(self, text: str, command: str,
                        commissioning: bool = False) -> QPushButton:
        button = QPushButton(text)
        button.clicked.connect(lambda _checked=False, value=command:
                               self.send_command(value))
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
        stop_button = QPushButton("停止电机")
        stop_button.clicked.connect(self.stop_motor)
        quick.addWidget(stop_button, 0, 4)
        quick.addWidget(self._command_button("帮助", "help"), 0, 5)
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
        grid = QGridLayout(tab)
        items = [
            ("参数差异", "motor commissioning diff"),
            ("接受候选", "motor commissioning accept"),
            ("保存参数", "motor commissioning save"),
            ("ADC零偏", "calibrate current_offset"),
            ("辨识Rs", "identify resistance"),
            ("辨识Ld/Lq", "identify inductance"),
            ("Hall角度", "calibrate hall angle"),
            ("Hall补偿", "calibrate hall offset"),
            ("计算电流PI", "calculate current_pi"),
            ("丢弃候选", "motor commissioning discard"),
            ("活动参数", "motor params active"),
            ("存储状态", "motor params storage"),
            ("加载Flash", "motor params load"),
            ("恢复默认", "motor params defaults"),
        ]
        passive = {"motor commissioning diff", "motor params active",
                   "motor params storage"}
        for index, item in enumerate(items):
            grid.addWidget(self._command_button(
                *item, commissioning=item[1] not in passive), index // 5, index % 5)
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
        form = QFormLayout()
        self.parameter_name = QComboBox()
        self.parameter_name.addItems(PARAMETER_NAMES)
        self.parameter_value = QLineEdit()
        form.addRow("参数名", self.parameter_name)
        form.addRow("数值", self.parameter_value)
        set_button = QPushButton("设置候选参数")
        set_button.clicked.connect(self.send_parameter)
        form.addRow(set_button)
        layout.addLayout(form)
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

    def disconnect(self) -> None:
        if self.serial_port:
            try:
                self.serial_port.close()
            except serial.SerialException:
                pass
        self.serial_port = None
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
        self.append_console(text)
        if ("OK commissioning complete" in text or
                "ERR commissioning rejected_or_failed" in text):
            self.set_commissioning_busy(False)

    def send_command(self, command: str) -> None:
        if not self.serial_port or not self.serial_port.is_open:
            QMessageBox.warning(self, "提示", "请先连接串口")
            return
        try:
            self.serial_port.write((command.strip() + "\r\n").encode("ascii"))
        except (serial.SerialException, UnicodeEncodeError) as exc:
            QMessageBox.critical(self, "发送失败", str(exc))
            return
        self.append_console(f"> {command.strip()}\n")
        if command.startswith(("calibrate ",
                               "identify ", "calculate current_pi")):
            self.set_commissioning_busy(True)

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
        action = "set" if self.mode_started[mode] else "start"
        if mode == "open":
            self.send_open(action)
        elif mode == "current":
            self.send_current(action)
        else:
            self.send_speed(action)
        if self.serial_port and self.serial_port.is_open:
            if action == "start":
                self.reset_mode_buttons()
            self.mode_started[mode] = True
            labels = {"open": "开环更新", "current": "电流更新",
                      "speed": "速度更新"}
            self.mode_buttons[mode].setText(labels[mode])

    def stop_motor(self) -> None:
        self.send_command("motor stop")
        if self.serial_port and self.serial_port.is_open:
            self.reset_mode_buttons()

    def reset_device(self) -> None:
        self.send_command("reset")
        if self.serial_port and self.serial_port.is_open:
            self.reset_mode_buttons()

    def reset_mode_buttons(self) -> None:
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
        self.send_command(f"motor param set {name} {value}")

    def toggle_debug(self) -> None:
        self.send_command("debug 0" if self.debug_enabled else "debug 1")
        if self.serial_port and self.serial_port.is_open:
            self.debug_enabled = not self.debug_enabled
            self.debug_button.setText(
                "关闭调试" if self.debug_enabled else "打开调试")

    def set_commissioning_busy(self, busy: bool) -> None:
        for button in self.commissioning_buttons:
            button.setEnabled(not busy)

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
