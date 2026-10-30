"""Bounded telemetry decoder and oscilloscope for the firmware @T protocol."""
from __future__ import annotations

import csv
import math
from collections import deque
from dataclasses import dataclass

from PySide6.QtCore import QPointF, QTimer, QRect, QMargins
from PySide6.QtGui import QPainter, QColor, QPen
from PySide6.QtCharts import QChart, QChartView, QLineSeries, QValueAxis
from PySide6.QtCore import Qt
from PySide6.QtWidgets import (QWidget, QVBoxLayout, QHBoxLayout, QComboBox,
    QPushButton, QLabel, QCheckBox, QFileDialog, QMessageBox, QRubberBand)

# Fixed per-channel palette, independent of Qt chart themes and unit grouping.
TRACE_COLORS = ('#d62728', '#008548', '#0066cc', '#a64d00', '#a020c0',
                '#008b95', '#d14b7c', '#6b5b20', '#4f46b8')


class ScopeChartView(QChartView):
    """Explicit axis ranges so incoming samples cannot undo a manual view."""
    def __init__(self, chart, x_axis, y_axis, owner):
        super().__init__(chart)
        self.x_axis, self.y_axis, self.owner = x_axis, y_axis, owner
        self.y_axes = [y_axis]
        self.manual_y_axes = set()
        self.auto_x = self.auto_y = True
        self.gesture = None
        self.band = QRubberBand(QRubberBand.Rectangle, self.viewport())
        self.setToolTip('靠近坐标轴滚轮只缩放该轴；图内滚轮缩放XY；左键框选；中键或Shift+左键平移；双击恢复')

    def chart_point(self, point):
        return self.chart().mapFromScene(self.mapToScene(point.toPoint()))

    def wheelEvent(self, event):
        area = self.chart().plotArea()
        point = self.chart_point(event.position())
        if area.width() <= 0 or area.height() <= 0:
            event.ignore()
            return
        candidates = []
        if area.left()-12 <= point.x() <= area.right()+12 and point.y() >= area.bottom()-18:
            candidates.append((abs(point.y()-area.bottom()), 'x', None))
        if area.top()-12 <= point.y() <= area.bottom()+12:
            if point.x() <= area.left()+18:
                candidates.append((abs(point.x()-area.left()), 'y', self.y_axes[0]))
            if len(self.y_axes)>1 and point.x() >= area.right()-18:
                right_axes = self.y_axes[1:]
                band_width = max(1, (self.chart().boundingRect().right()-area.right())/len(right_axes))
                index = min(len(right_axes)-1, int(max(0, point.x()-area.right())/band_width+.5))
                candidates.append((abs(point.x()-area.right()), 'y', right_axes[index]))
        if candidates:
            _, mode, target_axis = min(candidates, key=lambda item: item[0])
        elif area.contains(point):
            mode, target_axis = 'xy', None
        else:
            event.ignore()
            return
        steps = event.angleDelta().y() / 120
        if not steps:
            event.ignore()
            return
        factor = 1.2 ** (-max(-10, min(10, steps)))
        fx = max(0, min(1, (point.x()-area.left())/area.width()))
        fy = max(0, min(1, (area.bottom()-point.y())/area.height()))
        def zoom(axis, fraction):
            low, high = axis.min(), axis.max()
            anchor = low+(high-low)*fraction
            return anchor+(low-anchor)*factor, anchor+(high-anchor)*factor
        self.owner.manual_range(self,
            zoom(self.x_axis, fx) if mode != 'y' else None,
            zoom(target_axis or self.y_axis, fy) if mode != 'x' else None,
            y_axis=target_axis)
        event.accept()

    def mousePressEvent(self, event):
        point = self.chart_point(event.position())
        if self.chart().plotArea().contains(point) and event.button() in (Qt.LeftButton, Qt.MiddleButton):
            pan = event.button() == Qt.MiddleButton or bool(event.modifiers() & Qt.ShiftModifier)
            self.gesture = 'pan' if pan else 'box'
            self.origin = event.position()
            self.start_point = point
            self.start_area = self.chart().plotArea()
            self.start_ranges = (self.x_axis.min(), self.x_axis.max(), self.y_axis.min(), self.y_axis.max())
            if pan:
                self.viewport().setCursor(Qt.ClosedHandCursor)
            else:
                self.band.setGeometry(QRect(self.origin.toPoint(), self.origin.toPoint()))
                self.band.show()
            event.accept()
            return
        super().mousePressEvent(event)

    def mouseMoveEvent(self, event):
        if self.gesture:
            if self.gesture == 'box':
                self.band.setGeometry(QRect(self.origin.toPoint(), event.position().toPoint()).normalized())
            else:
                self.pan_to(event.position())
            event.accept()
            return
        super().mouseMoveEvent(event)

    def pan_to(self, position):
        area = self.start_area
        if area.width() <= 0 or area.height() <= 0:
            return
        delta = self.chart_point(position)-self.start_point
        low_x, high_x, low_y, high_y = self.start_ranges
        dx = -delta.x()/area.width()*(high_x-low_x)
        dy = delta.y()/area.height()*(high_y-low_y)
        self.owner.manual_range(self, (low_x+dx, high_x+dx), (low_y+dy, high_y+dy))

    def mouseReleaseEvent(self, event):
        if self.gesture and event.button() in (Qt.LeftButton, Qt.MiddleButton):
            if self.gesture == 'pan':
                self.pan_to(event.position())
            else:
                area = self.chart().plotArea()
                end = self.chart_point(event.position())
                start = self.start_point
                left, right = sorted((max(area.left(), min(area.right(), start.x())),
                                      max(area.left(), min(area.right(), end.x()))))
                top, bottom = sorted((max(area.top(), min(area.bottom(), start.y())),
                                      max(area.top(), min(area.bottom(), end.y()))))
                x_range = y_range = None
                if right-left >= 5:
                    span = self.x_axis.max()-self.x_axis.min()
                    x_range = (self.x_axis.min()+(left-area.left())/area.width()*span,
                               self.x_axis.min()+(right-area.left())/area.width()*span)
                if bottom-top >= 5:
                    span = self.y_axis.max()-self.y_axis.min()
                    y_range = (self.y_axis.min()+(area.bottom()-bottom)/area.height()*span,
                               self.y_axis.min()+(area.bottom()-top)/area.height()*span)
                self.owner.manual_range(self, x_range, y_range)
            self.gesture = None
            self.band.hide()
            self.viewport().unsetCursor()
            event.accept()
            return
        super().mouseReleaseEvent(event)

    def mouseDoubleClickEvent(self, event):
        if event.button() == Qt.LeftButton:
            self.gesture = None
            self.band.hide()
            self.viewport().unsetCursor()
            self.owner.restore_auto()
            event.accept()
            return
        super().mouseDoubleClickEvent(event)


# (wire index, label, engineering unit, raw-to-display scale)
def channels(logid, detail, firmware='hall'):
    if logid == 10:
        if detail:
            axis = 'Id' if detail == 1 else 'Iq'
            result = [(i, label, unit, .001) for i, label, unit in (
                (0, axis+'给定', 'A'), (1, axis+'反馈', 'A'), (2, axis+'误差', 'A'),
                (3, 'P项', 'V'), (4, 'I项', 'V'), (5, '轴限幅后请求', 'V'),
                (6, '矢量限幅后输出', 'V'), (7, '限幅前P+I', 'V'))]
            if firmware == 'demo':
                labels = {5: '原厂PI限幅前输出', 6: '实际电压指令', 7: 'P+I重建'}
                result = [(i, labels.get(i, label), unit, scale) for i, label, unit, scale in result]
            return result
        return [(i, label, unit, .001) for i, label, unit in (
            (0, 'Id给定', 'A'), (1, 'Id反馈', 'A'), (2, 'Iq给定', 'A'),
            (3, 'Iq反馈', 'A'), (4, 'Ud输出', 'V'), (5, 'Uq输出', 'V'))]
    if logid == 20:
        return [(i, label, unit, scale) for i, label, unit, scale in (
            (0, '目标转速', 'rpm', .001), (1, '斜坡转速', 'rpm', .001),
            (2, '反馈转速', 'rpm', .001), (3, 'Iq指令', 'A', .001),
            (4, 'Iq反馈', 'A', .001), (5, 'P项', 'A', .001),
            (6, 'I项', 'A', .001), (7, 'PI输出', 'A', .001),
            (8, 'Iq限幅', 'A', .001))]
    if logid == 30:
        return [(i, label, unit, scale) for i, label, unit, scale in (
            (0, 'U占空比', '%', .01), (1, 'V占空比', '%', .01),
            (2, 'W占空比', '%', .01), (3, 'Ia', 'A', .001),
            (4, 'Ib', 'A', .001), (5, 'Ic重构', 'A', .001),
            (6, '母线电压', 'V', .001))]
    return [(0, 'C / Iq反馈', 'A', .001), (1, 'V / 速度', 'rpm', .001),
            (2, 'P / Hall相对位置', 'deg', .001), (3, 'Iq指令', 'A', .001),
            (4, '斜坡转速', 'rpm', .001)]


@dataclass(frozen=True)
class Frame:
    logid: int
    detail: int
    seq: int
    us: int
    drops: int
    flags: int
    capture: int
    values: tuple

    @classmethod
    def parse(cls, line):
        parts = line.strip().split(',')
        if len(parts) != 18 or parts[0] != '@T':
            raise ValueError('Invalid telemetry frame')
        nums = tuple(int(v) for v in parts[1:])
        if nums[0] not in (10, 20, 30, 40) or nums[1] not in (0, 1, 2):
            raise ValueError('Unsupported telemetry group')
        if any(not 0 <= v <= 0xffffffff for v in nums[2:6]):
            raise ValueError('Invalid unsigned header')
        if nums[6] not in (0, 1) or any(not -2**31 <= v < 2**31 for v in nums[7:]):
            raise ValueError('Invalid telemetry value')
        return cls(*nums[:7], nums[7:])


class TraceView(QWidget):
    def __init__(self, send, parent=None):
        super().__init__(parent)
        self.send = send
        self.firmware = 'hall'
        self.pending = ''
        self.rows = deque(maxlen=100000)
        self.points = deque(maxlen=2000)
        self.key = None
        self.last_us = None
        self.elapsed = 0
        self.series = []
        self.chart_views = []
        self.plot_key = None
        self.bad_frames = 0
        self.lost_frames = 0
        self.last_seq = None
        self.dirty = False
        self.command_queue = deque()
        self.command_pending = False
        self.command_timer = QTimer(self)
        self.command_timer.setSingleShot(True)
        self.command_timer.timeout.connect(self.command_timeout)
        layout = QVBoxLayout(self)
        controls = QHBoxLayout()
        self.group = QComboBox()
        for label, ident in [('10 电流环PI', 10), ('20 速度环PI', 20),
                             ('30 占空比/电流', 30), ('40 CVP', 40)]:
            self.group.addItem(label, ident)
        controls.addWidget(self.group)
        self.detail = QComboBox()
        self.detail.addItems(['总览', 'D轴PI细节', 'Q轴PI细节'])
        controls.addWidget(self.detail)
        self.group.currentIndexChanged.connect(self.group_changed)
        self.rate = QComboBox()
        self.rate.addItems(['200', '500', '100', '50', '10'])
        controls.addWidget(self.rate)
        for label, handler in [('开始', self.start), ('停止', self.stop),
                ('高速抓取128点', self.capture), ('清空', self.clear), ('导出CSV', self.export)]:
            button = QPushButton(label)
            button.clicked.connect(handler)
            controls.addWidget(button)
            if label == '高速抓取128点':
                self.capture_button = button
        layout.addLayout(controls)
        navigation = QHBoxLayout()
        self.lock_x, self.lock_y = QCheckBox('锁定X'), QCheckBox('锁定Y')
        navigation.addWidget(self.lock_x)
        navigation.addWidget(self.lock_y)
        self.lock_x.toggled.connect(self.axis_lock_changed)
        self.lock_y.toggled.connect(self.axis_lock_changed)
        auto = QPushButton('恢复自动范围')
        auto.clicked.connect(self.restore_auto)
        navigation.addWidget(auto)
        navigation.addWidget(QLabel('靠近坐标轴滚轮缩放该轴 · 图内滚轮缩放XY · 左键框选 · 中键/Shift+左键平移 · 双击恢复'))
        navigation.addStretch()
        layout.addLayout(navigation)
        self.note = QLabel('C=逻辑Iq，V=逻辑机械转速，P=选择40组后的Hall相对位置；非绝对位置。')
        layout.addWidget(self.note)
        self.legend = QHBoxLayout()
        layout.addLayout(self.legend)
        self.plots = QHBoxLayout()
        layout.addLayout(self.plots, 1)
        self.status = QLabel('等待数据；标志包含运行、限压、PWM、采样及速度/位置有效性')
        self.status.setWordWrap(True)
        self.note.setWordWrap(True)
        layout.addWidget(self.status)
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.redraw)
        self.timer.start(50)
        self.detail.currentIndexChanged.connect(self.preview_selection)
        self.group_changed()

    def group_changed(self):
        current = self.group.currentData() == 10
        self.detail.setEnabled(current)
        self.capture_button.setEnabled(current)
        self.rate.setCurrentText('500' if self.group.currentData() in (10, 30) else '200')
        self.preview_selection()

    def set_firmware(self, firmware):
        self.firmware = firmware
        self.reset_connection()
        self.build_plots(self.group.currentData(),
                          self.detail.currentIndex() if self.group.currentData() == 10 else 0)

    def preview_selection(self):
        if not self.points:
            ident = self.group.currentData()
            detail = self.detail.currentIndex() if ident == 10 else 0
            if self.plot_key != (ident, detail):
                self.build_plots(ident, detail)

    def configure(self, last):
        if self.command_pending:
            return
        commands = ['debug 0', f'logid {self.group.currentData()}']
        if self.group.currentData() == 10:
            commands.append(f'logdetail {self.detail.currentIndex()}')
        commands.extend([f'lograte {self.rate.currentText()}', last])
        self.clear()
        self.command_queue.extend(commands)
        self.next_command()

    def next_command(self):
        if not self.command_queue:
            self.command_pending = False
            return
        self.command_pending = True
        command = self.command_queue.popleft()
        self.expected_reply = ('OK debug 0' if command == 'debug 0' else
                               'OK capture armed' if command.startswith('logcapture') else 'OK logid=')
        if self.send(command) is False:
            self.command_queue.clear()
            self.command_pending = False
            return
        self.command_timer.start(2000)

    def command_timeout(self):
        self.command_queue.clear()
        self.command_pending = False
        self.status.setText('调试命令超时，后续命令已取消；请检查设备后重试')

    def start(self):
        self.configure('debug 1')

    def capture(self):
        self.configure('logcapture 128')

    def stop(self):
        self.command_queue.clear()
        self.command_pending = False
        self.command_timer.stop()
        self.send('debug 0')

    def clear(self):
        self.rows.clear()
        self.points.clear()
        self.last_us = None
        self.elapsed = 0
        self.bad_frames = 0
        self.lost_frames = 0
        self.last_seq = None
        for series, *_ in self.series:
            series.clear()
        self.restore_auto()
        self.preview_selection()
        self.status.setText('数据已清空')

    def axis_lock_changed(self):
        self.dirty = True

    def manual_range(self, view, x_range=None, y_range=None, y_axis=None):
        def valid(bounds):
            return (bounds is not None and all(math.isfinite(v) for v in bounds)
                    and 1e-12 < bounds[1]-bounds[0] < 1e15)
        if not self.lock_x.isChecked() and valid(x_range):
            for target in self.chart_views:
                target.auto_x = False
                target.x_axis.setRange(*x_range)
        if not self.lock_y.isChecked() and valid(y_range):
            source = y_axis or view.y_axis
            low, high = source.min(), source.max()
            relative = ((y_range[0]-low)/(high-low), (y_range[1]-low)/(high-low))
            ranges = [(axis, (axis.min()+relative[0]*(axis.max()-axis.min()),
                              axis.min()+relative[1]*(axis.max()-axis.min())))
                      for axis in ([y_axis] if y_axis is not None else view.y_axes)]
            view.auto_y = False
            for axis, bounds in ranges:
                if valid(bounds):
                    view.manual_y_axes.add(axis)
                    axis.setRange(*bounds)

    def restore_auto(self):
        self.lock_x.setChecked(False)
        self.lock_y.setChecked(False)
        for view in self.chart_views:
            view.auto_x = view.auto_y = True
            view.manual_y_axes.clear()
            if not self.points:
                view.x_axis.setRange(0, 1)
                for axis in view.y_axes:
                    axis.setRange(0, 1)
        self.dirty = True
        self.redraw()

    def reset_connection(self):
        self.pending = ''
        self.command_queue.clear()
        self.command_pending = False
        self.command_timer.stop()
        self.clear()

    def consume(self, text):
        """Handle arbitrary UART chunks; return complete non-telemetry lines."""
        self.pending += text
        output = []
        while '\n' in self.pending:
            line, self.pending = self.pending.split('\n', 1)
            if line.startswith('@T,'):
                try:
                    self.add_frame(Frame.parse(line))
                except ValueError:
                    self.bad_frames += 1
            else:
                output.append(line + '\n')
                if line.startswith('OK logid='):
                    self.apply_status(line)
                if self.command_pending:
                    if line.startswith('ERR '):
                        self.command_timer.stop()
                        self.command_queue.clear()
                        self.command_pending = False
                        self.status.setText(line)
                    elif line.startswith(self.expected_reply):
                        self.command_timer.stop()
                        QTimer.singleShot(0, self.next_command)
        if len(self.pending) > 8192:
            self.bad_frames += 1
            self.pending = ''
        return ''.join(output)

    def apply_status(self, line):
        values = dict(token.split('=', 1) for token in line.split() if '=' in token)
        try:
            ident = int(values['logid'])
            detail = int(values.get('detail', 0))
            if ident not in (10, 20, 30, 40) or detail not in (0, 1, 2):
                return
            self.group.blockSignals(True)
            self.group.setCurrentIndex(self.group.findData(ident))
            self.group.blockSignals(False)
            self.detail.setCurrentIndex(detail)
            self.detail.setEnabled(ident == 10)
            self.capture_button.setEnabled(ident == 10)
            if 'rate' in values:
                rate = str(int(values['rate']))
                if self.rate.findText(rate) < 0:
                    self.rate.addItem(rate)
                self.rate.setCurrentText(rate)
        except (ValueError, KeyError):
            return

    def add_frame(self, frame):
        key = (frame.logid, frame.detail, frame.capture)
        if self.key != key or (self.last_us is not None and frame.seq == 0):
            self.points.clear()
            self.last_us = None
            self.elapsed = 0
            self.key = key
            self.last_seq = None
            if self.plot_key != (frame.logid, frame.detail):
                self.build_plots(frame.logid, frame.detail)
            else:
                self.restore_auto()
        if self.last_seq is not None:
            delta = (frame.seq-self.last_seq) & 0xffffffff
            if 1 < delta < 0x80000000:
                self.lost_frames += delta-1
        self.last_seq = frame.seq
        if self.last_us is not None:
            self.elapsed += ((frame.us - self.last_us) & 0xffffffff) / 1e6
        self.last_us = frame.us
        self.points.append((self.elapsed, frame))
        self.rows.append((self.elapsed, frame))
        self.dirty = True
        flags = frame.flags
        self.status.setText(f'ID={frame.logid} 序号={frame.seq} 固件丢帧={frame.drops} 序号缺口={self.lost_frames} '
            f'解析错误={self.bad_frames} 抓取={bool(frame.capture)} 电流环运行={bool(flags & 1)} '
            f'PWM={bool(flags & 4)} 限压={bool(flags & 2)} '
            f'限流={bool(flags & 64)} 速度有效={bool(flags & 32)} 位置有效={bool(flags & 128)} '
            f'保存最近 {len(self.rows)} / 100000 点')

    def build_plots(self, logid, detail):
        self.series = []
        self.chart_views = []
        self.plot_key = (logid, detail)
        self.lock_x.setChecked(False)
        self.lock_y.setChecked(False)
        self.note.setText({
            10: '电流为物理D/Q轴方向；实时流观察趋势，高速抓取按16 kHz采样，128点约8 ms。',
            20: '速度与Iq均为逻辑方向；PI输出与最终Iq指令之间还可能存在启动斜率限制。',
            30: '占空比为本周期写入、下一PWM更新点生效的预装载值；电流为本周期采样，Ic为重构值。',
            40: 'C=逻辑Iq，V=逻辑机械转速，P=选择40组后的Hall相对位置；当前分辨率15°，非绝对位置。',
        }[logid])
        if self.firmware == 'demo':
            self.note.setText({
                10: 'demo原厂电流PI；高速抓取16 kHz、128点约8 ms。轴请求为原厂out_start，细节不套用hall限幅语义。',
                20: 'demo原厂速度PI，速度rpm、电流A；显示原厂积分与输出，不包含hall启动Iq斜坡。',
                30: 'demo PWM比较值换算占空比；相电流来自原厂采样，母线来自实际ADC。',
                40: 'demo Hall插值角累计的相对机械位置，非绝对位置；停机重新识别或角度重定位可引入跳变。',
            }[logid])
        for box in (self.legend, self.plots):
            while box.count():
                item = box.takeAt(0)
                if item.widget():
                    item.widget().hide()
                    item.widget().deleteLater()
        plots = {}
        def make_axis():
            axis = QValueAxis()
            axis.setRange(0, 1)
            axis.setTickCount(6)
            axis.setMinorTickCount(4)
            axis.setLabelFormat('%.4g')
            axis.setLabelsColor(QColor('#7c8796'))
            axis.setLinePen(QPen(QColor('#9299a3')))
            axis.setGridLinePen(QPen(QColor('#dfe4ea'), .6))
            axis.setMinorGridLinePen(QPen(QColor('#f0f2f5'), .5))
            return axis
        for index, label, unit, scale in channels(logid, detail, self.firmware):
            if unit not in plots and plots:
                chart, x, _ = next(iter(plots.values()))
                y = make_axis()
                y.setTitleText(unit)
                y.setGridLineVisible(False)
                y.setMinorGridLineVisible(False)
                chart.addAxis(y, Qt.AlignRight)
                self.chart_views[0].y_axes.append(y)
                plots[unit] = chart, x, y
            elif unit not in plots:
                chart = QChart()
                chart.setBackgroundBrush(QColor('white'))
                chart.setBackgroundRoundness(0)
                chart.setMargins(QMargins(6, 6, 6, 6))
                chart.setTitle({10: '电流总览' if detail == 0 else ('D轴PI' if detail == 1 else 'Q轴PI'),
                                20: '速度环', 30: '占空比与相电流', 40: 'CVP'}[logid])
                chart.legend().hide()
                x, y = make_axis(), make_axis()
                x.setTitleText('采样时间 (s)')
                y.setTitleText(unit)
                chart.addAxis(x, Qt.AlignBottom)
                chart.addAxis(y, Qt.AlignLeft)
                view = ScopeChartView(chart, x, y, self)
                self.chart_views.append(view)
                view.setMinimumHeight(240)
                view.setRenderHint(QPainter.Antialiasing)
                self.plots.addWidget(view, 1)
                plots[unit] = chart, x, y
            chart, x, y = plots[unit]
            series = QLineSeries()
            series.setName(label)
            chart.addSeries(series)
            series.setPen(QPen(QColor(TRACE_COLORS[len(self.series)]), 1.6))
            series.attachAxis(x)
            series.attachAxis(y)
            check = QCheckBox(label)
            check.setStyleSheet(f'color:{series.color().name()};')
            check.setChecked(True)
            check.toggled.connect(series.setVisible)
            check.toggled.connect(lambda _v: setattr(self, 'dirty', True))
            self.legend.addWidget(check)
            self.series.append((series, index, scale, x, y))

    def redraw(self):
        if not self.dirty or not self.points:
            return
        self.dirty = False
        limits = {}
        for series, index, scale, x, y in self.series:
            data = []
            for t, f in self.points:
                # Show the latest contiguous valid segment; never bridge invalid data.
                valid = True
                if f.logid == 10:
                    valid = bool(f.flags & 1) and bool(f.flags & 256)
                if f.logid == 20 and index == 2:
                    valid = bool(f.flags & 32)
                if f.logid == 20 and index == 4:
                    valid = bool(f.flags & 1)
                if f.logid == 40:
                    valid = bool(f.flags & {0: 1, 1: 32, 2: 128}.get(index, 0xffffffff))
                if f.logid == 30 and index in (3, 4, 5):
                    valid = bool(f.flags & 8)
                if f.logid == 30 and index == 6:
                    valid = bool(f.flags & 16)
                if valid:
                    data.append(QPointF(t, f.values[index] * scale))
                else:
                    data.clear()
            series.replace(data)
            if series.isVisible() and data:
                low = min(p.y() for p in data)
                high = max(p.y() for p in data)
                prior = limits.get(y, (low, high))
                limits[y] = min(low, prior[0]), max(high, prior[1])
        for axis, (low, high) in limits.items():
            pad = max((high-low)*.1, .01)
            view = next(view for view in self.chart_views if axis in view.y_axes)
            if axis not in view.manual_y_axes and not self.lock_y.isChecked() and not view.gesture:
                axis.setRange(low-pad, high+pad)
        for view in self.chart_views:
            if view.auto_x and not self.lock_x.isChecked() and not view.gesture:
                view.x_axis.setRange(self.points[0][0], max(self.points[0][0]+.001, self.points[-1][0]))

    def export(self):
        path, _ = QFileDialog.getSaveFileName(self, '导出原始曲线数据', 'motor_trace.csv', 'CSV (*.csv)')
        if not path:
            return
        try:
            with open(path, 'w', newline='', encoding='utf-8-sig') as stream:
                writer = csv.writer(stream)
                writer.writerow(['elapsed_s', 'logid', 'detail', 'seq', 'sample_us',
                    'drops', 'flags', 'capture', *[f'v{i}' for i in range(10)], 'fields'])
                for elapsed, f in self.rows:
                    names = ';'.join(f'v{i}={name}[{unit}],scale={scale}'
                                    for i, name, unit, scale in channels(f.logid, f.detail, self.firmware))
                    writer.writerow([elapsed, f.logid, f.detail, f.seq, f.us, f.drops,
                                     f.flags, f.capture, *f.values, names])
        except OSError as exc:
            QMessageBox.warning(self, '导出失败', str(exc))
