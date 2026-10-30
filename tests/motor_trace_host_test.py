"""Protocol, command sequencing and Qt curve smoke tests (no motor required)."""
import csv
import os
import sys
import unittest
from pathlib import Path
from unittest.mock import patch

os.environ['QT_QPA_PLATFORM'] = 'offscreen'
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools' / 'motor_host'))
from PySide6.QtWidgets import QApplication, QPushButton
from PySide6.QtGui import QFontDatabase, QFont, QWheelEvent
from PySide6.QtCore import QPoint, QPointF, Qt
from PySide6.QtTest import QTest
from trace_view import Frame, TraceView, channels
from motor_host import MotorHost

APP = QApplication.instance() or QApplication([])
QFontDatabase.addApplicationFont('C:/Windows/Fonts/msyh.ttc')
APP.setFont(QFont('Microsoft YaHei', 9))


def wire(logid=10, detail=0, seq=0, us=100, flags=511, capture=0):
    values = [1000, 900, 2000, 1900, 100, 200, 0, 0, 0, seq]
    return ','.join(map(str, ['@T', logid, detail, seq, us, 0, flags, capture, *values]))+'\r\n'


class TraceTests(unittest.TestCase):
    def setUp(self):
        self.sent = []
        self.view = TraceView(self.sent.append)

    def tearDown(self):
        self.view.timer.stop()
        self.view.command_timer.stop()
        self.view.close()
        self.view.deleteLater()
        APP.processEvents()

    def test_protocol_chunks_and_invalid_frames(self):
        packet = wire()
        self.assertEqual(self.view.consume(packet[:12]), '')
        result = self.view.consume(packet[12:]+'OK debug 0\r\n')
        self.assertEqual(result, 'OK debug 0\r\n')
        self.assertEqual(len(self.view.rows), 1)
        self.view.consume('@T,broken\n')
        self.assertEqual(self.view.bad_frames, 1)
        for bad in [wire(logid=99), wire(capture=3), wire(us=-1), wire()[:-4]]:
            with self.assertRaises(ValueError):
                Frame.parse(bad)

    def test_timestamp_wrap_and_plots(self):
        for logid in (10, 20, 30, 40):
            self.view.consume(wire(logid=logid, us=0xfffffff0))
            self.view.consume(wire(logid=logid, seq=1, us=0x10))
            self.assertAlmostEqual(self.view.elapsed, 32e-6)
            self.view.redraw()
            self.assertEqual(len(self.view.series), len(channels(logid, 0)))
            self.assertGreater(self.view.series[0][0].count(), 0)
        self.view.consume(wire(logid=10, detail=2, capture=1))
        self.view.redraw()
        self.assertEqual(len(self.view.series), 8)
        for seq in range(1, 128):
            self.view.consume(wire(logid=10, detail=2, capture=1, seq=seq, us=100+seq*63))
        self.view.redraw()
        self.view.resize(1200, 500)
        self.view.show()
        APP.processEvents()
        target = Path(__file__).resolve().parents[1] / 'tmp' / 'trace_view_preview.png'
        self.assertTrue(self.view.grab().save(str(target)))

    def test_configuration_waits_for_each_ack_and_stop_cancels(self):
        self.view.start()
        self.assertEqual(self.sent, ['debug 0'])
        self.view.consume('OK debug 0\r\n')
        APP.processEvents()
        self.assertEqual(self.sent[-1], 'logid 10')
        for expected in ['logdetail 0', 'lograte 500', 'debug 1']:
            self.view.consume('OK logid=10 enabled=0\r\n')
            APP.processEvents()
            self.assertEqual(self.sent[-1], expected)
        self.view.consume('OK logid=10 enabled=1\r\n')
        APP.processEvents()
        self.assertFalse(self.view.command_pending)
        self.view.start()
        self.view.stop()
        count = len(self.sent)
        self.view.consume('OK debug 0\r\n')
        APP.processEvents()
        self.assertEqual(len(self.sent), count)

    def test_error_timeout_and_disconnect_cancel_commands(self):
        self.view.start()
        self.view.consume('ERR invalid_trace_command\r\n')
        APP.processEvents()
        self.assertFalse(self.view.command_pending)
        self.assertEqual(len(self.sent), 1)
        self.view.start()
        self.view.command_timeout()
        self.assertFalse(self.view.command_queue)
        self.view.start()
        self.view.reset_connection()
        self.assertFalse(self.view.command_pending)
        self.assertEqual(self.view.pending, '')

    def test_invalid_feedback_not_drawn_and_export(self):
        self.view.consume(wire(logid=40, flags=0))
        self.view.redraw()
        for series, *_ in self.view.series[:3]:
            self.assertEqual(series.count(), 0)
        target = Path(__file__).resolve().parents[1] / 'tmp' / 'trace_export_test.csv'
        with patch('trace_view.QFileDialog.getSaveFileName', return_value=(str(target), 'CSV')):
            self.view.export()
        with target.open(encoding='utf-8-sig', newline='') as stream:
            rows = list(csv.reader(stream))
        self.assertEqual(len(rows), 2)
        self.assertEqual(rows[1][1], '40')
        self.assertIn('scale=0.001', rows[1][-1])

    def test_main_window_integration(self):
        window = MotorHost()
        self.assertIsInstance(window.trace_view, TraceView)
        self.assertFalse(any('24V' in b.text() for b in window.findChildren(QPushButton)))
        window.resize(1280, 800)
        window.show()
        APP.processEvents()
        normal_height = window.tabs.height()
        window.tabs.setCurrentIndex(window.trace_tab_index)
        APP.processEvents()
        self.assertTrue(window.console_group.isHidden())
        self.assertTrue(window.command_bar.isHidden())
        self.assertGreater(window.tabs.height(), normal_height)
        self.assertGreater(len(window.trace_view.chart_views), 0)
        target = Path(__file__).resolve().parents[1] / 'tmp' / 'motor_host_empty_grid.png'
        self.assertTrue(window.grab().save(str(target)))
        window.tabs.setCurrentIndex(0)
        self.assertFalse(window.console_group.isHidden())
        self.assertFalse(window.command_bar.isHidden())
        window.close()
        window.deleteLater()

    def test_empty_grid_tracks_selection_and_keeps_channel_choices_on_first_frame(self):
        self.assertEqual(self.view.plot_key, (10, 0))
        for chart in self.view.chart_views:
            self.assertEqual((chart.x_axis.min(), chart.x_axis.max()), (0, 1))
            self.assertEqual((chart.y_axis.min(), chart.y_axis.max()), (0, 1))
            self.assertGreater(chart.x_axis.minorTickCount(), 0)
        self.view.group.setCurrentIndex(self.view.group.findData(30))
        self.assertEqual(self.view.plot_key, (30, 0))
        check = self.view.legend.itemAt(0).widget()
        check.setChecked(False)
        first_series = self.view.series[0][0]
        self.view.consume(wire(logid=30))
        self.assertIs(self.view.series[0][0], first_series)
        self.assertFalse(first_series.isVisible())
        self.view.clear()
        self.assertEqual(self.view.chart_views[0].x_axis.max(), 1)

    def test_mouse_zoom_pan_axis_locks_and_live_updates(self):
        for seq in range(100):
            self.view.consume(wire(logid=20, seq=seq, us=seq*2000))
        self.view.resize(1250, 600)
        self.view.show()
        APP.processEvents()
        self.view.redraw()
        chart = self.view.chart_views[0]
        other = self.view.chart_views[0]
        def bounds():
            return chart.x_axis.min(), chart.x_axis.max(), chart.y_axis.min(), chart.y_axis.max()
        def pos(fx, fy):
            area = chart.chart().plotArea()
            point = QPointF(area.left()+area.width()*fx, area.top()+area.height()*fy)
            return chart.mapFromScene(chart.chart().mapToScene(point))
        def wheel(fx=.5, fy=.5):
            center = pos(fx, fy)
            event = QWheelEvent(QPointF(center), QPointF(chart.viewport().mapToGlobal(center)),
                QPoint(), QPoint(0, 120), Qt.NoButton, Qt.NoModifier, Qt.NoScrollPhase, False)
            APP.sendEvent(chart.viewport(), event)
        original = bounds()
        wheel()
        zoomed = bounds()
        self.assertLess(zoomed[1]-zoomed[0], original[1]-original[0])
        self.assertLess(zoomed[3]-zoomed[2], original[3]-original[2])
        self.assertEqual((other.x_axis.min(), other.x_axis.max()), zoomed[:2])
        self.view.consume(wire(logid=20, seq=100, us=300000))
        self.view.redraw()
        self.assertEqual(bounds(), zoomed)
        self.view.lock_x.setChecked(True)
        wheel()
        self.assertEqual(bounds()[:2], zoomed[:2])
        self.assertNotEqual(bounds()[2:], zoomed[2:])
        self.view.lock_x.setChecked(False)
        before = bounds()
        wheel(.5, 1.03)
        self.assertNotEqual(bounds()[:2], before[:2])
        self.assertEqual(bounds()[2:], before[2:])
        before = bounds()
        wheel(-.03, .5)
        self.assertEqual(bounds()[:2], before[:2])
        self.assertNotEqual(bounds()[2:], before[2:])
        before = bounds()
        QTest.mousePress(chart.viewport(), Qt.LeftButton, Qt.NoModifier, pos(.2, .2))
        QTest.mouseMove(chart.viewport(), pos(.8, .8))
        QTest.mouseRelease(chart.viewport(), Qt.LeftButton, Qt.NoModifier, pos(.8, .8))
        self.assertLess(chart.x_axis.max()-chart.x_axis.min(), before[1]-before[0])
        before = bounds()
        start, end = pos(.4, .4), pos(.6, .6)
        QTest.mousePress(chart.viewport(), Qt.MiddleButton, Qt.NoModifier, start)
        QTest.mouseMove(chart.viewport(), end)
        QTest.mouseRelease(chart.viewport(), Qt.MiddleButton, Qt.NoModifier, end)
        after = bounds()
        self.assertNotEqual(after[:2], before[:2])
        self.assertAlmostEqual(after[1]-after[0], before[1]-before[0])
        self.assertAlmostEqual(after[3]-after[2], before[3]-before[2])
        self.view.lock_y.setChecked(True)
        before = bounds()
        wheel()
        self.assertEqual(bounds()[2:], before[2:])
        QTest.mouseDClick(chart.viewport(), Qt.LeftButton, Qt.NoModifier, pos(.5, .5))
        self.assertTrue(chart.auto_x and chart.auto_y and other.auto_x)
        self.assertFalse(self.view.lock_x.isChecked() or self.view.lock_y.isChecked())
        self.assertAlmostEqual(chart.x_axis.max(), .3)

    def test_lock_freezes_automatic_axis_and_invalid_ranges_are_ignored(self):
        self.view.consume(wire())
        self.view.consume(wire(seq=1, us=2000))
        self.view.redraw()
        chart = self.view.chart_views[0]
        self.view.lock_x.setChecked(True)
        old = (chart.x_axis.min(), chart.x_axis.max())
        self.view.consume(wire(seq=2, us=1000000))
        self.view.redraw()
        self.assertEqual((chart.x_axis.min(), chart.x_axis.max()), old)
        self.view.lock_x.setChecked(False)
        self.view.redraw()
        self.assertGreater(chart.x_axis.max(), old[1])
        self.view.manual_range(chart, (float('nan'), 1), (1, 1))
        self.assertTrue(chart.auto_x and chart.auto_y)

    def test_current_overview_has_one_plot_with_independent_current_voltage_axes(self):
        self.assertEqual(len(self.view.chart_views), 1)
        chart = self.view.chart_views[0]
        self.assertEqual(len(chart.y_axes), 2)
        current, voltage = chart.y_axes
        self.assertEqual((current.titleText(), voltage.titleText()), ('A', 'V'))
        for series, index, _, x, y in self.view.series:
            self.assertIs(y, current if index < 4 else voltage)
            self.assertIs(x, chart.x_axis)
        self.view.consume(wire())
        self.view.consume(wire(seq=1, us=2000))
        self.view.redraw()
        self.assertGreater(current.max(), voltage.max())
        old_current = current.min(), current.max()
        old_voltage = voltage.min(), voltage.max()
        self.view.manual_range(chart, None,
            (old_current[0]+.25*(old_current[1]-old_current[0]),
             old_current[1]-.25*(old_current[1]-old_current[0])))
        self.assertAlmostEqual(voltage.max()-voltage.min(), .5*(old_voltage[1]-old_voltage[0]))
        saved = voltage.min(), voltage.max()
        self.view.consume(wire(seq=2, us=4000))
        self.view.redraw()
        self.assertEqual((voltage.min(), voltage.max()), saved)
        self.view.restore_auto()
        self.assertAlmostEqual(voltage.max(), old_voltage[1])

    def test_wheel_near_right_axis_only_changes_voltage_axis(self):
        self.view.resize(1250, 600)
        self.view.show()
        APP.processEvents()
        chart = self.view.chart_views[0]
        current, voltage = chart.y_axes
        area = chart.chart().plotArea()
        point = chart.mapFromScene(chart.chart().mapToScene(QPointF(area.right()+12, area.center().y())))
        event = QWheelEvent(QPointF(point), QPointF(chart.viewport().mapToGlobal(point)),
            QPoint(), QPoint(0, 120), Qt.NoButton, Qt.NoModifier, Qt.NoScrollPhase, False)
        APP.sendEvent(chart.viewport(), event)
        self.assertEqual((current.min(), current.max()), (0, 1))
        self.assertEqual((chart.x_axis.min(), chart.x_axis.max()), (0, 1))
        self.assertLess(voltage.max()-voltage.min(), 1)
        self.assertIn(voltage, chart.manual_y_axes)
        self.assertNotIn(current, chart.manual_y_axes)
        saved = voltage.min(), voltage.max()
        self.view.consume(wire(seq=1))
        self.view.consume(wire(seq=2, us=2000))
        self.view.redraw()
        # A new stream restores auto; zoom the right axis again during acquisition.
        area = chart.chart().plotArea()
        point = chart.mapFromScene(chart.chart().mapToScene(QPointF(area.right()+12, area.center().y())))
        event = QWheelEvent(QPointF(point), QPointF(chart.viewport().mapToGlobal(point)),
            QPoint(), QPoint(0, 120), Qt.NoButton, Qt.NoModifier, Qt.NoScrollPhase, False)
        APP.sendEvent(chart.viewport(), event)
        saved = voltage.min(), voltage.max()
        self.view.consume(wire(seq=3, us=4000))
        self.view.redraw()
        self.assertEqual((voltage.min(), voltage.max()), saved)

    def test_all_channels_have_unique_stable_colors_and_matching_labels(self):
        for logid, detail in ((10, 0), (10, 1), (10, 2), (20, 0), (30, 0), (40, 0)):
            self.view.build_plots(logid, detail)
            colors = [series.color().name() for series, *_ in self.view.series]
            self.assertEqual(len(colors), len(set(colors)))
            for i, color in enumerate(colors):
                self.assertIn(color, self.view.legend.itemAt(i).widget().styleSheet())
            self.view.build_plots(logid, detail)
            self.assertEqual(colors, [series.color().name() for series, *_ in self.view.series])

    def test_every_group_has_one_chart_and_all_units_keep_separate_scales(self):
        for logid, detail in ((10, 0), (10, 1), (10, 2), (20, 0), (30, 0), (40, 0)):
            self.view.build_plots(logid, detail)
            self.assertEqual(len(self.view.chart_views), 1)
            self.assertEqual(self.view.plots.count(), 1)
            chart = self.view.chart_views[0]
            expected_units = {unit for _, _, unit, _ in channels(logid, detail)}
            self.assertEqual({axis.titleText() for axis in chart.y_axes}, expected_units)
            for series, _, _, x, y in self.view.series:
                self.assertIs(series.chart(), chart.chart())
                self.assertIs(x, chart.x_axis)
                self.assertIn(y, chart.y_axes)

    def test_three_unit_chart_can_zoom_each_right_axis_independently(self):
        self.view.build_plots(30, 0)
        self.view.resize(1250, 600)
        self.view.show()
        APP.processEvents()
        chart = self.view.chart_views[0]
        left, right_one, right_two = chart.y_axes
        for target, fraction in ((right_one, .02), (right_two, .5)):
            area = chart.chart().plotArea()
            right_width = chart.chart().boundingRect().right()-area.right()
            point = chart.mapFromScene(chart.chart().mapToScene(
                QPointF(area.right()+right_width*fraction, area.center().y())))
            before = {axis: (axis.min(), axis.max()) for axis in chart.y_axes}
            event = QWheelEvent(QPointF(point), QPointF(chart.viewport().mapToGlobal(point)),
                QPoint(), QPoint(0, 120), Qt.NoButton, Qt.NoModifier, Qt.NoScrollPhase, False)
            APP.sendEvent(chart.viewport(), event)
            for axis in chart.y_axes:
                if axis is target:
                    self.assertLess(axis.max()-axis.min(), before[axis][1]-before[axis][0])
                else:
                    self.assertEqual((axis.min(), axis.max()), before[axis])


if __name__ == '__main__':
    unittest.main()
