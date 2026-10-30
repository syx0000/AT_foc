"""Offscreen regression for fragmented identification and firmware-specific UI."""
import os
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
import sys
from pathlib import Path
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools" / "motor_host"))
from PySide6.QtWidgets import QApplication
from motor_host import MotorHost, PARAMETER_NAMES, DEMO_PARAMETER_NAMES
from trace_view import Frame, channels


class ProfilesTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QApplication.instance() or QApplication([])

    def setUp(self):
        self.window = MotorHost()

    def tearDown(self):
        self.window.close()

    def button(self, original_command):
        return next(button for button, _, command in self.window.command_buttons
                    if command == original_command)

    def test_identification_and_hall_roundtrip(self):
        w = self.window
        self.assertFalse(w.command_supported("speed start 100"))
        w.process_replies("OK fw=demo-host-1 firm")
        self.assertEqual(w.firmware_type, "unknown")
        w.process_replies("ware=demo protocol=hall-host-1\r\n")
        self.assertEqual(w.firmware_type, "demo")
        self.assertEqual(w.parameter_name.count(), len(DEMO_PARAMETER_NAMES))
        self.assertEqual(self.button("identify resistance").property("command"), "identify winding")
        self.assertTrue(self.button("identify flux").isHidden())
        self.assertTrue(self.button("motor commissioning accept").isHidden())
        self.assertEqual(w.open_accel.text(), "0")
        self.assertFalse(w.open_accel.isEnabled())
        self.assertFalse(w.command_supported("motor param set speed_ki_q20 64"))
        self.assertTrue(w.command_supported("motor param set demo_speed_ki 50"))
        self.assertFalse(w.command_supported("motor params defaults"))
        w.process_replies("OK fw=hall-current hw=AT32\r\n")
        self.assertEqual(w.firmware_type, "hall")
        self.assertEqual(w.parameter_name.count(), len(PARAMETER_NAMES))
        self.assertFalse(self.button("identify flux").isHidden())
        self.assertFalse(self.button("motor commissioning accept").isHidden())
        self.assertEqual(self.button("identify resistance").property("command"), "identify resistance")
        self.assertTrue(w.open_accel.isEnabled())

    def test_native_readback_and_busy_release(self):
        w = self.window
        w.apply_firmware_profile("demo")
        w.parse_active_parameters("OK params active demo_id_kp=6047 demo_id_kp_shift=12 hall_angle=5/1/3/2/6/4")
        self.assertEqual(w.active_values["demo_id_kp"], "6047")
        self.assertEqual(w.active_values["demo_id_kp_shift"], "12")
        w.set_commissioning_busy(True)
        self.assertFalse(self.button("identify resistance").isEnabled())
        w.process_replies("ERR commissioning rejected_")
        w.process_replies("or_failed\r\n")
        self.assertTrue(self.button("identify resistance").isEnabled())
        w.disconnect()
        self.assertEqual(w.firmware_type, "unknown")
        self.assertFalse(w.active_values)

    def test_native_trace_labels_and_capture_decode(self):
        self.assertEqual(channels(10, 1)[5][1], "轴限幅后请求")
        self.assertEqual(channels(10, 1, "demo")[5][1], "原厂PI限幅前输出")
        w = self.window
        w.apply_firmware_profile("demo")
        text = "@T,10,0,0,1000,0,301,1,0,0,1000,900,0,100,0,0,0,0\r\n"
        f = Frame.parse(text)
        self.assertEqual(f.capture, 1)
        self.assertEqual(f.values[2], 1000)
        w.trace_view.consume(text[:19])
        w.trace_view.consume(text[19:])
        self.assertEqual(len(w.trace_view.rows), 1)
        self.assertEqual(w.trace_view.bad_frames, 0)

    def test_rejected_edits_preserve_confirmed_run_until_stop_ack(self):
        w = self.window
        w.apply_firmware_profile("demo")
        w.process_replies("OK run requested mode=speed\r\n")
        self.assertTrue(w.mode_started["speed"])
        w.process_replies("ERR commissioning rejected_or_failed reason=state_requires_stopped\r\n")
        self.assertTrue(w.mode_started["speed"])
        self.assertEqual(w.mode_buttons["speed"].text(), "速度更新")
        w.process_replies("ERR open_voltage_exceeds_bus_or_invalid_bus\r\n")
        self.assertTrue(w.mode_started["speed"])
        w.process_replies("OK stop")
        self.assertTrue(w.mode_started["speed"])
        w.process_replies("ped\r\n")
        self.assertFalse(any(w.mode_started.values()))

    def test_start_and_update_wait_for_ack_and_rejections_do_not_invent_a_stop(self):
        class FakeSerial:
            is_open = True

            def write(self, data):
                return len(data)

            def close(self):
                self.is_open = False

        w = self.window
        w.apply_firmware_profile("demo")
        w.serial_port = FakeSerial()
        w.send_mode("current")
        self.assertEqual(w.pending_mode_request, "current")
        self.assertFalse(any(w.mode_started.values()))
        w.process_replies("ERR unsupported_or_invalid_command\r\n")
        self.assertIsNone(w.pending_mode_request)
        self.assertFalse(any(w.mode_started.values()))
        w.send_mode("current")
        w.process_replies("OK run requested mode=current\r\n")
        self.assertTrue(w.mode_started["current"])
        w.send_mode("current")
        w.process_replies("ERR invalid_current\r\n")
        self.assertTrue(w.mode_started["current"])
        w.stop_motor()
        self.assertTrue(w.mode_started["current"])
        w.process_replies("OK stopped\r\n")
        self.assertFalse(any(w.mode_started.values()))
        w.apply_firmware_profile("hall")
        w.send_mode("speed")
        self.assertFalse(w.mode_started["speed"])
        w.process_replies("OK speed requested target=150 rpm\r\n")
        self.assertTrue(w.mode_started["speed"])
        w.process_replies("ERR 3 invalid_parameter\r\n")
        self.assertTrue(w.mode_started["speed"])
        w.process_replies("OK motor stopped\r\n")
        self.assertFalse(any(w.mode_started.values()))


if __name__ == "__main__":
    unittest.main()
