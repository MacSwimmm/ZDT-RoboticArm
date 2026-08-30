import pathlib
import sys
import time
import unittest


PROJECT_DIR = pathlib.Path(__file__).resolve().parents[1]
FIRMWARE_DIR = PROJECT_DIR.parent / "F407-RoboticArm"
sys.path.insert(0, str(PROJECT_DIR))

from comm.serial_worker import SerialWorker
from protocol.host_protocol import CMD_SERVO, CMD_STOP, CMD_VELOCITY, CMD_ZERO, Parser, servo, stop, velocity, zero


class HostProtocolTests(unittest.TestCase):
    def test_zero_and_stop_have_one_byte_payloads(self):
        parser = Parser()
        frames = parser.feed(zero(2) + stop(1))
        self.assertEqual(frames, [(CMD_ZERO, b"\x02"), (CMD_STOP, b"\x01")])

    def test_velocity_frame_carries_speed_and_accel(self):
        parser = Parser()
        frames = parser.feed(velocity(2, 1, 120, 20))
        self.assertEqual(frames, [(CMD_VELOCITY, bytes((2, 1, 0, 120, 20)))])

    def test_servo_frame_clamps_to_180_degrees(self):
        self.assertEqual(Parser().feed(servo(220)), [(CMD_SERVO, b"\xb4")])

    def test_firmware_contract_matches_host_payloads_and_emm(self):
        host_source = (FIRMWARE_DIR / "MDK-ARM" / "APP" / "HostProtocol.c").read_text(encoding="utf-8")
        bus_source = (FIRMWARE_DIR / "MDK-ARM" / "APP" / "MotorBus.c").read_text(encoding="utf-8")
        self.assertEqual(host_source.count("if (length != 1U) return;"), 4)
        self.assertIn("#define MOTORBUS_X_FIRMWARE             0U", bus_source)


class PrepareWorkflowTests(unittest.TestCase):
    def setUp(self):
        self.worker = SerialWorker()
        self.worker.simulation_mode = True
        for state in self.worker.states:
            state.online = True

    def test_prepare_zeroes_then_enables_all_axes(self):
        self.worker.request_zero_all()
        self.worker._process_prepare_workflow()
        self.assertFalse(self.worker._prepare_all_pending)
        self.assertTrue(all(state.zero_valid for state in self.worker.states))
        self.assertTrue(all(state.enabled for state in self.worker.states))
        self.assertTrue(all(not state.stopped for state in self.worker.states))

    def test_prepare_timeout_does_not_enable(self):
        self.worker._prepare_all_pending = True
        self.worker._prepare_all_started = time.time() - self.worker.PREPARE_TIMEOUT_SECONDS - 1.0
        self.worker._process_prepare_workflow()
        self.assertFalse(self.worker._prepare_all_pending)
        self.assertTrue(all(not state.enabled for state in self.worker.states))

    def test_jog_release_stops_without_locking_axis(self):
        state = self.worker.states[0]
        state.zero_valid = True
        state.enabled = True
        state.stopped = False
        self.worker.request_velocity(0, 1, pressed=False)
        _cmd, description = self.worker.urgent_tx_queue.get_nowait()
        self.assertIn("不锁定", description)
        self.assertFalse(state.stopped)


if __name__ == "__main__":
    unittest.main()
