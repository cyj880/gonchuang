"""通信噪声、视频断流/阻塞及过期结果离线回归；无需串口或摄像头。"""
import multiprocessing
from pathlib import Path
import sys
import tempfile
import threading
import time
import unittest
from types import SimpleNamespace
from unittest.mock import Mock, patch

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import camera_base
import logistics_vision_main as main
from line_yaw import BandLineFinder


def fake_capture(connection, device, width, height, fps):
    """实际spawn进程：首轮停止交付或退出，第二轮恢复；不会打开真实设备。"""
    first = not Path(device).exists()
    if first:
        Path(device).touch()
    connection.send(('opened', (width, height, fps, 'MJPG')))
    connection.send(('frame', time.monotonic_ns(), np.full((height, width, 3), 1 if first else 2, np.uint8)))
    if first:
        if device.endswith('exit'):
            connection.close()
            return
        # 模拟驱动read永久阻塞；只有监督进程终止本进程才能恢复。
        threading.Event().wait()
    try:
        while True:
            connection.send(('frame', time.monotonic_ns(), np.full((height, width, 3), 2, np.uint8)))
            time.sleep(0.03)
    except (BrokenPipeError, EOFError, OSError):
        pass
    finally:
        connection.close()


def blocked_open(connection, *args):
    threading.Event().wait()


class ProtocolTests(unittest.TestCase):
    def test_poisoned_headers_do_not_hide_one_shot_color(self):
        good = main.encode_mode(main.MATERIAL_TRACK, 3)
        for command in (0x10, 0x11, 0xFF):
            for length in range(256):
                if command == 0x10 and length in (1, 2):
                    continue
                parser = main.FrameParser()
                self.assertEqual(parser.feed(bytes((0xAA, 0x55, command, length)) + good),
                                 [(main.COMMAND, bytes((main.MATERIAL_TRACK, 3)))])
        for prefix in (b'\xaa\x55\x10\x01\x03', b'\xaa\x55\x10\x02\x04',
                       b'\xaa\x55\x10\x02\x03\x00'):
            self.assertEqual(main.FrameParser().feed(prefix + good),
                             [(main.COMMAND, bytes((main.MATERIAL_TRACK, 3)))])

    def test_all_commands_and_all_split_points(self):
        commands = [main.encode_mode(mode, color) for mode, color in
                    [(mode, None) for mode in main.MODE_NAMES if mode != main.MATERIAL_TRACK] +
                    [(main.MATERIAL_TRACK, color) for color in range(1, 7)]]
        for command in commands:
            expected = main.FrameParser().feed(command)
            for split in range(len(command) + 1):
                parser = main.FrameParser()
                self.assertEqual(parser.feed(command[:split]) + parser.feed(command[split:]), expected)
        data = b''.join(commands)
        bad = bytearray(commands[-1]); bad[-3] ^= 1
        parser = main.FrameParser()
        self.assertEqual(len(parser.feed(bytes(bad) + data)), len(commands))
        self.assertLess(len(parser._buffer), 10)

    def test_callback_failure_does_not_drop_next_command_in_same_read(self):
        callback = Mock(side_effect=[RuntimeError('bad callback'), None])
        link = main.SerialLink('unused', 115200, callback)
        packet = main.encode_mode(main.RAW) + main.encode_mode(main.MATERIAL_TRACK, 1)
        handle = Mock(in_waiting=len(packet))
        handle.read.return_value = packet
        handle.write.side_effect = len
        link._handle = handle
        link._poll()
        self.assertEqual(callback.call_count, 2)
        self.assertEqual(callback.call_args.args, ('material_track 1',))
        handle.write.assert_called_once_with(main.encode_material_track_ack(1))


class PublisherTests(unittest.TestCase):
    def setUp(self):
        self.clock = patch.object(main.time, 'monotonic_ns', return_value=2_000_000_000)
        self.clock.start(); self.addCleanup(self.clock.stop)
        self.worker = SimpleNamespace(lock=threading.Lock(), latest_source_ns=2_000_000_000,
            latest={'detected': True, 'center_u': 320, 'center_v': 240,
                    'middle_center_px': {'x': 320, 'y': 240}, 'station_yaw_deg': 1.25},
            latest_sites=[{'id': 1, 'x': 220, 'y': 240}, {'id': 2, 'x': 320, 'y': 240},
                          {'id': 3, 'x': 420, 'y': 240}], latest_yaw=1.25)
        self.link = Mock(); self.link.send.return_value = True
        self.publisher = main.VisionPublisher(self.link,
            dict.fromkeys(('turntable', 'stacked', 'ring'), self.worker), SimpleNamespace(active_mode='raw'))
        self.publisher._mode_since_ns = 1_900_000_000

    def test_expired_future_and_pre_mode_frames_not_republished(self):
        for source in (0, 1_699_999_999, 1_899_999_999, 2_000_000_001):
            self.worker.latest_source_ns = source
            self.publisher._next_turntable_tx = self.publisher._next_stacked_tx = 0
            self.link.send.reset_mock()
            self.publisher._publish_turntable()
            self.link.send.assert_not_called()
            self.publisher._publish_stacked()
            self.assertEqual(self.link.send.call_args.args[0], main.encode_stacked_result({'detected': False}))
        self.worker.latest_source_ns = 2_000_000_000
        self.publisher._next_turntable_tx = self.publisher._next_stacked_tx = 0
        self.publisher._publish_turntable()
        self.assertEqual(self.link.send.call_args.args[0], main.legacy_center_frame((320, 240)))
        self.publisher._publish_stacked()
        self.assertEqual(self.link.send.call_args.args[0][4], 1)

    def test_ring_expiry_uses_capture_time_and_failed_tx_retries(self):
        self.worker.latest_source_ns = 1_950_000_000
        self.link.send.return_value = False
        self.publisher._publish_ring()
        self.assertTrue(self.publisher._pending)
        self.link.send.return_value = True
        self.publisher._publish_ring()
        self.assertEqual(self.link.send.call_args.args[0][4], 3)
        self.assertFalse(self.publisher._pending)
        with patch.object(main.time, 'monotonic_ns', return_value=2_250_000_001):
            self.publisher._publish_ring()
        self.assertEqual(self.link.send.call_args.args[0][4], 0)

    def test_dispatcher_preserves_camera_timestamp(self):
        camera = SimpleNamespace(read_sample=Mock(side_effect=[(np.zeros((4, 4, 3)), 1234), (None, 0)]))
        worker = Mock()
        manager = SimpleNamespace(_lock=threading.Lock(), workers={'ring': worker}, active_mode='ring')
        dispatcher = main.FrameDispatcher(camera, manager)
        worker.submit.side_effect = lambda *args: dispatcher._stop_evt.set()
        dispatcher.run()
        self.assertEqual(worker.submit.call_args.args[1], 1234)


class HoughTests(unittest.TestCase):
    def test_both_opencv_array_shapes(self):
        edge = np.zeros((480, 640), np.uint8)
        for shape in ((1, 1, 4), (1, 4)):
            with patch.object(cv2, 'HoughLinesP', return_value=np.array([50, 164, 590, 164]).reshape(shape)):
                result = BandLineFinder()._line(edge, 320, 164, 0, 10, 100)
            self.assertIsNotNone(result)
            self.assertEqual(result[1], 0)
        with patch.object(cv2, 'HoughLinesP', return_value=None):
            self.assertIsNone(BandLineFinder()._line(edge, 320, 164, 0, 10, 100))


class CameraTests(unittest.TestCase):
    def test_capture_exception_reports_and_releases(self):
        handle = Mock()
        handle.isOpened.return_value = True
        handle.get.side_effect = [0, 640, 480, 30]
        handle.read.side_effect = RuntimeError('read failed')
        connection = Mock()
        with patch.object(cv2, 'VideoCapture', return_value=handle):
            camera_base._camera_capture(connection, 'fake', 640, 480, 30)
        self.assertEqual(connection.send.call_args.args[0], ('error', 'read failed'))
        handle.release.assert_called_once()
        connection.close.assert_called_once()

    def test_failed_reads_report_and_retain_mjpg_setting_order(self):
        handle = Mock()
        handle.isOpened.return_value = True
        handle.get.side_effect = [0, 640, 480, 30]
        handle.read.return_value = (False, None)
        connection = Mock()
        with patch.object(cv2, 'VideoCapture', return_value=handle), \
                patch.object(camera_base.time, 'sleep'):
            camera_base._camera_capture(connection, 'fake', 640, 480, 30)
        self.assertEqual(handle.read.call_count, 50)
        self.assertEqual(connection.send.call_args.args[0][0], 'error')
        self.assertEqual([call.args[0] for call in handle.set.call_args_list],
                         [cv2.CAP_PROP_FOURCC, cv2.CAP_PROP_FRAME_WIDTH,
                          cv2.CAP_PROP_FRAME_HEIGHT, cv2.CAP_PROP_FPS])
        handle.release.assert_called_once()

    def test_open_fail_and_no_frame_have_bounded_recovery(self):
        for suffix in ('hang', 'exit'):
            with self.subTest(suffix=suffix), tempfile.TemporaryDirectory() as directory, \
                    patch.object(camera_base, '_camera_capture', fake_capture):
                camera = camera_base.Camera(str(Path(directory) / suffix), 640, 480, 30)
                camera.FRAME_TIMEOUT_S = 0.3
                camera.RETRY_S = 0.02
                try:
                    if suffix == 'hang':
                        # 相机底层仍阻塞时，真实串口命令解析和模式切换不受采集进程影响。
                        manager = main.ModeManager('raw', {}, None, True)
                        link = main.SerialLink('unused', 115200, manager.request_mode)
                        handle = Mock()
                        handle.read.return_value = main.encode_mode(main.TURNTABLE)
                        handle.in_waiting = len(handle.read.return_value)
                        link._handle = handle
                        link._poll()
                        self.assertEqual(manager.active_mode, 'turntable')
                    deadline = time.monotonic() + 8
                    recovered = False
                    while time.monotonic() < deadline:
                        frame, source = camera.read_sample()
                        if frame is not None and frame[0, 0, 0] == 2:
                            recovered = True
                            self.assertLess(time.monotonic_ns() - source, camera.FRAME_TTL_NS)
                            break
                        time.sleep(0.02)
                    self.assertTrue(recovered)
                    self.assertGreaterEqual(camera.recoveries, 1)
                finally:
                    started = time.monotonic(); camera.close()
                    self.assertLess(time.monotonic() - started, 2)
                    self.assertFalse(camera._thread.is_alive())
                    self.assertIsNone(camera._process)

    def test_reopen_failure_does_not_stop_supervisor_or_leak_processes(self):
        with tempfile.TemporaryDirectory() as directory, \
                patch.object(camera_base, '_camera_capture', fake_capture):
            camera = camera_base.Camera(str(Path(directory) / 'exit'), 640, 480, 30)
            camera.RETRY_S = 0.02
            original = camera._start_capture
            calls = []

            def flaky_open():
                calls.append(1)
                if len(calls) <= 2:
                    raise RuntimeError('temporary open failure')
                return original()

            try:
                with patch.object(camera, '_start_capture', side_effect=flaky_open):
                    deadline = time.monotonic() + 8
                    while time.monotonic() < deadline:
                        frame = camera.read()
                        if frame is not None and frame[0, 0, 0] == 2:
                            break
                        time.sleep(0.02)
                    else:
                        self.fail('相机重开失败后未恢复')
                self.assertGreaterEqual(len(calls), 3)
                self.assertTrue(camera._thread.is_alive())
            finally:
                camera.close()
            self.assertFalse(camera._thread.is_alive())
            self.assertIsNone(camera._process)

    def test_open_hang_times_out_and_leaves_no_child(self):
        children = {p.pid for p in multiprocessing.active_children()}
        with patch.object(camera_base, '_camera_capture', blocked_open), \
                patch.object(camera_base.Camera, 'OPEN_TIMEOUT_S', 0.4):
            with self.assertRaises(RuntimeError):
                camera_base.Camera('fake', 640, 480, 30)
        self.assertEqual({p.pid for p in multiprocessing.active_children()}, children)

    def test_read_drops_stale_or_future_frame(self):
        camera = camera_base.Camera.__new__(camera_base.Camera)
        camera._lock = threading.Lock()
        for source in (time.monotonic_ns() - 1_000_000_000, time.monotonic_ns() + 1_000_000_000):
            camera._frame = np.zeros((2, 2, 3)); camera._frame_ns = source
            self.assertEqual(camera.read_sample(), (None, 0))


if __name__ == '__main__':
    unittest.main()
