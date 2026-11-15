"""离线回归：python -m unittest discover -s tests -v（无需摄像头/串口）。"""

import argparse
from collections import deque
from dataclasses import replace
import json
from pathlib import Path
import sys
import tempfile
import threading
import time
import unittest
from unittest.mock import patch

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import logistics_vision_main as main
import material_color_detect as colors
import material_modes as material
from material_grasp import STATE_MULTI


class IdentityUndistorter:
    def remap(self, frame):
        return frame


class FakeCamera:
    real_w, real_h = 640, 480

    def __init__(self, *args):
        self.frames = deque()
        self.lock = threading.Lock()
        self.closed = False

    def read(self):
        with self.lock:
            return self.frames.popleft() if self.frames else None

    def put(self, frame):
        with self.lock:
            self.frames.append(frame)

    def close(self):
        self.closed = True


def six_colors():
    hsv = np.full((480, 640, 3), (0, 0, 180), dtype=np.uint8)
    for index, value in enumerate(((0, 220, 180), (26, 220, 180),
                                   (110, 220, 180), (70, 180, 180),
                                   (0, 20, 30), (99, 200, 180))):
        cv2.circle(hsv, (60 + index * 100, 230), 25, value, -1)
    return cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)


def blob(color=1, center=(100.0, 200.0), area=2000.0, confidence=0.9):
    return colors.MaterialBlob(color, colors.COLOR_NAMES[color], center, area,
                               confidence, (int(center[0]) - 20,
                                            int(center[1]) - 20, 40, 40))


def wait_for(predicate, timeout=3.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(0.005)
    raise AssertionError('等待工作线程结果超时')


class MaterialIntegrationTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.config_path = str(Path(self.tmp.name) / 'colors.json')
        self.config = colors.build_default_color_config()
        Path(self.config_path).write_text(json.dumps(self.config), encoding='utf-8')
        self.frame = six_colors()
        self.und = IdentityUndistorter()
        self.workers = []

    def tearDown(self):
        for worker in self.workers:
            worker.close()
            self.assertFalse(worker._thread.is_alive())
        self.tmp.cleanup()

    def worker(self, track=False, path=None):
        worker = material.MaterialModeWorker(self.und, path or self.config_path, track)
        self.workers.append(worker)
        return worker

    def process(self, worker, frame=None):
        source = time.monotonic_ns()
        worker.submit(self.frame if frame is None else frame, source)
        wait_for(lambda: worker.latest_source_ns == source)
        with worker.lock:
            return dict(worker.latest)

    def test_six_colors_and_filter_by_requested_color(self):
        all_worker, track_worker = self.worker(), self.worker(track=True)
        all_worker.start_mode()
        result = self.process(all_worker)
        self.assertEqual([b.color_id for b in result['materials']], [1, 2, 3, 4, 5, 6])
        self.assertFalse(result['ambiguous'])
        for color in range(1, 7):
            track_worker.start_mode(color=color)
            result = self.process(track_worker)
            self.assertEqual(result['status'], 'TRACKING')
            self.assertEqual(result['target'].color_id, color)
            self.assertEqual(len(result['materials']), 1)

    def test_overlap_winner_tie_and_same_color_neighbors(self):
        a = blob(color=3, confidence=0.9)
        b = replace(a, color_id=6, name='light_blue', confidence=0.8)
        accepted, ambiguous = material.resolve_color_overlaps([a, b])
        self.assertEqual(accepted, [a])
        self.assertEqual(ambiguous, [])
        accepted, ambiguous = material.resolve_color_overlaps(
            [a, replace(b, confidence=0.9)])
        self.assertEqual(accepted, [])
        self.assertEqual(len(ambiguous), 1)
        accepted, ambiguous = material.resolve_color_overlaps([a, replace(a)])
        self.assertEqual(len(accepted), 2)
        self.assertEqual(ambiguous, [])
        self.assertAlmostEqual(material.bbox_iou((0, 0, 3, 2), (1, 0, 3, 2)), 0.5)

    def test_real_threshold_overlap_does_not_become_track_target(self):
        self.config['colors'][2]['ranges'] = self.config['colors'][5]['ranges']
        Path(self.config_path).write_text(json.dumps(self.config), encoding='utf-8')
        worker = self.worker(track=True)
        worker.start_mode(color=3)
        result = self.process(worker)
        self.assertEqual(len(result['ambiguous']), 1)
        self.assertIsNone(result['target'])
        self.assertEqual(result['status'], 'LOST')

    def test_region_priority_and_multi_invalid(self):
        # 区域优先：区内唯一→目标=区内那个；区内≥2 同色→MULTI 无效（用户决定）
        worker = self.worker(track=True)
        worker.start_mode(color=3)
        result = self.process(worker)              # 蓝 blob 在 (260,230)：区内唯一
        self.assertEqual(result['status'], 'TRACKING')
        self.assertEqual(result['target'].color_id, 3)
        self.assertTrue(result['in_region'])
        self.assertEqual(result['state_name'], 'WAIT_MOTION')

        hsv = np.full((480, 640, 3), (0, 0, 180), np.uint8)
        for cx in (280, 380):                      # 两个蓝物料都在默认区内
            cv2.circle(hsv, (cx, 240), 25, (110, 220, 180), -1)
        result = self.process(worker, cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR))
        self.assertEqual(result['state'], STATE_MULTI)
        self.assertFalse(result['grasp_ready'])
        self.assertIsNone(result['grasp_center'])

        hsv = np.full((480, 640, 3), (0, 0, 180), np.uint8)
        cv2.circle(hsv, (60, 100), 25, (110, 220, 180), -1)   # 区外唯一
        result = self.process(worker, cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR))
        self.assertEqual(result['status'], 'TRACKING')
        self.assertFalse(result['in_region'])
        self.assertEqual(result['state_name'], 'OUT_OF_REGION')

    def test_current_missing_frame_never_keeps_old_target(self):
        worker = self.worker(track=True)
        worker.start_mode(color=1)
        self.assertIsNotNone(self.process(worker)['target'])
        empty = np.full_like(self.frame, 180)
        result = self.process(worker, empty)
        self.assertEqual(result['status'], 'LOST')
        self.assertIsNone(result['target'])

    def test_lazy_config_failure_and_retry(self):
        path = str(Path(self.tmp.name) / 'missing.json')
        with patch.object(material, 'load_color_config', wraps=colors.load_color_config) as load:
            worker = self.worker(path=path)
            self.assertEqual(load.call_count, 0)
            worker.start_mode()
            result = self.process(worker)
            self.assertEqual(result['status'], 'CONFIG_ERROR')
            self.assertIn(path, result['error'])
            self.process(worker)
            self.assertEqual(load.call_count, 1)
            Path(path).write_text('{broken json', encoding='utf-8')
            worker.stop_mode()
            worker.start_mode()
            self.assertEqual(self.process(worker)['status'], 'CONFIG_ERROR')
            Path(path).write_text(json.dumps(self.config), encoding='utf-8')
            worker.stop_mode()
            worker.start_mode()
            self.assertEqual(len(self.process(worker)['materials']), 6)

    def make_workers(self):
        args = argparse.Namespace(scale=1.0, roi=False, roi_full_scan_interval=15,
                                  min_dist=60, min_r=20, max_r=60, dp=1.0,
                                  param1=210, param2=60, no_verify=False,
                                  coverage=0.7, merge_dist=20)
        ring = main.RingModeWorker(self.und, args)
        turn = main.TurntableWorker(self.und)
        stacked = main.StackedModeWorker(self.und)
        self.workers.extend([ring, turn, stacked])
        return {'ring': ring, 'turntable': turn, 'stacked': stacked,
                'material_all': self.worker(),
                'material_track': self.worker(track=True)}

    def test_all_workers_discard_inflight_old_session(self):
        workers = self.make_workers()
        for name, worker in workers.items():
            with self.subTest(mode=name):
                entered, release, finished = (threading.Event() for _ in range(3))
                original = worker._process

                def blocked(frame, source, generation):
                    entered.set()
                    if not release.wait(3):
                        raise RuntimeError('测试未释放处理线程')
                    original(frame, source, generation)
                    finished.set()

                worker._process = blocked
                options = {'color': 1} if name == 'material_track' else {}
                worker.start_mode(**options)
                worker.submit(self.frame, time.monotonic_ns())
                self.assertTrue(entered.wait(3))
                worker.stop_mode()
                worker.start_mode(**options)
                release.set()
                self.assertTrue(finished.wait(3))
                self.assertEqual(worker.latest_source_ns, 0)
                self.assertIsNone(worker.latest_canvas)
                worker._process = original
                source = time.monotonic_ns()
                worker.submit(self.frame, source)
                wait_for(lambda: worker.latest_source_ns == source)
                self.assertEqual(worker.error_count, 0)
                worker.stop_mode()

    def test_camera_keyboard_serial_modes_and_silence(self):
        workers = self.make_workers()
        manager = main.ModeManager('raw', workers, None, True, track_color=2)
        link = main.SerialLink('unused', 115200, manager.request_mode)
        sent = []
        link.send = sent.append
        publisher = main.VisionPublisher(link, workers, manager)
        manager.publisher = publisher
        cam = FakeCamera()
        dispatcher = main.FrameDispatcher(cam, manager)
        publisher.start()
        dispatcher.start()
        try:
            for mode, color in ((main.MATERIAL_ALL, None), (main.MATERIAL_TRACK, 3),
                                (main.RAW, None), (main.TURNTABLE, None),
                                (main.RING, None), (main.MATERIAL_TRACK, 6)):
                payload = main.encode_mode(mode, color)
                # 经真实 SerialLink._poll 解析拆包及噪声，I/O由假串口提供。
                class Handle:
                    in_waiting = 1024

                    def read(self, count):
                        return self.parts.popleft() if self.parts else b''

                handle = Handle()
                handle.parts = deque([b'noise' + payload[:3], payload[3:]])
                link._handle = handle
                link._poll()
                link._poll()
                cam.put(self.frame)
                base = main.MODE_NAMES[mode]
                if base in workers:
                    wait_for(lambda: workers[base].latest_source_ns > 0)
                else:
                    wait_for(lambda: dispatcher.latest_frame is not None)
                before = len(sent)
                time.sleep(0.08)
                if mode in (main.RAW, main.MATERIAL_ALL):
                    self.assertEqual(len(sent), before)
                if mode == main.MATERIAL_TRACK:
                    # 静止合成场景：WAIT_MOTION（先见运动）→ 不发送
                    self.assertEqual(len(sent), before)
                for name, worker in workers.items():
                    self.assertEqual(worker._active, name == base)
                    self.assertEqual(worker.error_count, 0)
                image = main.display_frame(manager, dispatcher)
                self.assertEqual(image.shape, self.frame.shape)

            track = workers['material_track']
            generation = track.generation
            manager.request_mode('material_track 6')
            self.assertGreater(track.generation, generation)
            self.assertIsNone(track.latest['target'])
            self.assertIsNone(track.latest_canvas)
            main.handle_key(ord('c'), manager)
            self.assertEqual(manager.active_mode, 'material_track 1')
            main.handle_key(ord('0'), manager)
            self.assertFalse(track._active)
            raw = main.display_frame(manager, dispatcher)
            np.testing.assert_array_equal(raw[150:], self.frame[150:])
            for key in '12345':
                main.handle_key(ord(key), manager)
            main.handle_key(ord('0'), manager)
            self.assertTrue(main.handle_key(ord('q'), manager))
            self.assertTrue(all(not worker._active for worker in workers.values()))
        finally:
            dispatcher.stop()
            publisher.stop()
            cam.close()
        self.assertTrue(cam.closed)
        self.assertFalse(dispatcher.is_alive())
        self.assertFalse(publisher.is_alive())

    def test_ring_and_turntable_wire_formats_unchanged(self):
        self.assertEqual(main.encode_identified_ring_result([]).hex(),
                         'aa55130400000000170d0a')
        self.assertEqual(main.legacy_center_frame((320, 240)).hex(),
                         '6601014000f077')

    def test_main_entry_arguments_and_clean_exit(self):
        argv = ['logistics_vision_main.py', '--start-mode', 'material_track',
                '--track-color', '4', '--material-config', self.config_path,
                '--no-serial', '--display']
        with patch.object(sys, 'argv', argv):
            args = main.parse_args()
        self.assertEqual(args.track_color, 4)
        cam, managers = FakeCamera(), []
        keys = iter('2' '4' '1' '3' 'c' '0' '5' 'q')
        handle = main.handle_key

        def wait_key(delay):
            cam.put(self.frame)
            time.sleep(0.03)
            return ord(next(keys))

        def key_handler(key, manager):
            managers.append(manager)
            return handle(key, manager)

        with patch.object(main, 'parse_args', return_value=args), \
                patch.object(main, 'Camera', return_value=cam), \
                patch.object(main, 'Undistorter', return_value=self.und), \
                patch.object(main, 'handle_key', side_effect=key_handler), \
                patch('signal.signal'), \
                patch.object(cv2, 'namedWindow'), \
                patch.object(cv2, 'imshow'), \
                patch.object(cv2, 'waitKey', side_effect=wait_key), \
                patch.object(cv2, 'destroyAllWindows'):
            main.main()
        self.assertTrue(cam.closed)
        for worker in managers[-1].workers.values():
            self.assertTrue(worker._closed)
            self.assertFalse(worker._thread.is_alive())
            self.assertEqual(worker.error_count, 0)


if __name__ == '__main__':
    unittest.main()
