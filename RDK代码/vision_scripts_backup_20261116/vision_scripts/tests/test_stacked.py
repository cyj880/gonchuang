"""码垛模式离线验收：三环取中/基准线/侧站推算/平滑/0x14 帧（合成场景）。

场景几何：三工位 (168,240)/(320,240)/(472,240) 间距152px；每站同心圆
r=26/33/40/47；基准线上下过 (320, 240±74)（线偏移 75mm≈0.5×间距）。
"""

import math
import sys
import threading
import time
import unittest
from pathlib import Path
from unittest.mock import Mock

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import logistics_vision_main as main
from stacked_detector import StackedDetector

CENTERS = ((168, 240), (320, 240), (472, 240))
RADII = (26, 33, 40, 47)
LINE_Y = (166, 314)


def stacked_scene(line_specs=((0.0, 0.0), (0.0, 0.0)), with_lines=True,
                  shift=(0, 0)):
    """合成码垛场景。

    line_specs: 每条线 (角度deg, 过中心x处的附加dy)，None=该侧无线；
    shift: 三环+线整体平移（模拟小车运动）。"""
    frame = np.full((480, 640, 3), (80, 80, 80), np.uint8)
    cv2.rectangle(frame, (30, 50), (610, 430), (225, 225, 225), -1)
    for cx, cy in CENTERS:
        cv2.circle(frame, (cx + shift[0], cy + shift[1]), 0, (40, 40, 40), 0)
        for radius in RADII:
            cv2.circle(frame, (cx + shift[0], cy + shift[1]), radius,
                       (40, 40, 40), 3)
    if with_lines:
        for spec, base_y in zip(line_specs, LINE_Y):
            if spec is None:
                continue
            angle, dy = spec
            cx0 = 320 + shift[0]
            cy0 = base_y + dy + shift[1]
            slope = math.tan(math.radians(angle))
            cv2.line(frame, (cx0 - 260, int(round(cy0 - slope * 260))),
                     (cx0 + 260, int(round(cy0 + slope * 260))),
                     (30, 30, 30), 3)
    return frame


class StackedSceneTests(unittest.TestCase):
    def setUp(self):
        self.det = StackedDetector()

    def feed(self, frame, count):
        result = None
        for _ in range(count):
            result, _ = self.det.process(frame)
        return result

    def test_middle_ring_locked(self):
        # 全图霍夫三环取中: 首个确认窗后持续检出中环 (320,240), yaw≈0
        result = self.feed(stacked_scene(), 6)
        self.assertTrue(result['detected'], result.get('failure_reason'))
        self.assertEqual(result['state'], 'tracking')
        self.assertLess(math.dist((result['middle_center_px']['x'],
                                   result['middle_center_px']['y']),
                                  (320, 240)), 4)
        self.assertAlmostEqual(result['station_yaw_deg'], 0.0, delta=1.5)

    def test_side_stations_inferred(self):
        result = self.feed(stacked_scene(), 6)
        sides = result['side_centers_inferred']
        self.assertEqual([s['source'] for s in sides],
                         ['inferred', 'inferred'])
        self.assertLess(math.dist((sides[0]['x'], sides[0]['y']),
                                  (168, 240)), 6)
        self.assertLess(math.dist((sides[1]['x'], sides[1]['y']),
                                  (472, 240)), 6)

    def test_single_line_degrades(self):
        # 只有上长边可见: 单线定 yaw, 检出不断流
        result = self.feed(stacked_scene(line_specs=((0.0, 0.0), None)), 6)
        self.assertTrue(result['detected'])
        self.assertEqual(result['line_source'], 'single')
        self.assertAlmostEqual(result['station_yaw_deg'], 0.0, delta=1.5)

    def test_lines_missing_falls_back_to_baseline(self):
        # 双线全失: 回退三环基线角, 不拒帧不断流
        result = self.feed(stacked_scene(with_lines=False), 6)
        self.assertTrue(result['detected'])
        self.assertEqual(result['line_source'], 'baseline')
        self.assertAlmostEqual(result['station_yaw_deg'], 0.0, delta=1.5)

    def test_line_conflict_falls_back_to_baseline(self):
        # 上下线各自过预期偏移点但角度差 3° > 2°: 回退基线角, 不拒帧
        result = self.feed(stacked_scene(line_specs=((1.5, 0.0), (-1.5, 0.0))), 6)
        self.assertTrue(result['detected'])
        self.assertEqual(result['line_source'], 'baseline')
        self.assertAlmostEqual(result['station_yaw_deg'], 0.0, delta=1.5)

    def test_no_triple_outputs_empty(self):
        result = self.feed(np.full((480, 640, 3), 90, np.uint8), 8)
        self.assertFalse(result['detected'])
        self.assertTrue(result['failure_reason'])
        self.assertIsNone(result['middle_center_px'])

    def test_static_output_smooth(self):
        frame = stacked_scene()
        self.feed(frame, 10)
        centers = []
        for _ in range(10):
            result, _ = self.det.process(frame)
            centers.append((result['middle_center_px']['x'],
                            result['middle_center_px']['y']))
        xs = [c[0] for c in centers]
        ys = [c[1] for c in centers]
        self.assertLessEqual(max(xs) - min(xs), 1.5)
        self.assertLessEqual(max(ys) - min(ys), 1.5)

    def test_motion_tracking_no_loss(self):
        # 每帧 +6px 横移(约 13Hz 下 0.4m/s): 全程检出, alphabeta 收敛后跟随
        result = None
        for i in range(25):
            frame = stacked_scene(shift=(6 * i - 72, 0))
            result, _ = self.det.process(frame)
        self.assertTrue(result['detected'])
        true_x = 320 + (6 * 24 - 72)
        self.assertLess(abs(result['middle_center_px']['x'] - true_x), 10,
                        f"out={result['middle_center_px']} true_x={true_x}")

    def test_reset_clears_session_state(self):
        self.feed(stacked_scene(), 6)
        self.det.reset()
        self.assertIsNone(self.det._yaw)
        self.assertIsNone(self.det._gaps)
        result = self.feed(stacked_scene(), 4)
        self.assertTrue(result['detected'])     # 3 帧确认后重锁


class StackedWireTests(unittest.TestCase):
    def test_wire_shape(self):
        full = main.encode_stacked_result(dict(
            detected=True, middle_center_px={'x': 320.0, 'y': 240.0},
            station_yaw_deg=0.5))
        self.assertEqual(full.hex(), 'aa551407014001f00032007f0d0a')
        blank = main.encode_stacked_result({'detected': False})
        self.assertEqual(blank[2], 0x14)
        self.assertEqual(blank[3], 7)
        self.assertEqual(blank[4], 0)                    # FLAGS=0 空帧
        self.assertEqual(blank[11], (0x14 + 7) & 0xFF)   # 校验和
        self.assertEqual(blank[12:14], b'\x0d\x0a')

    def test_publisher_sends_continuous_stream(self):
        worker = Mock()
        worker.lock = threading.Lock()
        worker.latest = {'detected': True, 'stable': False,
                         'middle_center_px': {'x': 320, 'y': 240},
                         'station_yaw_deg': 0.5}
        manager = Mock(active_mode='stacked')
        link = Mock()
        publisher = main.VisionPublisher(link, {'stacked': worker}, manager)
        worker.latest_source_ns = time.monotonic_ns()
        publisher._next_stacked_tx = 0.0
        publisher._publish_stacked()
        self.assertEqual(link.send.call_count, 1)
        frame = link.send.call_args.args[0]
        self.assertEqual(frame[2], 0x14)
        self.assertEqual(frame[4] & 1, 1)                # detected 位
        publisher._publish_stacked()
        self.assertEqual(link.send.call_count, 1)        # 10Hz 限频
        worker.latest = {'detected': False}              # 未检出: 连续流空帧
        publisher._next_stacked_tx = 0.0
        publisher._publish_stacked()
        self.assertEqual(link.send.call_count, 2)
        self.assertEqual(link.send.call_args.args[0][4], 0)


if __name__ == '__main__':
    unittest.main()
