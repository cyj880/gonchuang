"""物料抓取判定离线验收：转盘运动状态机 / 抓取区 / 0x11 连续流（2026-10-31）。

历史沿革：几何链用例随 10-30 矩心化删除；10-31 停稳判断升级为
"固定抓取区 + 先见运动 + 低速证据"状态机（cyqm 移植适配），旧矩心门控
用例随之重写。转盘外圆检测器（turntable 模式在用）的两条用例保留。"""

import json
import math
from collections import deque
from pathlib import Path
import sys
import tempfile
import threading
import time
import unittest
from unittest.mock import Mock, patch

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import logistics_vision_main as main
import material_color_detect as colors
import material_grasp as grasp
from material_grasp import (STATE_MOVING, STATE_MULTI, STATE_NOT_VISIBLE,
                            STATE_OUT_OF_REGION, STATE_READY, STATE_SETTLING,
                            STATE_WAITING_MOTION)
from material_modes import MaterialModeWorker
from turntable_detector import TurntableDetector


class CalibratedUndistorter:
    size = calibration_size = (640, 480)
    new_K = np.array([[320., 0, 320], [0, 320., 240], [0, 0, 1.]])

    def remap(self, frame):
        return frame


def material_image(center=(400, 240), radius=30):
    """灰色背景上一个红色圆柱物料（顶视），色块矩质心即圆心。"""
    frame = np.full((480, 640, 3), 180, np.uint8)
    cv2.circle(frame, tuple(center), radius, (0, 0, 160), -1)
    return frame


def blob(center=(320.0, 240.0), bbox=None):
    cx, cy = center
    box = bbox or (int(cx - 40), int(cy - 40), 80, 80)
    return colors.MaterialBlob(color_id=4, name='green', center=tuple(center),
                               area_px=2000.0, confidence=0.9, bbox=box)


DT = 0.08     # 12.5fps 量级帧间隔


class StateMachineTests(unittest.TestCase):
    """GraspMotionJudge：先见运动 → 转动确认 → 低速证据 → READY。"""

    def setUp(self):
        self.judge = grasp.GraspMotionJudge(grasp.GraspParams())

    def feed(self, steps, t0=0.0):
        states, t = [], t0
        for step in steps:
            t += DT
            states.append(self.judge.update(t, *step)[0])
        return states

    def stationary(self, n):
        return [(blob(), True, False)] * n

    def moving(self, n):
        # 12px / 0.08s = 150px/s：落在规则折算的转动速度带内
        return [(blob((320.0 + 12.0 * i, 240.0)), True, False)
                for i in range(n)]

    def test_initial_static_never_ready(self):
        # 用户决定③：开机看到静止必须先等转盘转过，禁止直接 READY
        states = self.feed(self.stationary(12))
        self.assertTrue(all(s == STATE_WAITING_MOTION for s in states))
        self.assertFalse(self.judge.motion_confirmed)

    def test_motion_then_stop_ready(self):
        steps = self.moving(4)
        last = (320.0 + 12.0 * 3, 240.0)                    # 从末速位置停住
        states = self.feed(steps + [(blob(last), True, False)] * 8)
        self.assertEqual(states[0], STATE_WAITING_MOTION)   # 首帧建基线
        self.assertEqual(list(states[1:4]), [STATE_MOVING] * 3)
        self.assertTrue(self.judge.motion_confirmed)
        self.assertEqual(list(states[4:8]), [STATE_SETTLING] * 4)
        self.assertEqual(states[8], STATE_READY)            # 低速证据攒满
        self.assertTrue(all(s == STATE_READY for s in states[8:]))

    def test_ready_requires_region(self):
        self.feed(self.moving(4))                           # 确认转动
        t = 5 * DT                                          # 时间延续（单调）
        states = []
        for _ in range(6):
            t += DT
            states.append(self.judge.update(
                t, blob((80.0, 240.0)), False, False)[0])
        self.assertEqual(states[0], STATE_MOVING)   # 位置瞬变如实视为运动
        self.assertTrue(all(s == STATE_OUT_OF_REGION for s in states[1:]))
        states = self.feed(self.stationary(6), t)
        self.assertEqual(states[-1], STATE_READY)

    def test_loss_preserves_motion_confirmed(self):
        self.feed(self.moving(4))
        self.assertTrue(self.judge.motion_confirmed)
        states = self.feed([(None, False, False)] * 4)
        self.assertTrue(all(s == STATE_NOT_VISIBLE for s in states))
        states = self.feed(self.stationary(6))
        self.assertEqual(states[-1], STATE_READY)           # 无需重新确认转动

    def test_multi_in_region_invalid(self):
        self.feed(self.moving(4))
        states = self.feed([(None, False, True)] * 3)
        self.assertTrue(all(s == STATE_MULTI for s in states))
        states = self.feed(self.stationary(6))
        self.assertEqual(states[-1], STATE_READY)           # 恢复后重新攒证据

    def test_frame_gap_rebuilds_baseline(self):
        self.feed(self.moving(4))
        t = 5 * DT + 1.0                                    # 断流 1s 的大间隔
        state, speed = self.judge.update(t, blob(), True, False)
        self.assertEqual(state, STATE_SETTLING)
        self.assertEqual(speed, 0.0)
        self.assertTrue(self.judge.motion_confirmed)        # 已转动事实保留
        states = self.feed(self.stationary(6))
        self.assertEqual(states[-1], STATE_READY)

    def test_edge_guarded_blob_not_measured(self):
        self.feed(self.moving(4))
        clipped = blob((10.0, 240.0), bbox=(0, 200, 60, 80))  # 触左边距圈
        states = self.feed([(clipped, False, False)] * 3)
        self.assertTrue(all(s == STATE_OUT_OF_REGION for s in states))
        self.assertIsNone(self.judge._previous)               # 测速基线已重建

    def test_param_validation(self):
        with self.assertRaises(ValueError):
            grasp.GraspParams(moving_speed_px_s=30.0, near_stop_speed_px_s=30.0)
        with self.assertRaises(ValueError):
            grasp.GraspParams(required_low_speed_frames=0)
        p = grasp.GraspParams()
        self.assertTrue(p.in_region((320, 240)))
        self.assertFalse(p.in_region((100, 240)))


class TurntableDetectorTests(unittest.TestCase):
    """转盘外圆检测器（turntable 模式与其它用途共享）的回归。"""

    def tray_scene(self, radius_px=160):
        frame = np.full((480, 640, 3), 120, np.uint8)
        cv2.circle(frame, (320, 240), radius_px, (220, 220, 220), -1)
        return frame

    def test_extracted_tray_detector_and_current_miss(self):
        frame = self.tray_scene()
        detector = TurntableDetector()
        result = None
        for _ in range(6):
            result, _, _ = detector.detect(frame)
        self.assertIsNotNone(result)
        self.assertTrue(result['stable'])
        self.assertLess(math.dist((result['center_u'], result['center_v']), (320, 240)), 2)
        lost, _, _ = detector.detect(np.full_like(frame, 120))
        self.assertIsNone(lost)

    def test_current_tray_fit_is_not_hidden_by_history_median(self):
        detector = TurntableDetector()
        frame = self.tray_scene()

        def fit(u):
            return dict(u=u, v=240., radius=160., confidence=1., fit_rms_px=0.)

        with patch.object(detector, '_global_refine', return_value=fit(320.)), \
                patch.object(detector, '_refine', return_value=fit(320.)) as refine:
            for _ in range(6):
                detector.detect(frame)
            refine.return_value = fit(324.)
            result, _, _ = detector.detect(frame)
        self.assertTrue(result['stable'])
        self.assertEqual(result['center_u'], 320)
        self.assertEqual(result['current_center'], (324., 240.))
        self.assertEqual(result['current_radius_px'], 160.)

    def test_height_from_focal_and_rules_diameter(self):
        # 规则 ⌀300mm（半径150mm）：focal=320px、实测盘半径160px
        # → Z = 320×150/160 = 300mm
        detector = TurntableDetector(focal_px=320.0)
        frame = self.tray_scene(radius_px=160)
        result = None
        for _ in range(6):
            result, _, _ = detector.detect(frame)
        self.assertTrue(result['stable'])
        self.assertIsNotNone(result['height_mm'])
        self.assertLess(abs(result['height_mm'] - 300.0), 4.0)

    def test_height_none_without_focal(self):
        detector = TurntableDetector()               # 无内参兜底窗 → 无高度
        frame = self.tray_scene()
        result, _, _ = detector.detect(frame)
        self.assertIsNone(result['height_mm'])


class WireTests(unittest.TestCase):
    def test_material_state_frame_bytes(self):
        # 0x11 帧家族: AA 55 11 02 STATE COLOR CK 0D 0A
        # CK = (0x11 + 0x02 + STATE + COLOR) & 0xFF
        cases = ((grasp.STATE_READY, 3, 'aa55110205031b0d0a'),        # 周期报告
                 (grasp.STATE_WAITING_MOTION, 1, 'aa5511020201160d0a'),  # 回执
                 (grasp.STATE_NOT_VISIBLE, 1, 'aa5511020001140d0a'),     # 回执
                 (grasp.STATE_MULTI, 6, 'aa55110206061f0d0a'))           # 回执
        for state, color, expected in cases:
            frame = main.encode_material_state_frame(state, color)
            self.assertEqual(frame.hex(), expected, (state, color))
            self.assertEqual(len(frame), 9)
        for state, color in ((7, 1), (-1, 1), (5, 0), (5, 7)):
            with self.assertRaises(ValueError):
                main.encode_material_state_frame(state, color)

    @staticmethod
    def make_publisher(latest, active_mode='material_track 3'):
        worker = Mock()
        worker.lock = threading.Lock()
        worker.generation = 7
        worker.grasp_params = grasp.GraspParams()
        worker.latest = dict(latest)
        manager = Mock(active_mode=active_mode)
        link = Mock()
        link.send.return_value = True
        publisher = main.VisionPublisher(link, {'material_track': worker},
                                         manager)
        worker.latest_source_ns = time.monotonic_ns()   # 源帧在命令之后
        return publisher, worker, link

    def test_publisher_resends_while_settled(self):
        # 停稳期间 10Hz 持续发送（单片机收首帧即夹取，多余帧无害）
        publisher, worker, link = self.make_publisher(
            {'grasp_ready': True, 'settled': True, 'generation': 7,
             'track_color': 3, 'state': grasp.STATE_READY})
        publisher._next_material_tx = 0.0
        publisher._publish_material()
        self.assertEqual(link.send.call_count, 1)
        publisher._next_material_tx = 0.0
        publisher._publish_material()                 # 停稳期间持续重发
        self.assertEqual(link.send.call_count, 2)
        self.assertEqual(link.send.call_args.args[0],
                         main.encode_material_state_frame(
                             grasp.STATE_READY, 3))
        publisher.on_mode_changed()                   # 新命令 = 新周期
        worker.latest_source_ns = time.monotonic_ns()  # 新命令之后的新鲜源帧
        publisher._next_material_tx = 0.0
        publisher._publish_material()
        self.assertEqual(link.send.call_count, 3)

    def test_publisher_reports_empty_region_when_settled_outside(self):
        # 盘已停稳但目标在区外: 持续发送 STATE=1（抓取区内还没有物料）
        publisher, worker, link = self.make_publisher(
            {'grasp_ready': False, 'settled': True, 'generation': 7,
             'track_color': 3, 'state': grasp.STATE_OUT_OF_REGION})
        publisher._next_material_tx = 0.0
        publisher._publish_material()
        self.assertEqual(link.send.call_count, 1)
        self.assertEqual(link.send.call_args.args[0],
                         main.encode_material_state_frame(
                             grasp.STATE_OUT_OF_REGION, 3))
        publisher._next_material_tx = 0.0
        publisher._publish_material()                 # 停稳期间持续重发
        self.assertEqual(link.send.call_count, 2)

    def test_publisher_silent_when_not_settled(self):
        # 未停稳（转动中）/区内多物料/目标不可见 → 静默
        for state in (grasp.STATE_MOVING, grasp.STATE_MULTI,
                      grasp.STATE_NOT_VISIBLE):
            publisher, worker, link = self.make_publisher(
                {'grasp_ready': False, 'settled': False, 'generation': 7,
                 'track_color': 3, 'state': state})
            publisher._next_material_tx = 0.0
            publisher._publish_material()
            self.assertEqual(link.send.call_count, 0, state)

    def test_publisher_send_failure_retries_next_tick(self):
        # 写失败下个节拍用新鲜帧自动重试（无锁存状态，停稳期间一直重试）
        publisher, worker, link = self.make_publisher(
            {'grasp_ready': True, 'settled': True, 'generation': 7,
             'track_color': 3, 'state': grasp.STATE_READY})
        link.send.return_value = False
        publisher._next_material_tx = 0.0
        publisher._publish_material()
        self.assertEqual(link.send.call_count, 1)
        link.send.return_value = True
        publisher._next_material_tx = 0.0
        publisher._publish_material()
        self.assertEqual(link.send.call_count, 2)

    def test_material_track_ack_bytes(self):
        # 0x18 应答: AA 55 18 01 COLOR CK 0D 0A, CK=(0x18+0x01+COLOR)&0xFF
        self.assertEqual(main.encode_material_track_ack(3).hex(),
                         'aa551801031c0d0a')
        self.assertEqual(main.encode_material_track_ack(6).hex(),
                         'aa551801061f0d0a')
        self.assertEqual(len(main.encode_material_track_ack(1)), 8)
        for color in (0, 7):
            with self.assertRaises(ValueError):
                main.encode_material_track_ack(color)

    def test_serial_link_acks_track_commands(self):
        # 每条追踪命令（含同色重复）回一帧 ACK；非追踪命令不回
        sent = []
        link = main.SerialLink('unused', 115200, lambda s: None)
        link.send = sent.append

        class Handle:
            in_waiting = 1024

            def read(self, count):
                return self.parts.popleft() if self.parts else b''

        handle = Handle()
        handle.parts = deque([main.encode_mode(main.MATERIAL_TRACK, 3),
                              main.encode_mode(main.MATERIAL_TRACK, 3),
                              main.encode_mode(main.RAW, None)])
        link._handle = handle
        for _ in range(3):                 # 每次 _poll 读一个分段
            link._poll()
        self.assertEqual(sent, [main.encode_material_track_ack(3),
                                main.encode_material_track_ack(3)])

    def test_publisher_rejections(self):
        worker = Mock()
        worker.lock = threading.Lock()
        worker.generation = 7
        worker.grasp_params = grasp.GraspParams()
        worker.latest_source_ns = time.monotonic_ns()
        worker.latest = {'grasp_ready': True, 'generation': 7,
                         'track_color': 3}
        manager = Mock(active_mode='material_track 3')
        link = Mock()
        publisher = main.VisionPublisher(link, {'material_track': worker},
                                         manager)
        for change in ('stale', 'future', 'color', 'generation', 'precommand',
                       'not_ready'):
            worker.latest_source_ns = time.monotonic_ns()
            worker.latest = {'grasp_ready': True, 'settled': True,
                             'state': grasp.STATE_READY, 'generation': 7,
                             'track_color': 3,
                             'grasp_center': (320.0, 240.0)}
            if change == 'stale':
                worker.latest_source_ns -= 400_000_000
            elif change == 'future':
                worker.latest_source_ns += 1_000_000_000
            elif change == 'color':
                worker.latest['track_color'] = 2
            elif change == 'generation':
                worker.latest['generation'] = 6
            elif change == 'precommand':
                publisher._mode_since_ns = time.monotonic_ns() + 1_000_000_000
            elif change == 'not_ready':
                worker.latest['grasp_ready'] = False
                worker.latest['settled'] = False
            publisher._publish_material()
            self.assertFalse(link.send.called, change)

    def test_serial_send_reports_full_write_only(self):
        link = main.SerialLink('unused', 115200, lambda s: None)
        link._handle = Mock()
        link._handle.write.return_value = 3
        self.assertTrue(link.send(b'abc'))
        self.assertEqual(link.tx_count, 1)
        handle = link._handle
        handle.write.return_value = 1
        self.assertFalse(link.send(b'abc'))
        self.assertIsNone(link._handle)
        handle.close.assert_called_once()


class GraspWorkerTests(unittest.TestCase):
    def test_initial_static_waits_for_motion(self):
        # 真实检测管线 + 静止物料：先见运动语义（初始静止永远不 READY）
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / 'colors.json'
            path.write_text(json.dumps(colors.build_default_color_config()),
                            encoding='utf-8')
            worker = MaterialModeWorker(CalibratedUndistorter(), str(path), True)
            try:
                worker.start_mode(color=1)
                image = material_image()
                start = time.monotonic_ns()
                for i in range(13):
                    source = start + i * 100_000_000
                    worker.submit(image, source)
                    deadline = time.monotonic() + 3
                    while worker.latest_source_ns != source and \
                            time.monotonic() < deadline:
                        time.sleep(0.002)
                    self.assertEqual(worker.latest_source_ns, source,
                                     worker.last_error)
                result = worker.latest
                self.assertFalse(result['grasp_ready'], result.get('reason'))
                self.assertEqual(result['state'], grasp.STATE_WAITING_MOTION)
                self.assertTrue(result['in_region'])
                self.assertLess(math.dist(result['grasp_center'], (400, 240)), 2)
                source += 100_000_000
                worker.submit(np.full_like(image, 180), source)
                deadline = time.monotonic() + 3
                while worker.latest_source_ns != source and \
                        time.monotonic() < deadline:
                    time.sleep(0.002)
                self.assertEqual(worker.latest['state'], grasp.STATE_NOT_VISIBLE)
                self.assertIsNone(worker.latest.get('grasp_center'))
                worker.stop_mode()
                worker.start_mode(color=1)
                self.assertEqual(worker.latest['state'], 0)
                self.assertEqual(worker.error_count, 0)
            finally:
                worker.close()

    def test_settled_resends_until_material_gone(self):
        # 停稳期间持续发送（单片机收首帧即夹取，多余帧无害）；物料夹走→
        # 未停稳静默；新命令重臂新周期；区外停稳→STATE=1 持续发送
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / 'colors.json'
            path.write_text(json.dumps(colors.build_default_color_config()),
                            encoding='utf-8')
            worker = MaterialModeWorker(CalibratedUndistorter(), str(path), True)
            workers = {'material_track': worker}
            manager = main.ModeManager('raw', workers, None, True)
            link = main.SerialLink('unused', 115200, manager.request_mode)
            handle = Mock()
            handle.write.side_effect = len
            link._handle = handle
            publisher = main.VisionPublisher(link, workers, manager)
            manager.publisher = publisher
            publisher.start()

            def feed(frames):
                for image in frames:
                    source = time.monotonic_ns()
                    worker.submit(image, source)
                    deadline = time.monotonic() + 2
                    while worker.latest_source_ns != source and \
                            time.monotonic() < deadline:
                        time.sleep(0.002)
                    self.assertEqual(worker.latest_source_ns, source,
                                     worker.last_error)

            def cycle_frames(moving_xs, settle_x):
                # 时间戳用真实单调钟（合成时钟会超前真实时间，触发"源帧
                # 来自未来"拒收）。运动 3 帧（≈126px/s ≥60 确认转动）+
                # 静止 7 帧（攒满 5 帧低速证据）+ 5 帧（停稳期供持续重发）
                return ([material_image(center=(cx, 240))
                         for cx in moving_xs] +
                        [material_image(center=(settle_x, 240))] * 12)

            try:
                manager.request_mode('material_track 1')   # 颜色命令开启周期
                feed(cycle_frames((400, 412, 424), 424))   # 区内停稳
                deadline = time.monotonic() + 2
                while handle.write.call_count < 2 and \
                        time.monotonic() < deadline:
                    time.sleep(0.01)
                self.assertGreaterEqual(handle.write.call_count, 2)
                self.assertEqual(handle.write.call_args.args[0],
                                 main.encode_material_state_frame(
                                     grasp.STATE_READY, 1))
                count = handle.write.call_count
                # 物料夹走（空白帧）: 未停稳 → 静默（先等过渡期与 TTL 走完
                # 再取基线——最后一次停稳快照在转静默时还会发出一帧）
                feed([np.full((480, 640, 3), 180, np.uint8)] * 3)
                time.sleep(0.5)
                count = handle.write.call_count
                time.sleep(0.4)
                self.assertEqual(handle.write.call_count, count)
                # 新命令重臂新周期；区外停稳 → STATE=1 持续发送
                manager.request_mode('material_track 1')
                feed(cycle_frames((100, 112, 124), 124))
                deadline = time.monotonic() + 2
                while handle.write.call_count <= count and \
                        time.monotonic() < deadline:
                    time.sleep(0.01)
                self.assertGreater(handle.write.call_count, count)
                self.assertEqual(handle.write.call_args.args[0],
                                 main.encode_material_state_frame(
                                     grasp.STATE_OUT_OF_REGION, 1))
            finally:
                publisher.stop()
                worker.close()


if __name__ == '__main__':
    unittest.main()
