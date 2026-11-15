"""line_yaw 共享模块 + 圆环模式线角 yaw 移植回归（合成场景，无需相机/串口）。

场景几何（与 test_stacked 同款）：三工位 (168,240)/(320,240)/(472,240)
间距 152px；同心圆 r=26/33/40/47；上侧边界线过环心线上方 76px
（=半实测间距，对应理论垂距 75mm≈73.2px，实测学习值 74.8~75.7px）。
"""

import argparse
import math
import struct
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
from line_yaw import BandLineFinder

CENTERS = ((168, 240), (320, 240), (472, 240))
RADII = (26, 33, 40, 47)
LINE_OFFSET = 76.0          # 边界线到环心线的垂距（= 半实测间距）


def ring_scene(with_line=True, angle=0.0, single_ring=False):
    """合成圆环场景：三工位同心圆 + 可选上侧边界线。

    angle: 线角度 deg（绕画面中心 x=320 旋转）；single_ring: 只画最左
    一个工位（构造 PARTIAL）。"""
    frame = np.full((480, 640, 3), (80, 80, 80), np.uint8)
    cv2.rectangle(frame, (30, 50), (610, 430), (225, 225, 225), -1)
    centers = CENTERS[:1] if single_ring else CENTERS
    for cx, cy in centers:
        for radius in RADII:
            cv2.circle(frame, (cx, cy), radius, (40, 40, 40), 3)
    if with_line:
        y_mid = CENTERS[0][1] - LINE_OFFSET
        slope = math.tan(math.radians(angle))
        cv2.line(frame, (60, int(round(y_mid - slope * 260))),
                 (580, int(round(y_mid + slope * 260))), (30, 30, 30), 3)
    return frame


def band_edges(angle=0.0, y_offset=LINE_OFFSET, draw=True):
    """直接构造 BandLineFinder 输入：稀疏边缘图 + 已知直线。"""
    edges = np.zeros((480, 640), np.uint8)
    if draw:
        y_mid = 240.0 - y_offset
        slope = math.tan(math.radians(angle))
        cv2.line(edges, (60, int(round(y_mid - slope * 260))),
                 (580, int(round(y_mid + slope * 260))), 255, 1)
    return edges


def wait_for(predicate, timeout=3.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(0.005)
    raise AssertionError('等待工作线程结果超时')


class BandLineFinderTests(unittest.TestCase):
    def setUp(self):
        self.finder = BandLineFinder()

    def find(self, edges, angle_prior=0.0):
        return self.finder.find(edges, 320.0, 240.0, LINE_OFFSET, 94.0,
                                angle_prior, -1.0)

    def test_find_hits_drawn_line(self):
        line = self.find(band_edges())
        self.assertIsNotNone(line)
        self.assertAlmostEqual(line[1], 0.0, delta=0.5)
        self.assertIsNotNone(self.finder.seed)

    def test_find_angle(self):
        line = self.find(band_edges(angle=3.0))
        self.assertIsNotNone(line)
        self.assertAlmostEqual(line[1], 3.0, delta=0.5)

    def test_no_line_returns_none_and_no_seed(self):
        self.assertIsNone(self.find(band_edges(draw=False)))
        self.assertIsNone(self.finder.seed)

    def test_distance_gate_rejects(self):
        # 线存在但离环心 40px（预期 76px，容差 max(2, 76×5%)≈3.8px）→ 拒绝
        line = self.find(band_edges(y_offset=40.0))
        self.assertIsNone(line)
        self.assertIsNone(self.finder.seed)

    def test_clear_seed(self):
        self.find(band_edges())
        self.assertIsNotNone(self.finder.seed)
        self.finder.clear_seed()
        self.assertIsNone(self.finder.seed)


class IdentityUndistorter:
    def remap(self, frame):
        return frame


class RingWorkerLineYawTests(unittest.TestCase):
    def setUp(self):
        self.und = IdentityUndistorter()
        self.workers = []

    def tearDown(self):
        for worker in self.workers:
            worker.close()
            self.assertFalse(worker._thread.is_alive())

    def worker(self):
        det_args = argparse.Namespace(
            scale=1.0, roi=False, roi_full_scan_interval=15,
            min_dist=60.0, min_r=20, max_r=60, dp=1.0,
            param1=210.0, param2=60.0, no_verify=False, coverage=0.7,
            merge_dist=20.0)
        worker = main.RingModeWorker(self.und, det_args, blur_k=3)
        worker.start_mode()
        self.workers.append(worker)
        return worker

    def process(self, worker, frame, count=1):
        out = None
        for _ in range(count):
            source = time.monotonic_ns()
            worker.submit(frame, source)
            wait_for(lambda: worker.latest_source_ns == source)
            with worker.lock:
                out = (list(worker.latest_sites), worker.latest_yaw,
                       worker.latest_line_source)
        return out

    def test_full_uses_line_yaw(self):
        # confirm_hits=8 后 FULL；线角优先（EMA 收敛后 ≈ 线角）
        worker = self.worker()
        sites, yaw, source = self.process(worker, ring_scene(), 12)
        self.assertEqual([s['id'] for s in sites], [1, 2, 3])
        self.assertEqual(source, 'line')
        self.assertAlmostEqual(yaw, 0.0, delta=1.5)

    def test_line_angle_follows_drawn_line(self):
        worker = self.worker()
        sites, yaw, source = self.process(worker, ring_scene(angle=3.0), 12)
        self.assertEqual(source, 'line')
        self.assertAlmostEqual(yaw, 3.0, delta=1.5)

    def test_no_line_falls_back_to_baseline(self):
        # 无线：回退三环基线角（水平三工位 ≈ 0°），不断流
        worker = self.worker()
        sites, yaw, source = self.process(worker, ring_scene(with_line=False),
                                          12)
        self.assertEqual([s['id'] for s in sites], [1, 2, 3])
        self.assertEqual(source, 'baseline')
        self.assertAlmostEqual(yaw, 0.0, delta=1.5)

    def test_partial_keeps_encoder_computed_yaw(self):
        # PARTIAL（单工位）：无发布 yaw（编码器按坐标现算，单点 yaw_valid=0）
        worker = self.worker()
        sites, yaw, source = self.process(
            worker, ring_scene(with_line=False, single_ring=True), 12)
        self.assertEqual(len(sites), 1)
        self.assertEqual(sites[0]['id'], 0)
        self.assertIsNone(yaw)
        self.assertEqual(source, 'none')

    def test_start_mode_clears_yaw_session(self):
        worker = self.worker()
        self.process(worker, ring_scene(), 10)
        worker.start_mode()                     # 新会话：清空结果与 yaw 状态
        self.assertEqual(worker.latest_sites, [])
        self.assertIsNone(worker.latest_yaw)
        self.assertEqual(worker.latest_line_source, 'none')

    def test_over_three_confirmed_groups_publish_empty(self):
        # 矩形四角 4 个确认组、共线度全拒绝无合法三元组：确认组 >3 超出
        # 0x13 编码上限，按空观测上报（不编造数据、不触发编码异常）
        worker = self.worker()
        frame = np.full((480, 640, 3), (80, 80, 80), np.uint8)
        cv2.rectangle(frame, (30, 50), (610, 430), (225, 225, 225), -1)
        for cx, cy in ((120, 100), (520, 100), (120, 380), (520, 380)):
            for radius in RADII:
                cv2.circle(frame, (cx, cy), radius, (40, 40, 40), 3)
        sites, yaw, source = self.process(worker, frame, 12)
        self.assertEqual(sites, [])
        self.assertIsNone(yaw)
        self.assertEqual(source, 'none')


class RingEncoderYawTests(unittest.TestCase):
    def payload_of(self, frame):
        return frame[4:4 + frame[3]]

    def test_explicit_yaw_overrides_baseline(self):
        # 两点水平放置（坐标基线角=0）：显式线角 2.5° 应覆盖
        sites = [{'x': 100, 'y': 110, 'id': 1}, {'x': 300, 'y': 110, 'id': 2}]
        payload = self.payload_of(main.encode_identified_ring_result(
            sites, yaw=2.5))
        self.assertEqual(payload[0], 2)                      # count
        self.assertEqual(payload[1], 1)                      # yaw_valid
        self.assertEqual(struct.unpack('<h', payload[2:4])[0], 250)

    def test_default_recomputes_from_points(self):
        sites = [{'x': 100, 'y': 110, 'id': 1}, {'x': 300, 'y': 150, 'id': 2}]
        payload = self.payload_of(main.encode_identified_ring_result(sites))
        yaw = struct.unpack('<h', payload[2:4])[0] / 100.0
        self.assertAlmostEqual(yaw, main.ring_yaw(sites), delta=0.02)

    def test_single_point_has_no_yaw(self):
        payload = self.payload_of(main.encode_identified_ring_result(
            [{'x': 100, 'y': 110, 'id': 0}]))
        self.assertEqual(payload[1], 0)                      # yaw_valid=0


class PublisherResilienceTests(unittest.TestCase):
    """发送线程韧性回归（2026-11-14 漏洞：编码异常曾致 vision-publisher
    线程永久退出——识别正常但串口静默，切模式不可恢复）。"""

    def publisher(self):
        worker = Mock()
        worker.lock = threading.Lock()
        worker.latest_sites = []
        worker.latest_source_ns = 0
        worker.latest_yaw = None
        manager = Mock()
        manager._lock = threading.Lock()
        manager.active_mode = 'ring'
        link = Mock()
        pub = main.VisionPublisher(link, {'ring': worker}, manager)
        return pub, worker, link

    def test_over_three_sites_no_longer_kills_publisher(self):
        pub, worker, link = self.publisher()
        worker.latest_sites = [{'x': 100, 'y': 110, 'id': 0},
                               {'x': 200, 'y': 110, 'id': 0},
                               {'x': 300, 'y': 110, 'id': 0},
                               {'x': 400, 'y': 110, 'id': 0}]
        worker.latest_source_ns = time.monotonic_ns()
        pub._publish_ring()                    # 修复前：此处抛 ValueError
        self.assertEqual(link.send.call_count, 1)
        frame = link.send.call_args.args[0]
        self.assertEqual(frame[4], 3)          # 清洗后上限 3 个站点

    def test_duplicate_and_out_of_bounds_sites_dropped(self):
        pub, worker, link = self.publisher()
        worker.latest_sites = [{'x': 100.2, 'y': 110.4, 'id': 0},
                               {'x': 100.3, 'y': 110.2, 'id': 0},  # 取整重合
                               {'x': 9999, 'y': 110, 'id': 0},     # 越界
                               {'x': 300, 'y': 110, 'id': 0}]
        worker.latest_source_ns = time.monotonic_ns()
        pub._publish_ring()
        frame = link.send.call_args.args[0]
        self.assertEqual(frame[4], 2)          # 去重 + 越界剔除

    def test_publisher_thread_survives_and_recovers(self):
        pub, worker, link = self.publisher()
        worker.latest_sites = [{'x': 100, 'y': 110, 'id': 0},
                               {'x': 200, 'y': 110, 'id': 0},
                               {'x': 300, 'y': 110, 'id': 0},
                               {'x': 400, 'y': 110, 'id': 0},
                               {'x': 500, 'y': 110, 'id': 0}]
        worker.latest_source_ns = time.monotonic_ns()
        t = threading.Thread(target=pub.run, daemon=True)
        t.start()
        try:
            deadline = time.monotonic() + 2
            while link.send.call_count < 1 and time.monotonic() < deadline:
                time.sleep(0.005)
            self.assertTrue(t.is_alive())      # 修复前：首个坏结果即死
            self.assertGreaterEqual(link.send.call_count, 1)
            # 场景恢复为合法 3 站（新 source_ns）→ 真实帧恢复
            worker.latest_sites = [{'x': 100, 'y': 110, 'id': 1},
                                   {'x': 260, 'y': 110, 'id': 2},
                                   {'x': 420, 'y': 110, 'id': 3}]
            worker.latest_source_ns = time.monotonic_ns()
            count = link.send.call_count
            deadline = time.monotonic() + 2
            while link.send.call_count <= count and time.monotonic() < deadline:
                time.sleep(0.005)
            self.assertGreater(link.send.call_count, count)
            self.assertEqual(link.send.call_args.args[0][4], 3)
        finally:
            pub._stop_evt.set()                    # 停手动线程（pub.stop 会
            t.join(timeout=2.0)                    # join 未 start 的自身）

    def test_run_catch_all_skips_bad_tick(self):
        pub, worker, link = self.publisher()
        original = pub._publish_ring
        calls = {'n': 0}

        def flaky():
            calls['n'] += 1
            if calls['n'] == 1:
                raise RuntimeError('boom')
            original()

        pub._publish_ring = flaky
        t = threading.Thread(target=pub.run, daemon=True)
        t.start()
        try:
            deadline = time.monotonic() + 2
            while calls['n'] < 2 and time.monotonic() < deadline:
                time.sleep(0.005)
            self.assertGreaterEqual(calls['n'], 2)   # 第一拍炸、第二拍继续
            self.assertTrue(t.is_alive())
        finally:
            pub._stop_evt.set()
            t.join(timeout=2.0)


class SerialLinkResilienceTests(unittest.TestCase):
    """串口接收线程韧性回归（2026-11-14 自检：run() 原本无异常捕获，
    与 publisher 同类漏洞——RX 线程死亡 = 永久失聪，收不到任何模式
    命令且 TX 照常、症状隐蔽）。"""

    RAW_CMD = bytes.fromhex('aa55100100110d0a')   # 0x10 raw 模式命令

    def test_rx_thread_survives_callback_exception(self):
        import itertools
        on_mode = Mock(side_effect=[RuntimeError('boom'), None])
        link = main.SerialLink('/dev/null', 115200, on_mode=on_mode)
        handle = Mock()
        handle.in_waiting = 8
        handle.read.side_effect = itertools.chain(
            [self.RAW_CMD, b'', self.RAW_CMD], itertools.repeat(b''))
        link._handle = handle                 # 注入句柄，绕过真实串口
        link.start()
        try:
            deadline = time.monotonic() + 2
            while on_mode.call_count < 2 and time.monotonic() < deadline:
                time.sleep(0.005)
            # 第一条命令的回调炸、第二条照常收到——RX 线程自愈
            self.assertGreaterEqual(on_mode.call_count, 2)
            self.assertTrue(link.is_alive())
        finally:
            link.stop()


    def test_split_frame_across_long_gap_is_assembled(self):
        # 2026-11-15 随机丢指令漏洞回归：一帧分两次读取、间隔 80ms。
        # 旧代码的 50ms 空闲重置会把缓冲里的半帧整体丢弃（指令蒸发、
        # RDK 保持 raw）；修复后任意间隔都能拼接完成。
        on_mode = Mock()
        link = main.SerialLink('/dev/null', 115200, on_mode=on_mode)
        handle = Mock()
        link._handle = handle
        handle.in_waiting = 5
        handle.read.return_value = self.RAW_CMD[:5]     # 第一片 AA 55 10 01 00
        link._poll()
        time.sleep(0.08)                                # 间隔 > 旧 50ms 阈值
        handle.in_waiting = 3
        handle.read.return_value = self.RAW_CMD[5:]     # 第二片 11 0D 0A
        link._poll()
        on_mode.assert_called_once_with('raw')

    def test_noise_bytes_then_frame_still_parsed(self):
        # 噪声字节（含假帧头）之后的完整命令仍能解析——靠校验和与
        # 逐字节滑动重同步，不依赖空闲重置
        on_mode = Mock()
        link = main.SerialLink('/dev/null', 115200, on_mode=on_mode)
        handle = Mock()
        link._handle = handle
        handle.in_waiting = 20
        handle.read.return_value = b'\x00\xaa\x55\xff' + self.RAW_CMD
        link._poll()
        on_mode.assert_called_once_with('raw')


if __name__ == '__main__':
    unittest.main()
