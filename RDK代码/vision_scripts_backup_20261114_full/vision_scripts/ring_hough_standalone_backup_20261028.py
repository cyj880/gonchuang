#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ring_hough_standalone.py —— 独立圆环识别（经典梯度霍夫，无 ROS2）

用途：
    单文件脚本，仅依赖 opencv-python + numpy。
    用 cv2.HoughCircles 的经典梯度法（HOUGH_GRADIENT）识别场地圆环组
    （粗加工区/暂存区的 1~6 号同心圆环），并显示调试画面。

摄像头与去畸变参数与 logistics_ws 现役链路一致：
    - 打开方式照搬 camera_capture_node.py：CAP_V4L2，MJPG → 640x480 → 30fps
      （set 顺序保持 FOURCC → WIDTH → HEIGHT → FPS，C100 实测可行顺序）
    - 去畸变照搬 camera_undistort_node.py：读 camera_640x480.yaml，
      getOptimalNewCameraMatrix(alpha=0) + initUndistortRectifyMap(CV_16SC2)
      预计算 map 只算一次，每帧仅 remap(INTER_LINEAR)；
      yaml 缺失时回退到内嵌的同数值标定。

识别链路（每帧）：
    最新帧 → 去畸变 → 5x5 高斯 + CLAHE → 搜索区域(全图/ROI) → 梯度霍夫
    → 覆盖率验证 → 同心聚类 → 黑线辅助矫正 → 时域跟踪确认
    → 三环组判定(ID1/2/3) + yaw → 标注显示

关键机制：
    1. 后台线程只保留最新帧，处理不排队；
    2. 去畸变 map 预计算（CV_16SC2）；
    3. 高斯模糊（--blur 3/5/7，b 键运行时 3/5 切换）+ CLAHE(clip=3.0)
       应对局部反光（c 键 A/B 对比，--no-clahe 关闭）；环线在 640x480 下
       仅约 1.6px：核越大视觉越粗、霍夫票越少（blur_node 实测 3x3 保留
       97% 边缘、5x5 起变软、7x7 只剩 22%），默认 5x5 抑噪，求环线细腻
       用 --blur 3；若虚线外圈漏检可运行时收紧 param2；
    4. 霍夫半径窗 [minR, maxR] 收紧是耗时大头；--scale 0.5 半分辨率粗定位；
    5. --roi 确认后锁定局部检测，每 N 帧全图复查（复刻 hough_classic _search）；
    6. 覆盖率验证在搜索分辨率上做（霍夫在哪张图找到的圆就在哪张图验证）；
    7. 时域跟踪 GroupTracker（ROS RingTracker 语义）：最近邻关联 + EMA
       （--smooth 新值权重，默认 0.5 低延迟）+ hits>=8 确认（--confirm）+
       连丢 3 帧命中史清零（--miss-tol，防间歇假组攒数）+ 连丢 8 帧删除
       （--remove-misses）；确认后丢失 <=2 帧仍显示（比 ROS 宽松处，防真组
       单帧丢失闪烁）；
    8. 三环组判定 select_three（ROS ring_geometry 逐行移植）：间距
       152.1px±20%、等距比 >=0.85、共线度 <=0.08，评分最小者胜出；
       左→右 ID1/ID2/ID3；yaw=基线角 atan2(dy,dx)（右/下为正）；
    9. 黑线辅助 LineAssist（移植 stacked_ring_algorithm._line 思路，
       **默认关闭**，--line 开启）：场地事实（2026-10-27 实拍探针确认）：
       靠近地图边界的上边界黑线明显，其余三条边不显——开启后只搜图像
       上侧（--line-side top，可选 bottom / auto 双侧+侧记忆）；
       检出后 (a) yaw 优先取黑线角；(b) 环心垂直投影到理想线矫正
       （--line-corr）；垂距默认自动学习（--line-dist-px 手动覆盖，
       理论 75mm*0.9755≈73.2px，实拍 ≈76px）；(c) 垂距偏差 >25px 的组
       判离群不进跟踪（第三道假组闸门）。

调试画面（cv2.imshow）：
    暗红细圈=原始霍夫圆；暗黄圈=未确认轨迹；灰圈=黑线判离群组；
    绿色粗圈+ID1/2/3=确认三工位（PARTIAL 时暗黄绿 S 编号）；
    洋红粗线=检出黑线；橙色细线=矫正后理想环心线；
    底部状态行 RING FULL/PARTIAL/NONE yaw=xx.xx；
    按键：q/ESC 退出，p 暂停，s 截图，w/e 调 param2 ±5，a/d 调 param1 ±10，
    v 切换覆盖率验证，c 切换 CLAHE，r 切换 ROI。

示例：
    python3 ring_hough_standalone.py
    python3 ring_hough_standalone.py --device /dev/video1 --scale 0.5 --roi
    python3 ring_hough_standalone.py --image test.png        # 离线测试单张图
"""

import argparse
import math
import os
import signal
import sys
import threading
import time
from itertools import combinations

import cv2
import numpy as np

# ---------------------------------------------------------------------------
# 内嵌兜底标定（与 ~/logistics_vision_data/calibration/camera_640x480.yaml
# 即 c100_v7 完全一致，yaml 读取失败时使用）
# ---------------------------------------------------------------------------
FALLBACK_K = np.array(
    [[314.240644445762463, 0.0, 320.43482606113088],
     [0.0, 313.07831940288361, 241.51945297353271],
     [0.0, 0.0, 1.0]], dtype=np.float64)
FALLBACK_D = np.array([0.090434649497838485, -0.12048307090013888,
                       -0.0014616467467961872, 9.6284293522321047e-05,
                       0.035588081010554162], dtype=np.float64)

DEFAULT_CALIB = os.path.expanduser(
    '~/logistics_vision_data/calibration/camera_640x480.yaml')

WINDOW_NAME = 'Ring Hough Standalone'

# 三工位圆环组中心间距参考值（150mm × 1.014 mm/px，见 validate_node.py）
EXPECTED_SPACING_PX = 152.1
# 黑线与环心线的理论垂距：75mm × 0.9755 mm/px（工位标定实测比例尺）
THEORY_LINE_DIST_PX = 73.2


# ---------------------------------------------------------------------------
# 摄像头：V4L2 + MJPG + 640x480@30，后台线程只留最新帧
# ---------------------------------------------------------------------------
class Camera:
    def __init__(self, device, width, height, fps):
        self._open(device, width, height, fps)
        self._lock = threading.Lock()
        self._frame = None            # 最新一帧，被主循环取走即置 None
        self._stop_evt = threading.Event()
        self._fail_count = 0
        self._fail_cycles = 0
        self._thread = threading.Thread(target=self._read_loop, daemon=True)
        self._thread.start()

    def _open(self, device, width, height, fps):
        self.device = device
        self.cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
        if not self.cap.isOpened():
            raise RuntimeError(f'无法打开摄像头 {device}')
        fourcc = cv2.VideoWriter_fourcc(*'MJPG')
        self.cap.set(cv2.CAP_PROP_FOURCC, fourcc)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self.cap.set(cv2.CAP_PROP_FPS, fps)
        # 回读实际生效值（照搬 camera_capture_node.py 的做法）
        self.real_w = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        self.real_h = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        self.real_fps = self.cap.get(cv2.CAP_PROP_FPS)
        fourcc_int = int(self.cap.get(cv2.CAP_PROP_FOURCC))
        self.real_fourcc = ''.join(chr((fourcc_int >> (8 * i)) & 0xFF)
                                   for i in range(4))
        print(f'[camera] {device} 实际生效: {self.real_w}x{self.real_h}@'
              f'{self.real_fps:.0f} fourcc={self.real_fourcc}')
        if (self.real_w, self.real_h) != (width, height):
            print(f'[camera][警告] 实际分辨率与请求的 {width}x{height} 不一致，'
                  f'请以实际分辨率重新标定！')

    def _read_loop(self):
        while not self._stop_evt.is_set():
            ok, frame = self.cap.read()
            if not ok:
                self._fail_count += 1
                if self._fail_count >= 50:      # 连续失败自动重开（自救）
                    print('[camera][警告] 连续读帧失败，尝试重新打开设备')
                    self.cap.release()
                    time.sleep(0.3)
                    try:
                        self._open(self.device, self.real_w, self.real_h,
                                   self.real_fps)
                    except RuntimeError:
                        pass
                    self._fail_cycles += 1
                    self._fail_count = 0
                    if self._fail_cycles >= 3:
                        print('[camera][提示] 多次重开无效，可能是 UVC 状态卡死，'
                              '可执行: sudo modprobe -r uvcvideo && '
                              'sudo modprobe uvcvideo 重载驱动')
                time.sleep(0.02)
                continue
            self._fail_count = 0
            with self._lock:
                self._frame = frame             # 覆盖旧帧，只保最新

    def read(self):
        """取走最新帧；无新帧返回 None（主循环复用上一帧画面即可）。"""
        with self._lock:
            frame, self._frame = self._frame, None
        return frame

    def close(self):
        self._stop_evt.set()
        self._thread.join(timeout=1.0)
        self.cap.release()


# ---------------------------------------------------------------------------
# 去畸变：预计算 map，每帧仅 remap
# ---------------------------------------------------------------------------
class Undistorter:
    def __init__(self, calib_path, size):
        K, D, source = self._load(calib_path)
        self.new_K, _ = cv2.getOptimalNewCameraMatrix(K, D, size, 0, size)
        self.map_x, self.map_y = cv2.initUndistortRectifyMap(
            K, D, None, self.new_K, size, cv2.CV_16SC2)
        self.size = size
        self.source = source
        self._size_warned = False
        print(f'[undistort] 标定来源: {source}')
        print(f'[undistort] K={np.round(K, 2).tolist()} D={np.round(D, 6).tolist()}')

    @staticmethod
    def _load(calib_path):
        if calib_path and os.path.isfile(calib_path):
            try:
                fs = cv2.FileStorage(calib_path, cv2.FILE_STORAGE_READ)
                K = fs.getNode('camera_matrix').mat()
                D = fs.getNode('distortion_coefficients').mat()
                fs.release()
                if K is not None and D is not None and K.shape == (3, 3):
                    return K, D.ravel()[:5], calib_path
            except Exception as exc:            # yaml 损坏等，回退内嵌值
                print(f'[undistort][警告] 读取 {calib_path} 失败: {exc}')
        else:
            print(f'[undistort][警告] 标定文件不存在: {calib_path}，使用内嵌标定')
        return FALLBACK_K.copy(), FALLBACK_D.copy(), '内嵌兜底标定(c100_v7)'

    def remap(self, frame):
        # 帧尺寸与标定尺寸不符时无法使用预计算 map，原样跳过（只警告一次）
        if (frame.shape[1], frame.shape[0]) != self.size:
            if not self._size_warned:
                self._size_warned = True
                print(f'[undistort][警告] 帧 {frame.shape[1]}x{frame.shape[0]} '
                      f'≠ 标定尺寸 {self.size[0]}x{self.size[1]}，跳过去畸变')
            return frame
        return cv2.remap(frame, self.map_x, self.map_y, cv2.INTER_LINEAR)


# ---------------------------------------------------------------------------
# 圆环检测：梯度霍夫 + 覆盖率验证 + 同心聚类
# ---------------------------------------------------------------------------
def circle_coverage(canny, cx, cy, r, sectors=12, samples=30):
    """圆周覆盖率：12 扇区 × 每扇区 30 点采样，命中边缘的扇区占比。
    对应 ring_geometry.circle_groups 的轻量版（0.2ms 量级）。"""
    total = sectors * samples
    angs = np.arange(total, dtype=np.float64) * (2.0 * math.pi / total)
    h, w = canny.shape[:2]
    xs = np.clip(np.round(cx + r * np.cos(angs)).astype(np.int32), 0, w - 1)
    ys = np.clip(np.round(cy + r * np.sin(angs)).astype(np.int32), 0, h - 1)
    hits = canny[ys, xs] > 0
    return hits.reshape(sectors, samples).any(axis=1).mean()


def group_circles(circles, merge_dist=20.0):
    """同心聚类：圆心距 ≤ merge_dist 的圆并为一组（并查集思想的贪心实现）。
    输出 [{'cx','cy','r_inner','r_outer','members'}]，纯内存操作，开销可忽略。"""
    groups = []           # 每项: {'cx','cy','members':[ (x,y,r), ... ]}
    for x, y, r in sorted(circles, key=lambda c: -c[2]):
        placed = False
        for g in groups:
            if math.hypot(x - g['cx'], y - g['cy']) <= merge_dist:
                g['members'].append((x, y, r))
                xs = [m[0] for m in g['members']]
                ys = [m[1] for m in g['members']]
                g['cx'], g['cy'] = float(np.median(xs)), float(np.median(ys))
                placed = True
                break
        if not placed:
            groups.append({'cx': float(x), 'cy': float(y),
                           'members': [(x, y, r)]})
    for g in groups:
        rs = [m[2] for m in g['members']]
        g['r_inner'], g['r_outer'] = float(min(rs)), float(max(rs))
    return groups


def select_three(groups, image_width=640,
                 expected_spacing=EXPECTED_SPACING_PX, spacing_gate=False,
                 tolerance=0.2, min_span_ratio=0.35,
                 max_collinearity=0.08):
    """三工位三元组筛选（cyqm ring_station_detector._select_triple 移植
    + 可选绝对间距门）。

    门（全部通过才入选）：
    - 跨度下限：span >= image_width × min_span_ratio——挤在角落、间距碰巧
      合格的假三元组一票否决；
    - 中点分数：中点在两端连线上的投影分数落在 0.2~0.8——取代旧的
      "等距比 >=0.85" 硬门（斜视时近端间距天然小于远端，硬门会误杀真实
      三元组；透视不平衡改由评分中的间距平衡项惩罚）；
    - 共线度 <= max_collinearity；
    - 绝对间距门（可选，--spacing-gate 开启才生效）：
      |avg − 期望间距| <= 期望间距 × tolerance。
    评分（越小越好，cyqm 公式）：
      8.0×共线度 + 0.35×|log(间距比)| + 2.0×环径惩罚 − 0.015×同心支持数；
      环径惩罚：每环 2×r_outer 与该环间距之比超出 [0.25, 1.10] 的部分求和，
      把环尺寸与工位间距绑定的物理一致性检查。
    返回 (按 x 排序三元组 | None, (first, second, col, 基线角deg) | None,
    其余组)。"""
    best = None
    for triple in combinations(groups, 3):
        pts = list(triple)
        # 最远点对作为端点（cyqm 原逻辑：三点斜排时最远对未必是 x 两端）
        span, ia, ic = max(
            (math.hypot(pts[a]['cx'] - pts[b]['cx'],
                        pts[a]['cy'] - pts[b]['cy']), a, b)
            for a in range(3) for b in range(a))
        if span < image_width * min_span_ratio or span <= 0:
            continue
        ib = 3 - ia - ic
        p1, pm, p2 = pts[ia], pts[ib], pts[ic]
        lx, ly = p2['cx'] - p1['cx'], p2['cy'] - p1['cy']
        ox, oy = pm['cx'] - p1['cx'], pm['cy'] - p1['cy']
        fraction = (ox * lx + oy * ly) / (span * span)
        if not 0.2 < fraction < 0.8:
            continue
        col = abs(lx * oy - ly * ox) / (span * span)
        if col > max_collinearity:
            continue
        ordered = sorted(pts, key=lambda g: g['cx'])
        first = math.hypot(ordered[1]['cx'] - ordered[0]['cx'],
                           ordered[1]['cy'] - ordered[0]['cy'])
        second = math.hypot(ordered[2]['cx'] - ordered[1]['cx'],
                            ordered[2]['cy'] - ordered[1]['cy'])
        avg = (first + second) / 2
        if spacing_gate and abs(avg - expected_spacing) > \
                expected_spacing * tolerance:
            continue
        diameter_penalty = 0.0
        for item, spacing in zip(ordered, (first, avg, second)):
            ratio = (2.0 * item['r_outer']) / spacing if spacing > 0 else 99.0
            if ratio < 0.25:
                diameter_penalty += 0.25 - ratio
            elif ratio > 1.10:
                diameter_penalty += ratio - 1.10
        support = sum(len(g.get('members', [])) or 1 for g in ordered)
        score = (8.0 * col + 0.35 * abs(math.log(first / second)) +
                 2.0 * diameter_penalty - 0.015 * support)
        if best is None or score < best[0]:
            best = (score, ordered, (first, second, col, math.degrees(
                math.atan2(ordered[2]['cy'] - ordered[0]['cy'],
                           ordered[2]['cx'] - ordered[0]['cx']))))
    if best is None:
        return None, None, list(groups)
    return best[1], best[2], [g for g in groups if g not in best[1]]


def baseline_yaw(sites):
    """工位基线角（ROS ring_serial_protocol.ring_yaw 移植）：
    最左→最右站点连线的 atan2 角度，图像系右/下为正，(-90,90) 度。"""
    pts = sorted((float(g['cx']), float(g['cy'])) for g in sites)
    if len(pts) < 2 or pts[0] == pts[-1]:
        return None
    return math.degrees(math.atan2(pts[-1][1] - pts[0][1],
                                   pts[-1][0] - pts[0][0]))


class GroupTracker:
    """时域稳定层（ROS RingTracker 语义移植，防间歇假组）。

    - 最近邻关联（assoc_dist 内贪心配对）+ 圆心平滑（--smooth-mode 选择）：
        ema：smoothed = w*new + (1-w)*smoothed（w=--smooth，1=不平滑）；
        median：最近 --median-window 帧原始位置的分量中位数（压尖峰最强，
                但运动时滞后大，适合纯静态观察）；
        alphabeta：常速模型 α-β 滤波（默认，2026-10-28 实测帕累托优于
                EMA w=0.5：静止抖动 −41%、运动偏差 −26%；匀速跟随好）；
    - 半径通道一律弱滤波（EMA w=0.5，突变 >8px 吸附）——半径波动不影响圆心；
    - 中心跳变 >jump_reset 直接吸附并复位滤波器状态；
    - 新轨迹累计 hits >= confirm_hits（ROS 默认 8）才确认；
    - 连丢 > miss_tol 帧命中史清零（防间歇假组攒数，ROS 同款机制）；
    - 连丢 > remove_misses 帧删除轨迹；
    - 已确认轨迹丢失 <= display_hold 帧内仍输出（比 ROS 的 misses==0 宽松，
      防真组单帧丢失造成画面闪烁）；
    - 输出按中心 x 从左到右排序；另维护 yaw 的 EMA（跳变 >15° 吸附）。
    """

    def __init__(self, assoc_dist=60.0, confirm_hits=8, miss_tol=3,
                 remove_misses=8, display_hold=2, smooth_w=0.5,
                 jump_reset=20.0, smooth_mode='alphabeta',
                 median_window=7, alpha=0.3, beta=0.15,
                 kalman_q=0.15, kalman_r=4.8):
        self.assoc_dist = assoc_dist
        self.confirm_hits = confirm_hits
        self.miss_tol = miss_tol
        self.remove_misses = remove_misses
        self.display_hold = display_hold
        self.w = smooth_w
        self.jump_reset = jump_reset
        self.smooth_mode = smooth_mode
        self.median_window = median_window
        self.alpha = alpha
        self.beta = beta
        self.kalman_q = kalman_q
        self.kalman_r = kalman_r
        self.tracks = []          # {'cx','cy','r_inner','r_outer','hits','missed'}
        self.last_confirmed = []  # 上帧确认轨迹（黑线锚点用）
        self.smoothed_yaw = None

    # 完整卡尔曼常量矩阵（CV 模型，dt=1 帧）
    _KF_F = np.array([[1, 0, 1, 0], [0, 1, 0, 1],
                      [0, 0, 1, 0], [0, 0, 0, 1]], dtype=float)
    _KF_H = np.array([[1.0, 0, 0, 0], [0, 1.0, 0, 0]])
    _KF_QBLOCK = np.array([[0.25, 0, 0.5, 0], [0, 0.25, 0, 0.5],
                           [0.5, 0, 1.0, 0], [0, 0.5, 0, 1.0]], dtype=float)

    def _kalman_predict(self, x, P):
        Q = self.kalman_q * self._KF_QBLOCK
        x = self._KF_F @ x
        P = self._KF_F @ P @ self._KF_F.T + Q
        return x, P

    def _kalman_update(self, x, P, z):
        H = self._KF_H
        y = np.array(z, dtype=float) - H @ x
        S = H @ P @ H.T + self.kalman_r * np.eye(2)
        K = P @ H.T @ np.linalg.inv(S)
        x = x + K @ y
        P = (np.eye(4) - K @ H) @ P
        return x, P

    def _smooth_center(self, t, g, jumped):
        """按 smooth_mode 更新轨迹圆心（jumped=吸附时复位滤波状态）。"""
        gx, gy = g['cx'], g['cy']
        if jumped:
            t['cx'], t['cy'] = gx, gy
            t['_hist'] = [(gx, gy)]
            t['_fx'] = np.array([gx, gy], dtype=float)
            t['_fv'] = np.zeros(2)
            t['_kx'] = np.array([gx, gy, 0.0, 0.0])
            t['_kP'] = np.eye(4) * 10.0
            return
        if self.smooth_mode == 'median':
            hist = t.get('_hist') or [(gx, gy)]
            hist.append((gx, gy))
            t['_hist'] = hist[-self.median_window:]
            arr = np.asarray(t['_hist'], dtype=float)
            t['cx'] = float(np.median(arr[:, 0]))
            t['cy'] = float(np.median(arr[:, 1]))
        elif self.smooth_mode == 'alphabeta':
            fx = t.get('_fx')
            if fx is None:
                fx = np.array([gx, gy], dtype=float)
                fv = np.zeros(2)
            else:
                fv = t.get('_fv', np.zeros(2))
                resid = np.array([gx, gy], dtype=float) - (fx + fv)
                fx = fx + fv + self.alpha * resid
                fv = fv + self.beta * resid
            t['_fx'], t['_fv'] = fx, fv
            t['cx'], t['cy'] = float(fx[0]), float(fx[1])
        elif self.smooth_mode == 'kalman':
            kx = t.get('_kx')
            if kx is None:
                kx = np.array([gx, gy, 0.0, 0.0])
                kP = np.eye(4) * 10.0
            else:
                kx, kP = self._kalman_predict(kx, t['_kP'])
                kx, kP = self._kalman_update(kx, kP, (gx, gy))
            t['_kx'], t['_kP'] = kx, kP
            t['cx'], t['cy'] = float(kx[0]), float(kx[1])
        else:                                   # ema（原有行为）
            w = self.w
            t['cx'] = w * gx + (1 - w) * t['cx']
            t['cy'] = w * gy + (1 - w) * t['cy']

    def update(self, groups):
        # 1) 贪心最近邻关联
        pairs = []
        for ti, t in enumerate(self.tracks):
            for gi, g in enumerate(groups):
                d = math.hypot(g['cx'] - t['cx'], g['cy'] - t['cy'])
                if d <= self.assoc_dist:
                    pairs.append((d, ti, gi))
        pairs.sort()
        used_t, used_g = set(), set()
        for d, ti, gi in pairs:
            if ti in used_t or gi in used_g:
                continue
            used_t.add(ti)
            used_g.add(gi)
            t, g = self.tracks[ti], groups[gi]
            if d > self.jump_reset:                     # 大幅移动：吸附+复位滤波
                self._smooth_center(t, g, jumped=True)
                t['r_inner'], t['r_outer'] = g['r_inner'], g['r_outer']
            else:
                self._smooth_center(t, g, jumped=False)
                # 半径弱滤波：EMA w=0.5；突变 >8px（NMS 内外圈胜者翻转）直接吸附
                if abs(g['r_inner'] - t['r_inner']) > 8:
                    t['r_inner'] = g['r_inner']
                else:
                    t['r_inner'] = 0.5 * g['r_inner'] + 0.5 * t['r_inner']
                if abs(g['r_outer'] - t['r_outer']) > 8:
                    t['r_outer'] = g['r_outer']
                else:
                    t['r_outer'] = 0.5 * g['r_outer'] + 0.5 * t['r_outer']
            t['hits'] += 1
            t['missed'] = 0
        # 2) 未关联轨迹：计丢失；连丢超容忍帧数命中史清零（防假组攒数）
        for ti, t in enumerate(self.tracks):
            if ti not in used_t:
                t['missed'] += 1
                if t['missed'] > self.miss_tol:
                    t['hits'] = 0
                if self.smooth_mode == 'kalman' and '_kx' in t:
                    # 丢帧仅预测：按速度外推保持位置连续；P 增大→恢复后增益自动升高
                    t['_kx'], t['_kP'] = self._kalman_predict(t['_kx'], t['_kP'])
                    t['cx'], t['cy'] = float(t['_kx'][0]), float(t['_kx'][1])
        # 3) 删除超时轨迹
        self.tracks = [t for t in self.tracks
                       if t['missed'] <= self.remove_misses]
        # 4) 未配对的检出组建新轨迹
        for gi, g in enumerate(groups):
            if gi not in used_g:
                self.tracks.append({'cx': g['cx'], 'cy': g['cy'],
                                    'r_inner': g['r_inner'],
                                    'r_outer': g['r_outer'],
                                    'hits': 1, 'missed': 0})
        # 5) 确认/候选划分（确认轨迹允许 display_hold 帧内的短暂丢失）
        confirmed = [t for t in self.tracks
                     if t['hits'] >= self.confirm_hits
                     and t['missed'] <= self.display_hold]
        confirmed_ids = {id(t) for t in confirmed}
        candidates = [t for t in self.tracks if id(t) not in confirmed_ids]
        confirmed.sort(key=lambda t: t['cx'])
        candidates.sort(key=lambda t: t['cx'])
        self.last_confirmed = list(confirmed)
        return confirmed, candidates

    def smooth_yaw(self, angle):
        """yaw 的 EMA；跳变 >15° 或清空（PARTIAL/NONE）时直接置值/复位。"""
        if angle is None:
            self.smoothed_yaw = None
            return None
        if self.smoothed_yaw is None or abs(angle - self.smoothed_yaw) > 15:
            self.smoothed_yaw = angle
        else:
            self.smoothed_yaw += self.w * (angle - self.smoothed_yaw)
        return round(self.smoothed_yaw, 2)


class LineAssist:
    """黑线辅助矫正（移植 stacked_ring_algorithm._line 思路）。

    场地边界黑线与三环组中心连线平行、垂距固定。每帧：
    1. 以上帧确认轨迹为锚点估基线角 θ0，在两侧 ±d 处各开 ±band 斜带掩膜；
    2. 带内 Huber fitLine 亚像素拟合，不达标回退 HoughLinesP（评分=长度−3×
       位置偏差），两侧取优；
    3. 检出黑线后：yaw 优先取线角；各组垂直投影到与黑线平行、垂距 target
       的理想线上（corr 强度混合）；垂距偏差 > reject_dev 的组判离群。
    垂距 target：--line-dist-px 手动指定，否则首次"有效线+锚点"时取锚点
    垂距中位数自动学习并冻结（连续丢失过久则重新学习）。"""

    def __init__(self, enabled=False, dist_px=None, band=18, corr=1.0,
                 min_span=120.0, angle_tol=10.0, reject_dev=25.0,
                 side='auto'):
        self.enabled = enabled
        self.dist_px = dist_px          # None = 自动学习
        self.band = band
        self.corr = corr
        self.min_span = min_span
        self.angle_tol = angle_tol
        self.reject_dev = reject_dev
        # 搜索侧：'top' 只搜图像上侧(-1)、'bottom' 只搜下侧(+1)、'auto' 双侧
        # 实车场地事实：靠近地图边界的上边界黑线明显，其余三条边不显——
        # 固定单侧可杜绝上下两条平行边互相翻转（A2）。
        self.side = side
        self._last_side = None          # auto 模式的侧记忆
        self.learned = None             # 自动学习到的垂距（绝对值 px）
        self._miss_streak = 0
        self._had_anchors = False       # 是否有过锚点（区分开机首冻与真饥饿）
        self._starved = False           # 锚点饿死过；恢复时须重新冻结垂距（A1）

    @staticmethod
    def _unit(theta_deg):
        r = math.radians(theta_deg)
        return (math.cos(r), math.sin(r)), (-math.sin(r), math.cos(r))

    @staticmethod
    def _fit_line_robust(pts):
        """两趟截断最小二乘拟合（DIST_HUBER 在 ARM 上实测 26ms/千点，
        改用 L2 拟合 + 剔除 >2.5px 残差点重拟合，鲁棒性接近、快两个量级）。
        返回 (vx, vy, px, py)，方向规范为 vx>0。"""
        vx, vy, px, py = (float(v) for v in cv2.fitLine(
            pts, cv2.DIST_L2, 0, 0.01, 0.01).flatten())
        if vx < 0:
            vx, vy = -vx, -vy
        d = np.abs((pts[:, 0] - px) * vy - (pts[:, 1] - py) * vx)
        keep = d <= 2.5
        if 40 <= keep.sum() < len(pts):
            vx, vy, px, py = (float(v) for v in cv2.fitLine(
                pts[keep], cv2.DIST_L2, 0, 0.01, 0.01).flatten())
            if vx < 0:
                vx, vy = -vx, -vy
        return vx, vy, px, py

    def _fit_in_band(self, canny_sub, x_off, y_off, theta0, target_pt):
        """在已裁剪的斜带边缘图内拟合直线（坐标已加回全图偏移）。
        返回 dict(score, angle, point, span, segment) 或 None。"""
        ys, xs = np.nonzero(canny_sub)
        if not len(xs):
            return None
        xs = xs.astype(np.float64) + x_off
        ys = ys.astype(np.float64) + y_off
        result = None
        if len(xs) >= 40 and np.ptp(xs) >= self.min_span * 0.6:
            pts = np.column_stack((xs, ys)).astype(np.float32)
            vx, vy, px, py = self._fit_line_robust(pts)
            if vx > 1e-6:
                angle = math.degrees(math.atan2(vy, vx))
                # 位置校验：拟合线在带中心 x 处的 y 距目标线不超过 band+10
                y_at = py + vy / vx * (target_pt[0] - px)
                dev = abs(y_at - target_pt[1])
                if abs(angle - theta0) <= self.angle_tol and dev <= self.band + 10:
                    slope = math.tan(math.radians(angle))
                    x0, x1 = int(xs.min()), int(xs.max())
                    seg = ((x0, int(round(y_at + slope * (x0 - target_pt[0])))),
                           (x1, int(round(y_at + slope * (x1 - target_pt[0])))))
                    result = dict(score=float(np.ptp(xs)) - 3.0 * dev,
                                  angle=angle, point=(float(target_pt[0]),
                                                      float(y_at)),
                                  span=float(np.ptp(xs)), segment=seg)
        if result is None:                      # HoughLinesP 兜底
            found = cv2.HoughLinesP(canny_sub, 1, np.pi / 180, 40,
                                    minLineLength=int(self.min_span),
                                    maxLineGap=25)
            best = None
            for item in found[:, 0] if found is not None else []:
                x1, y1, x2, y2 = map(int, item)
                x1, y1, x2, y2 = x1 + x_off, y1 + y_off, x2 + x_off, y2 + y_off
                if x2 < x1:
                    x1, y1, x2, y2 = x2, y2, x1, y1
                angle = math.degrees(math.atan2(y2 - y1, x2 - x1))
                if abs(angle - theta0) > self.angle_tol + 5:
                    continue
                length = math.hypot(x2 - x1, y2 - y1)
                (tx, ty), (nx, ny) = self._unit(theta0)
                mid = ((x1 + x2) / 2, (y1 + y2) / 2)
                dev = abs((mid[0] - target_pt[0]) * nx +
                          (mid[1] - target_pt[1]) * ny)
                score = length - 3.0 * dev
                if best is None or score > best['score']:
                    best = dict(score=float(score), angle=float(angle),
                                point=mid, span=float(length),
                                segment=((x1, y1), (x2, y2)))
            result = best
        return result

    def process(self, groups, anchors, enhanced, canny_lo, canny_hi):
        """返回 (矫正后组, 离群组, 黑线段|None, 理想线段|None, info)。"""
        if not self.enabled:
            return groups, [], None, None, {'line': 'off'}
        if len(anchors) < 2 or not groups:
            # 曾有过锚点才算"饿死"（开机首冻不算），恢复时重新冻结垂距
            if self._had_anchors:
                self._starved = True
            return groups, [], None, None, {'line': 'wait'}
        # 锚点估计基线角 θ0
        pts = np.array([(t['cx'], t['cy']) for t in anchors], np.float32)
        vx, vy, px, py = (float(v) for v in cv2.fitLine(
            pts, cv2.DIST_L2, 0, 0.01, 0.01).flatten())
        if vx < 0:
            vx, vy = -vx, -vy
        theta0 = math.degrees(math.atan2(vy, vx))
        center = (float(np.median(pts[:, 0])), float(np.median(pts[:, 1])))
        d_guess = self.dist_px if self.dist_px is not None else \
            (self.learned if self.learned is not None else THEORY_LINE_DIST_PX)
        # 只在斜带包围盒内做 Canny + 掩膜（省 9 倍像素）：x 取锚点跨度 ±80，
        # y 取中心线 y(x) ± (d_guess + band + 5)
        h, w = enhanced.shape[:2]
        slope0 = math.tan(math.radians(theta0))
        x_lo = int(max(0, pts[:, 0].min() - 80))
        x_hi = int(min(w, pts[:, 0].max() + 80))
        y_mid_lo = center[1] + slope0 * (x_lo - center[0])
        y_mid_hi = center[1] + slope0 * (x_hi - center[0])
        pad = d_guess + self.band + 5
        y_lo = int(max(0, min(y_mid_lo, y_mid_hi) - pad))
        y_hi = int(min(h, max(y_mid_lo, y_mid_hi) + pad))
        if x_hi - x_lo < 40 or y_hi - y_lo < 2 * 2:
            self._miss_streak += 1
            return groups, [], None, None, {'line': 'miss'}
        sub = cv2.Canny(enhanced[y_lo:y_hi, x_lo:x_hi], canny_lo, canny_hi)
        # 锚线（过 center、角 θ0）的带符号垂距图，float32，两侧共用
        (tx0, ty0), (nx0, ny0) = self._unit(theta0)
        cols = np.arange(x_lo, x_hi, dtype=np.float32)[None, :]
        rows = np.arange(y_lo, y_hi, dtype=np.float32)[:, None]
        perp0 = (cols - center[0]) * nx0 + (rows - center[1]) * ny0
        # 搜索侧：固定单侧（top/bottom）或 auto 双侧+侧记忆（A2 修复：
        # 上下两条平行边得分接近时，优先保持上帧同侧，防 s_target 符号翻转）
        if self.side == 'top':
            sides = (-1.0,)
        elif self.side == 'bottom':
            sides = (+1.0,)
        else:
            sides = (+1.0, -1.0)
        results = []
        for side in sides:
            filtered = np.where(np.abs(perp0 - side * d_guess) <= self.band,
                                sub, 0).astype(np.uint8)
            target_pt = (center[0] + side * d_guess * nx0,
                         center[1] + side * d_guess * ny0)
            cand = self._fit_in_band(filtered, x_lo, y_lo, theta0, target_pt)
            if cand is not None:
                cand['side'] = side
                results.append(cand)
        if self.side == 'auto':
            if len(results) == 2 and self._last_side is not None:
                prev = [c for c in results if c['side'] == self._last_side]
                other = [c for c in results if c['side'] != self._last_side]
                # 同侧分数不低于对侧一半就保持同侧
                if prev and other and prev[0]['score'] >= other[0]['score'] * 0.5:
                    best = prev[0]
                else:
                    best = max(results, key=lambda c: c['score'])
            elif results:
                best = max(results, key=lambda c: c['score'])
            else:
                best = None
        else:
            best = results[0] if results else None
        if best is not None:
            self._last_side = best['side']
        if best is None:
            self._miss_streak += 1
            if self._miss_streak > 45 and self.dist_px is None:
                self.learned = None             # 丢失过久，允许重新学习
            return groups, [], None, None, {'line': 'miss'}
        self._miss_streak = 0
        # 锚点到黑线的带符号垂距 → 目标垂距
        (tx, ty), (nx, ny) = self._unit(best['angle'])
        s_anchor = [float((t['cx'] - best['point'][0]) * nx +
                          (t['cy'] - best['point'][1]) * ny) for t in anchors]
        s_bar = float(np.median(s_anchor))
        if self.dist_px is not None:
            s_target = math.copysign(self.dist_px, s_bar)
        else:
            # A1 修复：learned 从未学习（开机首次）或刚从饥饿中恢复时，
            # 用新鲜锚点（饿死期间未矫正的原始位置）重新冻结垂距，
            # 打破"冻结值过时 → 全组被拒 → 饿死 → 重确认 → 再全拒"的死循环
            if self.learned is None or self._starved:
                self.learned = abs(s_bar)
                reason = '饥饿恢复，重新冻结' if self._starved else '垂距自动学习'
                print(f'[line] {reason}: {self.learned:.1f}px')
            s_target = math.copysign(self.learned, s_bar)
        self._starved = False
        self._had_anchors = True
        # 矫正各组：沿线分量保留、垂直分量吸附到理想线；离群组剔除
        corrected, rejected = [], []
        for g in groups:
            s_i = (g['cx'] - best['point'][0]) * nx + \
                  (g['cy'] - best['point'][1]) * ny
            if abs(s_i - s_target) > self.reject_dev:
                rejected.append(g)
                continue
            along = (g['cx'] - best['point'][0]) * tx + \
                    (g['cy'] - best['point'][1]) * ty
            gx = best['point'][0] + along * tx + s_target * nx
            gy = best['point'][1] + along * ty + s_target * ny
            corrected.append({**g, 'cx': self.corr * gx + (1 - self.corr) * g['cx'],
                              'cy': self.corr * gy + (1 - self.corr) * g['cy']})
        # 画线段：黑线 + 理想环心线（过锚点中心、平行黑线）
        seg_len = 300.0
        cx0 = center[0] - seg_len / 2
        ideal = ((int(round(cx0)), int(round(best['point'][1] + s_target * ny +
                                             math.tan(math.radians(
                                                 best['angle'])) * (cx0 - center[0])))),
                 (int(round(cx0 + seg_len)), int(round(
                     best['point'][1] + s_target * ny +
                     math.tan(math.radians(best['angle'])) *
                     (cx0 + seg_len - center[0])))))
        info = {'line': 'ok', 'angle': round(best['angle'], 2),
                'd': round(abs(s_bar), 1), 'span': round(best['span'], 0),
                'side': 'top' if best['side'] < 0 else 'bottom'}
        return corrected, rejected, best['segment'], ideal, info


class RingDetector:
    """封装霍夫参数与搜索区域（全图 / ROI / 缩放）状态。"""

    def __init__(self, args):
        self.args = args
        self.last_groups = []         # 上帧确认的环组，用于 ROI 锁定
        self.roi_frame_count = 0

    def detect(self, gray, blur):
        """返回 (原始圆数组 Nx3, 验证后圆数组, 环组列表, 搜索区域说明)。"""
        a = self.args
        # ---- 选择搜索区域 ----
        search = blur
        off_x, off_y = 0, 0
        scale = a.scale
        region = 'full'
        if a.roi and self.last_groups:
            self.roi_frame_count += 1
            # ROI：上帧全部确认环组的外接包围盒，外扩 margin
            margin = a.max_r + 32
            xs = [g['cx'] for g in self.last_groups]
            ys = [g['cy'] for g in self.last_groups]
            rs = [g['r_outer'] for g in self.last_groups]
            x0 = int(max(0, min(xs) - max(rs) - margin))
            y0 = int(max(0, min(ys) - max(rs) - margin))
            x1 = int(min(gray.shape[1], max(xs) + max(rs) + margin))
            y1 = int(min(gray.shape[0], max(ys) + max(rs) + margin))
            if x1 - x0 > 2 * a.min_r and y1 - y0 > 2 * a.min_r:
                search = blur[y0:y1, x0:x1]
                off_x, off_y = x0, y0
                region = f'roi[{x0},{y0},{x1 - x0}x{y1 - y0}]'
                if self.roi_frame_count % a.roi_full_scan_interval == 0:
                    region += '+periodic-full'
                    search = blur
                    off_x = off_y = 0
        else:
            self.roi_frame_count = 0

        # ---- 可选缩放（只影响霍夫这一步）----
        hough_min_dist = a.min_dist
        hough_r = (a.min_r, a.max_r)
        if scale != 1.0:
            if search is not blur:
                search = cv2.resize(search, None, fx=scale, fy=scale,
                                    interpolation=cv2.INTER_AREA)
            else:
                search = cv2.resize(blur, None, fx=scale, fy=scale,
                                    interpolation=cv2.INTER_AREA)
            # 缩放后距离/半径窗同步缩小，还原时坐标 ÷ scale
            hough_min_dist = max(3.0, a.min_dist * scale)
            hough_r = (max(3, int(a.min_r * scale)),
                       max(4, int(a.max_r * scale)))

        # ---- 梯度霍夫（经典 HOUGH_GRADIENT）----
        t0 = time.perf_counter()
        raw = cv2.HoughCircles(
            search, cv2.HOUGH_GRADIENT, dp=a.dp, minDist=hough_min_dist,
            param1=a.param1, param2=a.param2,
            minRadius=hough_r[0], maxRadius=hough_r[1])
        t_hough = (time.perf_counter() - t0) * 1000.0

        # ---- 覆盖率验证：在搜索分辨率上做（霍夫在哪张图上找到的圆，
        #      就在哪张图上验证），缩放/ROI 时 Canny 也只算对应区域 ----
        t0 = time.perf_counter()
        raw_circles, verified = [], []
        if raw is not None:
            canny = cv2.Canny(search, a.param1 * 0.5, a.param1)
            for x, y, r in raw[0]:          # 此时 x,y,r 为搜索坐标
                # 还原到全图坐标
                fx, fy, fr = x / scale + off_x, y / scale + off_y, r / scale
                raw_circles.append((fx, fy, fr))
                if a.no_verify or \
                        circle_coverage(canny, x, y, r) >= a.coverage:
                    verified.append((fx, fy, fr))
        t_verify = (time.perf_counter() - t0) * 1000.0

        groups = group_circles(verified, a.merge_dist)
        return np.array(raw_circles, dtype=np.float32).reshape(-1, 3), \
            verified, groups, region, t_hough, t_verify


# ---------------------------------------------------------------------------
# 画面标注与 HUD
# ---------------------------------------------------------------------------
def annotate(canvas, raw, candidates, line_rejected, sites, status, yaw,
             line_seg, ideal_seg, region, stats, args, undist_src):
    h, w = canvas.shape[:2]
    # 原始霍夫圆：暗红细圈
    for x, y, r in raw:
        cv2.circle(canvas, (int(x), int(y)), int(r), (0, 0, 160), 1)
    # 黑线判离群组：灰圈
    for g in line_rejected:
        cv2.circle(canvas, (int(g['cx']), int(g['cy'])),
                   int(g['r_inner']), (128, 128, 128), 1)
    # 未确认轨迹：暗黄圈
    for g in candidates:
        cv2.circle(canvas, (int(g['cx']), int(g['cy'])),
                   int(g['r_inner']), (0, 170, 170), 1)
    # 检出黑线（洋红粗）与理想环心线（橙细）
    if line_seg is not None:
        cv2.line(canvas, line_seg[0], line_seg[1], (255, 0, 255), 2)
    if ideal_seg is not None:
        cv2.line(canvas, ideal_seg[0], ideal_seg[1], (0, 140, 255), 1)
    # 输出工位：FULL 时绿色粗圈 + ID1/2/3，PARTIAL 时暗黄绿 S 编号
    for i, g in enumerate(sites):
        color = (0, 255, 0) if status == 'FULL' else (0, 220, 220)
        cv2.circle(canvas, (int(g['cx']), int(g['cy'])),
                   int(g['r_inner']), color, 2)
        cv2.circle(canvas, (int(g['cx']), int(g['cy'])),
                   int(g['r_outer']), (255, 255, 0), 1)
        cv2.drawMarker(canvas, (int(g['cx']), int(g['cy'])), (255, 255, 0),
                       cv2.MARKER_CROSS, 12, 2)
        label = f'ID{i + 1}' if status == 'FULL' else f'S{i + 1}'
        label_org = (int(g['cx']) - 20, int(g['cy'] - g['r_outer']) - 6)
        cv2.putText(canvas, f'{label} ({int(g["cx"])},{int(g["cy"])})',
                    label_org, cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1)
    # 工位间距线
    for i in range(1, len(sites)):
        p0 = (int(sites[i - 1]['cx']), int(sites[i - 1]['cy']))
        p1 = (int(sites[i]['cx']), int(sites[i]['cy']))
        cv2.line(canvas, p0, p1, (255, 0, 255), 1)
        dist = math.hypot(p1[0] - p0[0], p1[1] - p0[1])
        cv2.putText(canvas, f'{dist:.0f}px', ((p0[0] + p1[0]) // 2 + 4,
                                              (p0[1] + p1[1]) // 2 - 4),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 0, 255), 1)

    # HUD
    a = args
    clahe_txt = 'off' if stats.get('clahe') is None else f'clip={stats["clahe"]}'
    li = stats.get('line_info', {})
    if li.get('line') == 'ok':
        line_txt = f"line=ok th={li['angle']:.1f} d={li['d']:.0f}"
    else:
        line_txt = f"line={li.get('line', 'off')}"
    hud = [
        f'FPS {stats["fps"]:.1f} | undist {stats["undist"]:.1f}ms '
        f'blur+clahe {stats["blur"]:.1f}ms hough {stats["hough"]:.1f}ms '
        f'verify {stats["verify"]:.1f}ms line {stats["line"]:.1f}ms '
        f'total {stats["total"]:.1f}ms',
        f'HOUGH_GRADIENT dp={a.dp} minDist={a.min_dist} p1={a.param1} '
        f'p2={a.param2} r=[{a.min_r},{a.max_r}] scale={a.scale} '
        f'blur={stats["blur_k"]}x{stats["blur_k"]} {clahe_txt} region={region}',
        f'raw={len(raw)} cand={len(candidates)} '
        f'rejline={len(line_rejected)} conf={len(sites)} {line_txt} '
        f'| calib={undist_src} | {w}x{h}',
        # cv2.putText 仅支持 ASCII，中文会画成乱码，HUD 一律用英文
        'q:quit p:pause s:snapshot w/e:p2-/+5 a/d:p1-/+10 v:verify '
        'c:CLAHE b:blur3/5 r:ROI',
    ]
    for i, line in enumerate(hud):
        y = 20 + i * 20
        cv2.putText(canvas, line, (8, y), cv2.FONT_HERSHEY_SIMPLEX, 0.45,
                    (0, 0, 0), 3, cv2.LINE_AA)
        cv2.putText(canvas, line, (8, y), cv2.FONT_HERSHEY_SIMPLEX, 0.45,
                    (0, 255, 255), 1, cv2.LINE_AA)
    # 底部状态行（仿 ROS annotate_ring）
    status_color = {'FULL': (0, 255, 0), 'PARTIAL': (0, 165, 255),
                    'NONE': (0, 0, 255)}[status]
    yaw_txt = f'{yaw:.2f}' if yaw is not None else '--'
    status_line = f'RING {status} yaw={yaw_txt}deg'
    cv2.putText(canvas, status_line, (8, h - 12), cv2.FONT_HERSHEY_SIMPLEX,
                0.6, (0, 0, 0), 4, cv2.LINE_AA)
    cv2.putText(canvas, status_line, (8, h - 12), cv2.FONT_HERSHEY_SIMPLEX,
                0.6, status_color, 2, cv2.LINE_AA)
    return canvas


# ---------------------------------------------------------------------------
# 主循环
# ---------------------------------------------------------------------------
def run_camera(args):
    cam = Camera(args.device, args.width, args.height, args.fps)
    und = Undistorter(args.calib, (cam.real_w, cam.real_h))
    det = RingDetector(args)
    # CLAHE 实例只创建一次（clipLimit 越大对局部对比度拉伸越猛）
    clahe = None if args.no_clahe else cv2.createCLAHE(
        clipLimit=args.clip, tileGridSize=(8, 8))
    tracker = GroupTracker(assoc_dist=args.assoc, confirm_hits=args.confirm,
                           miss_tol=args.miss_tol,
                           remove_misses=args.remove_misses,
                           display_hold=args.hold, smooth_w=args.smooth,
                           smooth_mode=args.smooth_mode,
                           median_window=args.median_window,
                           alpha=args.alpha, beta=args.beta,
                           kalman_q=args.kalman_q, kalman_r=args.kalman_r)
    line_assist = LineAssist(enabled=args.line,
                             dist_px=args.line_dist_px, band=args.line_band,
                             corr=args.line_corr, side=args.line_side)
    # 高斯核：3=保边（环线视觉最细、霍夫票数最足），5=抑噪；b 键运行时切换
    blur_k = args.blur
    blur_sigma = {3: 1.0, 5: 1.2, 7: 2.0}[blur_k] \
        if args.blur_sigma is None else args.blur_sigma

    cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_AUTOSIZE)
    print('[hint] 调试窗口已打开；对准圆环区域后按 w/e、a/d 可实时调参。')

    fps, last_t = 0.0, time.perf_counter()
    perf_win_t, perf_frames, perf_total = time.perf_counter(), 0, 0.0
    paused = False
    last_canvas = None
    t_undist = 0.0
    try:
        while True:
            frame = cam.read()
            if not paused and frame is not None:
                # ---- 去畸变（预计算 map，仅 remap）----
                t0 = time.perf_counter()
                undist = und.remap(frame)
                t_undist = (time.perf_counter() - t0) * 1000.0

                # ---- 灰度 + 高斯 + CLAHE（应对局部反光/曝光不均）----
                t0 = time.perf_counter()
                gray = cv2.cvtColor(undist, cv2.COLOR_BGR2GRAY)
                blur = cv2.GaussianBlur(gray, (blur_k, blur_k), blur_sigma)
                if clahe is not None:
                    blur = clahe.apply(blur)    # 后续霍夫/验证/画布共用此图
                t_blur = (time.perf_counter() - t0) * 1000.0

                # ---- 梯度霍夫 + 覆盖率验证 + 同心聚类 ----
                raw, verified, groups, region, t_hough, t_verify = \
                    det.detect(gray, blur)

                # ---- 黑线辅助：矫正 + 离群剔除（先于跟踪，单条平滑路径）----
                t0 = time.perf_counter()
                corrected, line_rejected, line_seg, ideal_seg, line_info = \
                    line_assist.process(groups, tracker.last_confirmed,
                                        blur, args.param1 * 0.5, args.param1)
                t_line = (time.perf_counter() - t0) * 1000.0

                # ---- 时域稳定层（ROS RingTracker 语义，防间歇假组）----
                confirmed, candidates = tracker.update(corrected)

                # ---- 三环组判定 + ID1/2/3 + yaw ----
                triple, relation, rest = select_three(
                    confirmed, gray.shape[1], expected_spacing=args.spacing,
                    spacing_gate=args.spacing_gate,
                    min_span_ratio=args.min_span_ratio)
                if triple is not None:
                    status = 'FULL'
                    sites = [dict(t, id=i + 1) for i, t in enumerate(triple)]
                    # yaw 优先取黑线角（基线最长最稳），无线时用环心基线角
                    yaw_raw = line_info['angle'] \
                        if line_info.get('line') == 'ok' else relation[3]
                elif 1 <= len(confirmed) <= 2:
                    status, sites = 'PARTIAL', confirmed
                    yaw_raw = baseline_yaw(confirmed)
                else:
                    status, sites, yaw_raw = 'NONE', [], None
                yaw = tracker.smooth_yaw(yaw_raw if status == 'FULL'
                                         else yaw_raw if status == 'PARTIAL'
                                         else None)

                det.last_groups = sites if sites else confirmed
                total = t_undist + t_blur + t_hough + t_verify + t_line

                now = time.perf_counter()
                dt, last_t = now - last_t, now
                fps = 0.9 * fps + 0.1 * (1.0 / dt) if dt > 0 else fps

                canvas = cv2.cvtColor(blur, cv2.COLOR_GRAY2BGR)
                annotate(canvas, raw, candidates, line_rejected, sites,
                         status, yaw, line_seg, ideal_seg, region,
                         {'fps': fps, 'undist': t_undist, 'blur': t_blur,
                          'hough': t_hough, 'verify': t_verify,
                          'line': t_line, 'total': total,
                          'line_info': line_info, 'blur_k': blur_k,
                          'clahe': None if clahe is None else args.clip},
                         args, os.path.basename(und.source))
                last_canvas = canvas

                # 每 5 秒打印一次性能统计
                perf_frames += 1
                perf_total += total
                if now - perf_win_t >= 5.0:
                    yaw_txt = f'{yaw:.2f}' if yaw is not None else '--'
                    print(f'[RING PERF] process={fps:.1f}Hz '
                          f'avg_total={perf_total / max(perf_frames, 1):.1f}ms '
                          f'undist={t_undist:.1f}ms blur={t_blur:.1f}ms '
                          f'hough={t_hough:.1f}ms verify={t_verify:.1f}ms '
                          f'line={t_line:.1f}ms region={region} '
                          f'raw={len(groups)} conf={len(confirmed)} '
                          f'status={status} yaw={yaw_txt} {line_info}')
                    perf_win_t, perf_frames, perf_total = now, 0, 0.0
            elif paused and frame is not None:
                pass                    # 暂停期间照样取走新帧，恢复时画面即最新

            if last_canvas is not None:
                cv2.imshow(WINDOW_NAME, last_canvas)

            key = cv2.waitKey(1) & 0xFF
            if key in (ord('q'), 27):
                break
            if key == ord('p'):
                paused = not paused
                print(f'[hint] {"已暂停" if paused else "继续"}')
            elif key == ord('s'):
                if last_canvas is not None:
                    name = time.strftime('snapshot_%Y%m%d_%H%M%S.png')
                    cv2.imwrite(name, last_canvas)
                    print(f'[hint] 截图已保存: {name}')
            elif key == ord('w'):
                args.param2 = max(1, args.param2 - 5)
                print(f'[hint] param2 -> {args.param2}')
            elif key == ord('e'):
                args.param2 += 5
                print(f'[hint] param2 -> {args.param2}')
            elif key == ord('a'):
                args.param1 = max(1, args.param1 - 10)
                print(f'[hint] param1 -> {args.param1}')
            elif key == ord('d'):
                args.param1 += 10
                print(f'[hint] param1 -> {args.param1}')
            elif key == ord('b'):
                blur_k = 3 if blur_k != 3 else 5
                blur_sigma = {3: 1.0, 5: 1.2, 7: 2.0}[blur_k] \
                    if args.blur_sigma is None else args.blur_sigma
                print(f'[hint] 高斯核 -> {blur_k}x{blur_k} σ{blur_sigma}')
            elif key == ord('v'):
                args.no_verify = not args.no_verify
                print(f'[hint] 覆盖率验证 {"关闭" if args.no_verify else "开启"}')
            elif key == ord('c'):
                if clahe is None:
                    clahe = cv2.createCLAHE(clipLimit=args.clip,
                                            tileGridSize=(8, 8))
                else:
                    clahe = None
                print(f'[hint] CLAHE {"开启 clip=" + str(args.clip) if clahe is not None else "关闭"}')
            elif key == ord('r'):
                args.roi = not args.roi
                det.last_groups = []
                print(f'[hint] ROI 模式 {"开启" if args.roi else "关闭"}')
    finally:
        cam.close()
        cv2.destroyAllWindows()


# ---------------------------------------------------------------------------
# 离线单图测试：不出窗口，直接打印结果并保存标注图
# ---------------------------------------------------------------------------
def run_image(args):
    frame = cv2.imread(args.image)
    if frame is None:
        print(f'无法读取图片: {args.image}')
        return 1
    und = Undistorter(args.calib, (frame.shape[1], frame.shape[0]))
    undist = und.remap(frame)
    gray = cv2.cvtColor(undist, cv2.COLOR_BGR2GRAY)
    blur_k = args.blur
    blur_sigma = {3: 1.0, 5: 1.2, 7: 2.0}[blur_k] \
        if args.blur_sigma is None else args.blur_sigma
    blur = cv2.GaussianBlur(gray, (blur_k, blur_k), blur_sigma)
    if not args.no_clahe:
        blur = cv2.createCLAHE(clipLimit=args.clip,
                               tileGridSize=(8, 8)).apply(blur)
    det = RingDetector(args)
    raw, verified, groups, region, t_hough, t_verify = det.detect(gray, blur)
    print(f'原始霍夫圆 {len(raw)} 个: '
          f'{[(int(x), int(y), int(r)) for x, y, r in raw]}')
    print(f'验证通过 {len(verified)} 个，环组 {len(groups)} 组:')
    for i, g in enumerate(groups):
        print(f'  S{i + 1}: 中心=({g["cx"]:.1f},{g["cy"]:.1f}) '
              f'r_inner={g["r_inner"]:.1f} r_outer={g["r_outer"]:.1f} '
              f'成员数={len(g["members"])}')
    # 三环组判定（单图无时域层，直接对环组判定）
    triple, relation, rest = select_three(
        groups, frame.shape[1], expected_spacing=args.spacing,
        spacing_gate=args.spacing_gate, min_span_ratio=args.min_span_ratio)
    if triple is not None:
        status = 'FULL'
        sites = [dict(t, id=i + 1) for i, t in enumerate(triple)]
        yaw = relation[3]
        print(f'三环组判定: FULL 间距=({relation[0]:.1f},{relation[1]:.1f})px '
              f'共线度={relation[2]:.3f} yaw={yaw:.2f}deg')
    elif groups:
        status, sites = 'PARTIAL', groups
        yaw = baseline_yaw(groups)
        print(f'三环组判定: PARTIAL（{len(groups)} 组，无达标三元组）')
    else:
        status, sites, yaw = 'NONE', [], None
    print(f'耗时: hough={t_hough:.1f}ms verify={t_verify:.1f}ms '
          f'region={region}')
    canvas = cv2.cvtColor(blur, cv2.COLOR_GRAY2BGR)
    annotate(canvas, raw, [], [], sites, status, yaw, None, None, region,
             {'fps': 0, 'undist': 0, 'blur': 0, 'hough': t_hough,
              'verify': t_verify, 'line': 0, 'total': t_hough + t_verify,
              'line_info': {'line': 'off'}, 'blur_k': blur_k,
              'clahe': None if args.no_clahe else args.clip},
             args, und.source)
    out = 'ring_hough_result.png'
    cv2.imwrite(out, canvas)
    print(f'标注图已保存: {out}')
    return 0


# ---------------------------------------------------------------------------
def parse_args():
    p = argparse.ArgumentParser(
        description='独立圆环识别（经典梯度霍夫，无 ROS2），'
                    '摄像头与去畸变参数同 logistics_ws 现役链路')
    p.add_argument('--device', default='/dev/video0', help='摄像头设备节点')
    p.add_argument('--calib', default=DEFAULT_CALIB, help='标定 yaml 路径')
    p.add_argument('--width', type=int, default=640)
    p.add_argument('--height', type=int, default=480)
    p.add_argument('--fps', type=int, default=30)
    p.add_argument('--scale', type=float, default=1.0,
                   help='霍夫检测缩放（0.5 = 半分辨率粗定位后还原）')
    p.add_argument('--roi', action='store_true',
                   help='确认环组后锁定局部区域检测（再按 r 键可切换）')
    p.add_argument('--roi-full-scan-interval', type=int, default=15,
                   help='ROI 模式下每 N 帧强制全图复查')
    p.add_argument('--blur', type=int, choices=(3, 5, 7), default=5,
                   help='高斯核尺寸：3=环线视觉最细且边缘票数最足（实测保留97%%），'
                        '5=抑噪优先；按 b 键运行时在 3/5 间切换')
    p.add_argument('--blur-sigma', type=float, default=None,
                   help='高斯 σ（默认按核配对：3→1.0、5→1.2、7→2.0）')
    p.add_argument('--no-clahe', action='store_true',
                   help='关闭 CLAHE（按 c 键可运行时切换）')
    p.add_argument('--clip', type=float, default=3.0,
                   help='CLAHE clipLimit，越大局部对比度拉伸越强')
    p.add_argument('--no-verify', action='store_true',
                   help='关闭覆盖率验证门（按 v 键可切换）')
    p.add_argument('--coverage', type=float, default=0.7,
                   help='覆盖率门限（12 扇区命中占比）')
    p.add_argument('--dp', type=float, default=1.0)
    p.add_argument('--min-dist', type=float, default=60.0)
    p.add_argument('--param1', type=float, default=210.0,
                   help='Canny 高阈值（2026-10-28 起默认 210）')
    p.add_argument('--param2', type=float, default=60.0,
                   help='累加器票数阈值（经典法语义，越大越严；'
                        '2026-10-28 起默认 60）')
    p.add_argument('--min-r', type=int, default=20)
    p.add_argument('--max-r', type=int, default=60)
    p.add_argument('--merge-dist', type=float, default=20.0,
                   help='同心聚类圆心距门限 px')
    p.add_argument('--smooth', type=float, default=0.5,
                   help='EMA 新值权重 0~1（仅 --smooth-mode ema 时生效）')
    p.add_argument('--smooth-mode', choices=('ema', 'median', 'alphabeta',
                                             'kalman'),
                   default='alphabeta',
                   help='圆心平滑算法（2026-10-28 实测 alphabeta 帕累托优于'
                        ' ema：静止抖动 -41%%、运动偏差 -26%%；kalman=完整'
                        '自适应卡尔曼，干净场景下与 alphabeta 打平，遮挡/'
                        '丢帧场景有自愈优势）')
    p.add_argument('--median-window', type=int, default=7,
                   help='median 模式窗口帧数（运动时滞后大，适合纯静态）')
    p.add_argument('--alpha', type=float, default=0.3,
                   help='alphabeta 模式位置增益（实测最优 0.3）')
    p.add_argument('--beta', type=float, default=0.15,
                   help='alphabeta 模式速度增益（实测最优 0.15）')
    p.add_argument('--kalman-q', type=float, default=0.15,
                   help='kalman 模式过程噪声（实测均衡点 0.15）')
    p.add_argument('--kalman-r', type=float, default=4.8,
                   help='kalman 模式测量噪声 px²（本机静止段实测，'
                        '换相机/场景后重估）')
    p.add_argument('--confirm', type=int, default=8,
                   help='轨迹累计命中 N 帧才确认（ROS 默认 8，防间歇假组）')
    p.add_argument('--miss-tol', type=int, default=3,
                   help='连丢超 N 帧命中史清零（防间歇假组攒数）')
    p.add_argument('--remove-misses', type=int, default=8,
                   help='连丢超 N 帧删除轨迹')
    p.add_argument('--hold', type=int, default=2,
                   help='确认后丢失 N 帧内仍显示')
    p.add_argument('--assoc', type=float, default=60.0,
                   help='帧间轨迹关联距离 px（须小于工位间距152）')
    p.add_argument('--spacing', type=float, default=EXPECTED_SPACING_PX,
                   help='三工位期望间距 px（仅 --spacing-gate 开启时生效）')
    p.add_argument('--spacing-gate', action='store_true',
                   help='启用绝对间距门（|avg−152.1|≤20%%，默认关闭：'
                        '绝对像素间距随拍摄高度变化，默认不以此否决）')
    p.add_argument('--min-span-ratio', type=float, default=0.35,
                   help='三环组跨度下限占画面宽度的比例（cyqm 移植，'
                        '角落小跨度假三元组一票否决）')
    p.add_argument('--line', action='store_true',
                   help='开启黑线辅助矫正（默认关闭，2026-10-27 起改为默认关闭；'
                        '开启后需同时满足锚点条件才生效）')
    p.add_argument('--line-side', choices=('auto', 'top', 'bottom'),
                   default='top',
                   help='黑线搜索侧：默认 top=只搜图像上侧（实拍确认仅地图'
                        '边界侧黑线明显）；auto=双侧+侧记忆')
    p.add_argument('--line-dist-px', type=float, default=None,
                   help='黑线与环心线垂距 px（默认自动学习，理论 73.2）')
    p.add_argument('--line-corr', type=float, default=1.0,
                   help='黑线垂直矫正强度 0~1（0=只测不矫正）')
    p.add_argument('--line-band', type=int, default=18,
                   help='黑线搜索斜带半宽 px')
    p.add_argument('--image', default=None,
                   help='离线测试：喂一张静态图，不出窗口直接报结果')
    return p.parse_args()


def main():
    args = parse_args()
    # SIGTERM/SIGINT 转 SystemExit，保证 run_camera 的 finally 执行
    # cam.close()——被 timeout/kill 强杀时若无释放动作，UVC 相机会停在
    # streaming 状态，下一次 STREAMON 报 Protocol error（须重载 uvcvideo）。
    for sig in (signal.SIGTERM, signal.SIGINT):
        signal.signal(sig, lambda s, f: sys.exit(0))
    if args.image:
        return run_image(args)
    run_camera(args)
    return 0


if __name__ == '__main__':
    sys.exit(main())
