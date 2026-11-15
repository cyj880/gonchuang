#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""共享转盘外圆检测器：保留原有霍夫、径向精修和稳定窗口；含离盘高度解算。"""
import math
from collections import deque
import cv2
import numpy as np

# 规则《竞赛命题与运行》P4 表2：原料区顶面为直径300mm的圆盘（盘总高
# 80~100mm 为盘体结构高度，与相机距离无关）。高度解算用半径 150mm。
TRAY_RADIUS_MM = 150.0

TURNTABLE_P = dict(  # 源自 turntable_detect_node 默认参数；coarse 半径窗默认值
    # 仅为无内参时的兜底（构造时传入焦距会按相机高度带重新推导，见下）
    reference_center=(320.0, 240.0), reference_radius=185.0,
    coarse_scale=0.35, coarse_min_radius=120.0, coarse_max_radius=230.0,
    coarse_param1=60.0, coarse_param2=28.0, coarse_min_dist=80.0,
    refine_half_width=28.0, angular_step_deg=4.0, min_visible_arc=90.0,
    max_fit_rms=7.0, min_edge_strength=4.0, stable_frames=5,
    max_center_jitter=10.0, max_radius_jitter=5.0,
    saturated_pixel_threshold=90,  # cyqm 同款：高饱和（彩色物料）像素不参与外圆扫描
    recheck_conf_below=0.70, recheck_rms_above=4.0,
    recheck_interval=5, reacquire_jump=15.0, seed_hold_frames=3)


def radius_window_for_height(focal_px, height_min_mm, height_max_mm):
    """由相机高度带换算转盘像素半径窗：r = f × 150 / Z（规则转盘 ⌀300mm）。

    先验写在物理量上（相机离盘多近多远），不写死像素——像素窗随内参
    分辨率自动适配，换相机/换分辨率不需要重标窗口。"""
    if not (focal_px > 0 and 0 < height_min_mm < height_max_mm):
        raise ValueError('焦距或高度带无效')
    r_min = focal_px * 150.0 / height_max_mm
    r_max = focal_px * 150.0 / height_min_mm
    return max(20.0, r_min), r_max


def focal_px_of(und):
    """从去畸变内参取焦距；对象不含 new_K 时返回 None（回退兜底像素窗）。"""
    try:
        return float((und.new_K[0, 0] + und.new_K[1, 1]) / 2)
    except (AttributeError, TypeError, IndexError, ValueError):
        return None


def _fit_circle(points):
    if len(points) < 3:
        return None
    x, y = points[:, 0], points[:, 1]
    matrix = np.column_stack((x, y, np.ones(len(points))))
    target = x * x + y * y
    solution, _, _, _ = np.linalg.lstsq(matrix, target, rcond=None)
    cx, cy = solution[0] / 2.0, solution[1] / 2.0
    radius_sq = solution[2] + cx * cx + cy * cy
    if radius_sq <= 0 or not np.isfinite(radius_sq):
        return None
    return np.array([cx, cy], dtype=np.float32), float(math.sqrt(radius_sq))


def _robust_fit_circle(points, min_r, max_r):
    """5 轮 MAD 剔野最小二乘圆拟合（turntable_detect_node:356-381）。"""
    if len(points) < 20:
        return None
    keep = np.ones(len(points), dtype=bool)
    circle = _fit_circle(points)
    if circle is None:
        return None
    for _ in range(5):
        center, radius = circle
        residuals = np.abs(np.linalg.norm(points - center, axis=1) - radius)
        active = residuals[keep]
        median = float(np.median(active))
        mad = float(np.median(np.abs(active - median))) + 1e-6
        threshold = max(1.5, min(5.0, median + 3.0 * 1.4826 * mad))
        updated = residuals <= threshold
        if np.count_nonzero(updated) < 20:
            break
        keep = updated
        circle = _fit_circle(points[keep])
        if circle is None:
            return None
    center, radius = circle
    if not (min_r <= radius <= max_r):
        return None
    return center, radius, keep


def _longest_arc_angles(angles):
    """最长连续可见弧（5° 分箱，roll 弥补跨零，返回度数）。"""
    if len(angles) == 0:
        return 0.0
    bins = np.zeros(72, dtype=bool)
    bins[np.mod((angles / (2 * np.pi) * 72).astype(int), 72)] = True
    bins = bins | (np.roll(bins, 1) & np.roll(bins, -1))
    best = current = 0
    for value in np.tile(bins, 2):
        current = current + 1 if value else 0
        best = max(best, current)
    return float(min(best, 72) * 5.0)


class TurntableDetector:
    """无相机/线程依赖；调用方传入去畸变后的新鲜图像。

    focal_px 传入去畸变内参焦距时：①检测半径窗按相机高度带
    [tray_height_min_mm, tray_height_max_mm] 推导（台架默认 120~600mm，
    上车后按实际安装高度收紧）；②每帧结果带 height_mm = f×150/r_px
    （相机离盘面高度，规则转盘 ⌀300mm）。不传 focal_px 则用兜底像素窗，
    height_mm=None。"""
    def __init__(self, focal_px=None,
                 tray_height_min_mm=120.0, tray_height_max_mm=600.0):
        self.p = dict(TURNTABLE_P)
        self.focal_px = float(focal_px) if focal_px and focal_px > 0 else None
        if self.focal_px is not None:
            r_min, r_max = radius_window_for_height(
                self.focal_px, tray_height_min_mm, tray_height_max_mm)
            self.p['coarse_min_radius'] = r_min
            self.p['coarse_max_radius'] = r_max
        self.reference = np.array(self.p['reference_center'], np.float32)
        self.reset()

    def reset(self):
        self._samples = deque(maxlen=self.p['stable_frames'])
        self._seed = None
        self._seed_miss = 0
        self._since_recheck = 0

    def _global_refine(self, frame):
        """全局霍夫粗定位 → 逐候选精修 → 取置信度最高者
        （turntable_detect_node:208-225，未含 _material_seed 种子路径）。"""
        best = None
        for candidate in self._global_coarse(frame):
            fit = self._refine(frame, candidate)
            if fit is not None and (best is None or
                                    fit['confidence'] > best['confidence']):
                best = fit
        if best is None and self._seed is not None:
            # 粗定位候选全部失败时回退种子精修：个别帧的霍夫漏检不应清空
            # 稳定窗（实测约2%帧漏检，曾是 STABLE<->ACQUIRING 跳动的根源）
            best = self._refine(frame, self._seed)
        return best

    def detect(self, frame):
        p = self.p
        annotated = frame.copy()
        best = None
        had_seed = self._seed is not None
        if had_seed:
            best = self._refine(frame, self._seed)
            self._since_recheck += 1
            needs = (best is None or best['confidence'] < p['recheck_conf_below']
                     or best['fit_rms_px'] > p['recheck_rms_above'])
            if best is None or (needs and
                                self._since_recheck >= p['recheck_interval']):
                best = self._global_refine(frame)
                mode = 'reacquired'
                self._since_recheck = 0
            else:
                mode = 'local'
        else:
            best = self._global_refine(frame)
            mode = 'reacquired'

        if best is None:
            # 单帧失败不清稳定窗：静止场景下一帧即恢复（实测霍夫单帧漏检率
            # ~0.7%，曾与全局复查失败叠加成 STABLE<->ACQUIRING 跳动）；
            # 连续失败才视为真丢失清窗。
            self._seed_miss += 1
            if self._seed_miss > 1:
                self._samples.clear()
            if self._seed_miss > p['seed_hold_frames']:
                self._seed = None
            tracking_mode = 'search'
            cv2.putText(annotated, 'TURNTABLE: SEARCHING', (10, 28),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 0, 255), 2,
                        cv2.LINE_AA)
            return None, tracking_mode, annotated

        if had_seed:
            jump = max(math.hypot(best['u'] - self._seed[0],
                                  best['v'] - self._seed[1]),
                       abs(best['radius'] - self._seed[2]))
            if jump > p['reacquire_jump']:
                self._samples.clear()
        self._seed = np.array([best['u'], best['v'], best['radius']],
                              dtype=np.float32)
        self._seed_miss = 0
        self._samples.append(best)
        center = np.median(np.array([[s['u'], s['v']] for s in self._samples]),
                           axis=0)
        radius = float(np.median([s['radius'] for s in self._samples]))
        center_jitter = max(float(np.linalg.norm(
            np.array([s['u'], s['v']]) - center)) for s in self._samples)
        radius_jitter = max(abs(s['radius'] - radius)
                            for s in self._samples)
        stable = (len(self._samples) == self._samples.maxlen and
                  center_jitter <= p['max_center_jitter'] and
                  radius_jitter <= p['max_radius_jitter'])
        # 离盘高度解算（针孔模型）：Z = f × 盘半径150mm / 实测半径px。
        # 前提：光轴垂直盘面且过盘心（比赛安装满足），倾斜/偏心引入误差。
        height_mm = None
        if self.focal_px is not None and radius > 0:
            height_mm = self.focal_px * TRAY_RADIUS_MM / radius
        cv2.circle(annotated, (int(round(center[0])), int(round(center[1]))),
                   int(round(radius)), (0, 220, 0), 2)
        cv2.drawMarker(annotated, (int(round(center[0])), int(round(center[1]))),
                       (0, 0, 255), cv2.MARKER_CROSS, 24, 2)
        text = f'TURNTABLE {"STABLE" if stable else "ACQUIRING"} ' \
               f'center=({round(center[0])},{round(center[1])})'
        if height_mm is not None:
            text += f' h={height_mm:.0f}mm'
        cv2.putText(annotated, text, (10, 28), cv2.FONT_HERSHEY_SIMPLEX,
                    0.6, (0, 255, 0) if stable else (0, 200, 255), 2,
                    cv2.LINE_AA)
        result = dict(best)
        result.update({'detected': True, 'stable': stable,
                       # 夹取门控使用当前帧拟合值，不能让历史中值掩盖几何抖动。
                       'current_center': (float(best['u']), float(best['v'])),
                       'current_radius_px': float(best['radius']),
                       'center_u': int(round(center[0])),
                       'center_v': int(round(center[1])),
                       'radius_px': radius,
                       'height_mm': round(height_mm, 1)
                       if height_mm is not None else None})
        result.pop('u', None)
        result.pop('v', None)
        result.pop('radius', None)
        return result, mode, annotated

    def _global_coarse(self, frame):
        """全局霍夫粗定位候选（未移植 _material_seed 种子路径）。"""
        p = self.p
        scale = p['coarse_scale']
        small = cv2.resize(frame, None, fx=scale, fy=scale,
                           interpolation=cv2.INTER_AREA)
        gray = cv2.GaussianBlur(cv2.cvtColor(small, cv2.COLOR_BGR2GRAY),
                                (7, 7), 0)
        min_r = max(2, int(p['coarse_min_radius'] * scale))
        max_r = max(min_r + 1, int(p['coarse_max_radius'] * scale))
        circles = cv2.HoughCircles(
            gray, cv2.HOUGH_GRADIENT, dp=1.2,
            minDist=max(3, int(p['coarse_min_dist'] * scale)),
            param1=p['coarse_param1'], param2=p['coarse_param2'],
            minRadius=min_r, maxRadius=max_r)
        output = []
        if circles is not None:
            for x, y, radius in circles[0][:10]:
                candidate = np.array([x / scale, y / scale, radius / scale],
                                     dtype=np.float32)
                if (self._seed is not None and np.linalg.norm(
                        candidate[:2] - self._seed[:2]) > 120.0):
                    continue  # 远离上帧盘心的候选视为杂弧（环板图案等）
                output.append(candidate)
        if self._seed is not None and not output:
            output.append(self._seed.copy())
        return output

    def _refine(self, frame, candidate):
        """极坐标径向扫描精修（turntable_detect_node:305-354）。"""
        if candidate is None:
            return None
        p = self.p
        half = p['refine_half_width']
        step = p['angular_step_deg']
        angles = np.deg2rad(np.arange(0.0, 360.0, step))
        radii = np.arange(max(5.0, candidate[2] - half),
                          candidate[2] + half + 1.0, 1.0)
        map_x = (candidate[0] + np.cos(angles)[:, None] *
                 radii[None, :]).astype(np.float32)
        map_y = (candidate[1] + np.sin(angles)[:, None] *
                 radii[None, :]).astype(np.float32)
        h, w = frame.shape[:2]
        valid = ((map_x >= 3) & (map_x < w - 3) &
                 (map_y >= 3) & (map_y < h - 3))
        colors = cv2.remap(frame, map_x, map_y, cv2.INTER_LINEAR,
                           borderMode=cv2.BORDER_CONSTANT).astype(np.float32)
        colors = cv2.GaussianBlur(colors, (5, 1), 1.0)
        # cyqm 同款饱和度剔除：在采样矩阵上就地按 BGR max/min 算饱和度，
        # 盘上的彩色物料像素不参与外圆边缘扫描（否则物料压边/近边时其
        # 彩色梯度会抢走径向峰值，致稳定窗反复掉落）。
        maximum = np.max(colors, axis=2)
        minimum = np.min(colors, axis=2)
        saturations = (maximum - minimum) * 255.0 / np.maximum(maximum, 1.0)
        gradients = np.linalg.norm(colors[:, 2:, :] - colors[:, :-2, :], axis=2)
        valid = valid[:, 1:-1] & valid[:, :-2] & valid[:, 2:]
        sample_radii = radii[1:-1]
        prior = np.exp(-0.5 * ((sample_radii - candidate[2]) /
                               max(8.0, half * 0.7)) ** 2)
        scores = gradients * prior[None, :]
        scores[~valid] = -np.inf
        scores[saturations[:, 1:-1] >=
               self.p['saturated_pixel_threshold']] = -np.inf
        index = np.argmax(scores, axis=1)
        rows = np.arange(len(angles))
        strength = gradients[rows, index]
        eligible = np.isfinite(scores[rows, index]) & \
            (strength >= p['min_edge_strength'])
        if np.count_nonzero(eligible) < 20:
            return None
        selected_angles = angles[eligible]
        selected_radii = sample_radii[index[eligible]]
        points = np.column_stack((
            candidate[0] + selected_radii * np.cos(selected_angles),
            candidate[1] + selected_radii * np.sin(selected_angles)))
        strengths = strength[eligible]
        circle = _robust_fit_circle(points, p['coarse_min_radius'],
                                    p['coarse_max_radius'])
        if circle is None:
            return None
        center, radius, inliers = circle
        distances = np.linalg.norm(points[inliers] - center, axis=1)
        rms = float(np.sqrt(np.mean((distances - radius) ** 2)))
        if rms > p['max_fit_rms']:
            return None
        point_angles = np.mod(np.arctan2(points[inliers, 1] - center[1],
                                         points[inliers, 0] - center[0]),
                              2 * np.pi)
        longest = _longest_arc_angles(point_angles)
        if longest < p['min_visible_arc']:
            return None
        edge_score = min(1.0, float(np.median(strengths[inliers])) / 20.0)
        confidence = min(1.0, 0.35 * min(1.0, longest / 180.0) +
                         0.50 * max(0.0, 1.0 - rms / (p['max_fit_rms'] * 1.5)) +
                         0.15 * edge_score)
        return {'u': float(center[0]), 'v': float(center[1]),
                'radius': float(radius), 'visible_arc_deg': float(longest),
                'fit_rms_px': rms, 'confidence': confidence,
                'detail': 'outer circle refined'}

