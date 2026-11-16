#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""line_yaw.py —— 工位基准线检测（cyqm stacked_ring_detector 思路，共享模块）。

从 stacked_detector.py 原样抽出的线检测三件套（种子精修 / 带内找线 /
物理距离门），供两个模式共用：
    - stacked（码垛）：中环上/下 ±0.5×实测间距各找一条长边基准线，
      双线 2° 一致性配对取跨度加权 yaw，单线退化，全失回退三环基线角；
    - ring（圆环，2026-11-14 移植）：三环 FULL 后在上侧（2026-10-27 实拍
      探针实锤：粗加工/暂存区仅地图边界侧黑线明显）距环心线半个实测
      间距处找边界线，线角优先定 yaw，失线回退基线角。

与已删除的 ring LineAssist 的本质区别：本模块**无冻结状态**——目标线
位置每帧由调用方用实测间距现算（offset 参数），线种子丢失即清，
不存在"学习垂距过时 → 永不重捕获"一类缺陷（原 A1.5）。
"""

import math

import cv2
import numpy as np


class BandLineFinder:
    """单侧基准线检测器：上帧线种子 ±6px 精修优先，HoughLinesP 兜底，
    物理距离门复核；命中过门后更新内部种子，未命中清种子。"""

    def __init__(self, band_ratio=0.14, min_span_ratio=1.4,
                 dist_check_ratio=0.05):
        self.band_ratio = float(band_ratio)
        self.min_span_ratio = float(min_span_ratio)
        self.dist_check_ratio = float(dist_check_ratio)
        self.seed = None               # 上帧线 (angle_deg, y_at) 或 None

    def clear_seed(self):
        """三环/中环丢失时调用：防过时种子把下一帧带偏（码垛原语义）。"""
        self.seed = None

    def find(self, edges, center_x, center_y, offset, diameter,
             prior_angle, sign):
        """在圆心 (center_x, center_y) 沿 sign(±1) 方向 offset 像素处
        ±band 斜带内找基准线，返回 (score, angle_deg, segment, y_at)
        或 None（未命中/被距离门拒绝）。

        band 与最小跨度由环径推导（码垛原公式，随识别目标尺寸自适应）：
            band     = max(8, diameter × band_ratio)
            min_span = max(60, diameter × min_span_ratio)
        prior_angle：斜带方向与角度门先验（上帧平滑 yaw 或三环基线角）。
        """
        band = max(8, int(round(diameter * self.band_ratio)))
        min_span = max(60.0, diameter * self.min_span_ratio)
        line = self._line(edges, center_x, center_y + sign * offset,
                          prior_angle, band, min_span)
        line = self._check_line_distance(line, center_y, offset, sign)
        self.seed = (line[1], line[3]) if line else None
        return line

    # ------------------------------------------------------------------
    # 以下三个方法自 stacked_detector.py 原样移植（self._band/_min_span/
    # _initial_angle 实例状态改为显式参数，逻辑逐行不变）
    # ------------------------------------------------------------------
    def _refine_line_seed(self, filtered, center_x, target_y, seed,
                          prior_angle, band, min_span):
        """上帧线 ±6px 内的边缘点做 Huber fitLine；角度/位置双校验。

        cyqm 增强: 精修前按 x 列去重——每列只留距线最近的一点,
        防止竖直纹理聚簇拉偏拟合。"""
        seed_angle, seed_y = seed
        yy, xx = np.nonzero(filtered)
        if not len(xx):
            return None
        points = np.column_stack((xx, yy)).astype(np.float32)
        seed_slope = math.tan(math.radians(seed_angle))
        residual = np.abs(points[:, 1] - (seed_y + seed_slope *
                                          (points[:, 0] - center_x)))
        mask = residual <= 6.0
        near, near_residual = points[mask], residual[mask]
        if len(near) < 40 or np.ptp(near[:, 0]) < min_span:
            return None
        order = np.lexsort((near_residual, near[:, 0]))
        dedup = near[order]
        first = np.ones(len(dedup), dtype=bool)
        first[1:] = dedup[1:, 0] != dedup[:-1, 0]
        near = dedup[first]
        if len(near) < 40 or np.ptp(near[:, 0]) < min_span:
            return None
        vx, vy, px, py = (float(v) for v in cv2.fitLine(
            near, cv2.DIST_HUBER, 0.0, 0.01, 0.01).reshape(-1))
        if vx < 0:
            vx, vy = -vx, -vy
        if abs(vx) <= 1e-6:
            return None
        angle = math.degrees(math.atan2(vy, vx))
        if abs(angle - prior_angle) > 15 or abs(angle - seed_angle) > 5:
            return None
        y_at = py + vy / vx * (center_x - px)
        if abs(y_at - target_y) > band + 10.0:
            return None
        x0, x1 = int(near[:, 0].min()), int(near[:, 0].max())
        slope = math.tan(math.radians(angle))
        segment = ((x0, int(round(y_at + slope * (x0 - center_x)))),
                   (x1, int(round(y_at + slope * (x1 - center_x)))))
        score = float(np.ptp(near[:, 0])) - 3.0 * abs(y_at - target_y)
        return score, angle, segment, float(y_at)

    def _line(self, edges, center_x, target_y, prior_angle, band, min_span):
        """在目标线 ±band 斜带内找基准线：种子精修优先，HoughLinesP 兜底。"""
        slope = math.tan(math.radians(prior_angle))
        rows = np.arange(edges.shape[0])[:, None]
        cols = np.arange(edges.shape[1])[None, :]
        mask = np.abs(rows - (target_y + slope * (cols - center_x))) <= band
        filtered = np.where(mask, edges, 0).astype(np.uint8)
        if self.seed is not None:
            refined = self._refine_line_seed(filtered, center_x, target_y,
                                             self.seed, prior_angle, band,
                                             min_span)
            if refined is not None:
                return refined
        found = cv2.HoughLinesP(filtered, 1, np.pi / 180, 40,
                                minLineLength=int(min_span),
                                maxLineGap=25)
        best = None
        # OpenCV4常见(N,1,4)，OpenCV5可返回(N,4)，统一为逐条端点。
        for item in np.asarray(found).reshape(-1, 4) if found is not None else []:
            x1, y1, x2, y2 = map(int, item)
            if x2 < x1:
                x1, y1, x2, y2 = x2, y2, x1, y1
            angle = math.degrees(math.atan2(y2 - y1, x2 - x1))
            if abs(angle - prior_angle) > 15:
                continue
            length = math.hypot(x2 - x1, y2 - y1)
            expected = target_y + slope * ((x1 + x2) / 2 - center_x)
            score = length - 3 * abs((y1 + y2) / 2 - expected)
            y_at = y1 + (y2 - y1) * (center_x - x1) / max(1, x2 - x1)
            if best is None or score > best[0]:
                best = (score, angle, ((x1, y1), (x2, y2)), float(y_at))
        return best

    def _check_line_distance(self, line, center_y, expected_offset, sign):
        """cyqm 物理距离复核: 线到圆心距离须等于预期偏移 ±ratio，且方向正确。"""
        if line is None:
            return None
        actual = (line[3] - center_y) * sign
        tolerance = max(2.0, expected_offset * self.dist_check_ratio)
        if actual <= 0 or abs(actual - expected_offset) > tolerance:
            return None
        return line
