#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""stacked_detector.py —— 码垛识别（模式5）：全图霍夫三环取中 + 基准线 + 侧站推算。

三步流程（2026-10-31 设计定稿）：
    1. 定中环：每帧全图霍夫（复用 ring_hough_standalone 的 RingDetector，实车
       验证参数原样）→ GroupTracker 时域确认 → select_three 三环排列关系取
       中间环。每帧全图执行、无搜索窗——目标不可能"移出搜索区域"，运动丢失
       是结构性消除；
    2. 定角度：中环上下 ±0.5×实测间距（规则几何 75mm=150mm/2）开带找工位
       上下长边基准线（cyqm stacked_ring_detector 思路）：上帧线种子 ±6px
       精修 + 按列去重（防竖直纹理聚簇拉偏 Huber）→ HoughLinesP 兜底 →
       角度/位置/物理距离门（±5%）→ 双线 2° 一致性配对 + 跨度加权 yaw，
       单线退化，双线全失或方向冲突回退三环基线角（不断流）；
    3. 推算侧站：左右工位 = 中环心 ± 实测间距 × (cos yaw, sin yaw)，
       source=inferred（间距每帧实测，无需预拍标定——cyqm 依赖预拍布局
       旋转，本方案不需要）。

设计决定（讨论定稿）：
    - 全分辨率霍夫起步；分辨率缩放经 RingDetector 的 scale 参数留作调参 A/B；
    - 无三帧稳定窗/到位判定——输出只看当帧是否检出，杜绝"识别到了但条件
      不通过导致间歇丢失"（v1 教训）；
    - 无遮挡回退（三环失败即如实断流，保持极简）；
    - 平滑：中环圆心由 GroupTracker 内置 alphabeta 常速滤波（运动/静止一套
      滤波，实测运动偏差比中值窗小 6 倍）；yaw EMA（15° 跳变吸附）；间距弱 EMA。

输入：去畸变后的 640x480 BGR 帧（去畸变由调用方 Worker 负责）。
"""

import argparse
import math

import cv2
import numpy as np

from ring_hough_standalone import RingDetector, GroupTracker, select_three
from line_yaw import BandLineFinder


class StackedDetector:
    """码垛工位跟踪器：全图霍夫三环取中 → 上下基准线定 yaw → 推算左右工位。"""

    def __init__(self, *, min_span_ratio=0.35, confirm_hits=2,
                 line_band_ratio=0.14, min_line_span_ratio=1.4,
                 line_disagreement=2.0, line_dist_check_ratio=0.05,
                 yaw_ema_w=0.8, spacing_ema_w=0.5):
        self.min_span_ratio = float(min_span_ratio)
        self.line_band_ratio = float(line_band_ratio)
        self.min_line_span_ratio = float(min_line_span_ratio)
        self.line_disagreement = float(line_disagreement)
        self.line_dist_check_ratio = float(line_dist_check_ratio)
        self.yaw_ema_w = float(yaw_ema_w)
        self.spacing_ema_w = float(spacing_ema_w)
        # 上/下基准线各一个 finder（种子与状态互相独立，line_yaw.py 共享模块）
        self._top_line = BandLineFinder(line_band_ratio, min_line_span_ratio,
                                        line_dist_check_ratio)
        self._bottom_line = BandLineFinder(line_band_ratio,
                                           min_line_span_ratio,
                                           line_dist_check_ratio)
        # 全图霍夫内核：圆环模式实车验证参数原样（p1=210/p2=60/r[20,60]）
        self._ring_det = RingDetector(argparse.Namespace(
            scale=1.0, roi=False, roi_full_scan_interval=15,
            min_dist=60.0, min_r=20, max_r=60, dp=1.0,
            param1=210.0, param2=60.0, no_verify=False, coverage=0.7,
            merge_dist=20.0))
        self.clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8, 8))
        # 时域确认与平滑参数 = 2026-10-31 实机录像(5场景2680帧)网格寻优最优组：
        # ch=2 检出率99.2%/断流1帧/重捕获1帧（ch=8 时 94.7%/7帧/7帧）；
        # α=0.5/β=0.25 运动跟随 1.5px（0.3/0.15 为 2.2px），静止 std 1.3px 仍
        # 压缩原始霍夫噪声（原始 std 1.8）；yaw EMA 0.8 的 yaw 偏差 0.24°。
        # Assoc 60 须小于工位间距 152。
        self._tracker_kwargs = dict(
            assoc_dist=60.0, confirm_hits=int(confirm_hits), miss_tol=3,
            remove_misses=8, display_hold=2, smooth_w=0.5,
            smooth_mode='alphabeta', alpha=0.5, beta=0.25)
        self.reset()

    def reset(self):
        self._tracker = GroupTracker(**self._tracker_kwargs)
        self._yaw = None               # yaw EMA 状态（跳变>15°吸附）
        self._gaps = None              # (左间距, 右间距) 弱 EMA
        self._top_line.clear_seed()
        self._bottom_line.clear_seed()

    @staticmethod
    def _empty(state, reason):
        return dict(detected=False, stable=False, state=state,
                    middle_center_px=None, middle_radius_px=None,
                    station_yaw_deg=None, side_centers_inferred=None,
                    top_line_angle_deg=None, bottom_line_angle_deg=None,
                    line_source='none', failure_reason=reason)

    # ------------------------------------------------------------------
    # 主流程
    # ------------------------------------------------------------------
    def process(self, frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (5, 5), 0)
        enhanced = self.clahe.apply(blur)
        edges = cv2.Canny(blur, 30, 100)
        _, _, groups, _, _, _ = self._ring_det.detect(gray, enhanced)
        return self.consume(groups, edges, frame)

    def consume(self, groups, edges, frame):
        """从霍夫环组继续：时域确认→三环取中→基准线→推算侧站。

        独立成方法供离线回放调参与测试直接喂缓存组（groups 为
        RingDetector.detect 的第三返回值，edges 为 Canny(blur,30,100)）。"""
        annotated = frame.copy()

        # ---- 步骤1：时域确认 → 三环取中（霍夫环组由 process 缓存或外部喂入）----
        confirmed, candidates = self._tracker.update(groups)
        triple, relation, _ = select_three(
            confirmed, frame.shape[1], spacing_gate=False,
            min_span_ratio=self.min_span_ratio)
        for track in candidates:               # 未确认轨迹：暗黄细圈
            cv2.circle(annotated, (int(track['cx']), int(track['cy'])),
                       int(track['r_inner']), (0, 170, 170), 1)
        if triple is None:
            self._top_line.clear_seed()        # 丢失清线种子，防过时种子拉偏
            self._bottom_line.clear_seed()
            for track in confirmed:            # 已确认但未成三元组：暗黄绿圈
                cv2.circle(annotated, (int(track['cx']), int(track['cy'])),
                           int(track['r_inner']), (0, 220, 220), 1)
            cv2.putText(annotated, 'STACKED: SEARCHING', (10, 28),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 0, 255), 2,
                        cv2.LINE_AA)
            return self._empty('searching', '等待三环确认'), annotated

        middle = triple[1]                     # select_three 按 x 排序取中环
        cx, cy = middle['cx'], middle['cy']
        gap_left, gap_right = relation[0], relation[1]
        if self._gaps is None:
            self._gaps = (gap_left, gap_right)
        else:
            w = self.spacing_ema_w
            self._gaps = tuple(w * g + (1 - w) * e
                               for g, e in zip((gap_left, gap_right), self._gaps))
        offset = (self._gaps[0] + self._gaps[1]) / 4.0     # 75mm = 150mm/2
        diameter = 2.0 * middle['r_outer']
        prior = self._yaw if self._yaw is not None else relation[3]

        # ---- 步骤2：上下基准线定 yaw（line_yaw.BandLineFinder 共享模块）----
        top = self._top_line.find(edges, cx, cy, offset, diameter, prior, -1.0)
        bottom = self._bottom_line.find(edges, cx, cy, offset, diameter,
                                        prior, +1.0)
        top_span = top[2][1][0] - top[2][0][0] if top else 0.0
        bottom_span = bottom[2][1][0] - bottom[2][0][0] if bottom else 0.0
        if top and bottom and abs(top[1] - bottom[1]) <= self.line_disagreement:
            raw_yaw = (top[1] * top_span + bottom[1] * bottom_span) / max(
                1.0, top_span + bottom_span)
            line_source = 'dual'
        elif top or bottom:
            raw_yaw = (top if top else bottom)[1]
            line_source = 'single'
        else:
            raw_yaw = relation[3]              # 线全失：三环基线角兜底，不断流
            line_source = 'baseline'
        if top and bottom and abs(top[1] - bottom[1]) > self.line_disagreement:
            raw_yaw = relation[3]              # 方向冲突：回退基线角，不拒帧
            line_source = 'baseline'
        if self._yaw is None or abs(raw_yaw - self._yaw) > 15:
            self._yaw = raw_yaw
        else:
            self._yaw += self.yaw_ema_w * (raw_yaw - self._yaw)

        # ---- 步骤3：推算左右工位 ----
        theta = math.radians(self._yaw)
        sides = [
            {'x': round(cx - self._gaps[0] * math.cos(theta), 2),
             'y': round(cy - self._gaps[0] * math.sin(theta), 2),
             'source': 'inferred'},
            {'x': round(cx + self._gaps[1] * math.cos(theta), 2),
             'y': round(cy + self._gaps[1] * math.sin(theta), 2),
             'source': 'inferred'}]

        # ---- 标注与输出 ----
        cv2.circle(annotated, (int(cx), int(cy)), int(middle['r_inner']),
                   (0, 255, 0), 2)
        cv2.drawMarker(annotated, (int(cx), int(cy)), (255, 255, 0),
                       cv2.MARKER_CROSS, 12, 2)
        for line in (top, bottom):
            if line:
                cv2.line(annotated, *line[2], (255, 0, 255), 2)
        for side in sides:
            cv2.drawMarker(annotated, (round(side['x']), round(side['y'])),
                           (0, 165, 255), cv2.MARKER_CROSS, 16, 1)
        cv2.putText(annotated, f'STACKED TRACKING yaw={self._yaw:.1f}',
                    (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2,
                    cv2.LINE_AA)
        result = dict(detected=True, stable=False, state='tracking',
                      middle_center_px={'x': round(cx, 2), 'y': round(cy, 2)},
                      middle_radius_px=round(middle['r_inner'], 2),
                      station_yaw_deg=round(self._yaw, 2),
                      side_centers_inferred=sides,
                      top_line_angle_deg=round(top[1], 2) if top else None,
                      bottom_line_angle_deg=round(bottom[1], 2) if bottom else None,
                      line_source=line_source,
                      spacing_px=round(self._gaps[0] + self._gaps[1], 1),
                      failure_reason='')
        return result, annotated
