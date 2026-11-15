"""椭圆拟合圆环检测器(ellipse_v6 后端): cyqm 轮廓路线移植。

移植自 cyqm 开源工程 ring_station_detector.py 的检测内核:
    提取: 灰度→模糊→Canny→findContours(RETR_LIST)→每条≥min_points轮廓
          fitEllipse(长短轴/扁率过滤)——椭圆模型天然容忍相机倾斜;
    聚类: 并查集按圆心距并簇, "同心支持数≥N 且 直径档≥M"才成为候选——
          单圈杂物(车轮/圆标)凑不齐同心结构, 是防误识别的核心闸门;
    ROI:  成功后在结果包围盒+余量内找, 连续 K 次失败回全图。

与 cyqm 原版的差异(适配本管线):
    - 不做三选/毫米标定/底盘纠偏——三元组几何由下游 select_three 的
      152.1px 绝对间距先验把关, 输出保持像素制;
    - 簇内保留最内档半径作 r_inner(数字识别ROI用, 对齐霍夫路径语义);
    - 输出为管线标准 group 字典, 直接进 RingTracker/RingValidation。
"""
import math

import cv2
import numpy as np


def _find_root(parent, index):
    while parent[index] != index:
        parent[index] = parent[parent[index]]
        index = parent[index]
    return index


class EllipseRingDetector:
    def __init__(self, canny_low=60, canny_high=160,
                 min_contour_points=25, min_minor_axis_px=20.0,
                 max_major_axis_px=320.0, min_aspect=0.25,
                 cluster_radius_px=22.0, band_gap_px=5.0,
                 min_support=5, min_bands=2,
                 roi_margin_px=100, roi_fallback_after_failures=2):
        # min_support=5: 实测640x480实拍帧, 真环同心椭圆n=10~11,
        # 背景杂物(车轮/圆标)n<=3, 门槛5既杀杂物又留双倍余量(cyqm原版同值)。
        self.canny_low = int(canny_low)
        self.canny_high = int(canny_high)
        self.min_contour_points = int(min_contour_points)
        self.min_minor_axis_px = float(min_minor_axis_px)
        self.max_major_axis_px = float(max_major_axis_px)
        self.min_aspect = float(min_aspect)
        self.cluster_radius_px = float(cluster_radius_px)
        self.band_gap_px = float(band_gap_px)
        self.min_support = int(min_support)
        self.min_bands = int(min_bands)
        self.roi_margin_px = int(roi_margin_px)
        self.roi_fallback_after_failures = int(roi_fallback_after_failures)
        self.roi = None
        self.roi_failures = 0

    def reset(self):
        self.roi = None
        self.roi_failures = 0

    def _extract(self, bgr):
        gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (5, 5), 1.2)
        edges = cv2.Canny(blurred, self.canny_low, self.canny_high)
        contours, _ = cv2.findContours(edges, cv2.RETR_LIST,
                                       cv2.CHAIN_APPROX_NONE)
        observations = []
        for contour in contours:
            if len(contour) < self.min_contour_points:
                continue
            center, axes, angle = cv2.fitEllipse(contour)
            major, minor = max(axes), min(axes)
            if (minor < self.min_minor_axis_px or
                    major > self.max_major_axis_px or
                    minor / major < self.min_aspect):
                continue
            observations.append((float(center[0]), float(center[1]),
                                 float(major), float(minor)))
        return observations

    def _cluster(self, observations):
        parent = list(range(len(observations)))
        for first in range(len(observations)):
            for second in range(first):
                if (abs(observations[first][0] - observations[second][0]) ** 2 +
                        abs(observations[first][1] - observations[second][1]) ** 2
                        ) <= self.cluster_radius_px ** 2:
                    first_root, second_root = (_find_root(parent, first),
                                               _find_root(parent, second))
                    if first_root != second_root:
                        parent[second_root] = first_root
        groups = {}
        for index, observation in enumerate(observations):
            groups.setdefault(_find_root(parent, index), []).append(observation)
        candidates = []
        for members in groups.values():
            majors = sorted(item[2] for item in members)
            bands = []
            for diameter in majors:
                if not bands or diameter - bands[-1] > self.band_gap_px:
                    bands.append(diameter)
            if len(members) < self.min_support or len(bands) < self.min_bands:
                continue
            centers = np.asarray([[item[0], item[1]] for item in members])
            candidates.append({
                # 簇心中位数; 最内档半径作r_inner(数字ROI语义同霍夫路径)
                'x': round(float(np.median(centers[:, 0])), 1),
                'y': round(float(np.median(centers[:, 1])), 1),
                'r_inner': round(bands[0] * 0.5, 1),
                'r_outer': round(bands[-1] * 0.5, 1),
                'coverage': round(min(1.0, len(members) / max(1.0, self.min_support)), 3),
                'n': len(members),
            })
        candidates.sort(key=lambda g: (g['x'], g['y']))
        return candidates

    def _detect_region(self, frame):
        return self._cluster(self._extract(frame))

    def detect(self, frame):
        """返回 (groups, scope)。scope: ellipse / ellipse_roi / ellipse_full_fallback。"""
        if self.roi is None:
            groups = self._detect_region(frame)
            return groups, 'ellipse'
        x0, y0, x1, y1 = self.roi
        groups = self._detect_region(frame[y0:y1, x0:x1])
        if groups:
            for group in groups:
                group['x'] = round(group['x'] + x0, 1)
                group['y'] = round(group['y'] + y0, 1)
            self.roi_failures = 0
            return groups, 'ellipse_roi'
        self.roi_failures += 1
        if self.roi_failures < self.roi_fallback_after_failures:
            return [], 'ellipse_roi'
        self.roi_failures = 0
        return self._detect_region(frame), 'ellipse_full_fallback'

    def update_roi(self, groups, shape):
        """由本帧结果更新下一帧 ROI 包围盒(外半径+余量)。"""
        if not groups:
            return
        height, width = shape[:2]
        margin = self.roi_margin_px + max(g.get('r_outer', g['r_inner'])
                                          for g in groups)
        self.roi = (
            max(0, int(math.floor(min(g['x'] for g in groups) - margin))),
            max(0, int(math.floor(min(g['y'] for g in groups) - margin))),
            min(width, int(math.ceil(max(g['x'] for g in groups) + margin))),
            min(height, int(math.ceil(max(g['y'] for g in groups) + margin))),
        )
