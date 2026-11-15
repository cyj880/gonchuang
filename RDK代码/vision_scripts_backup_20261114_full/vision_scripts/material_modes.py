#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""物料模式：HSV跟踪、跨颜色仲裁、固定抓取区+转盘运动状态机与连续流回传。"""

import math
import time

import cv2

from mode_worker import ModeWorker
from material_grasp import (GraspMotionJudge, GraspParams, STATE_NAMES,
                            STATE_MULTI, STATE_READY)
from material_color_detect import (COLOR_NAMES, ColorConfigError,
                                   annotate_blobs, draw_debug_label, find_material_blobs,
                                   load_color_config)


def bbox_iou(a, b):
    ax, ay, aw, ah = a
    bx, by, bw, bh = b
    overlap = max(0, min(ax + aw, bx + bw) - max(ax, bx)) * max(
        0, min(ay + ah, by + bh) - max(ay, by))
    return overlap / max(1, aw * ah + bw * bh - overlap)


def resolve_color_overlaps(blobs):
    """跨颜色 IoU>=0.5 连通簇仲裁；同分最高候选作为一处歧义显示。"""
    remaining = set(range(len(blobs)))
    accepted, ambiguous = [], []
    while remaining:
        first = min(remaining)
        remaining.remove(first)
        component, pending = [first], [first]
        while pending:
            current = blobs[pending.pop()]
            neighbors = [i for i in sorted(remaining)
                         if current.color_id != blobs[i].color_id
                         and bbox_iou(current.bbox, blobs[i].bbox) >= 0.5]
            remaining.difference_update(neighbors)
            component.extend(neighbors)
            pending.extend(neighbors)
        candidates = [blobs[i] for i in component]
        score = max(b.confidence for b in candidates)
        winners = [b for b in candidates
                   if math.isclose(b.confidence, score, rel_tol=0, abs_tol=1e-9)]
        if len(winners) == 1:
            accepted.append(winners[0])
        else:
            ambiguous.append(winners)
    accepted.sort(key=lambda b: (b.color_id, *b.center))
    return accepted, ambiguous


class MaterialModeWorker(ModeWorker):
    def __init__(self, und, config_path, track=False, grasp_params=None):
        self.mode = 'material_track' if track else 'material_all'
        super().__init__(self.mode + '-worker')
        self.und = und
        self.config_path = config_path
        self.track = track
        self.grasp_params = grasp_params or GraspParams()
        self._config = None
        self._config_error = ''
        self._clear_output()
        self._start_worker()

    def _clear_output(self):
        super()._clear_output()
        self.latest = {'status': 'WAITING', 'materials': [], 'ambiguous': [],
                       'target': None, 'track_color': None, 'elapsed_ms': 0.0,
                       'error': '', 'grasp_ready': False, 'grasp_center': None,
                       'state': 0, 'state_name': 'NOT_VISIBLE',
                       'speed_px_s': None, 'in_region': False,
                       'settled': False, 'generation': self.generation}

    def _reset_state(self, color=1):
        self._color = color
        if self.track:
            self._judge = GraspMotionJudge(self.grasp_params)
        self._stats_time = time.monotonic()
        self._stats_count = 0
        self._stats_ms = 0.0
        self._config_error = ''
        # 首次激活才读取；读取失败后重新进入模式可重试，不影响其余模式。
        if self._config is None:
            try:
                self._config = load_color_config(self.config_path)
                print(f'[{self.mode}] HSV配置: {self.config_path}')
            except ColorConfigError as exc:
                self._config_error = str(exc)
                print(f'[{self.mode}][错误] {exc}')

    def _draw_region(self, canvas):
        """抓取区可视化：画面中心矩形（--grasp-region-* 可调）。"""
        p = self.grasp_params
        x0 = int(round(p.region_cx - p.region_w / 2))
        y0 = int(round(p.region_cy - p.region_h / 2))
        x1 = int(round(p.region_cx + p.region_w / 2))
        y1 = int(round(p.region_cy + p.region_h / 2))
        cv2.rectangle(canvas, (x0, y0), (x1, y1), (255, 255, 0), 1, cv2.LINE_AA)
        cv2.putText(canvas, 'GRASP REGION', (x0 + 4, y0 + 16),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 0), 1,
                    cv2.LINE_AA)

    def _process(self, frame, source_ns, generation):
        started = time.monotonic()
        undist = self.und.remap(frame)
        canvas = undist.copy()
        if self.track:
            self._draw_region(canvas)
        if self._config_error:
            cv2.putText(canvas, 'HSV CONFIG ERROR - see terminal', (8, 100),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
            self._publish(generation, latest={
                'status': 'CONFIG_ERROR', 'materials': [], 'ambiguous': [],
                'target': None, 'track_color': self._color if self.track else None,
                'elapsed_ms': 0.0, 'error': self._config_error,
                'grasp_ready': False, 'generation': generation},
                latest_canvas=canvas, latest_source_ns=source_ns)
            return

        blobs, ambiguous = resolve_color_overlaps(
            find_material_blobs(undist, self._config))
        visible = ([b for b in blobs if b.color_id == self._color]
                   if self.track else blobs)
        annotate_blobs(canvas, visible)
        for group in ambiguous:
            x, y, w, h = group[0].bbox
            cv2.rectangle(canvas, (x, y), (x + w, y + h), (0, 165, 255), 2)
            cv2.putText(canvas, 'AMBIGUOUS', (x, max(14, y - 6)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 165, 255), 1)

        multi = False
        target = None
        in_region = []
        if self.track:
            # 区域优先选择：区内恰好1个→目标；≥2个→MULTI无效（用户决定）；
            # 区内0个→最大面积候选（仅追踪显示，不可能READY）
            in_region = [b for b in visible
                         if self.grasp_params.in_region(b.center)]
            multi = len(in_region) >= 2
            if multi:
                target = max(visible, key=lambda b: b.area_px) if visible else None
            elif in_region:
                target = in_region[0]
            else:
                target = max(visible, key=lambda b: b.area_px) if visible else None
            status = 'TRACKING' if visible else 'LOST'
        else:
            status = 'DETECTED' if blobs else 'NONE'
        if target is not None:
            center = tuple(round(v) for v in target.center)
            cv2.drawMarker(canvas, center, (255, 255, 255),
                           cv2.MARKER_CROSS, 24, 2)
            if multi:
                cv2.putText(canvas, 'MULTI', (center[0] + 14, center[1] - 14),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 165, 255), 1)
        grasp = {}
        if self.track:
            grasp = self._judge_result(target, in_region, multi, source_ns)
        counts = {cid: sum(b.color_id == cid for b in visible) for cid in COLOR_NAMES}
        hud = [('R%d Y%d B%d G%d K%d L%d  ambiguous=%d' % (
            *counts.values(), len(ambiguous)), 78)]
        if target is not None:
            hud.append(('TARGET (centroid)', 104))
        if self.track:
            hud.append((f"GRASP {grasp['state_name']}", 128))
            speed = grasp.get('speed_px_s')
            if speed is not None:
                hud.append((f'speed={speed:.1f}px/s', 152))
        for text, y in hud:
            draw_debug_label(canvas, text, (8, y))
        elapsed_ms = (time.monotonic() - started) * 1000
        result = {'status': status, 'materials': visible, 'ambiguous': ambiguous,
                  'target': target, 'track_color': self._color if self.track else None,
                  'elapsed_ms': elapsed_ms, 'error': '', 'generation': generation,
                  'grasp_ready': False, **grasp}
        if not self._publish(generation, latest=result, latest_canvas=canvas,
                             latest_source_ns=source_ns, last_error=''):
            return
        self._stats_count += 1
        self._stats_ms += elapsed_ms
        now = time.monotonic()
        if now - self._stats_time >= 5.0:
            center = None if target is None else tuple(round(v, 1) for v in target.center)
            print(f'[{self.mode}] fps={self._stats_count / (now - self._stats_time):.1f} '
                  f'ms={self._stats_ms / self._stats_count:.1f} counts={counts} '
                  f'ambiguous={len(ambiguous)} status={status} centroid={center} '
                  f'grasp={grasp.get("reason", "--")} center={grasp.get("grasp_center")}')
            self._stats_time, self._stats_count, self._stats_ms = now, 0, 0.0

    def _judge_result(self, target, in_region, multi, source_ns):
        """转盘运动状态机：multi/不可见/出区时测速对象与判定口径见类文档。"""
        if multi:
            measure, in_reg = None, False
        elif in_region:
            measure, in_reg = in_region[0], True
        else:
            # 区外目标也参与测速（提供转动证据）；触边距圈的由判定器剔除
            measure, in_reg = target, False
        state, speed = self._judge.update(source_ns / 1e9, measure, in_reg,
                                          multi)
        return {'grasp_ready': state == STATE_READY,
                'reason': STATE_NAMES[state],
                'state': state, 'state_name': STATE_NAMES[state],
                'speed_px_s': speed if measure is not None else None,
                'grasp_center': (tuple(float(v) for v in target.center)
                                 if target is not None and not multi else None),
                'in_region': bool(in_region),
                'settled': self._judge.settled,
                'track_color': self._color}
