#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
material_color_calib.py —— 物料六色 HSV 现场调阈值工具（无 ROS，cyqm 移植）

移植自 cyqm 开源 `颜色标定.py`，适配 640x480 去畸变画面 + 320px 自绘控制栏。
自绘滑条（而非 cv2.createTrackbar）的原因与 cyqm 相同：鼠标框选坐标与
去畸变图像像素一一对应，滑条不占窗口顶部。

操作（与 cyqm 一致）：
    1~6      选择 红/黄/蓝/绿/黑/浅蓝
    左键拖框 在物料内部拖出采样区域，自动统计 HSV 分布重估该色阈值
             （每色最多 20 个样本，可累积顶面/侧面/阴影）
    拖右侧滑条 人工微调 H/S/V 六值；H_LOW>H_HIGH 显示 WRAP（环绕，红色）
    u / c / r  撤销最近采样 / 清空该色采样 / 恢复出厂预设
    s        原子保存 JSON（保存后识别脚本重启即生效）
    q / ESC  退出（有未保存修改会提示）

注意：与主程序（logistics_vision_main.py）/其它采集脚本互斥抢相机，
运行前先停掉占用 /dev/video0 的进程（本工具启动时会自检提示）。

用法：
    python3 material_color_calib.py
    python3 material_color_calib.py --config ~/xxx.json
"""

import argparse
from copy import deepcopy
from datetime import datetime, timezone
import math
import os
import signal
import sys

import cv2
import numpy as np

from material_color_detect import (
    COLOR_SPECS,
    ColorConfigError,
    DEFAULT_CONFIG,
    PRESET_BOUNDS,
    _mask_from_hsv,
    bounds_to_ranges,
    build_default_color_config,
    estimate_hsv_bounds,
    load_color_config,
    ranges_to_bounds,
    save_color_config,
    validate_color_config,
)

WINDOW_NAME = 'Material HSV Calibrator'
SLIDER_NAMES = ('H_LOW', 'H_HIGH', 'S_LOW', 'S_HIGH', 'V_LOW', 'V_HIGH')
SLIDER_MAXIMA = (179, 179, 255, 255, 255, 255)
# 640x480 画面 + 右侧 320px 控制栏拼成 960x480（画面保持原始坐标不偏移）
IMAGE_WIDTH = 640
IMAGE_HEIGHT = 480
PANEL_WIDTH = 320
CANVAS_WIDTH = IMAGE_WIDTH + PANEL_WIDTH
SLIDER_X_MIN = IMAGE_WIDTH + 25
SLIDER_X_MAX = CANVAS_WIDTH - 25
SLIDER_Y_POSITIONS = (115, 165, 215, 265, 315, 365)


class CalibrationSession:
    """不依赖窗口的多颜色采样、撤销、重置和阈值状态（cyqm 原类移植）。"""

    MAX_SAMPLES_PER_COLOR = 20
    MAX_PIXELS_PER_SAMPLE = 50000

    def __init__(self, document: dict):
        self.document = deepcopy(validate_color_config(document))
        self.samples = {color_id: [] for color_id in COLOR_SPECS}
        self.baseline_bounds = {color_id: None for color_id in COLOR_SPECS}
        self.dirty = False

    def _item(self, color_id: int) -> dict:
        for item in self.document['colors']:
            if int(item['color_id']) == int(color_id):
                return item
        raise ColorConfigError(f'未知颜色：{color_id}')

    def bounds(self, color_id: int) -> tuple:
        return ranges_to_bounds(self._item(color_id)['ranges'])

    def set_bounds(self, color_id: int, bounds) -> None:
        ranges = bounds_to_ranges(bounds)
        if ranges != self._item(color_id)['ranges']:
            self._item(color_id)['ranges'] = ranges
            self.dirty = True

    def _recalculate(self, color_id: int) -> None:
        self.set_bounds(color_id,
                        estimate_hsv_bounds(self.samples[color_id], color_id))

    def add_sample(self, color_id: int, hsv_pixels: np.ndarray) -> None:
        color_id = int(color_id)
        if len(self.samples[color_id]) >= self.MAX_SAMPLES_PER_COLOR:
            raise ColorConfigError(
                f'每种颜色最多{self.MAX_SAMPLES_PER_COLOR}个样本')
        pixels = np.asarray(hsv_pixels, dtype=np.uint8).reshape(-1, 3)
        if len(pixels) < 100:
            raise ColorConfigError('框选区域太小，有效像素不足100')
        if len(pixels) > self.MAX_PIXELS_PER_SAMPLE:
            step = int(math.ceil(len(pixels) / self.MAX_PIXELS_PER_SAMPLE))
            pixels = pixels[::step]
        if not self.samples[color_id]:
            self.baseline_bounds[color_id] = self.bounds(color_id)
        self.samples[color_id].append(pixels)
        self._recalculate(color_id)

    def undo_sample(self, color_id: int) -> bool:
        color_id = int(color_id)
        if not self.samples[color_id]:
            return False
        self.samples[color_id].pop()
        if self.samples[color_id]:
            self._recalculate(color_id)
        else:
            baseline = self.baseline_bounds[color_id]
            if baseline is not None:
                self.set_bounds(color_id, baseline)
            self.baseline_bounds[color_id] = None
        return True

    def clear_samples(self, color_id: int) -> None:
        color_id = int(color_id)
        if self.samples[color_id]:
            baseline = self.baseline_bounds[color_id]
            self.samples[color_id].clear()
            if baseline is not None:
                self.set_bounds(color_id, baseline)
        self.baseline_bounds[color_id] = None

    def reset_to_preset(self, color_id: int) -> None:
        color_id = int(color_id)
        self.samples[color_id].clear()
        self.baseline_bounds[color_id] = None
        self.set_bounds(color_id, PRESET_BOUNDS[color_id])

    def mark_saved(self) -> None:
        self.dirty = False


class CalibratorWindow:
    """把标定会话连接到摄像头窗口、鼠标框选和右侧滑条（cyqm 原类移植）。"""

    def __init__(self, session: CalibrationSession, config_path: str):
        self.session = session
        self.config_path = config_path
        self.selected_color_id = 1
        self.current_frame = None
        self.frozen_frame = None
        self.drag_start = None
        self.drag_current = None
        self.active_slider_index = None
        self.sample_rectangles = {color_id: [] for color_id in COLOR_SPECS}
        self.status_message = 'Select color 1-6, then drag inside material'
        self.slider_values = self.session.bounds(self.selected_color_id)

    # ---- 滑条 ----
    def update_manual_thresholds(self) -> bool:
        values = tuple(int(value) for value in self.slider_values)
        if not (values[2] <= values[3] and values[4] <= values[5]):
            self.status_message = 'INVALID: S_LOW<=S_HIGH and V_LOW<=V_HIGH'
            return False
        self.session.set_bounds(self.selected_color_id, values)
        return True

    def _slider_index_at(self, x, y):
        """返回鼠标命中的右侧滑条编号，未命中返回 None。"""
        if not SLIDER_X_MIN - 12 <= x <= SLIDER_X_MAX + 12:
            return None
        for index, slider_y in enumerate(SLIDER_Y_POSITIONS):
            if abs(int(y) - slider_y) <= 22:
                return index
        return None

    def _set_slider_from_x(self, slider_index: int, x) -> None:
        x = max(SLIDER_X_MIN, min(SLIDER_X_MAX, int(x)))
        ratio = (x - SLIDER_X_MIN) / (SLIDER_X_MAX - SLIDER_X_MIN)
        maximum = SLIDER_MAXIMA[slider_index]
        values = list(self.slider_values)
        values[slider_index] = int(round(ratio * maximum))
        self.slider_values = tuple(values)
        if self.update_manual_thresholds():
            self.status_message = (
                f'{SLIDER_NAMES[slider_index]}={values[slider_index]}')

    def _sync_sliders(self) -> None:
        self.slider_values = self.session.bounds(self.selected_color_id)

    # ---- 鼠标框选采样 ----
    def _clamp_point(self, x, y):
        return (max(0, min(IMAGE_WIDTH - 1, int(x))),
                max(0, min(IMAGE_HEIGHT - 1, int(y))))

    def _normalised_rectangle(self):
        if self.drag_start is None or self.drag_current is None:
            return None
        x1, y1 = self.drag_start
        x2, y2 = self.drag_current
        return min(x1, x2), min(y1, y2), max(x1, x2), max(y1, y2)

    def _on_mouse(self, event, x, y, _flags, _userdata) -> None:
        if event == cv2.EVENT_LBUTTONDOWN:
            slider_index = self._slider_index_at(x, y)
            if slider_index is not None:
                self.active_slider_index = slider_index
                self._set_slider_from_x(slider_index, x)
                return
        if self.active_slider_index is not None:
            if event in (cv2.EVENT_MOUSEMOVE, cv2.EVENT_LBUTTONUP):
                self._set_slider_from_x(self.active_slider_index, x)
            if event == cv2.EVENT_LBUTTONUP:
                self.active_slider_index = None
            return
        # 右侧控制栏除滑条以外的区域不参与图像框选
        if x >= IMAGE_WIDTH and self.drag_start is None:
            return
        point = self._clamp_point(x, y)
        if event == cv2.EVENT_LBUTTONDOWN and self.current_frame is not None:
            self.frozen_frame = self.current_frame.copy()
            self.drag_start = point
            self.drag_current = point
        elif event == cv2.EVENT_MOUSEMOVE and self.drag_start is not None:
            self.drag_current = point
        elif event == cv2.EVENT_LBUTTONUP and self.drag_start is not None:
            self.drag_current = point
            rectangle = self._normalised_rectangle()
            try:
                self._accept_rectangle(rectangle)
            except ColorConfigError as error:
                self.status_message = f'Sample rejected: {error}'
            finally:
                self.drag_start = None
                self.drag_current = None
                self.frozen_frame = None

    def _accept_rectangle(self, rectangle) -> None:
        x1, y1, x2, y2 = rectangle
        if x2 - x1 < 10 or y2 - y1 < 10:
            raise ColorConfigError('ROI width and height must be at least 10px')
        roi = self.frozen_frame[y1:y2, x1:x2]
        hsv_pixels = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV).reshape(-1, 3)
        self.session.add_sample(self.selected_color_id, hsv_pixels)
        self.sample_rectangles[self.selected_color_id].append(rectangle)
        self._sync_sliders()
        count = len(self.session.samples[self.selected_color_id])
        self.status_message = f'Sample accepted: {count}/20'

    # ---- 按键动作 ----
    def select_color(self, color_id: int) -> None:
        self.selected_color_id = int(color_id)
        self._sync_sliders()
        name = COLOR_SPECS[self.selected_color_id][0]
        self.status_message = f'Selected {self.selected_color_id} {name}'

    def undo(self) -> None:
        if self.session.undo_sample(self.selected_color_id):
            self.sample_rectangles[self.selected_color_id].pop()
            self._sync_sliders()
            self.status_message = 'Last sample removed'
        else:
            self.status_message = 'No sample to undo'

    def clear(self) -> None:
        self.session.clear_samples(self.selected_color_id)
        self.sample_rectangles[self.selected_color_id].clear()
        self._sync_sliders()
        self.status_message = 'Current color samples cleared'

    def reset(self) -> None:
        self.session.reset_to_preset(self.selected_color_id)
        self.sample_rectangles[self.selected_color_id].clear()
        self._sync_sliders()
        self.status_message = 'Current color restored to preset'

    def save(self) -> None:
        if not self.update_manual_thresholds():
            raise ColorConfigError('滑条上下限无效，配置尚未保存')
        self.session.document['saved_at_utc'] = datetime.now(
            timezone.utc).isoformat()
        save_color_config(self.config_path, self.session.document)
        self.session.mark_saved()
        self.status_message = f'Saved: {self.config_path.split("/")[-1]}'

    # ---- 渲染 ----
    def _preview_ranges(self) -> list:
        values = self.slider_values
        if not (values[2] <= values[3] and values[4] <= values[5]):
            return []
        return bounds_to_ranges(values)

    def render(self, frame: np.ndarray) -> np.ndarray:
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = _mask_from_hsv(hsv, self._preview_ranges())
        # 未命中区域只保留 25% 亮度，命中像素保持原色，直观看出漏选和串色
        image_output = (frame.astype(np.float32) * 0.25).astype(np.uint8)
        image_output[mask > 0] = frame[mask > 0]
        color = COLOR_SPECS[self.selected_color_id][1]
        # 黑色本身适合做蒙版标签色，却不适合在暗色信息板上画文字和框
        interface_color = ((220, 220, 220)
                           if self.selected_color_id == 5 else color)
        for rectangle in self.sample_rectangles[self.selected_color_id]:
            x1, y1, x2, y2 = rectangle
            cv2.rectangle(image_output, (x1, y1), (x2, y2), interface_color, 2)
        active_rectangle = self._normalised_rectangle()
        if active_rectangle is not None:
            x1, y1, x2, y2 = active_rectangle
            cv2.rectangle(image_output, (x1, y1), (x2, y2), (255, 255, 255), 2)

        name = COLOR_SPECS[self.selected_color_id][0].upper()
        h_low, h_high, s_low, s_high, v_low, v_high = self.slider_values
        hue_text = f'H={h_low}..{h_high}' + (' WRAP' if h_low > h_high else '')
        state = 'DIRTY' if self.session.dirty else 'SAVED'
        canvas = np.full((IMAGE_HEIGHT, CANVAS_WIDTH, 3), (26, 26, 26),
                         dtype=np.uint8)
        canvas[:, :IMAGE_WIDTH] = image_output
        cv2.line(canvas, (IMAGE_WIDTH, 0), (IMAGE_WIDTH, IMAGE_HEIGHT - 1),
                 (90, 90, 90), 2)

        # 标题区（画面 480 高：标题 3 行 + 滑条 6 行 + 帮助 6 行刚好排下）
        title_lines = [
            f'Color {self.selected_color_id}: {name}',
            f'Samples={len(self.session.samples[self.selected_color_id])}/20 '
            f'State={state}',
            self.status_message[:32],
        ]
        for index, text in enumerate(title_lines):
            cv2.putText(canvas, text, (IMAGE_WIDTH + 14, 24 + index * 24),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.48,
                        (255, 255, 255) if index else interface_color, 1,
                        cv2.LINE_AA)

        for index, (slider_name, value, maximum, slider_y) in enumerate(zip(
                SLIDER_NAMES, self.slider_values, SLIDER_MAXIMA,
                SLIDER_Y_POSITIONS)):
            cv2.putText(canvas, f'{slider_name}: {value}',
                        (SLIDER_X_MIN, slider_y - 14),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.42, (220, 220, 220), 1,
                        cv2.LINE_AA)
            cv2.rectangle(canvas, (SLIDER_X_MIN, slider_y - 5),
                          (SLIDER_X_MAX, slider_y + 7), (70, 70, 70), -1)
            knob_x = int(round(SLIDER_X_MIN
                               + value / maximum
                               * (SLIDER_X_MAX - SLIDER_X_MIN)))
            cv2.rectangle(canvas, (SLIDER_X_MIN, slider_y - 5),
                          (knob_x, slider_y + 7), interface_color, -1)
            cv2.circle(canvas, (knob_x, slider_y + 1), 7, (255, 255, 255), -1,
                       cv2.LINE_AA)

        help_lines = [
            hue_text,
            f'S={s_low}..{s_high}  V={v_low}..{v_high}',
            '1-6: select color',
            'U: undo  C: clear  R: preset',
            'S: save  Q/Esc: quit',
            'Drag mouse inside material to sample',
        ]
        for index, text in enumerate(help_lines):
            cv2.putText(canvas, text, (IMAGE_WIDTH + 14, 400 + index * 13),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.38, (225, 225, 225), 1,
                        cv2.LINE_AA)
        return canvas


def parse_args():
    p = argparse.ArgumentParser(description='物料六色HSV现场标定工具(640x480)')
    p.add_argument('--device', default='/dev/video0')
    p.add_argument('--config', default=DEFAULT_CONFIG,
                   help='阈值 JSON 路径（默认设备侧数据目录）')
    p.add_argument('--calib', default=None,
                   help='相机标定 yaml（默认 camera_640x480.yaml）')
    return p.parse_args()


def run(args) -> int:
    # 相机延迟导入 + 占用自检：主程序在跑时给出明确提示而不是裸报错
    from camera_base import Camera, Undistorter, DEFAULT_CALIB
    try:
        cam = Camera(args.device, IMAGE_WIDTH, IMAGE_HEIGHT, 30)
    except RuntimeError as error:
        print(f'错误：{error}')
        print('提示：相机可能被占用。先停掉主程序/其它采集脚本：')
        print('       pgrep -af "logistics_vision_main|ring_hough"')
        return 1
    if (cam.real_w, cam.real_h) != (IMAGE_WIDTH, IMAGE_HEIGHT):
        print(f'错误：摄像头实际分辨率 {cam.real_w}x{cam.real_h} '
              f'≠ {IMAGE_WIDTH}x{IMAGE_HEIGHT}，请检查相机模式')
        cam.close()
        return 1
    und = Undistorter(args.calib or DEFAULT_CALIB, (cam.real_w, cam.real_h))

    config_path = os.path.expanduser(str(args.config))
    if os.path.isfile(config_path):
        document = load_color_config(config_path)
    else:
        print(f'[hint] 配置不存在，用出厂预设创建: {config_path}')
        document = build_default_color_config()
    session = CalibrationSession(document)
    interface = CalibratorWindow(session, config_path)
    cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_AUTOSIZE)
    cv2.setMouseCallback(WINDOW_NAME, interface._on_mouse)
    print('标定工具按键：1-6 选色；物料内左键拖框采样；u 撤销 c 清空 r 预设；'
          's 保存；q 退出')

    last_frame = None
    try:
        while True:
            frame = cam.read()
            if frame is not None:
                last_frame = und.remap(frame)
            if last_frame is None:
                if cv2.waitKey(10) & 0xFF in (ord('q'), 27):
                    break
                continue
            interface.current_frame = last_frame
            displayed = (interface.frozen_frame
                         if interface.frozen_frame is not None
                         else last_frame)
            cv2.imshow(WINDOW_NAME, interface.render(displayed))
            key = cv2.waitKey(1) & 0xFF
            if key in (ord('q'), ord('Q'), 27):
                if session.dirty:
                    print('未保存的HSV修改已放弃')
                break
            if ord('1') <= key <= ord('6'):
                interface.select_color(key - ord('0'))
            elif key in (ord('u'), ord('U')):
                interface.undo()
            elif key in (ord('c'), ord('C')):
                interface.clear()
            elif key in (ord('r'), ord('R')):
                interface.reset()
            elif key in (ord('s'), ord('S')):
                try:
                    interface.save()
                    print(f'颜色配置已保存：{interface.config_path}')
                except ColorConfigError as error:
                    interface.status_message = f'Save failed: {error}'
                    print(f'保存失败：{error}')
    finally:
        cam.close()
        cv2.destroyAllWindows()
    return 0


def main() -> int:
    args = parse_args()
    # SIGTERM/SIGINT 转 SystemExit：保证 run 的 finally 执行 cam.close()，
    # 防止强杀把 UVC 相机留在 streaming 状态导致僵死
    for sig in (signal.SIGTERM, signal.SIGINT):
        signal.signal(sig, lambda s, f: sys.exit(0))
    try:
        return run(args)
    except KeyboardInterrupt:
        print('\n已停止HSV标定工具')
        return 0
    except ColorConfigError as error:
        print(f'错误：{error}')
        return 1


if __name__ == '__main__':
    raise SystemExit(main())
