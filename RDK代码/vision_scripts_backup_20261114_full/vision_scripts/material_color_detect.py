#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
material_color_detect.py —— 物料六色 HSV 阈值识别库（无 ROS，照 cyqm 移植）

方案来源：cyqm 开源 `颜色标定.py`（配置/掩膜/估阈值）+ `material_pickup_detector.py`
（分割过滤管线），适配 640x480。颜色编号与串口协议一致：
    1 red  2 yellow  3 blue  4 green  5 black  6 light_blue

识别管线（find_material_blobs）：
    BGR → 单次 HSV 转换 → 六色 inRange（Hue 环绕自动拆双区间）
    → 开(3)闭(7)椭圆核 → RETR_EXTERNAL 轮廓
    → 面积/最短边/长宽比/solidity/extent 五重过滤 → MaterialBlob 列表

注意：blob 的矩质心即物料追踪/抓取判定的位置输出（去畸变图像像素）。

阈值配置 JSON（save/load，本库与 material_color_calib.py 共用一份）：
    默认 ~/logistics_vision_data/color_model/material_color_thresholds.json
    现场调阈值用 material_color_calib.py，保存后本脚本重启即生效。

用法：
    # 库方式（后续 logistics_vision_main.py 的 material 模式直接 import）：
    import material_color_detect as mcd
    blobs = mcd.find_material_blobs(frame, config)

    # 独立调试：
    python3 material_color_detect.py --image test.png    # 离线单图
    python3 material_color_detect.py                     # 摄像头实时（先停主程序）
"""

import argparse
import json
import math
import os
import signal
import sys
import tempfile
import time
from dataclasses import dataclass, field
from typing import Iterable

import cv2
import numpy as np

DEFAULT_CONFIG = os.path.expanduser(
    '~/logistics_vision_data/color_model/material_color_thresholds.json')

# 颜色编号与 BGR 显示色（BGR 只用于画框和文字，与阈值无关）
COLOR_SPECS = {
    1: ('red', (0, 0, 255)),
    2: ('yellow', (0, 255, 255)),
    3: ('blue', (255, 0, 0)),
    4: ('green', (0, 255, 0)),
    5: ('black', (80, 80, 80)),
    6: ('light_blue', (255, 255, 0)),
}
COLOR_NAMES = {cid: name for cid, (name, _) in COLOR_SPECS.items()}

# 出厂预设（cyqm 2026-09-01 六件物料矫正画面实测值，HSV 与分辨率无关，
# 可直接迁移作起点；R 键恢复 / 配置缺失时构造默认配置）
PRESET_BOUNDS = {
    1: (172, 8, 150, 255, 90, 255),      # 红：Hue 跨 179→0 边界（WRAP）
    2: (20, 32, 170, 255, 120, 255),
    3: (104, 116, 160, 255, 80, 230),
    4: (63, 78, 110, 255, 80, 220),
    5: (0, 179, 0, 130, 0, 60),          # 黑：全 Hue，只约束 S/V 亮度
    6: (94, 104, 130, 255, 100, 255),
}

# 分割过滤参数：cyqm 1280x720 → 640x480 换算（面积×0.333、线性×0.667）
DEFAULT_DETECT_PARAMS = dict(
    processing_scale=1.0,
    min_area_px=600.0,          # cyqm 450×0.333
    max_area_px=15000.0,        # cyqm 26000×0.333（放宽到 15000）
    min_dimension_px=12.0,      # cyqm 18×0.667
    max_dimension_px=140.0,     # cyqm 210×0.667
    max_aspect_ratio=2.35,
    min_solidity=0.55,
    min_extent=0.28,
    open_kernel_px=3,           # cyqm 5×0.667
    close_kernel_px=7,          # cyqm 11×0.667
)


class ColorConfigError(ValueError):
    """颜色配置或用户采样不满足要求。"""


def _require(condition, message):
    if not condition:
        raise ColorConfigError(message)


# ---------------------------------------------------------------------------
# 滑条值 ↔ inRange 区间（Hue 是 0~179 的圆环，支持环绕）
# ---------------------------------------------------------------------------
def bounds_to_ranges(bounds: Iterable[int]) -> list:
    """六个滑条值 (H_LOW,H_HIGH,S_LOW,S_HIGH,V_LOW,V_HIGH) → 1~2 个 inRange 区间。

    H_LOW > H_HIGH 时范围跨越红色的 179→0 边界，拆成
    ``0..H_HIGH`` 与 ``H_LOW..179`` 两段（S/V 相同）。"""
    values = tuple(int(value) for value in bounds)
    _require(len(values) == 6, 'HSV滑条值必须正好包含6个整数')
    h_low, h_high, s_low, s_high, v_low, v_high = values
    _require(0 <= h_low <= 179 and 0 <= h_high <= 179, 'Hue必须位于0~179')
    _require(0 <= s_low <= s_high <= 255, 'S上下限无效')
    _require(0 <= v_low <= v_high <= 255, 'V上下限无效')
    if h_low <= h_high:
        return [{'lower': [h_low, s_low, v_low],
                 'upper': [h_high, s_high, v_high]}]
    return [{'lower': [0, s_low, v_low], 'upper': [h_high, s_high, v_high]},
            {'lower': [h_low, s_low, v_low], 'upper': [179, s_high, v_high]}]


def ranges_to_bounds(ranges: list) -> tuple:
    """配置区间恢复为滑条值，严格拒绝无法表达的组合（双区间必须合法环绕）。"""
    _require(isinstance(ranges, list) and 1 <= len(ranges) <= 2,
             '每种颜色必须包含1或2个HSV区间')

    def triplet(hsv_range, key):
        try:
            values = tuple(int(value) for value in hsv_range[key])
        except (KeyError, TypeError, ValueError) as error:
            raise ColorConfigError('HSV区间字段格式无效') from error
        _require(len(values) == 3, 'HSV上下限必须正好包含3个整数')
        return values

    if len(ranges) == 1:
        lower = triplet(ranges[0], 'lower')
        upper = triplet(ranges[0], 'upper')
        return (lower[0], upper[0], lower[1], upper[1], lower[2], upper[2])

    low_hue_part, high_hue_part = ranges
    first_lower = triplet(low_hue_part, 'lower')
    first_upper = triplet(low_hue_part, 'upper')
    second_lower = triplet(high_hue_part, 'lower')
    second_upper = triplet(high_hue_part, 'upper')
    _require(first_lower[0] == 0 and second_upper[0] == 179,
             '双HSV区间必须表示跨越Hue边界的范围')
    _require(second_lower[0] > first_upper[0],
             '双HSV区间必须在Hue边界两侧且不能互相重叠')
    _require(first_lower[1:] == second_lower[1:]
             and first_upper[1:] == second_upper[1:],
             '跨Hue边界的两个区间必须使用相同S/V范围')
    return (second_lower[0], first_upper[0],
            first_lower[1], first_upper[1],
            first_lower[2], first_upper[2])


# ---------------------------------------------------------------------------
# 配置构造 / 校验 / 加载 / 原子保存
# ---------------------------------------------------------------------------
def build_default_color_config() -> dict:
    """用出厂预设构造一份全新配置（配置文件缺失时的兜底）。"""
    colors = []
    for color_id, (name, display_bgr) in COLOR_SPECS.items():
        colors.append({'color_id': color_id, 'name': name,
                       'display_bgr': list(display_bgr),
                       'ranges': bounds_to_ranges(PRESET_BOUNDS[color_id])})
    return {'format_version': 1, 'color_space': 'OpenCV_HSV', 'colors': colors}


def validate_color_config(document: dict) -> dict:
    """校验公共配置格式（与分辨率无关），返回原字典供调用方继续使用。"""
    try:
        _require(isinstance(document, dict), '颜色配置根节点必须是对象')
        _require(int(document['format_version']) == 1, '不支持的颜色配置版本')
        _require(document['color_space'] == 'OpenCV_HSV',
                 '颜色空间必须为OpenCV_HSV')
        colors = document['colors']
        _require(isinstance(colors, list) and len(colors) == 6,
                 '颜色配置必须正好包含6种颜色')
        ids = {int(item['color_id']) for item in colors}
        _require(ids == set(COLOR_SPECS), '六色编号不完整或重复')
        for item in colors:
            color_id = int(item['color_id'])
            expected_name = COLOR_SPECS[color_id][0]
            _require(item['name'] == expected_name,
                     f'颜色{color_id}名称必须为{expected_name}')
            display_bgr = item['display_bgr']
            _require(len(display_bgr) == 3 and all(
                0 <= int(value) <= 255 for value in display_bgr),
                f'{expected_name}显示颜色无效')
            # ranges_to_bounds 同时验证双区间是否是合法的 Hue 环绕
            bounds = ranges_to_bounds(item['ranges'])
            bounds_to_ranges(bounds)
            for hsv_range in item['ranges']:
                lower = tuple(int(value) for value in hsv_range['lower'])
                upper = tuple(int(value) for value in hsv_range['upper'])
                _require(len(lower) == 3 and len(upper) == 3,
                         f'{expected_name} HSV区间长度无效')
                limits = (179, 255, 255)
                _require(all(0 <= lower[index] <= upper[index] <= limits[index]
                             for index in range(3)),
                         f'{expected_name} HSV区间越界')
    except (IndexError, KeyError, TypeError, ValueError) as error:
        if isinstance(error, ColorConfigError):
            raise
        raise ColorConfigError('颜色配置字段格式无效') from error
    return document


def load_color_config(path) -> dict:
    """读取并校验一份六色 HSV JSON 配置。"""
    resolved = os.path.abspath(os.path.expanduser(str(path)))
    try:
        with open(resolved, encoding='utf-8') as fh:
            document = json.load(fh)
    except OSError as error:
        raise ColorConfigError(f'无法读取颜色配置：{resolved}: {error}') from error
    except json.JSONDecodeError as error:
        raise ColorConfigError(f'颜色配置不是有效JSON：{resolved}') from error
    return validate_color_config(document)


def save_color_config(path, document: dict) -> None:
    """原子保存配置（tempfile + fsync + replace），避免中断留下半个文件。"""
    validate_color_config(document)
    resolved = os.path.abspath(os.path.expanduser(str(path)))
    _require(os.path.isdir(os.path.dirname(resolved)),
             f'配置目录不存在：{os.path.dirname(resolved)}')
    temporary_name = None
    try:
        with tempfile.NamedTemporaryFile(
                mode='w', encoding='utf-8', dir=os.path.dirname(resolved),
                prefix=f'.{os.path.basename(resolved)}.', suffix='.tmp',
                delete=False) as temporary:
            temporary_name = temporary.name
            json.dump(document, temporary, ensure_ascii=False, indent=2)
            temporary.write('\n')
            temporary.flush()
            os.fsync(temporary.fileno())
        # NamedTemporaryFile 默认 0600，统一改为普通配置权限供其他进程读取
        os.chmod(temporary_name, 0o644)
        os.replace(temporary_name, resolved)
    except OSError as error:
        if temporary_name is not None:
            try:
                os.unlink(temporary_name)
            except OSError:
                pass
        raise ColorConfigError(f'无法保存颜色配置：{resolved}: {error}') from error


def _find_color(document: dict, color) -> dict:
    if isinstance(color, str):
        matches = [item for item in document['colors'] if item['name'] == color]
    else:
        matches = [item for item in document['colors']
                   if int(item['color_id']) == int(color)]
    if len(matches) != 1:
        raise ColorConfigError(f'未知颜色：{color}')
    return matches[0]


# ---------------------------------------------------------------------------
# 掩膜生成（识别与调参工具共用）
# ---------------------------------------------------------------------------
def _mask_from_hsv(hsv: np.ndarray, ranges: list) -> np.ndarray:
    mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
    for hsv_range in ranges:
        lower = np.asarray(hsv_range['lower'], dtype=np.uint8)
        upper = np.asarray(hsv_range['upper'], dtype=np.uint8)
        mask = cv2.bitwise_or(mask, cv2.inRange(hsv, lower, upper))
    return mask


def make_color_mask(frame: np.ndarray, color, config: dict) -> np.ndarray:
    """使用公共配置从 BGR 图像生成指定颜色的单通道蒙版。"""
    validate_color_config(config)
    _require(frame is not None and frame.ndim == 3 and frame.shape[2] == 3,
             '颜色识别要求BGR三通道图像')
    item = _find_color(config, color)
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    return _mask_from_hsv(hsv, item['ranges'])


def make_all_color_masks(frame: np.ndarray, config: dict) -> dict:
    """只做一次 BGR→HSV 转换并生成全部六色蒙版，返回 {color_id: mask}。

    实时识别同时处理六色时必须用本接口，避免六次整图转换的开销。"""
    validate_color_config(config)
    _require(frame is not None and frame.ndim == 3 and frame.shape[2] == 3,
             '颜色识别要求BGR三通道图像')
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    return {int(item['color_id']): _mask_from_hsv(hsv, item['ranges'])
            for item in config['colors']}


# ---------------------------------------------------------------------------
# 采样自动估阈值（调参工具的"框选重估"核心）
# ---------------------------------------------------------------------------
def estimate_hsv_bounds(sample_arrays: Iterable[np.ndarray],
                        color_id: int) -> tuple:
    """根据同色多个 ROI 的 HSV 像素稳健估计六个滑条值。

    Hue 用圆形均值展开到均值附近再取 2%/98% 百分位 ±3（红色环绕不会被
    扩成全环）；S ±15、V ±20；黑色特殊：Hue 不稳定无分类意义，全 Hue
    开放，只用 S/V 的 98 分位 +20 约束亮度。"""
    arrays = [np.asarray(item, dtype=np.float64).reshape(-1, 3)
              for item in sample_arrays if np.asarray(item).size]
    _require(bool(arrays), '没有可用于统计的HSV样本')
    values = np.concatenate(arrays, axis=0)
    _require(len(values) >= 100, '有效HSV样本至少需要100个像素')

    saturation = values[:, 1]
    value = values[:, 2]
    if int(color_id) == 5:
        s_high = min(255, int(math.ceil(np.percentile(saturation, 98))) + 20)
        v_high = min(255, int(math.ceil(np.percentile(value, 98))) + 20)
        return (0, 179, 0, s_high, 0, v_high)

    hue = values[:, 0]
    angles = hue * (2.0 * math.pi / 180.0)
    mean_angle = math.atan2(np.mean(np.sin(angles)), np.mean(np.cos(angles)))
    center_hue = (mean_angle * 180.0 / (2.0 * math.pi)) % 180.0
    delta = ((hue - center_hue + 90.0) % 180.0) - 90.0
    h_low_unwrapped = math.floor(np.percentile(delta, 2)) + center_hue - 3
    h_high_unwrapped = math.ceil(np.percentile(delta, 98)) + center_hue + 3
    if h_high_unwrapped - h_low_unwrapped >= 179.0:
        h_low, h_high = 0, 179
    else:
        h_low = int(math.floor(h_low_unwrapped)) % 180
        h_high = int(math.ceil(h_high_unwrapped)) % 180

    s_low = max(0, int(math.floor(np.percentile(saturation, 2))) - 15)
    s_high = min(255, int(math.ceil(np.percentile(saturation, 98))) + 15)
    v_low = max(0, int(math.floor(np.percentile(value, 2))) - 20)
    v_high = min(255, int(math.ceil(np.percentile(value, 98))) + 20)
    return (h_low, h_high, s_low, s_high, v_low, v_high)


# ---------------------------------------------------------------------------
# 分割过滤管线（cyqm material_pickup_detector._find_regions 移植）
# ---------------------------------------------------------------------------
@dataclass
class MaterialBlob:
    """一个通过五重过滤的物料色块。矩质心即追踪/抓取判定的位置输出。"""
    color_id: int
    name: str
    center: tuple                 # (cx, cy) 轮廓矩质心（全图坐标）
    area_px: float
    confidence: float
    bbox: tuple                   # (x, y, w, h)
    contour: np.ndarray = field(repr=False, default=None)


def _odd_kernel(full_size: int, scale: float) -> int:
    value = max(1, int(round(full_size * scale)))
    return value if value % 2 else value + 1


def find_material_blobs(frame: np.ndarray, config: dict,
                        **param_overrides) -> list:
    """六色阈值分割 → 形态学 → 轮廓五重过滤 → MaterialBlob 列表。

    过滤（cyqm 原逻辑，参数默认值见 DEFAULT_DETECT_PARAMS，可按名覆盖）：
    面积窗、最短/最长边、长宽比、凸包 solidity、extent（轮廓面积/外接矩形）。
    置信度 = 0.55×solidity + 0.45×min(1, extent/0.75)。
    返回按 (color_id, 质心x, 质心y) 排序的列表。"""
    params = dict(DEFAULT_DETECT_PARAMS)
    params.update(param_overrides)
    validate_color_config(config)
    _require(frame is not None and frame.ndim == 3 and frame.shape[2] == 3,
             '颜色识别要求BGR三通道图像')
    scale = float(params['processing_scale'])
    if scale < 1.0:
        working = cv2.resize(frame, None, fx=scale, fy=scale,
                             interpolation=cv2.INTER_AREA)
    else:
        working = frame
    masks = make_all_color_masks(working, config)
    open_size = _odd_kernel(int(params['open_kernel_px']), scale)
    close_size = _odd_kernel(int(params['close_kernel_px']), scale)
    open_kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,
                                            (open_size, open_size))
    close_kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,
                                             (close_size, close_size))

    blobs = []
    scale_area = scale * scale
    for color_id, raw_mask in masks.items():
        mask = cv2.morphologyEx(raw_mask, cv2.MORPH_OPEN, open_kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, close_kernel)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                       cv2.CHAIN_APPROX_SIMPLE)
        for contour in contours:
            working_area = float(cv2.contourArea(contour))
            area = working_area / scale_area
            if not (float(params['min_area_px']) <= area
                    <= float(params['max_area_px'])):
                continue
            bx, by, bw, bh = cv2.boundingRect(contour)
            full_width, full_height = bw / scale, bh / scale
            if (min(full_width, full_height)
                    < float(params['min_dimension_px'])
                    or max(full_width, full_height)
                    > float(params['max_dimension_px'])):
                continue
            aspect = max(full_width, full_height) / max(
                1.0, min(full_width, full_height))
            if aspect > float(params['max_aspect_ratio']):
                continue
            hull_area = float(cv2.contourArea(cv2.convexHull(contour)))
            if hull_area <= 0.0:
                continue
            solidity = working_area / hull_area
            extent = working_area / max(1.0, float(bw * bh))
            if (solidity < float(params['min_solidity'])
                    or extent < float(params['min_extent'])):
                continue
            moments = cv2.moments(contour)
            if abs(moments['m00']) < 1e-6:
                continue
            center = (float(moments['m10'] / moments['m00']) / scale,
                      float(moments['m01'] / moments['m00']) / scale)
            confidence = max(0.0, min(
                1.0, 0.55 * solidity + 0.45 * min(1.0, extent / 0.75)))
            blobs.append(MaterialBlob(
                color_id=int(color_id), name=COLOR_NAMES[int(color_id)],
                center=center, area_px=area, confidence=confidence,
                bbox=(int(round(bx / scale)), int(round(by / scale)),
                      int(round(full_width)), int(round(full_height))),
                contour=np.rint(contour.astype(np.float32) / scale)
                .astype(np.int32)))
    blobs.sort(key=lambda item: (item.color_id, item.center[0],
                                 item.center[1]))
    return blobs


def draw_debug_label(canvas, text, origin, color=(255, 255, 255), scale=0.48):
    """ASCII调试文字加深色底，在浅色场地上也能辨认。"""
    width, height = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, scale, 1)[0]
    x, y = origin
    cv2.rectangle(canvas, (x, y - height - 2), (x + width + 2, y + 3),
                  (25, 25, 25), -1)
    cv2.putText(canvas, text, origin, cv2.FONT_HERSHEY_SIMPLEX,
                scale, color, 1, cv2.LINE_AA)


def annotate_blobs(canvas: np.ndarray, blobs: list) -> np.ndarray:
    """在 BGR 画布上画色块轮廓/外接框/标签（标签一律 ASCII，cv2.putText 限制）。"""
    for blob in blobs:
        color = COLOR_SPECS[blob.color_id][1]
        if blob.contour is not None:
            cv2.drawContours(canvas, [blob.contour], -1, color, 2)
        x, y, w, h = blob.bbox
        cv2.rectangle(canvas, (x, y), (x + w, y + h), color, 1)
        # blue/black不再共用bl缩写；两行显示避免面积/分数挤在颜色名后面。
        labels = (f'{blob.color_id}:{blob.name}',
                  f'{int(blob.area_px)}px2 {blob.confidence:.2f}')
        for row, label in enumerate(labels):
            text_width = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX,
                                         0.4, 1)[0][0]
            org = (max(0, min(x, canvas.shape[1] - text_width - 2)),
                   max(12, y - 22) + row * 15)
            text_color = (200, 200, 200) if blob.color_id == 5 else color
            draw_debug_label(canvas, label, org, text_color, 0.4)
    return canvas


def render_mask_preview(frame: np.ndarray, masks: dict) -> np.ndarray:
    """掩膜预览：命中像素平铺为该色显示色，其余压到 25% 亮度。"""
    preview = (frame.astype(np.float32) * 0.25).astype(np.uint8)
    for color_id, mask in masks.items():
        preview[mask > 0] = COLOR_SPECS[color_id][1]
    return preview


# ---------------------------------------------------------------------------
# 独立调试入口（离线单图 / 实时摄像头）
# ---------------------------------------------------------------------------
def run_image(args):
    config = load_color_config(args.config)
    frame = cv2.imread(args.image)
    if frame is None:
        print(f'无法读取图片: {args.image}')
        return 1
    blobs = find_material_blobs(frame, config, processing_scale=args.scale)
    counts = {cid: 0 for cid in COLOR_SPECS}
    for blob in blobs:
        counts[blob.color_id] += 1
        print(f'  {blob.name:<10s} area={blob.area_px:7.0f}px² '
              f'conf={blob.confidence:.2f} center=({blob.center[0]:.0f},'
              f'{blob.center[1]:.0f}) bbox={blob.bbox}')
    print(f'共 {len(blobs)} 个色块: '
          + ' '.join(f'{COLOR_NAMES[cid]}={counts[cid]}'
                     for cid in COLOR_SPECS))
    masks = make_all_color_masks(frame, config)
    canvas = render_mask_preview(frame, masks)
    annotate_blobs(canvas, blobs)
    cv2.imwrite('material_color_result.png', canvas)
    print('标注图已保存: material_color_result.png')
    return 0


def run_camera(args):
    # 相机模块延迟导入：库被主程序 import 时不引入采集依赖
    from camera_base import Camera, Undistorter, DEFAULT_CALIB
    cam = Camera(args.device, 640, 480, 30)
    und = Undistorter(args.calib or DEFAULT_CALIB, (cam.real_w, cam.real_h))
    config = load_color_config(args.config)
    print(f'[mat] 阈值配置: {args.config}')

    window = 'Material Color Detect'
    cv2.namedWindow(window, cv2.WINDOW_AUTOSIZE)
    print('[hint] q 退出，m 切换掩膜预览，s 截图')
    fps, last_t = 0.0, None
    total_ms = 0.0
    show_mask = False
    last_canvas = None
    stats_t, stats_count = None, 0
    try:
        while True:
            frame = cam.read()
            if frame is not None:
                t0 = cv2.getTickCount()
                undist = und.remap(frame)
                blobs = find_material_blobs(undist, config,
                                            processing_scale=args.scale)
                ms = (cv2.getTickCount() - t0) / cv2.getTickFrequency() * 1000
                total_ms = 0.9 * total_ms + 0.1 * ms if last_t else ms
                now = time.monotonic()
                if last_t is not None:
                    dt = now - last_t
                    fps = 0.9 * fps + 0.1 * (1.0 / dt) if dt > 0 else fps
                last_t = now

                masks = make_all_color_masks(undist, config)
                canvas = render_mask_preview(undist, masks) if show_mask \
                    else undist.copy()
                annotate_blobs(canvas, blobs)
                counts = {cid: 0 for cid in COLOR_SPECS}
                for blob in blobs:
                    counts[blob.color_id] += 1
                hud1 = (f'FPS {fps:.1f} total {total_ms:.1f}ms '
                        f'blobs {len(blobs)}')
                hud2 = 'R%d Y%d B%d G%d K%d L%d' % tuple(
                    counts[cid] for cid in sorted(counts))
                hud3 = 'q quit  m mask-preview  s snapshot'
                for i, line in enumerate((hud1, hud2, hud3)):
                    y = 20 + i * 20
                    cv2.putText(canvas, line, (8, y),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 0), 3,
                                cv2.LINE_AA)
                    cv2.putText(canvas, line, (8, y),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.45,
                                (0, 255, 255), 1, cv2.LINE_AA)
                last_canvas = canvas

                stats_count += 1
                if stats_t is None:
                    stats_t = now
                elif now - stats_t >= 5.0:
                    print(f'[MAT PERF] fps={fps:.1f} ms={total_ms:.1f} '
                          + ' '.join(f'{COLOR_NAMES[cid]}={counts[cid]}'
                                     for cid in sorted(counts)))
                    stats_t, stats_count = now, 0

            if last_canvas is not None:
                cv2.imshow(window, last_canvas)
            key = cv2.waitKey(1) & 0xFF
            if key in (ord('q'), 27):
                break
            if key == ord('m'):
                show_mask = not show_mask
                print(f'[hint] 掩膜预览 {"开启" if show_mask else "关闭"}')
            elif key == ord('s') and last_canvas is not None:
                name = time.strftime('mat_snapshot_%Y%m%d_%H%M%S.png')
                cv2.imwrite(name, last_canvas)
                print(f'[hint] 截图已保存: {name}')
    finally:
        cam.close()
        cv2.destroyAllWindows()


def parse_args():
    p = argparse.ArgumentParser(
        description='物料六色 HSV 阈值识别（cyqm 方案移植，640x480）')
    p.add_argument('--image', default=None, help='离线测试单张图')
    p.add_argument('--config', default=DEFAULT_CONFIG, help='阈值 JSON 路径')
    p.add_argument('--device', default='/dev/video0')
    p.add_argument('--calib', default=None,
                   help='相机标定 yaml（默认 camera_640x480.yaml）')
    p.add_argument('--scale', type=float, default=1.0,
                   help='处理缩放（0.5 可提速，实测慢时再用）')
    return p.parse_args()


def main():
    args = parse_args()
    # SIGTERM/SIGINT 转 SystemExit：保证 run_camera 的 finally 执行
    # cam.close()——被强杀时 UVC 相机会停在 streaming 状态导致僵死
    # （须重载 uvcvideo 驱动，与 ring_hough_standalone 同款保护）
    for sig in (signal.SIGTERM, signal.SIGINT):
        signal.signal(sig, lambda s, f: sys.exit(0))
    try:
        if args.image:
            return run_image(args)
        run_camera(args)
        return 0
    except KeyboardInterrupt:
        print('\n已停止')
        return 0
    except ColorConfigError as error:
        print(f'错误：{error}')
        return 1


if __name__ == '__main__':
    raise SystemExit(main())
