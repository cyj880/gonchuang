#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""物料抓取判定：三抓取区（近端/左远端/右远端）+ 转盘运动状态机。

2026-11-15 三区扩展（cyqm 三槽位思路移植，用户定案）：
    - REGION 1=近端（原单抓取区）、2=左远端、3=右远端，优先级 1>2>3；
    - 逐区独立判定有效性：区内恰好 1 个目标色物料=有效候选，≥2=歧义
      区跳过；上报取优先级最高的有效区（0x11 REGION 字段，LEN=3）；
    - 判稳状态机不变（先见运动→转动确认→低速证据→settled）；
    - 乙路线：同色重臂只清低速证据/测速基线（soft_rearm），保留
      motion_confirmed——盘停后重发命令 ≈0.4s 重攒证据即 READY。

2026-10-31 修订（cyqm material_pickup_detector 状态机移植适配）：
停稳判断从"单目标矩心速度门"升级为状态机——固定抓取区（画面中心矩形，
--grasp-* 可调）+ "先见运动"（开机静止必须等转盘转过才允许判停，用户决定）
+ 低速证据窗。依据：规则《竞赛命题与运行》P4 转盘匀速 6~10 秒/圈
（⌀300mm），转动时物料 ~126-210px/s（0.5mm/px 折算），HSV 矩心噪声实测
≤25px/s——moving/near_stop 阈值为折算起点，待实机录像校准。

与 cyqm 的差异（适配单色追踪 + 部分视野）：
    - cyqm 对盘上全部物料取最大匹配速度、抓取区在盘缘不出画；本方案相机
      只见部分盘、物料会出入画，抓取判定只发生在目标位于区内时——区内
      必偏心，单目标速度即可信（盘转矩心必动），无需全场 max；
    - bbox 触及边距防护圈的物料视为正在出入画，不参与测速与判定；
    - 串口 0x11 帧（AA 55 11 03 STATE COLOR REGION，logistics_vision_main.
      encode_material_state_frame）：转盘停稳（settled）后 10Hz 持续
      上报——有有效区 STATE=5+REGION 可夹取、无有效区 STATE=1/0/6，
      其余静默。

STATE 枚举（encode_material_state_frame 使用）：
    0 NOT_VISIBLE   目标色物料不可见
    1 OUT_OF_REGION 可见但不在抓取区
    2 WAITING_MOTION 在区内但本周期未确认转动（禁止 READY）
    3 MOVING        转盘转动中
    4 SETTLING      减速中 / 低速证据积累
    5 READY         在区内低速证据满（可抓取）
    6 MULTI         区内同色多物料（无效，等待下一圈）
"""

from dataclasses import dataclass
import math

IMAGE_W, IMAGE_H = 640, 480

STATE_NOT_VISIBLE = 0
STATE_OUT_OF_REGION = 1
STATE_WAITING_MOTION = 2
STATE_MOVING = 3
STATE_SETTLING = 4
STATE_READY = 5
STATE_MULTI = 6
STATE_NAMES = ('NOT_VISIBLE', 'OUT_OF_REGION', 'WAIT_MOTION', 'MOVING',
               'SETTLING', 'READY', 'MULTI')

# 抓取区编号（0x11 REGION 字段；2026-11-15 用户定案）
REGION_NONE = 0
REGION_NEAR = 1     # 近端（原单抓取区）
REGION_LEFT = 2     # 左远端
REGION_RIGHT = 3    # 右远端
REGION_PRIORITY = (REGION_NEAR, REGION_LEFT, REGION_RIGHT)


@dataclass(frozen=True)
class GraspParams:
    # 近端抓取区（REGION 1；2026-11-15 用户定：中心 (320,320)，260x200）
    region_cx: float = 320.0
    region_cy: float = 320.0
    region_w: float = 260.0
    region_h: float = 200.0
    # 左远端抓取区（REGION 2；粗略初值，位置尺寸待现场微调）
    region2_cx: float = 150.0
    region2_cy: float = 160.0
    region2_w: float = 190.0
    region2_h: float = 160.0
    # 右远端抓取区（REGION 3；粗略初值，位置尺寸待现场微调）
    region3_cx: float = 490.0
    region3_cy: float = 160.0
    region3_w: float = 190.0
    region3_h: float = 160.0
    # 边距防护圈：bbox 触圈的物料视为正在出入画，不参与测速
    edge_margin_px: float = 20.0
    # 运动阈值（px/s）：规则 6~10s/圈折算起点，待实机录像校准
    moving_speed_px_s: float = 60.0
    near_stop_speed_px_s: float = 30.0
    required_moving_frames: int = 2
    required_low_speed_frames: int = 5
    # 测速帧间隔有效性门（断流后大间隔会低估速度，必须重建基线）
    min_frame_interval_s: float = 0.02
    max_frame_interval_s: float = 0.30
    # 连续流源帧新鲜度
    result_ttl_s: float = 0.3

    def __post_init__(self):
        values = vars(self)
        if not all(math.isfinite(v) and v >= 0 for v in values.values()):
            raise ValueError('抓取参数必须为有限非负数')
        for name in ('region_w', 'region_h', 'region2_w', 'region2_h',
                     'region3_w', 'region3_h', 'moving_speed_px_s',
                     'near_stop_speed_px_s', 'min_frame_interval_s',
                     'max_frame_interval_s'):
            if values[name] <= 0:
                raise ValueError(f'{name} 必须为正数')
        for name in ('required_moving_frames', 'required_low_speed_frames'):
            if type(values[name]) is not int or values[name] < 1:
                raise ValueError(f'{name} 必须为正整数')
        if self.near_stop_speed_px_s >= self.moving_speed_px_s:
            raise ValueError('near_stop_speed_px_s 必须小于 moving_speed_px_s')

    def in_region(self, center):
        cx, cy = float(center[0]), float(center[1])
        return (abs(cx - self.region_cx) <= self.region_w / 2 and
                abs(cy - self.region_cy) <= self.region_h / 2)

    def regions(self):
        """三个抓取区 (region_id, cx, cy, w, h)，按优先级 1>2>3 排列。"""
        return ((REGION_NEAR, self.region_cx, self.region_cy,
                 self.region_w, self.region_h),
                (REGION_LEFT, self.region2_cx, self.region2_cy,
                 self.region2_w, self.region2_h),
                (REGION_RIGHT, self.region3_cx, self.region3_cy,
                 self.region3_w, self.region3_h))

    def region_of(self, center):
        """点所在的抓取区编号（0=不在任何抓取区）。

        同点落在多个区时取优先级较高（编号较小）者。"""
        cx, cy = float(center[0]), float(center[1])
        for rid, rcx, rcy, rw, rh in self.regions():
            if abs(cx - rcx) <= rw / 2 and abs(cy - rcy) <= rh / 2:
                return rid
        return REGION_NONE


class GraspMotionJudge:
    """转盘运动状态机：先见运动 → 转动确认 → 低速证据 → 停稳。

    任一异常（丢目标/多物料/超速/间隔无效）清零低速证据与测速基线，
    但 motion_confirmed 保留（cyqm："重建测速基线，保留本周期已转动事实"）。
    出区不清零低速证据——停稳判定与区域无关（settled），READY 仍要求
    目标在区内。"""

    def __init__(self, params):
        self.p = params
        self.motion_confirmed = False
        self._reset_counters()

    def _reset_counters(self):
        self._moving_frames = 0
        self._low_speed_frames = 0
        self._previous = None      # {'time','cx','cy'} 测速基线

    def reset(self):
        self.motion_confirmed = False
        self._reset_counters()

    def soft_rearm(self):
        """同色重臂（乙路线，2026-11-15）：只清低速证据与测速基线，
        保留 motion_confirmed——盘已停时单片机重发命令后重新攒满低速
        证据（≈0.4s）即可 READY，无需再等"先见运动"；若重发时盘恰在
        转动，速度门照常挡住。换色/进入模式用 reset()（全清）。"""
        self._reset_counters()

    @property
    def settled(self):
        """本周期停稳判定：已确认转动且低速证据攒满（与区域无关）。

        供持续上报触发：区内停稳=READY（可夹取），区外停稳=抓取区内
        还没有目标物料。"""
        return self.motion_confirmed and \
            self._low_speed_frames >= self.p.required_low_speed_frames

    @staticmethod
    def _edge_guarded(params, blob):
        x, y, w, h = blob.bbox
        m = params.edge_margin_px
        return (x < m or y < m or x + w > IMAGE_W - m or
                y + h > IMAGE_H - m)

    def update(self, timestamp, blob, in_region, multi):
        """喂入一帧。blob=测速对象（multi/不可见时 None，边距判断在本类做）；
        in_region=blob 是否位于抓取区。返回 (state, speed)。"""
        p = self.p
        if multi:
            self._low_speed_frames = 0
            self._previous = None
            return STATE_MULTI, 0.0
        if blob is None:
            self._low_speed_frames = 0
            self._previous = None
            return STATE_NOT_VISIBLE, 0.0
        if self._edge_guarded(p, blob):
            # 正在出入画：质心受裁剪不连续跳变，测速不可信，只如实报区外
            self._low_speed_frames = 0
            self._previous = None
            return STATE_OUT_OF_REGION, 0.0
        cx, cy = float(blob.center[0]), float(blob.center[1])
        baseline_state = (STATE_WAITING_MOTION if in_region
                          else STATE_OUT_OF_REGION)
        if self._previous is None:
            self._previous = {'time': timestamp, 'cx': cx, 'cy': cy}
            if self.motion_confirmed:
                return STATE_SETTLING, 0.0   # 速度未明但已确认过转动
            return baseline_state, 0.0
        interval = timestamp - self._previous['time']
        if not (p.min_frame_interval_s <= interval <= p.max_frame_interval_s):
            self._reset_counters()
            self._previous = {'time': timestamp, 'cx': cx, 'cy': cy}
            state = STATE_SETTLING if self.motion_confirmed else baseline_state
            return state, 0.0
        speed = math.hypot(cx - self._previous['cx'],
                           cy - self._previous['cy']) / interval
        self._previous = {'time': timestamp, 'cx': cx, 'cy': cy}
        if speed >= p.moving_speed_px_s:
            self._moving_frames += 1
            self._low_speed_frames = 0
            if self._moving_frames >= p.required_moving_frames:
                self.motion_confirmed = True
            return STATE_MOVING, speed
        if not self.motion_confirmed:
            # 初始静止不能视为转动结束，必须先见过运动（cyqm 同款）
            self._moving_frames = 0
            self._low_speed_frames = 0
            return baseline_state, speed
        if not in_region:
            # 停稳判定与区域无关：区外慢速继续攒低速证据（供"区内还没有
            # 物料"的停稳上报），超速才清零；READY 仍要求目标在区内。
            if speed > p.near_stop_speed_px_s:
                self._low_speed_frames = 0
            else:
                self._low_speed_frames += 1
            return STATE_OUT_OF_REGION, speed
        if speed > p.near_stop_speed_px_s:
            self._low_speed_frames = 0
            return STATE_SETTLING, speed
        self._low_speed_frames += 1
        if self._low_speed_frames >= p.required_low_speed_frames:
            return STATE_READY, speed
        return STATE_SETTLING, speed
