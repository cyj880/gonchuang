#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
logistics_vision_main.py —— 单进程多模式视觉主程序（无 ROS2）

定位：
    以 logistics_ws 独立圆环脚本（ring_hough_standalone.py，检测算法来源）
    为内核，移植 ROS2 桥接层（ring_serial_bridge_node / ring_serial_protocol /
    mode_gate）的语义，构成单进程多模式框架。实现 raw/ring/material_all/
    material_track/turntable/stacked 全部六模式。物料追踪以固定抓取区+
    转盘运动状态机判停稳（material_grasp.py），0x11 以 10Hz 连续流回传
    状态与矩质心（草案）。

线程架构（照抄 ROS 节点语义，线程替代节点）：
    Camera 线程 ──取帧──> FrameDispatcher ──投递给激活模式──> RingModeWorker
                                                                │ sites+yaw
    SerialLink 线程（RX 模式命令→ModeManager；TX 自愈）<────────┘
    RingPublisher 线程（30Hz ring 发送 / 过期空帧 / 防重发）

模式隔离（各模式互不影响的三重保障）：
    1. 每模式独立工作线程 + 独立最新帧信箱 + 独立状态对象，切换模式时
       stop 旧线程并清空其状态，不共享任何可变对象；
    2. 未激活模式收不到帧 → 零 CPU（等价 ROS 的停订语义）；
    3. 模式内部异常被 worker 捕获上报，不杀死相机线程/串口线程；
       串口故障自愈重开（告警一次），也不影响视觉。

串口协议（移植自 ring_serial_protocol.py，语义原样）：
    帧 = AA 55 + 命令 + 长度 + payload + 校验和 + 0D 0A；
    checksum = (命令+长度+payload和) & 0xFF；
    0x10 模式命令（STM32→视觉）：payload[0]=模式号（material_track 附颜色）；
    0x11 物料状态/夹取帧（视觉→STM32，三抓取区版 2026-11-15）：
         AA 55 11 03 STATE COLOR REGION CK 0D 0A（STATE 用 material_grasp
         枚举，5=READY；REGION 0=无/1=近端/2=左远端/3=右远端，仅 STATE=5
         有意义）；转盘确认转动且停稳后以 10Hz 持续发送——目标在优先级
         最高的有效区为 STATE=5+REGION（可夹取），无有效区为
         STATE=0/1/6+REGION=0；单片机收到首帧即夹取并关闭通道，多余帧
         被丢弃（对 RDK 无影响，RDK 不需要单片机任何回传）；未停稳静默；
         单片机抓取条件 = STATE==5 且 COLOR==期望，按 REGION 对应机构
         位姿；乙路线：每抓一次重发一条同色命令开新周期（同色重臂保留
         motion_confirmed）。
    0x13 已识别圆环结果（视觉→STM32）：
         payload = count + yaw_valid + yaw×100(int16) + 每环[id, x:u16, y:u16]，
         站点按 (x,y) 排序、最多 3 个、绝对 640x480 像素坐标。

ring 发送语义（逐条复刻 ring_serial_bridge_node）：
    仅 ring 模式发送；30Hz；结果时间戳超 0.3s 视为过期发空帧；ring_pending
    防止重发同一结果；乱序/过期/早于模式切换时刻的源帧拒收（不重臂）；
    写失败关串口自愈；串口不可用时视觉照常（仅告警一次）。

用法：
    python3 logistics_vision_main.py --start-mode ring            # 带串口闭环
    python3 logistics_vision_main.py --no-serial --display        # 纯视觉调试
    python3 logistics_vision_main.py --start-mode raw             # 上电静默（只收命令）
"""

import argparse
import math
import os
import struct
import sys
import threading
import time
from collections import deque

import cv2
import numpy as np

try:
    import serial
except ImportError:      # 串口不可用时视觉照常，仅告警一次
    serial = None

HERE = os.path.dirname(os.path.abspath(__file__))
if HERE not in sys.path:
    sys.path.insert(0, HERE)
from ring_hough_standalone import (Camera, Undistorter, RingDetector,
                                   GroupTracker, select_three)
from mode_worker import ModeWorker
from material_modes import MaterialModeWorker
from turntable_detector import TurntableDetector, focal_px_of
from material_grasp import GraspParams, STATE_READY
from material_color_detect import DEFAULT_CONFIG, COLOR_NAMES, draw_debug_label
from stacked_detector import StackedDetector
from line_yaw import BandLineFinder


# ---------------------------------------------------------------------------
# 串口协议（移植自 ring_serial_protocol.py，语义原样）
# ---------------------------------------------------------------------------
HEAD = b"\xAA\x55"
TAIL = b"\x0D\x0A"
COMMAND = 0x10
RING_RESULT = 0x12
RING_IDENTIFIED_RESULT = 0x13
MATERIAL_GRASP_RESULT = 0x11
STACKED_RESULT = 0x14
MATERIAL_TRACK_ACK = 0x18

RAW, RING = 0, 1
MATERIAL_ALL, MATERIAL_TRACK, TURNTABLE, STACKED = 2, 3, 4, 5

MODE_NAMES = {
    RAW: "raw", RING: "ring", MATERIAL_ALL: "material_all",
    MATERIAL_TRACK: "material_track", TURNTABLE: "turntable", STACKED: "stacked",
}
NAME_TO_MODE = {v: k for k, v in MODE_NAMES.items()}
IMAGE_W, IMAGE_H = 640, 480


def _checksum(command, payload):
    return (command + len(payload) + sum(payload)) & 0xFF


def encode_frame(command, payload=b""):
    if not 0 <= command <= 0xFF:
        raise ValueError("command must fit in one byte")
    if len(payload) > 0xFF:
        raise ValueError("payload is too long")
    return (HEAD + bytes((command, len(payload))) + payload
            + bytes((_checksum(command, payload),)) + TAIL)


def encode_mode(mode, color=None):
    """0x10 模式选择帧（STM32→视觉）。"""
    if mode not in MODE_NAMES:
        raise ValueError("unknown mode: %s" % mode)
    if mode == MATERIAL_TRACK:
        if color is None or not 1 <= int(color) <= 6:
            raise ValueError("material_track requires color 1..6")
        payload = bytes((mode, int(color)))
    elif color is not None:
        raise ValueError("color is only valid for material_track")
    else:
        payload = bytes((mode,))
    return encode_frame(COMMAND, payload)


def decode_mode(payload):
    if not payload or payload[0] not in MODE_NAMES:
        return None
    mode = payload[0]
    if mode == MATERIAL_TRACK:
        if len(payload) != 2 or not 1 <= payload[1] <= 6:
            return None
        return mode, payload[1]
    if len(payload) != 1:
        return None
    return mode, None


def mode_command(mode, color=None):
    decoded = decode_mode(bytes((mode,)) if color is None else
                          bytes((mode, color)))
    if decoded is None:
        raise ValueError("invalid mode payload")
    selected, selected_color = decoded
    command = MODE_NAMES[selected]
    if selected == MATERIAL_TRACK:
        command += " %d" % selected_color
    return command


def ring_yaw(sites):
    """站点基线角：右/下为正；单点无方向。"""
    points = sorted((float(s["x"]), float(s["y"])) for s in sites)
    if len(points) < 2 or points[0] == points[-1]:
        return None
    dx = points[-1][0] - points[0][0]
    dy = points[-1][1] - points[0][1]
    return round(math.degrees(math.atan2(dy, dx)), 2)


def encode_ring_result(sites, yaw=None):
    """0x12：0~3 环快照，绝对 640x480 像素，小端。

    yaw=None 时按站点坐标现算基线角（原行为）；显式传入（如基准线角）
    则覆盖——0x13 线角优先移植使用，帧格式不变。"""
    points = []
    for site in sites:
        x, y = float(site["x"]), float(site["y"])
        if not math.isfinite(x) or not math.isfinite(y) or \
                not (0 <= x < IMAGE_W and 0 <= y < IMAGE_H):
            raise ValueError("ring coordinate outside 640x480")
        points.append((min(639, int(round(x))), min(479, int(round(y)))))
    if len(points) > 3 or len(set(points)) != len(points):
        raise ValueError("expected at most three distinct ring centers")
    points.sort()
    if yaw is None:
        yaw = ring_yaw([{"x": x, "y": y} for x, y in points])
    payload = struct.pack("<BBh", len(points), int(yaw is not None),
                          0 if yaw is None else int(round(yaw * 100)))
    payload += b"".join(struct.pack("<HH", x, y) for x, y in points)
    return encode_frame(RING_RESULT, payload)


def encode_identified_ring_result(sites, yaw=None):
    """0x13：固定工位编号（0=未知），部分观测不重编号；yaw 显式传入时
    覆盖坐标现算值（圆环模式基准线角优先），None 保持原行为。"""
    sites = list(sites)
    legacy = encode_ring_result(sites, yaw)
    ordered = sorted(sites, key=lambda s: (min(639, int(round(float(s['x'])))),
                                           min(479, int(round(float(s['y']))))))
    payload = legacy[4:8]
    seen = set()
    for index, site in enumerate(ordered):
        identity = site.get('id', 0)
        if type(identity) is not int or not 0 <= identity <= 3:
            raise ValueError('ring id must be 0..3')
        if identity and identity in seen:
            raise ValueError('duplicate fixed ring id')
        seen.add(identity)
        payload += bytes((identity,)) + legacy[8 + 4 * index:12 + 4 * index]
    return encode_frame(RING_IDENTIFIED_RESULT, payload)


def encode_material_state_frame(state, color, region=0):
    """0x11：物料状态/夹取帧（视觉→STM32，三抓取区版，2026-11-15）。

    AA 55 11 03 STATE COLOR REGION CK 0D 0A（总长 10 字节，LEN=3）；
    STATE 用 material_grasp 状态枚举（5=READY），COLOR 为本周期跟踪
    颜色 1~6，REGION 为抓取区编号（0=无 / 1=近端 / 2=左远端 /
    3=右远端；仅 STATE=5 时有意义）。转盘停稳（settled）后以 10Hz
    持续发送：目标在优先级最高的有效区 → STATE=5+REGION（可夹取），
    无有效区 → STATE=0/1/6 且 REGION=0；单片机抓取条件 = STATE==5
    且 COLOR==期望，按 REGION 对应机构位姿（乙路线：每抓一次重发
    一条同色命令开新周期，同色重臂保留 motion_confirmed）。"""
    if not 0 <= int(state) <= 6:
        raise ValueError("state must fit material_grasp enum 0..6")
    if not 1 <= int(color) <= 6:
        raise ValueError("color must be 1..6")
    if not 0 <= int(region) <= 3:
        raise ValueError("region must be 0..3")
    payload = struct.pack('<BBB', int(state), int(color), int(region))
    return encode_frame(MATERIAL_GRASP_RESULT, payload)


def encode_material_track_ack(color):
    """0x18：追踪命令应答（视觉→STM32）。收到六色追踪命令之一并完成重臂
    （新周期已开启）后立即回一帧；每条命令（含同色重复）各回一帧，
    键盘/启动参数切换不回（仅串口命令回）。

    AA 55 18 01 COLOR CK 0D 0A，CK = (0x18 + 0x01 + COLOR) & 0xFF；
    COLOR = 已接受并进入追踪的颜色 1~6。单片机收到即知"命令已收到、
    已切换追踪色"，报告等待超时窗可从 ACK 到达起算。注意：抓取触发
    接收通道打开期间单片机解析器也要接受本帧（旧固件通道期只认 0x11
    会把 ACK 丢掉）。"""
    if not 1 <= int(color) <= 6:
        raise ValueError("color must be 1..6")
    return encode_frame(MATERIAL_TRACK_ACK, bytes((int(color),)))


def encode_stacked_result(result):
    """0x14：码垛连续流帧（草案，待下位机确认冻结）；未检出发 FLAGS=0 空帧。

    FLAGS bit0=detected（bit1 预留恒 0）；U/V=平滑中环圆心 u16 小端；
    YAW=station_yaw_deg×100 int16 小端。"""
    detected = bool(result.get('detected'))
    flags = 1 if detected else 0
    center = result.get('middle_center_px') or {}
    u = v = yaw = 0
    if detected and center:
        u = min(639, max(0, int(round(float(center.get('x', 0.0))))))
        v = min(479, max(0, int(round(float(center.get('y', 0.0))))))
    station_yaw = result.get('station_yaw_deg')
    if detected and station_yaw is not None:
        yaw = max(-32768, min(32767, int(round(float(station_yaw) * 100))))
    return encode_frame(STACKED_RESULT, struct.pack('<BHHh', flags, u, v, yaw))


class FrameParser:
    """STM32模式命令增量解析；合法长度只有1/2，保留跨间隔拆包。"""

    def __init__(self):
        self._buffer = bytearray()

    def feed(self, data):
        self._buffer.extend(data)
        frames = []
        while True:
            start = self._buffer.find(HEAD)
            if start < 0:
                if self._buffer[-1:] == HEAD[:1]:
                    self._buffer.clear()
                    self._buffer.extend(HEAD[:1])
                else:
                    self._buffer.clear()
                break
            if start:
                del self._buffer[:start]
            if len(self._buffer) < 4:
                break
            length = self._buffer[3]
            if self._buffer[2] != COMMAND or length not in (1, 2):
                del self._buffer[0]
                continue
            if len(self._buffer) >= 5:
                mode = self._buffer[4]
                if mode not in MODE_NAMES or length != (2 if mode == MATERIAL_TRACK else 1):
                    del self._buffer[0]
                    continue
            if length == 2 and len(self._buffer) >= 6 and not 1 <= self._buffer[5] <= 6:
                del self._buffer[0]
                continue
            total = 2 + 1 + 1 + length + 1 + len(TAIL)
            if len(self._buffer) < total:
                break
            frame = bytes(self._buffer[:total])
            command = frame[2]
            payload = frame[4:4 + length]
            if frame[-2:] != TAIL:
                del self._buffer[0]
                continue
            if frame[4 + length] != _checksum(command, payload):
                del self._buffer[0]
                continue
            del self._buffer[:total]
            frames.append((command, payload))
        return frames


# ---------------------------------------------------------------------------
# RingModeWorker：ring 模式工作线程（最新帧信箱 + 检测管道）
# ---------------------------------------------------------------------------
class RingModeWorker(ModeWorker):
    """圆环检测 + 码垛式基准线定 yaw（线角优先、基线角兜底，2026-11-14）。

    yaw 来源（码垛模式同款语义，line_yaw.BandLineFinder 共享模块）：
    三环 FULL 后在上侧（2026-10-27 实拍探针实锤：粗加工/暂存区仅地图
    边界侧黑线明显）距环心线半个实测间距（=75mm，理论 73.2px）处找
    边界线——线角优先，失线回退三环基线角；EMA 平滑 + 15° 跳变吸附。
    无冻结状态（offset 每帧由实测间距现算），原 LineAssist 的
    "学习垂距过时→永不重捕获"一类缺陷结构性不存在。"""

    # yaw EMA 新值权重（码垛录像寻优值移植；圆环现场偏滞后/偏抖可调）
    YAW_EMA_W = 0.8

    def __init__(self, und, det_args, blur_k=3):
        super().__init__('ring-worker')
        self.und = und
        self.det_args = det_args
        self.blur_k = blur_k
        self._clear_output()
        self._start_worker()

    def _clear_output(self):
        super()._clear_output()
        self.latest_sites = []
        self.latest_time = None
        self.latest_yaw = None
        self.latest_line_source = 'none'

    def _reset_state(self):
        self._tracker = GroupTracker(assoc_dist=60.0, confirm_hits=8,
                                     miss_tol=3, remove_misses=8,
                                     display_hold=2, smooth_w=0.5,
                                     smooth_mode='alphabeta')
        self._det = RingDetector(self.det_args)
        self._clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8, 8))
        self._line_finder = BandLineFinder()
        self._line_sign = -1.0     # 搜环心线上侧；上车朝向不同改此值
        self._yaw = None

    def _process(self, frame, source_ns, generation):
        # 去畸变先行：与独立脚本/物料/转盘路径及标定几何（间距152.1px等）
        # 的前提一致。预计算 map，每帧仅 remap（实测约 +5.4ms）。
        frame = self.und.remap(frame)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (self.blur_k, self.blur_k), 1.0)
        blur = self._clahe.apply(blur)
        # 线检测边缘图按码垛原配方（5x5 高斯 + Canny 30/100）单独计算，
        # 与 CLAHE 后的霍夫输入解耦，保证移植行为一致。
        edges = cv2.Canny(cv2.GaussianBlur(gray, (5, 5), 0), 30, 100)
        raw, verified, groups, region, t_hough, t_verify = \
            self._det.detect(gray, blur)
        confirmed, candidates = self._tracker.update(groups)
        triple, relation, rest = select_three(
            confirmed, gray.shape[1], expected_spacing=152.1,
            spacing_gate=False, min_span_ratio=0.35)
        canvas = cv2.cvtColor(blur, cv2.COLOR_GRAY2BGR)
        yaw_out = None
        line_source = 'none'
        if triple is not None:
            sites = []
            for i, t in enumerate(triple, start=1):
                t = dict(t)
                t['id'] = i
                sites.append({'x': t['cx'], 'y': t['cy'], 'id': i})
                cv2.circle(canvas, (int(t['cx']), int(t['cy'])),
                           int(t['r_inner']), (0, 255, 0), 2)
                cv2.drawMarker(canvas, (int(t['cx']), int(t['cy'])),
                               (255, 255, 0), cv2.MARKER_CROSS, 12, 2)
                cv2.putText(canvas, f'ID{i}', (int(t['cx']) - 12,
                                               int(t['cy']) - int(t['r_outer']) - 6),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
            # 基准线定 yaw：offset=半实测间距（75mm），线角优先，
            # 失线回退三环基线角（不断流）。
            cx, cy = triple[1]['cx'], triple[1]['cy']
            offset = (relation[0] + relation[1]) / 4.0
            diameter = 2.0 * triple[1]['r_outer']
            prior = self._yaw if self._yaw is not None else relation[3]
            line = self._line_finder.find(edges, cx, cy, offset, diameter,
                                          prior, self._line_sign)
            if line is not None:
                yaw_raw, line_source = line[1], 'line'
                cv2.line(canvas, *line[2], (255, 0, 255), 2)
            else:
                yaw_raw, line_source = relation[3], 'baseline'
            if self._yaw is None or abs(yaw_raw - self._yaw) > 15:
                self._yaw = yaw_raw
            else:
                self._yaw += self.YAW_EMA_W * (yaw_raw - self._yaw)
            yaw_out = self._yaw
        else:
            self._line_finder.clear_seed()   # 丢失清种子，防过时种子拉偏
            # PARTIAL/NONE：发已确认组的未知编号观测（协议允许 0=未知）；
            # yaw 维持原行为——latest_yaw=None，由编码器按 ≥2 点坐标现算。
            # 确认组 >3 超出 0x13 编码上限（≤3 且互异），按空观测上报——
            # 不编造数据，也不触发编码异常（历史漏洞：曾致发送线程死亡）。
            if len(confirmed) > 3:
                sites = []
            else:
                sites = [{'x': t['cx'], 'y': t['cy'], 'id': 0}
                         for t in confirmed]
            status = 'PARTIAL' if confirmed else 'NONE'
            cv2.putText(canvas, f'RING {status}', (10, 460),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 165, 255), 2)
        self._publish(generation, latest_sites=sites,
                      latest_time=time.monotonic(), latest_source_ns=source_ns,
                      latest_yaw=yaw_out, latest_line_source=line_source,
                      latest_canvas=canvas, last_error='')


# ---------------------------------------------------------------------------
# SerialLink：串口自愈线程（RX 模式命令；TX 由 send() 调用，锁保护）
# ---------------------------------------------------------------------------
class SerialLink(threading.Thread):
    def __init__(self, port, baudrate, on_mode):
        super().__init__(daemon=True, name='serial-link')
        self.port, self.baudrate = port, baudrate
        self.on_mode = on_mode              # 回调 mode_command 字符串
        self._lock = threading.Lock()
        self._stop_evt = threading.Event()
        self._handle = None
        self._parser = FrameParser()
        self._last_open_error = ''
        self.tx_count = 0
        self._last_rx_err = ''
        self._last_rx_err_t = 0.0

    def run(self):
        while not self._stop_evt.is_set():
            # 异常自愈：接收/模式切换链路上的未知异常只跳过本拍——
            # RX 线程退出 = 永久失聪（TX 照常，症状隐蔽且切模式自救
            # 依赖的正是本线程），与 publisher 同类漏洞统一防护。
            try:
                self._poll()                # RX：20ms 轮询（照搬 bridge）
            except Exception as exc:
                now = time.monotonic()
                if str(exc) != self._last_rx_err or \
                        now - self._last_rx_err_t >= 5.0:
                    print(f'[serial][警告] 接收处理异常（本拍跳过，'
                          f'线程继续）: {exc}')
                    self._last_rx_err, self._last_rx_err_t = str(exc), now
            time.sleep(0.02)

    def _open(self):
        if self._handle is not None or serial is None:
            return
        try:
            self._handle = serial.Serial(self.port, self.baudrate,
                                         timeout=0, write_timeout=0)
            self._last_open_error = ''
            print(f'[serial] 已连接 STM32 串口 {self.port} @ {self.baudrate}')
        except (serial.SerialException, OSError, ValueError) as exc:
            if str(exc) != self._last_open_error:
                print(f'[serial][警告] 打开串口失败: {exc}')
                self._last_open_error = str(exc)

    def _close(self):
        if self._handle is not None:
            try:
                self._handle.close()
            except (serial.SerialException, OSError):
                pass
        self._handle = None
        self._parser = FrameParser()

    def _poll(self):
        try:
            with self._lock:
                self._open()
                if self._handle is None:
                    return
                waiting = self._handle.in_waiting
                data = self._handle.read(waiting or 1)
        except (serial.SerialException, OSError) as exc:
            print(f'[serial][警告] 串口读取失败: {exc}')
            with self._lock:
                self._close()
            return
        # 无空闲重置（2026-11-15 修复随机丢指令漏洞）：一帧分多次到达
        # （任意间隔）都由缓冲拼接完成；假帧由长度上限/帧尾/校验和/
        # 逐字节滑动重同步四道防线挡住。原 50ms 空闲重置会把跨间隙的
        # 半帧整体丢弃（实测分片 80ms 指令蒸发、RDK 保持 raw），且
        # 违背"解析容忍噪声/拆包/粘包"的协议契约。
        for command, payload in self._parser.feed(data):
            if command != COMMAND:
                continue
            decoded = decode_mode(payload)
            if decoded is None:
                continue
            try:
                self.on_mode(mode_command(*decoded))
            except Exception as exc:
                # 一条命令失败不丢弃本次read中已经解析出的后续命令。
                print(f'[serial][警告] 模式命令处理失败: {exc}')
                continue
            if decoded[0] == MATERIAL_TRACK:
                # 命令应答（0x18）：重臂完成后立即回执，含同色重复命令
                self.send(encode_material_track_ack(decoded[1]))

    def send(self, frame):
        with self._lock:
            self._open()
            if self._handle is None:
                return False
            try:
                written = self._handle.write(frame)
                if written != len(frame):
                    raise serial.SerialTimeoutException(
                        f'partial write: {written}/{len(frame)}')
                self.tx_count += 1
                return True
            except (serial.SerialException, OSError, AttributeError) as exc:
                print(f'[serial][警告] 串口发送失败: {exc}')
                self._close()
                return False

    def stop(self):
        self._stop_evt.set()
        self.join(timeout=1.0)
        self._close()


# ---------------------------------------------------------------------------
# TurntableWorker：转盘外圆检测（移植 turntable_detect_node.py:144-422 算法）
# 流程：霍夫粗定位（0.35 缩放找 r∈[120,230]px 大圆）→ 极坐标径向扫描精修
# （MAD 稳健圆拟合 + 可见弧长校验 + 置信度）→ 5 帧中位数稳定判定。
# 未移植：_material_seed（Lab 前景粗种子，全局霍夫+精修路径可独立工作）。
# ---------------------------------------------------------------------------
class TurntableWorker(ModeWorker):
    """转盘模式只负责去畸变和发布，算法实例与物料模式相互独立。"""
    def __init__(self, und, blur_k=5):
        super().__init__('turntable-worker')
        self.und = und
        self._clear_output()
        self._start_worker()

    def _clear_output(self):
        super()._clear_output()
        self.latest = {'detected': False, 'center_u': None, 'center_v': None,
                       'radius_px': None, 'stable': False,
                       'confidence': 0.0, 'tracking_mode': 'search'}

    def _reset_state(self):
        # 半径窗由相机高度带推导（r = f×150/Z），不写死像素
        self._detector = TurntableDetector(focal_px=focal_px_of(self.und))
        # 单帧检测失败保持额度：静止盘上 33ms 陈旧输出无影响，
        # 连续失败立即如实置空（防止画面状态绿/黄/红闪烁）。
        self._hold = 1
        self._last_latest = None
        self._last_canvas = None

    def _process(self, frame, source_ns, generation):
        best, mode, canvas = self._detector.detect(self.und.remap(frame))
        if best is None and self._hold > 0 and self._last_latest is not None:
            self._hold -= 1
            self._publish(generation, latest=self._last_latest,
                          latest_canvas=self._last_canvas,
                          latest_source_ns=source_ns, last_error='')
            return
        self._hold = 1
        latest = {'detected': best is not None,
                  'center_u': best['center_u'] if best else None,
                  'center_v': best['center_v'] if best else None,
                  'radius_px': best['radius_px'] if best else None,
                  'stable': best['stable'] if best else False,
                  'confidence': best['confidence'] if best else 0.0,
                  'tracking_mode': mode}
        self._last_latest = dict(latest)
        self._last_canvas = canvas
        self._publish(generation, latest=latest, latest_canvas=canvas,
                      latest_source_ns=source_ns, last_error='')


# ---------------------------------------------------------------------------
# StackedModeWorker：码垛模式工作线程（全图霍夫三环取中 + 基准线 + 推算侧站）
# 算法与设计决定见 stacked_detector.py / 交接文档 §15。
# ---------------------------------------------------------------------------
class StackedModeWorker(ModeWorker):
    """码垛模式只负责去畸变和发布，算法实例独立。"""
    def __init__(self, und):
        super().__init__('stacked-worker')
        self.und = und
        self._clear_output()
        self._start_worker()

    def _clear_output(self):
        super()._clear_output()
        self.latest = {'detected': False, 'stable': False, 'state': 'WAITING',
                       'middle_center_px': None, 'station_yaw_deg': None,
                       'side_centers_inferred': None, 'failure_reason': '',
                       'tracking_mode': 'search'}

    def _reset_state(self):
        self._detector = StackedDetector()

    def _process(self, frame, source_ns, generation):
        result, canvas = self._detector.process(self.und.remap(frame))
        latest = dict(result)
        latest.setdefault('tracking_mode', result.get('state', 'search'))
        self._publish(generation, latest=latest, latest_canvas=canvas,
                      latest_source_ns=source_ns, last_error='')


# ---------------------------------------------------------------------------
# RingPublisher → VisionPublisher：各模式各自的 TX 语义
# ---------------------------------------------------------------------------
LEGACY_HEAD = 0x66
LEGACY_CMD_CENTER = 0x01
LEGACY_TAIL = 0x77


def legacy_center_frame(center):
    """转盘旧版中心帧（视觉→STM32）：66 01 <x:int16大端> <y:int16大端> 77。"""
    x, y = center
    payload = struct.pack(">hh", int(x), int(y))
    return bytes((LEGACY_HEAD, LEGACY_CMD_CENTER)) + payload + \
        bytes((LEGACY_TAIL,))


class VisionPublisher(threading.Thread):
    """ring 30Hz 0x13 / turntable 10Hz旧帧 / track 停稳期间10Hz持续上报
    STATE/COLOR/REGION / stacked 10Hz 0x14。"""

    def __init__(self, serial_link, workers, mode_manager):
        super().__init__(daemon=True, name='vision-publisher')
        self.link = serial_link
        self.workers = workers
        self.manager = mode_manager
        self.ring_period = 1.0 / 30.0
        self.turntable_period = 1.0 / 10.0
        self.stacked_period = 1.0 / 10.0
        self.material_period = 1.0 / 10.0
        self._stop_evt = threading.Event()
        self._pending = False
        self._latest_frame = encode_identified_ring_result([])
        self._latest_time = None
        self._last_source_ns = 0
        self._mode_since = time.monotonic()
        self._mode_since_ns = time.monotonic_ns()
        self._tx_ages = deque(maxlen=512)
        self._stats_time = time.monotonic()
        self._next_turntable_tx = 0.0
        self._next_stacked_tx = 0.0
        self._tt_reported = False
        self._next_material_tx = 0.0
        self._last_warn = ''
        self._last_warn_t = 0.0

    def _warn_tx(self, mode, exc):
        """发送异常节流告警：同一原因 5s 内只打一次。"""
        now = time.monotonic()
        msg = f'{mode}: {exc}'
        if msg != self._last_warn or now - self._last_warn_t >= 5.0:
            print(f'[publisher][警告] {mode} 发送异常（本拍跳过，'
                  f'线程继续）: {exc}')
            self._last_warn, self._last_warn_t = msg, now

    def on_mode_changed(self):
        """模式切换：清空缓存结果（复刻 bridge 语义，防旧结果泄漏新模式）。"""
        self._latest_frame = encode_identified_ring_result([])
        self._latest_time = None
        self._pending = False
        self._last_source_ns = 0
        self._mode_since = time.monotonic()
        self._mode_since_ns = time.monotonic_ns()
        self._tt_reported = False
        self._next_material_tx = 0.0

    def run(self):
        while not self._stop_evt.is_set():
            loop_start = time.monotonic()
            # 切换与发送互斥，避免已切到物料/raw后仍发送一次旧模式结果。
            with self.manager._lock:
                mode = self.manager.active_mode
                if self.link is not None:
                    # 异常自愈：单个坏结果（如编码契约 violation）只跳过
                    # 本拍，发送线程绝不退出——退出即该模式永久静默且
                    # 切模式无法恢复（2026-11-13 实测复现的漏洞）。
                    try:
                        if mode == 'ring':
                            self._publish_ring()
                        elif mode == 'turntable':
                            self._publish_turntable()
                        elif mode == 'stacked':
                            self._publish_stacked()
                        elif mode.startswith('material_track '):
                            self._publish_material()
                    except Exception as exc:
                        self._warn_tx(mode, exc)
            self._stop_evt.wait(max(
                0.0, min(self.ring_period, self.turntable_period) -
                (time.monotonic() - loop_start)))

    def _publish_ring(self):
        # 从 worker 取最新结果（校验：过期/乱序/早于切换时刻拒收）
        with self.workers['ring'].lock:
            sites = list(self.workers['ring'].latest_sites)
            source_ns = self.workers['ring'].latest_source_ns
            yaw = self.workers['ring'].latest_yaw
        valid = self._source_fresh(source_ns) and source_ns > self._last_source_ns
        if valid:
            # yaw：worker 发布值（FULL=线角/基线角 EMA）优先，PARTIAL 为
            # None 时编码器按站点坐标现算（原行为）。
            # 防御性清洗（保持原站点对象与 id 不变）：丢弃取整后重复/
            # 越界/非有限坐标的站点并上限 3——保证 0x13 编码契约
            # （≤3 且互异且在 640x480 内）恒成立，编码不可能抛异常。
            seen, clean = set(), []
            for s in sites:
                try:
                    px, py = int(round(float(s['x']))), int(round(float(s['y'])))
                except (TypeError, ValueError, OverflowError):
                    continue
                if not (0 <= px < IMAGE_W and 0 <= py < IMAGE_H):
                    continue
                if (px, py) in seen:
                    continue
                seen.add((px, py))
                clean.append(s)
                if len(clean) == 3:
                    break
            self._latest_frame = encode_identified_ring_result(clean, yaw)
            self._latest_time = time.monotonic()
            self._last_source_ns = source_ns
            self._pending = True
        fresh = self._source_fresh(self._last_source_ns)
        if not fresh:
            self.link.send(encode_identified_ring_result([]))
        elif self._pending:
            if self.link.send(self._latest_frame):
                self._tx_ages.append((time.monotonic_ns() - self._last_source_ns) / 1e6)
                self._pending = False
        now = time.monotonic()
        if now - self._stats_time >= 5.0:
            ages = sorted(self._tx_ages)
            if ages:
                print(f'[RING TX] source_age_p95='
                      f'{ages[int((len(ages) - 1) * 0.95)]:.1f}ms '
                      f'samples={len(ages)}')
            self._tx_ages.clear()
            self._stats_time = now

    def _source_fresh(self, source_ns):
        return (type(source_ns) is int and source_ns > 0 and
                source_ns >= self._mode_since_ns and
                0 <= time.monotonic_ns() - source_ns <= 300_000_000)

    def _publish_turntable(self):
        now = time.monotonic()
        if now < self._next_turntable_tx:
            return
        self._next_turntable_tx = now + self.turntable_period
        with self.workers['turntable'].lock:
            latest = dict(self.workers['turntable'].latest)
            source = self.workers['turntable'].latest_source_ns
        if (self._source_fresh(source) and latest.get('detected') and
                latest.get('center_u') is not None):
            frame = legacy_center_frame((latest['center_u'],
                                         latest['center_v']))
            self.link.send(frame)
            if not self._tt_reported:
                print(f'[TURNTABLE TX] 首帧: {frame.hex(" ").upper()}')
                self._tt_reported = True

    def _publish_material(self):
        """0x11 停稳期间 10Hz 持续发送（三抓取区版）：

        转盘确认转动且低速证据攒满（停稳，与目标是否在区内无关）后持续
        发送：目标在优先级最高的有效区 → STATE=5+REGION（可夹取，乙路线
        下单片机抓完重发同色命令开新周期）；无有效区 → STATE=0/1/6 且
        REGION=0。单片机收到首帧即夹取并关闭通道，后续重复帧被丢弃——
        对 RDK 无影响，RDK 不需要单片机任何回传。物料被夹走/盘再转动
        （未停稳）即静默，恢复停稳自动继续；写失败下个节拍用新鲜帧重试；
        未停稳/全部歧义/目标色不可见静默。"""
        now = time.monotonic()
        if now < self._next_material_tx:
            return
        self._next_material_tx = now + self.material_period
        color = int(self.manager.active_mode.split()[1])
        worker = self.workers['material_track']
        with worker.lock:
            result = dict(worker.latest)
            source = worker.latest_source_ns
        if (not result.get('settled') or
                result.get('generation') != worker.generation or
                result.get('track_color') != color or
                source < self._mode_since_ns or
                not 0 <= time.monotonic_ns() - source <=
                worker.grasp_params.result_ttl_s * 1e9):
            return
        state = STATE_READY if result.get('grasp_ready') else \
            int(result.get('state', 0))
        region = int(result.get('region', 0) or 0)
        self.link.send(encode_material_state_frame(state, color, region))

    def _publish_stacked(self):
        """码垛连续流: 10Hz 每周期发一帧, 未检测发 FLAGS=0 空帧(0x14 草案)。"""
        now = time.monotonic()
        if now < self._next_stacked_tx:
            return
        self._next_stacked_tx = now + self.stacked_period
        with self.workers['stacked'].lock:
            result = dict(self.workers['stacked'].latest)
            source = self.workers['stacked'].latest_source_ns
        if not self._source_fresh(source):
            result = {'detected': False}
        self.link.send(encode_stacked_result(result))

    def stop(self):
        self._stop_evt.set()
        self.join(timeout=1.0)


# ---------------------------------------------------------------------------
# ModeManager：active_mode 状态机（模式切换互不影响的核心）
# ---------------------------------------------------------------------------
class ModeManager:
    def __init__(self, start_mode, workers, publisher, serial_enabled,
                 track_color=1):
        self.workers = workers
        self.publisher = publisher
        self.serial_enabled = serial_enabled
        self.track_color = track_color
        self._lock = threading.Lock()
        self.implemented = set(workers.keys())
        self.active_mode = self._normalize(start_mode)
        self._activate(self.active_mode)
        print(f'[mode] 初始模式: {self.active_mode}'
              + ('' if serial_enabled else '（串口已禁用，仅视觉）'))

    def _normalize(self, mode_str):
        parts = mode_str.split()
        if not parts or parts[0] not in NAME_TO_MODE:
            raise ValueError('unknown mode: ' + mode_str)
        if parts[0] == 'material_track':
            if len(parts) > 2:
                raise ValueError('material_track requires one color')
            color = int(parts[1]) if len(parts) == 2 else self.track_color
            if color not in COLOR_NAMES:
                raise ValueError('track color must be 1..6')
            return f'material_track {color}'
        if len(parts) != 1:
            raise ValueError('color is only valid for material_track')
        return parts[0]

    def _activate(self, mode_str, preserve_motion=False):
        parts = mode_str.split()
        worker = self.workers.get(parts[0])
        if parts[0] == 'material_track':
            self.track_color = int(parts[1])
            if worker is not None:
                worker.start_mode(color=self.track_color,
                                  preserve_motion=preserve_motion)
        elif worker is not None:
            worker.start_mode()

    def request_mode(self, mode_str):
        """串口/键盘共用入口；material_track 命令重臂新周期——同色重发
        为软重臂（清低速证据/测速基线/区域选择，保留 motion_confirmed，
        乙路线：盘停后重发 ≈0.4s 重攒证据即可 READY）；换色或从其他模式
        进入为全清重臂（先见运动重新要求）。"""
        with self._lock:
            mode_str = self._normalize(mode_str)
            base = mode_str.split()[0]
            if self.active_mode == mode_str and base != 'material_track':
                return
            old = self.active_mode
            old_parts, new_parts = old.split(), mode_str.split()
            rearm = (base == 'material_track'
                     and old_parts[0] == 'material_track'
                     and len(old_parts) == 2
                     and old_parts[1] == new_parts[1])
            old_worker = self.workers.get(old.split()[0])
            if old_worker is not None:
                old_worker.stop_mode()
            self.active_mode = mode_str
            if self.publisher is not None:
                self.publisher.on_mode_changed()
            self._activate(mode_str, preserve_motion=rearm)
            if base in self.workers:
                print(f'[mode] {old} -> {mode_str}（工作线程已启动）')
            elif base == 'raw':
                print(f'[mode] {old} -> raw（相机画面，全部识别停止）')
            else:
                print(f'[mode] {old} -> {mode_str}（该模式尚未实现，'
                      f'视觉处理停止，等待后续命令）')

    def cycle_track_color(self):
        # request_mode 自己获取锁，不能在持锁状态下调用。
        with self._lock:
            if not self.active_mode.startswith('material_track '):
                return
            next_color = self.track_color % 6 + 1
        self.request_mode(f'material_track {next_color}')


# ---------------------------------------------------------------------------
# FrameDispatcher：取帧 → 投递给激活模式的 worker
# ---------------------------------------------------------------------------
class FrameDispatcher(threading.Thread):
    def __init__(self, cam, manager):
        super().__init__(daemon=True, name='frame-dispatcher')
        self.cam = cam
        self.manager = manager
        self._stop_evt = threading.Event()
        self.seq = 0
        self.lock = threading.Lock()
        self.latest_frame = None

    def run(self):
        while not self._stop_evt.is_set():
            read_sample = getattr(self.cam, 'read_sample', None)
            if callable(read_sample):
                frame, source_ns = read_sample()
            else:
                frame, source_ns = self.cam.read(), time.monotonic_ns()
            if frame is None:
                time.sleep(0.002)
                continue
            self.seq += 1
            with self.lock:
                self.latest_frame = frame
            with self.manager._lock:
                worker = self.manager.workers.get(
                    self.manager.active_mode.split()[0])
                if worker is not None:
                    worker.submit(frame, source_ns)

    def stop(self):
        self._stop_evt.set()
        self.join(timeout=1.0)


# ---------------------------------------------------------------------------
def display_frame(manager, dispatcher):
    """取得一致的模式/结果快照；始终在副本上叠字，避免污染工作线程画布。"""
    with manager._lock:
        mode = manager.active_mode
        base = mode.split()[0]
        worker = manager.workers.get(base)
        canvas, sites, result, error = None, [], {}, ''
        stale_material = False
        ring_yaw_out, ring_line_source = None, 'none'
        if worker is not None:
            with worker.lock:
                canvas = worker.latest_canvas
                if canvas is not None:
                    canvas = canvas.copy()
                error = worker.last_error
                if base == 'ring':
                    sites = list(worker.latest_sites)
                    ring_yaw_out = getattr(worker, 'latest_yaw', None)
                    ring_line_source = getattr(worker, 'latest_line_source',
                                               'none')
                else:
                    result = dict(worker.latest)
                if base == 'material_track' and worker.latest_source_ns:
                    stale_material = (time.monotonic_ns() - worker.latest_source_ns >
                                      worker.grasp_params.result_ttl_s * 1e9)
        elif base == 'raw':
            with dispatcher.lock:
                if dispatcher.latest_frame is not None:
                    canvas = dispatcher.latest_frame.copy()
    if canvas is None:
        canvas = np.zeros((IMAGE_H, IMAGE_W, 3), np.uint8)
    color = (0, 200, 255)
    if base == 'ring':
        # FULL 显示 worker 发布的 yaw（与 TX 一致，标题带线/基线来源）；
        # PARTIAL 无发布值时按坐标现算（与编码器回退一致）。
        yaw = ring_yaw_out if ring_yaw_out is not None else \
            (ring_yaw(sites) if len(sites) >= 2 else None)
        full = len(sites) == 3 and all(s['id'] for s in sites)
        status = 'FULL' if full else ('PARTIAL' if sites else 'NONE')
        src = f'[{ring_line_source}]' if full else ''
        yaw_txt = f'{yaw:.2f}' if yaw is not None else '--'
        info = f'MODE=ring {status}{src} yaw={yaw_txt}'
        color = (0, 255, 0) if full else color
    elif base == 'turntable':
        status = ('STABLE' if result.get('stable') else
                  'ACQUIRING' if result.get('detected') else 'SEARCHING')
        info = f'MODE=turntable {status}'
        color = (0, 255, 0) if result.get('stable') else color
        # 转盘算法自己已在y=28画状态；公共标题另起一行。
    elif base in ('material_all', 'material_track'):
        status = result.get('status', 'WAITING')
        info = f'MODE={mode} {status}'
        if base == 'material_track':
            status = ('STALE FRAME' if stale_material else
                      result.get('state_name', 'WAITING'))
            info = f'MODE={mode} {status}'
            target = result.get('target')
            if target is not None:
                info += ' C=(%d,%d)' % tuple(round(v) for v in target.center)
        color = (0, 255, 0) if status in ('READY', 'DETECTED') else color
    elif base == 'stacked':
        status = 'TRACKING' if result.get('detected') else 'SEARCHING'
        info = f'MODE=stacked {status}'
        if result.get('detected'):
            yaw = result.get('station_yaw_deg')
            info += f' yaw={yaw:.1f}' if yaw is not None else ' yaw=--'
        color = (0, 255, 0) if result.get('detected') else color
    elif base == 'raw':
        info, color = 'MODE=raw CAMERA', (255, 255, 255)
    else:
        info = f'MODE={base} NOT IMPLEMENTED'
    if error:
        info += ' ERROR - see terminal'
        color = (0, 0, 255)
    if base in ('turntable', 'stacked'):
        # 转盘/码垛算法自己在 y=28 画状态；公共标题另起一行。
        title_y, help_y = 56, 80
    else:
        title_y, help_y = 24, 50
    draw_debug_label(canvas, info, (8, title_y), color, 0.52)
    help_line = '0:raw 1:ring 2:all 3:track 4:turn 5:stk c:color q:quit'
    draw_debug_label(canvas, help_line, (8, help_y))
    if stale_material:
        # 覆盖旧画布上的READY提示，避免断流后窗口仍显示旧的放行结论。
        draw_debug_label(canvas, 'GRASP STALE FRAME - TX BLOCKED',
                         (8, 128), (0, 0, 255))
    return canvas


def handle_key(key, manager):
    """返回True表示退出；键盘与串口共用模式切换入口。"""
    if key in (ord('q'), 27):
        return True
    key_map = {ord('0'): 'raw', ord('1'): 'ring', ord('2'): 'material_all',
               ord('3'): 'material_track', ord('4'): 'turntable',
               ord('5'): 'stacked'}
    if key in key_map:
        manager.request_mode(key_map[key])
    elif key == ord('c'):
        manager.cycle_track_color()
    return False


def parse_args():
    p = argparse.ArgumentParser(
        description='单进程多模式视觉主程序（无 ROS2）：圆环/物料/转盘')
    p.add_argument('--device', default='/dev/video0')
    p.add_argument('--width', type=int, default=640)
    p.add_argument('--height', type=int, default=480)
    p.add_argument('--fps', type=int, default=30)
    p.add_argument('--blur', type=int, choices=(3, 5, 7), default=3)
    p.add_argument('--start-mode', default='raw',
                   choices=('raw', 'ring', 'material_all', 'material_track',
                            'turntable', 'stacked'),
                   help='上电初始模式（默认 raw=上电静默，等 STM32 命令）')
    p.add_argument('--serial', default='/dev/ttyS1',
                   help='STM32 串口（与 ROS 桥一致）')
    p.add_argument('--no-serial', action='store_true',
                   help='禁用串口（纯视觉调试）')
    p.add_argument('--material-config', default=DEFAULT_CONFIG,
                   help='物料六色 HSV 阈值 JSON；首次进入物料模式时加载')
    p.add_argument('--track-color', type=int, choices=range(1, 7), default=1,
                   help='启动物料跟踪颜色（默认1红色；运行时c循环切色）')
    # 夹取参数集中定义于GraspParams，命令行同名映射，避免两套默认值。
    for name, value in vars(GraspParams()).items():
        p.add_argument('--grasp-' + name.replace('_', '-'), type=type(value),
                       default=value, help=f'夹取参数{name}（默认{value}）')
    p.add_argument('--baudrate', type=int, default=115200)
    p.add_argument('--display', action='store_true',
                   help='调试窗口（raw/圆环/物料/转盘）')
    return p.parse_args()


def main():
    args = parse_args()
    grasp_params = GraspParams(**{name: getattr(args, 'grasp_' + name)
                                  for name in vars(GraspParams())})
    # SIGTERM/SIGINT 转 SystemExit：所有线程为 daemon，主线程退出即收尾
    import signal
    signal.signal(signal.SIGTERM, lambda s, f: sys.exit(0))
    signal.signal(signal.SIGINT, lambda s, f: sys.exit(0))

    det_args = argparse.Namespace(
        scale=1.0, roi=False, roi_full_scan_interval=15,
        min_dist=60.0, min_r=20, max_r=60, dp=1.0,
        param1=210.0, param2=60.0, no_verify=False, coverage=0.7,
        merge_dist=20.0)

    cam = None
    attempt = 0
    while cam is None:               # 开机自启动场景：无限等待相机（每 2s 重试）
        attempt += 1
        try:
            cam = Camera(args.device, args.width, args.height, args.fps)
            break
        except RuntimeError as exc:
            if attempt == 1 or attempt % 5 == 0:
                print(f'[main][等待相机] {exc}（已等待 {attempt * 2}s，'
                      f'相机接上后自动继续——检查 USB 线缆/插拔）')
            time.sleep(2)
    if (cam.real_w, cam.real_h) != (IMAGE_W, IMAGE_H):
        print(f'[警告] 实际分辨率 {cam.real_w}x{cam.real_h} ≠ 协议假设 '
              f'{IMAGE_W}x{IMAGE_H}，坐标编码将越界拒发！')
    und = Undistorter('/home/sunrise/logistics_vision_data/calibration/'
                      'camera_640x480.yaml', (cam.real_w, cam.real_h))

    ring_worker = RingModeWorker(und, det_args, blur_k=args.blur)
    turntable_worker = TurntableWorker(und)
    stacked_worker = StackedModeWorker(und)
    # ModeManager 与 VisionPublisher 互相引用：先建 Manager（publisher 占位），
    # 再建 Publisher 并回填——request_mode 只在两者就绪后才可能被调用
    workers = {'ring': ring_worker, 'turntable': turntable_worker,
               'stacked': stacked_worker,
               'material_all': MaterialModeWorker(und, args.material_config),
               'material_track': MaterialModeWorker(und, args.material_config,
                                                     track=True,
                                                     grasp_params=grasp_params)}
    manager = ModeManager(args.start_mode, workers,
                          publisher=None,
                          serial_enabled=args.no_serial is False,
                          track_color=args.track_color)
    publisher = VisionPublisher(serial_link=None, workers=workers,
                                mode_manager=manager)
    manager.publisher = publisher

    serial_link = None
    if not args.no_serial:
        serial_link = SerialLink(args.serial, args.baudrate,
                                 on_mode=lambda s: manager.request_mode(s))
        publisher.link = serial_link

    if serial_link is not None:
        serial_link.start()
    publisher.start()
    dispatcher = FrameDispatcher(cam, manager)
    dispatcher.start()

    print(f'[main] 运行中：模式={args.start_mode} 串口='
          f'{"关" if serial_link is None else args.serial} '
          f'（Ctrl+C 退出；--display 时按 q 退出）')

    try:
        if args.display:
            cv2.namedWindow('Logistics Vision Main', cv2.WINDOW_AUTOSIZE)
            while True:
                cv2.imshow('Logistics Vision Main', display_frame(manager, dispatcher))
                if handle_key(cv2.waitKey(30) & 0xFF, manager):
                    break
        else:
            threading.Event().wait()    # 无窗口：阻塞至 SIGTERM/SIGINT
    finally:
        dispatcher.stop()
        publisher.stop()
        if serial_link is not None:
            serial_link.stop()
        for worker in workers.values():
            worker.close()
        cam.close()
        cv2.destroyAllWindows()
        print('[main] 已退出')


if __name__ == '__main__':
    main()
