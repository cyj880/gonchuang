#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
camera_base.py —— 相机采集与去畸变公共模块（vision_scripts 文件夹自包含）

从 ~/logistics_ws/ring_hough_standalone.py 原样提取的两个类，行为不变：
    Camera       V4L2 + MJPG + 640x480@30，独立采集进程+监督线程，只保留最新帧（信箱模式），
                 异常或2秒无新帧后有界终止并重开，启动超时5秒；
    Undistorter  读 c100_v7 标定 yaml → getOptimalNewCameraMatrix(alpha=0) +
                 initUndistortRectifyMap(CV_16SC2) 预计算 map，每帧仅 remap；
                 yaml 缺失/损坏时回退到内嵌同数值标定。

C100 实测要点（沿用 ring_hough_standalone 的结论）：
    - set 顺序必须 FOURCC → WIDTH → HEIGHT → FPS；
    - MJPG 640x480 只有 30fps 一档，请求其它格式/帧率会僵死固件（须物理重插）；
    - 被强杀后相机卡死且重开无效：
      sudo modprobe -r uvcvideo && sudo modprobe uvcvideo 重载驱动。

用法：
    from camera_base import Camera, Undistorter, DEFAULT_CALIB
    cam = Camera('/dev/video0', 640, 480, 30)
    und = Undistorter(DEFAULT_CALIB, (cam.real_w, cam.real_h))
    frame = cam.read()          # 无新帧返回 None
    undist = und.remap(frame)   # 尺寸与标定不符时原样跳过（警告一次）
"""

import os
import multiprocessing
import threading
import time

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

DEFAULT_CALIB = '/home/sunrise/logistics_vision_data/calibration/camera_640x480.yaml'


# ---------------------------------------------------------------------------
# 摄像头：V4L2 + MJPG + 640x480@30，后台线程只留最新帧
# ---------------------------------------------------------------------------
def _camera_capture(connection, device, width, height, fps):
    """唯一拥有V4L2句柄的进程；父进程可终止被底层read/open阻塞的采集。"""
    cap = None
    try:
        cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
        if not cap.isOpened():
            raise RuntimeError(f'无法打开摄像头 {device}')
        for prop, value in ((cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG')),
                            (cv2.CAP_PROP_FRAME_WIDTH, width),
                            (cv2.CAP_PROP_FRAME_HEIGHT, height),
                            (cv2.CAP_PROP_FPS, fps)):
            cap.set(prop, value)
        fourcc = int(cap.get(cv2.CAP_PROP_FOURCC))
        metadata = (int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)),
                    int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)),
                    cap.get(cv2.CAP_PROP_FPS),
                    ''.join(chr((fourcc >> (8 * i)) & 0xFF) for i in range(4)))
        connection.send(('opened', metadata))
        failed = 0
        while True:
            ok, frame = cap.read()
            if not ok or frame is None:
                failed += 1
                if failed >= 50:
                    raise RuntimeError('连续读帧失败50次')
                time.sleep(0.02)
                continue
            failed = 0
            connection.send(('frame', time.monotonic_ns(), frame))
    except Exception as exc:
        try:
            connection.send(('error', str(exc)))
        except (OSError, EOFError):
            pass
    finally:
        connection.close()
        if cap is not None:
            cap.release()  # 即使驱动在release卡住，父进程的有界终止仍可接管。


class Camera:
    OPEN_TIMEOUT_S = 5.0
    FRAME_TIMEOUT_S = 2.0
    FRAME_TTL_NS = 300_000_000
    RETRY_S = 0.3

    def __init__(self, device, width, height, fps):
        self.device = device
        self._requested = (width, height, fps)  # 重开固定沿用请求，不将坏回读值写回相机。
        self._ctx = multiprocessing.get_context('spawn')
        self._lock = threading.Lock()
        self._frame = None
        self._frame_ns = 0
        self._stop_evt = threading.Event()
        self._process = self._connection = None
        self._last_warning = ''
        self.recoveries = 0
        try:
            self._start_capture()
        except Exception as exc:
            self._stop_capture()
            raise RuntimeError(f'相机初始化失败: {exc}') from exc
        self._thread = threading.Thread(target=self._read_loop, daemon=True,
                                        name='camera-supervisor')
        self._thread.start()

    def _start_capture(self):
        reader, writer = self._ctx.Pipe(duplex=False)
        process = self._ctx.Process(target=_camera_capture,
                                    args=(writer, self.device, *self._requested),
                                    daemon=True, name='camera-capture')
        self._connection, self._process = reader, process
        try:
            process.start()
        finally:
            writer.close()
        deadline = time.monotonic() + self.OPEN_TIMEOUT_S
        while not self._stop_evt.is_set() and time.monotonic() < deadline:
            if reader.poll(0.05):
                message = reader.recv()
                if message[0] != 'opened':
                    raise RuntimeError(str(message[1]))
                metadata = message[1]
                if hasattr(self, 'real_w') and metadata[:2] != (self.real_w, self.real_h):
                    raise RuntimeError('重开后分辨率变化，拒绝沿用旧去畸变映射')
                self.real_w, self.real_h, self.real_fps, self.real_fourcc = metadata
                print(f'[camera] {self.device} 实际生效: {self.real_w}x{self.real_h}@'
                      f'{self.real_fps:.0f} fourcc={self.real_fourcc}')
                self._last_frame_at = time.monotonic()
                return
            if not process.is_alive():
                raise RuntimeError('相机采集进程在打开阶段退出')
        raise RuntimeError('相机打开超时或已中止')

    def _stop_capture(self):
        # 只由初始化/监督线程或监督线程退出后的close调用，不跨线程release句柄。
        process, connection = self._process, self._connection
        if process is not None and process.pid is not None:
            if process.is_alive():
                process.terminate()
            process.join(timeout=0.5)
            if process.is_alive():
                process.kill()
                process.join(timeout=0.5)
            if process.is_alive():
                # 不能不断产生新进程占用同一个设备；旧进程退出前只保持离线。
                return False
            process.close()
        if connection is not None:
            connection.close()
        self._process = self._connection = None
        return True

    def _read_loop(self):
        try:
            while not self._stop_evt.is_set():
                try:
                    if self._connection is None:
                        self._start_capture()
                    if self._connection.poll(0.05):
                        message = self._connection.recv()
                        if message[0] == 'error':
                            raise RuntimeError(str(message[1]))
                        if message[0] == 'frame':
                            _, source_ns, frame = message
                            if 0 <= time.monotonic_ns() - source_ns <= self.FRAME_TTL_NS:
                                with self._lock:
                                    self._frame, self._frame_ns = frame, source_ns
                                self._last_frame_at = time.monotonic()
                                self._last_warning = ''
                    if not self._process.is_alive():
                        raise RuntimeError('相机采集进程退出')
                    if time.monotonic() - self._last_frame_at >= self.FRAME_TIMEOUT_S:
                        raise RuntimeError('相机2秒无新帧，重建采集进程')
                except Exception as exc:
                    with self._lock:
                        self._frame, self._frame_ns = None, 0
                    if str(exc) != self._last_warning:
                        print(f'[camera][警告] {exc}')
                        self._last_warning = str(exc)
                    # 底层进程未退出时不再打开设备；所有等待均可被close唤醒。
                    while not self._stop_capture():
                        if self._stop_evt.wait(self.RETRY_S):
                            return
                    self.recoveries += 1
                    self._stop_evt.wait(self.RETRY_S)
        finally:
            self._stop_capture()

    def read_sample(self):
        """取最新帧和真实采集时刻；断流/过期不重复交付旧画面。"""
        with self._lock:
            frame, source = self._frame, self._frame_ns
            self._frame, self._frame_ns = None, 0
        if source and 0 <= time.monotonic_ns() - source <= self.FRAME_TTL_NS:
            return frame, source
        return None, 0

    def read(self):
        return self.read_sample()[0]

    def close(self):
        self._stop_evt.set()
        self._thread.join(timeout=2.0)
        # 设备由子进程独占，禁止父线程并发调用cap.release/read。


# ---------------------------------------------------------------------------
# 去畸变：预计算 map，每帧仅 remap
# ---------------------------------------------------------------------------
class Undistorter:
    def __init__(self, calib_path, size):
        K, D, source = self._load(calib_path)
        # 内嵌标定明确来自640x480；外部文件若未声明尺寸则不批准自动夹取。
        self.calibration_size = (640, 480) if source.startswith('内嵌') else None
        if self.calibration_size is None:
            fs = cv2.FileStorage(calib_path, cv2.FILE_STORAGE_READ)
            try:
                self.calibration_size = (int(fs.getNode('image_width').real()),
                                         int(fs.getNode('image_height').real()))
            finally:
                fs.release()
        self.new_K, _ = cv2.getOptimalNewCameraMatrix(K, D, size, 0, size)
        self.map_x, self.map_y = cv2.initUndistortRectifyMap(
            K, D, None, self.new_K, size, cv2.CV_16SC2)
        self.size = size
        self.source = source
        self._size_warned = False
        print(f'[undistort] 标定来源: {source}')
        print(f'[undistort] K={np.round(K, 2).tolist()} '
              f'D={np.round(D, 6).tolist()}')

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
