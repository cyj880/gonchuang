#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
camera_base.py —— 相机采集与去畸变公共模块（vision_scripts 文件夹自包含）

从 ~/logistics_ws/ring_hough_standalone.py 原样提取的两个类，行为不变：
    Camera       V4L2 + MJPG + 640x480@30，后台线程只保留最新帧（信箱模式），
                 连续读帧失败自动重开（自救）；
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
class Camera:
    def __init__(self, device, width, height, fps):
        self._open(device, width, height, fps)
        self._lock = threading.Lock()
        self._frame = None            # 最新一帧，被主循环取走即置 None
        self._stop_evt = threading.Event()
        self._fail_count = 0
        self._fail_cycles = 0
        self._thread = threading.Thread(target=self._read_loop, daemon=True)
        self._thread.start()

    def _open(self, device, width, height, fps):
        self.device = device
        self.cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
        if not self.cap.isOpened():
            raise RuntimeError(f'无法打开摄像头 {device}')
        fourcc = cv2.VideoWriter_fourcc(*'MJPG')
        self.cap.set(cv2.CAP_PROP_FOURCC, fourcc)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self.cap.set(cv2.CAP_PROP_FPS, fps)
        # 回读实际生效值（照搬 camera_capture_node.py 的做法）
        self.real_w = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        self.real_h = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        self.real_fps = self.cap.get(cv2.CAP_PROP_FPS)
        fourcc_int = int(self.cap.get(cv2.CAP_PROP_FOURCC))
        self.real_fourcc = ''.join(chr((fourcc_int >> (8 * i)) & 0xFF)
                                   for i in range(4))
        print(f'[camera] {device} 实际生效: {self.real_w}x{self.real_h}@'
              f'{self.real_fps:.0f} fourcc={self.real_fourcc}')
        if (self.real_w, self.real_h) != (width, height):
            print(f'[camera][警告] 实际分辨率与请求的 {width}x{height} 不一致，'
                  f'请以实际分辨率重新标定！')

    def _read_loop(self):
        while not self._stop_evt.is_set():
            ok, frame = self.cap.read()
            if not ok:
                self._fail_count += 1
                if self._fail_count >= 50:      # 连续失败自动重开（自救）
                    print('[camera][警告] 连续读帧失败，尝试重新打开设备')
                    self.cap.release()
                    time.sleep(0.3)
                    try:
                        self._open(self.device, self.real_w, self.real_h,
                                   self.real_fps)
                    except RuntimeError:
                        pass
                    self._fail_cycles += 1
                    self._fail_count = 0
                    if self._fail_cycles >= 3:
                        print('[camera][提示] 多次重开无效，可能是 UVC 状态卡死，'
                              '可执行: sudo modprobe -r uvcvideo && '
                              'sudo modprobe uvcvideo 重载驱动')
                time.sleep(0.02)
                continue
            self._fail_count = 0
            with self._lock:
                self._frame = frame             # 覆盖旧帧，只保最新

    def read(self):
        """取走最新帧；无新帧返回 None（主循环复用上一帧画面即可）。"""
        with self._lock:
            frame, self._frame = self._frame, None
        return frame

    def close(self):
        self._stop_evt.set()
        self._thread.join(timeout=1.0)
        self.cap.release()


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
