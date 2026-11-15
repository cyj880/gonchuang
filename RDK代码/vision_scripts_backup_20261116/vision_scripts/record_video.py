#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""record_video.py —— 码垛调参用实机录像（RAW 帧，MJPG/AVI）。

录的是**未去畸变的原始帧**：离线回放时用同一份标定重做去畸变，与实机
管线逐帧一致。MJPG 编码体积小、无转码质量损失。

用法（每个场景一条命令，文件名自动带时间戳，存 /home/sunrise/stacked_tune/）：
    python3 record_video.py --output 01_static    --duration 30
    python3 record_video.py --output 02_approach  --duration 40
按 q 可提前结束；SIGTERM/SIGINT 优雅退出（相机不僵死）。
"""

import argparse
import os
import signal
import sys
import time

import cv2

from camera_base import Camera


def parse_args():
    p = argparse.ArgumentParser(description='实机录像（RAW MJPG/AVI，调参用）')
    p.add_argument('--device', default='/dev/video0')
    p.add_argument('--output', default='stacked_raw',
                   help='输出文件名前缀（自动加时间戳）')
    p.add_argument('--duration', type=float, default=30.0,
                   help='计划录像秒数（q 可提前结束）')
    p.add_argument('--outdir', default='/home/sunrise/stacked_tune')
    p.add_argument('--no-preview', action='store_true',
                   help='关闭预览窗口（预览占耗时，追求满 30fps 时加）')
    p.add_argument('--hint', default='',
                   help='动作提示（仅 ASCII，显示在预览画面上）')
    p.add_argument('--lead', type=float, default=5.0,
                   help='开始前倒计时秒数（用于准备动作）')
    return p.parse_args()


def main():
    args = parse_args()
    for sig in (signal.SIGTERM, signal.SIGINT):
        signal.signal(sig, lambda s, f: sys.exit(0))
    os.makedirs(args.outdir, exist_ok=True)
    cam = Camera(args.device, 640, 480, 30)
    path = os.path.join(args.outdir, time.strftime(
        args.output + '_%Y%m%d_%H%M%S.avi'))
    writer = cv2.VideoWriter(path, cv2.VideoWriter_fourcc(*'MJPG'),
                             30, (cam.real_w, cam.real_h))
    if not writer.isOpened():
        print(f'[rec] 错误: VideoWriter 打开失败 ({path})')
        cam.close()
        return 1
    window = None
    if not args.no_preview:
        window = 'REC (q=stop)'
        cv2.namedWindow(window, cv2.WINDOW_AUTOSIZE)
    hint = args.hint.upper()
    print(f'[rec] 输出: {path}  计划 {args.duration:.0f}s'
          + ('' if window else '（无预览，按 Ctrl+C 结束）'))
    if hint:
        print(f'[rec] 动作: {hint}')

    if window and args.lead > 0:      # 开始前倒计时：大字读秒 + 动作提示
        deadline = time.monotonic() + args.lead
        while time.monotonic() < deadline:
            remain = deadline - time.monotonic()
            frame = cam.read()
            if frame is not None:
                preview = frame.copy()
                cv2.putText(preview, f'START IN {remain:3.0f}s', (8, 70),
                            cv2.FONT_HERSHEY_SIMPLEX, 1.3, (0, 0, 255), 4,
                            cv2.LINE_AA)
                if hint:
                    cv2.putText(preview, hint, (8, 120),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                                (0, 255, 255), 2, cv2.LINE_AA)
                cv2.imshow(window, preview)
            cv2.waitKey(1)
    elif args.lead > 0:
        print(f'[rec] {args.lead:.0f}s 后开始，请准备...')
        time.sleep(args.lead)

    frames, started, last_report = 0, time.monotonic(), time.monotonic()
    try:
        while True:
            elapsed = time.monotonic() - started
            frame = cam.read()
            if frame is not None:
                writer.write(frame)
                frames += 1
                if window:
                    preview = frame.copy()
                    cv2.putText(preview, f'REC {elapsed:5.1f}s frames={frames}',
                                (8, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                                (0, 0, 255), 2, cv2.LINE_AA)
                    if hint:
                        cv2.putText(preview, hint, (8, 466),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                                    (0, 255, 255), 1, cv2.LINE_AA)
                    cv2.imshow(window, preview)
            if elapsed >= args.duration:
                break
            if window and cv2.waitKey(1) & 0xFF == ord('q'):
                print('[rec] 手动结束')
                break
            now = time.monotonic()
            if now - last_report >= 5.0:
                print(f'[rec] {elapsed:5.1f}s  frames={frames}  '
                      f'({frames / elapsed:.1f}fps)')
                last_report = now
    finally:
        writer.release()
        cam.close()
        cv2.destroyAllWindows()
    seconds = time.monotonic() - started
    size_mb = os.path.getsize(path) / 1e6
    print(f'[rec] 完成: {path}\n'
          f'[rec] {frames} 帧 / {seconds:.1f}s = {frames / max(seconds, 0.1):.1f}fps, '
          f'{size_mb:.0f}MB')
    return 0


if __name__ == '__main__':
    sys.exit(main())
