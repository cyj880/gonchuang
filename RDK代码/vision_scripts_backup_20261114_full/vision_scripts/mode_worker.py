#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""模式工作线程：最新帧信箱、会话隔离、工作线程内复位。"""

import threading


class ModeWorker:
    def __init__(self, name):
        self.name = name
        self._cond = threading.Condition()
        self.lock = threading.Lock()
        self._pending = None
        self._active = False
        self._closed = False
        self._options = {}
        self.generation = 0
        self.latest_canvas = None
        self.latest_source_ns = 0
        self.error_count = 0
        self.last_error = ''

    def _start_worker(self):
        # 子类全部字段初始化完成后再起线程。
        self._thread = threading.Thread(target=self._run, name=self.name,
                                        daemon=True)
        self._thread.start()

    def start_mode(self, **options):
        with self._cond:
            self.generation += 1
            self._active = True
            self._options = dict(options)
            self._pending = None
            with self.lock:
                self._clear_output()
                self.last_error = ''

    def stop_mode(self):
        with self._cond:
            self.generation += 1
            self._active = False
            self._pending = None
            with self.lock:
                self._clear_output()

    def submit(self, frame, source_ns):
        with self._cond:
            if self._active and not self._closed:
                self._pending = (self.generation, self._options,
                                 source_ns, frame)
                self._cond.notify()

    def _clear_output(self):
        """调用方持有输出锁；只清输出，不触碰算法状态。"""
        self.latest_canvas = None
        self.latest_source_ns = 0

    def _publish(self, generation, **output):
        # 与 start/stop 使用相同锁序，校验与提交不可被切换打断。
        with self._cond:
            if (not self._active or self._closed or
                    generation != self.generation):
                return False
            with self.lock:
                for key, value in output.items():
                    setattr(self, key, value)
            return True

    def _run(self):
        last_generation = None
        while True:
            with self._cond:
                self._cond.wait_for(lambda: self._closed or
                                    self._pending is not None)
                if self._closed:
                    return
                item, self._pending = self._pending, None
            generation, options, source_ns, frame = item
            try:
                if generation != last_generation:
                    self._reset_state(**options)
                    last_generation = generation
                self._process(frame, source_ns, generation)
            except Exception as exc:
                # 中断的检测不能保留上一段连续停稳/跟踪证据；下帧在本线程复位。
                last_generation = None
                with self._cond:
                    if generation != self.generation or not self._active:
                        continue
                    with self.lock:
                        self._clear_output()
                        self.error_count += 1
                        if self.last_error != str(exc):
                            print(f'[{self.name}][错误] {exc}')
                        self.last_error = str(exc)

    def close(self):
        with self._cond:
            self._closed = True
            self._active = False
            self._pending = None
            self._cond.notify()
        self._thread.join()
