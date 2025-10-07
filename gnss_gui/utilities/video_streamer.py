"""Video streaming and local camera utilities.

This module contains a lightweight camera abstraction suitable for
embedding a local (device) camera into the GUI for development and
testing.  It also retains a simple ``VideoStreamer`` stub for the
ffmpeg-based streaming path used for remote cameras.

The important addition here is ``LocalCamera`` and a tiny
``CameraManager`` registry so callers can request a camera by name
(``"local"``) and attach it to the GUI's ``VideoViewer``.  The
implementation uses OpenCV (if available) to capture frames in a
background thread and exposes a non-blocking ``read_frame`` method.
"""

from __future__ import annotations

import threading
import time
from typing import Optional, Dict

try:
    import cv2
    import numpy as np
except Exception:  # pragma: no cover - optional dependency at runtime
    cv2 = None  # type: ignore
    np = None  # type: ignore

import subprocess


class CameraSource:
    """Minimal interface for a camera-like source used by VideoViewer.

    Implementations should provide ``start()``, ``stop()`` and
    ``read_frame() -> Optional[np.ndarray]`` where a frame is an RGB
    ``(H, W, 3)`` uint8 array.
    """

    def start(self) -> None:
        raise NotImplementedError()

    def stop(self) -> None:
        raise NotImplementedError()

    def read_frame(self):
        raise NotImplementedError()


class LocalCamera(CameraSource):
    """Capture from the default local camera using OpenCV.

    This class captures frames on a background thread and keeps the
    most recent frame in memory; ``read_frame`` returns a copy of that
    frame or ``None`` if no frame is available.
    """

    def __init__(self, index: int = 0) -> None:
        if cv2 is None:
            raise RuntimeError("OpenCV (cv2) is required for LocalCamera but is not installed")
        self.index = int(index)
        # don't reference cv2 types in annotations (some type checkers
        # don't like library-specific classes); keep a simple runtime
        # value instead.
        self._cap = None
        self._thread = None
        self._stop_event = threading.Event()
        self._lock = threading.Lock()
        self._frame = None

    def start(self) -> None:
        if self._thread and self._thread.is_alive():
            return
        self._stop_event.clear()
        self._cap = cv2.VideoCapture(self.index)
        # Try a few default capture properties for consistency
        try:
            self._cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
            self._cap.set(cv2.CAP_PROP_FPS, 30)
        except Exception:
            pass

        def _run() -> None:
            while not self._stop_event.is_set():
                try:
                    ok, frame = self._cap.read()  # BGR
                    if not ok:
                        time.sleep(0.05)
                        continue
                    # Convert to RGB and store
                    with self._lock:
                        self._frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                except Exception:
                    time.sleep(0.1)

        self._thread = threading.Thread(target=_run, daemon=True)
        self._thread.start()

    def read_frame(self):
        with self._lock:
            if self._frame is None:
                return None
            # return a copy to avoid race conditions
            return self._frame.copy()

    def stop(self) -> None:
        self._stop_event.set()
        if self._thread:
            self._thread.join(timeout=1.0)
            self._thread = None
        if self._cap is not None:
            try:
                self._cap.release()
            except Exception:
                pass
            self._cap = None
        with self._lock:
            self._frame = None


class CameraManager:
    """Simple registry for named cameras.

    The module registers a ``local`` camera instance by default to make
    it easy for the GUI code to discover the local device.
    """

    _registry: Dict[str, CameraSource] = {}

    @classmethod
    def register(cls, name: str, source: CameraSource) -> None:
        cls._registry[name] = source

    @classmethod
    def get_camera(cls, name: str) -> Optional[CameraSource]:
        return cls._registry.get(name)


# Register a default local camera (index 0) if OpenCV is available.
try:
    if cv2 is not None:
        cam = LocalCamera(index=0)
        CameraManager.register("local", cam)
        # Do NOT start the camera at import time. The camera will be
        # opened when the user explicitly requests streaming (Start
        # Stream). This avoids leaving the device active without user
        # consent.
except Exception:
    # If camera registration fails don't crash import-time; consumers
    # will get a helpful error when they attempt to use the camera.
    pass


class VideoStreamer:
    """Manage the lifecycle of a video stream using ffmpeg.

    This class is unchanged from the prototype: it provides a small
    wrapper that in future can spawn ffmpeg to push frames to a
    network endpoint.  It's kept for parity with the earlier design.
    """

    def __init__(self, camera_device: str = "/dev/video0", host: str = "127.0.0.1", port: int = 5000) -> None:
        self.camera_device = camera_device
        self.host = host
        self.port = port
        self._process: Optional[subprocess.Popen] = None

    def start_stream(self, bitrate: int = 2000) -> None:
        print(f"[VideoStreamer] Starting stream from {self.camera_device} to {self.host}:{self.port} at {bitrate} kbps")

    def stop_stream(self) -> None:
        print("[VideoStreamer] Stopping stream")
        if self._process is not None:
            self._process.terminate()
            self._process.wait()
            self._process = None