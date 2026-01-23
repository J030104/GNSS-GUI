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
import shutil
from dataclasses import dataclass
from typing import List


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
        self._cv2 = cv2
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
        self._cap = self._cv2.VideoCapture(self.index)
        # Try a few default capture properties for consistency
        try:
            self._cap.set(self._cv2.CAP_PROP_FRAME_WIDTH, 640)
            self._cap.set(self._cv2.CAP_PROP_FRAME_HEIGHT, 480)
            self._cap.set(self._cv2.CAP_PROP_FPS, 30)
        except Exception:
            pass

        def _run() -> None:
            while not self._stop_event.is_set():
                try:
                    if self._cap is None:
                        time.sleep(0.02)
                        continue
                    ok, frame = self._cap.read()  # BGR
                    if not ok:
                        time.sleep(0.05)
                        continue
                    # Convert to RGB and store
                    with self._lock:
                        self._frame = self._cv2.cvtColor(frame, self._cv2.COLOR_BGR2RGB)
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


@dataclass
class NetworkStreamOptions:
    """Options for low-latency network streaming via OpenCV/FFmpeg.

    Attributes
    ----------
    proto : str
        Protocol: "rtsp", "udp", or "srt" (SRT requires FFmpeg/OpenCV support).
    host : str
        Remote host/IP to connect to (ignored for certain URL forms).
    port : int
        Port number.
    path : str
        Path segment for RTSP (e.g., "live.sdp").
    rtsp_transport : str
        For RTSP: "udp" (default) or "tcp".
    buffer_size : int
        Internal frame buffer size in frames (CAP_PROP_BUFFERSIZE). Use 1 for low latency.
    target_size : Optional[str]
        Optional target view size like "1280x720"; not applied at capture time but kept for UI context.
    extra_query : Optional[str]
        Extra query parameters appended to the URL (e.g., "latency=20").
    """

    proto: str = "rtsp"
    host: str = "127.0.0.1"
    port: int = 8554
    path: str = "live.sdp"
    rtsp_transport: str = "udp"
    buffer_size: int = 1
    target_size: Optional[str] = None
    extra_query: Optional[str] = None


class NetworkStreamCamera(CameraSource):
    """Receive a network stream via OpenCV's FFmpeg backend and yield RGB frames.

    Designed to behave like LocalCamera so it can be attached to VideoViewer.
    """

    def __init__(self, options: NetworkStreamOptions) -> None:
        if cv2 is None:
            raise RuntimeError("OpenCV (cv2) is required for network streaming but is not installed")
        self._cv2 = cv2  # store module reference to avoid Optional typing issues
        self.options = options
        self._cap = None
        self._thread = None
        self._stop_event = threading.Event()
        self._lock = threading.Lock()
        self._frame = None

    def _build_url(self) -> str:
        o = self.options
        proto = (o.proto or "rtsp").lower()
        q = []
        # RTSP-specific: choose transport
        if proto == "rtsp" and o.rtsp_transport in {"udp", "tcp"}:
            q.append(f"rtsp_transport={o.rtsp_transport}")
        if o.extra_query:
            q.append(o.extra_query.strip("?&"))

        if proto == "rtsp":
            base = f"rtsp://{o.host}:{int(o.port)}/{o.path.strip('/')}" if o.path else f"rtsp://{o.host}:{int(o.port)}/"
            if q:
                sep = '&' if '?' in base else '?'
                return f"{base}{sep}{'&'.join(q)}"
            return base
        if proto == "udp":
            # Basic UDP URL
            base = f"udp://{o.host}:{int(o.port)}"
            if q:
                sep = '&' if '?' in base else '?'
                return f"{base}{sep}{'&'.join(q)}"
            return base
        if proto == "srt":
            # SRT URL (support depends on build)
            base = f"srt://{o.host}:{int(o.port)}"
            if q:
                sep = '&' if '?' in base else '?'
                return f"{base}{sep}{'&'.join(q)}"
            return base
        # Fallback: assume options.host is raw URL
        return proto

    def start(self) -> None:
        if self._thread and self._thread.is_alive():
            return
        self._stop_event.clear()
        
        # NOTE: We do NOT open VideoCapture here because it blocks checking for the stream.
        # Instead we let the thread open it.

        def _run() -> None:
            # Generate URL in the thread (or before)
            url = self._build_url()
            # Initial open attempt
            try:
                if self._cap is None:
                    # Pass a timeout to ffmpeg via environment or url if supported?
                    # For now just open it. This might block THIS thread, but not the GUI.
                    self._cap = self._cv2.VideoCapture(url)
                    if not self._cap.isOpened():
                         self._cap = None
                    else:
                        # Reduce internal buffering to lower latency
                        if hasattr(self._cv2, 'CAP_PROP_BUFFERSIZE'):
                            self._cap.set(self._cv2.CAP_PROP_BUFFERSIZE, max(1, int(self.options.buffer_size)))
            except Exception:
                pass

            no_frame_strikes = 0
            while not self._stop_event.is_set():
                try:
                    if self._cap is None or not self._cap.isOpened():
                        # Try to open if not open
                        try:
                            if self._cap:
                                self._cap.release()
                        except Exception:
                            pass
                        
                        # Use a small sleep to avoid hammering
                        time.sleep(0.5)
                        try: 
                             self._cap = self._cv2.VideoCapture(url)
                             if hasattr(self._cv2, 'CAP_PROP_BUFFERSIZE'):
                                self._cap.set(self._cv2.CAP_PROP_BUFFERSIZE, max(1, int(self.options.buffer_size)))
                        except Exception:
                             pass
                        
                        if self._cap is None or not self._cap.isOpened():
                             continue

                    ok, frame = self._cap.read()  # BGR
                    if not ok or frame is None:
                        no_frame_strikes += 1
                        # Avoid busy loop when stream stalls
                        time.sleep(0.02)
                        # After a few misses, try to reopen
                        if no_frame_strikes >= 50: # 50 * 20ms = 1s
                            no_frame_strikes = 0
                            try:
                                if self._cap is not None:
                                    self._cap.release()
                            except Exception:
                                pass
                            self._cap = None # trigger reopen loop
                        continue
                    
                    no_frame_strikes = 0
                    
                    # Store latest frame (convert BGR -> RGB)
                    with self._lock:
                        self._frame = self._cv2.cvtColor(frame, self._cv2.COLOR_BGR2RGB)
                except Exception:
                     time.sleep(0.1)

        self._thread = threading.Thread(target=_run, daemon=True)
        self._thread.start()

    def read_frame(self):
        with self._lock:
            if self._frame is None:
                return None
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
        # Determine correct index for MacBook Pro Camera on MacOS
        cam_idx = 0
        import platform
        if platform.system() == "Darwin":
            try:
                # Run ffmpeg to list devices
                # Note: ffmpeg prints to stderr
                result = subprocess.run(
                    ["ffmpeg", "-f", "avfoundation", "-list_devices", "true", "-i", ""],
                    stderr=subprocess.PIPE,
                    stdout=subprocess.PIPE,
                    text=True
                )
                output = result.stderr
                # Parse output looking for "MacBook Pro Camera"
                # Pattern: [AVFoundation indev @ ...] [index] Name
                for line in output.split('\n'):
                     if "MacBook Pro Camera" in line and "Desk View" not in line:
                         # Extract index inside brackets e.g. [2]
                         import re
                         match = re.search(r'\[(\d+)\]', line)
                         if match:
                             cam_idx = int(match.group(1))
                             print(f"Found MacBook Pro Camera at index {cam_idx}")
                             break
            except Exception as e:
                print(f"Error auto-detecting camera: {e}")
                # Fallback to 1 if detection fails (common secondary index)
                cam_idx = 1

        cam = LocalCamera(index=cam_idx)
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
# Register remote cameras
try:
    if cv2 is not None:
        # Left USB Camera (UDP Port 5000)
        # Note: 0.0.0.0 binds to all interfaces to receive packets
        left_cam = NetworkStreamCamera(NetworkStreamOptions(
            proto="udp",
            host="0.0.0.0",
            port=5000,
            buffer_size=1
        ))
        CameraManager.register("Left USB Camera", left_cam)

        # Right USB Camera (UDP Port 5001)
        right_cam = NetworkStreamCamera(NetworkStreamOptions(
            proto="udp",
            host="0.0.0.0",
            port=5001,
            buffer_size=1
        ))
        CameraManager.register("Right USB Camera", right_cam)
except Exception:
    pass

@dataclass
class FfplayOptions:
    """Configurable ffplay options for receiving a stream.

    Attributes
    ----------
    low_latency : bool
        Apply a set of flags to reduce latency (nobuffer, low_delay, etc.).
    rtsp_transport : str
        One of {"udp", "tcp"}; only used for RTSP URLs.
    video_size : Optional[str]
        Target window size, e.g. "1280x720". If None, native.
    show_stats : bool
        Show on-screen stats (ffplay default enabled with 's' hotkey). We don't
        toggle via CLI, but this helps future extension.
    extra_args : List[str]
        Any additional raw ffplay CLI arguments to append.
    window_title : Optional[str]
        Title for the ffplay window.
    """

    low_latency: bool = True
    rtsp_transport: str = "udp"
    video_size: Optional[str] = None
    show_stats: bool = False
    extra_args: Optional[List[str]] = None
    window_title: Optional[str] = None


class FfplayReceiver:
    """Spawn and manage an ffplay process that receives a remote stream.

    This opens an external ffplay window (not embedded in the Qt widget). It is
    a pragmatic solution for low-latency preview. The GUI retains its own video
    widgets for local or simulated cameras.
    """

    def __init__(self) -> None:
        self._proc: Optional[subprocess.Popen] = None

    def _which_ffplay(self) -> str:
        # Try to find ffplay in PATH. On Windows, users typically install
        # FFmpeg and add its bin folder to PATH.
        exe = "ffplay"
        if shutil.which(exe) is None:
            # Try common Windows name
            exe = "ffplay.exe"
        if shutil.which(exe) is None:
            raise RuntimeError("ffplay not found in PATH. Please install FFmpeg and ensure ffplay is available.")
        return exe

    def start(self, url: str, options: Optional[FfplayOptions] = None) -> None:
        if self._proc and self._proc.poll() is None:
            # already running
            return
        exe = self._which_ffplay()
        opts = options or FfplayOptions()
        args: List[str] = [exe, "-nostdin"]

        # Window title
        if opts.window_title:
            args += ["-window_title", str(opts.window_title)]

        # Size
        if opts.video_size:
            # -x and -y in ffplay specify window size; alternatively -vf scale.
            try:
                w, h = str(opts.video_size).lower().split("x")
                args += ["-x", str(int(w)), "-y", str(int(h))]
            except Exception:
                # ignore malformed size
                pass

        # Low-latency flags
        if opts.low_latency:
            args += [
                "-fflags", "nobuffer",
                "-flags", "low_delay",
                "-framedrop",
                "-strict", "experimental",
                "-probesize", "32",
                "-analyzeduration", "0",
            ]

        # RTSP transport selection
        if url.lower().startswith("rtsp://") and opts.rtsp_transport in {"udp", "tcp"}:
            args += ["-rtsp_transport", opts.rtsp_transport]

        # Additional raw args
        if opts.extra_args:
            args += list(opts.extra_args)

        # Input URL
        args += ["-i", url]

        # Disable audio for simplicity (optional)
        args += ["-an"]

        # Auto exit if stream ends or fails quickly to avoid zombie windows
        args += ["-autoexit"]

        try:
            # Use a new process group so we can terminate cleanly on Windows
            # TODO: Solve the compatibility issues
            creationflags = 0
            try:
                # 0x00000200 CREATE_NEW_PROCESS_GROUP (Windows only)
                import sys
                if sys.platform.startswith("win"):
                    creationflags = 0x00000200
            except Exception:
                pass
            self._proc = subprocess.Popen(
                args,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.PIPE,
                creationflags=creationflags,
            )
        except FileNotFoundError:
            raise RuntimeError("ffplay executable not found. Install FFmpeg and ensure ffplay is on PATH.")

    def stop(self, timeout: float = 2.0) -> None:
        proc = self._proc
        if not proc:
            return
        try:
            if proc.poll() is None:
                proc.terminate()
                try:
                    proc.wait(timeout=timeout)
                except Exception:
                    proc.kill()
        finally:
            self._proc = None