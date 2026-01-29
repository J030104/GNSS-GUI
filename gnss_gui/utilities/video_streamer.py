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

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.executors import MultiThreadedExecutor
    from sensor_msgs.msg import Image
    from cv_bridge import CvBridge
    from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy, QoSDurabilityPolicy
    HAS_ROS = True
except ImportError:
    HAS_ROS = False

# Global ROS context management for the GUI
_ros_init_lock = threading.Lock()
_ros_initialized = False
_ros_executor: "MultiThreadedExecutor | None" = None
_ros_executor_thread: "threading.Thread | None" = None


def _ensure_ros_initialized() -> bool:
    """Thread-safe ROS initialization. Returns True if ROS is available."""
    global _ros_initialized, _ros_executor, _ros_executor_thread
    
    if not HAS_ROS:
        return False
    
    with _ros_init_lock:
        if _ros_initialized and rclpy.ok():
            return True
        
        try:
            if not rclpy.ok():
                rclpy.init()
            
            # Create a shared multi-threaded executor for all subscriber nodes
            _ros_executor = MultiThreadedExecutor()
            
            def _spin_executor():
                try:
                    _ros_executor.spin()
                except Exception as e:
                    print(f"[ROS] Executor spin error: {e}")
            
            _ros_executor_thread = threading.Thread(target=_spin_executor, daemon=True)
            _ros_executor_thread.start()
            
            _ros_initialized = True
            print("[ROS] Context initialized with shared executor")
            return True
        except Exception as e:
            print(f"[ROS] Failed to initialize: {e}")
            return False


def _add_node_to_executor(node: "Node") -> None:
    """Add a ROS node to the shared executor."""
    global _ros_executor
    if _ros_executor is not None:
        try:
            _ros_executor.add_node(node)
        except Exception as e:
            print(f"[ROS] Failed to add node to executor: {e}")


def _remove_node_from_executor(node: "Node") -> None:
    """Remove a ROS node from the shared executor."""
    global _ros_executor
    if _ros_executor is not None:
        try:
            _ros_executor.remove_node(node)
        except Exception as e:
            print(f"[ROS] Failed to remove node from executor: {e}")

from ..config import CAMERA_NAMES

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


class VideoSubscriberNode(Node if HAS_ROS else object):
    """ROS2 Node for receiving video frames from a specified topic."""
    def __init__(self, topic: str = '/rover/camera_feed'):
        if not HAS_ROS:
            return
        # Create unique node name based on topic
        node_name = 'video_subscriber_' + topic.replace('/', '_').strip('_')
        super().__init__(node_name)
        self.topic = topic
        self.latest_frame = None
        self.bridge = CvBridge()
        self._lock = threading.Lock()
        
        # QoS Profile: Must match Publisher (Best Effort, Volatile)
        qos_profile = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE
        )
        
        self.subscription = self.create_subscription(
            Image,
            self.topic,
            self.listener_callback,
            qos_profile
        )
        self.get_logger().info(f"Subscribing to {self.topic}")

    def listener_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV image
            # We want RGB for the GUI, but cv_bridge usually returns BGR for "bgr8".
            # If the publisher sends "bgr8", we receive it here.
            # The GUI expects RGB.
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
            
            with self._lock:
                self.latest_frame = cv_image
        except Exception as e:
            self.get_logger().error(f"Error converting image: {e}")

    def get_frame_count(self):
        """Debug helper to check if frames are being received."""
        with self._lock:
            return self.latest_frame is not None

class ROSCameraSubscriber(CameraSource):
    """Receives video from a ROS2 topic.
    
    Uses a shared ROS executor to avoid spawning multiple spin threads.
    The executor is managed globally for the entire GUI application.
    """
    
    def __init__(self, topic: str = '/rover/camera_feed'):
        if not HAS_ROS:
             raise RuntimeError("ROS2 libraries not available")
        self.topic = topic
        self.node: "VideoSubscriberNode | None" = None
        
    def start(self) -> None:
        if self.node is not None:
            return
        
        # Use the shared ROS context and executor
        if not _ensure_ros_initialized():
            print(f"[ROSCameraSubscriber] Failed to initialize ROS for {self.topic}")
            return
            
        self.node = VideoSubscriberNode(topic=self.topic)
        _add_node_to_executor(self.node)
        print(f"[ROSCameraSubscriber] Created and registered subscriber node for {self.topic}")

    def stop(self) -> None:
        """Stop the subscriber and remove the node from the executor."""
        if self.node:
            try:
                _remove_node_from_executor(self.node)
                self.node.destroy_node()
            except Exception:
                pass
            self.node = None
        # We don't shutdown rclpy here as it's shared by other components

    def read_frame(self):
        if self.node:
            with self.node._lock:
                 if self.node.latest_frame is not None:
                      return self.node.latest_frame.copy()
        return None


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

    The module registers camera instances based on CAMERA_CONFIG in config.py.
    Supports multiple transport types: local, udp, ros2, rtsp.
    """

    _registry: Dict[str, CameraSource] = {}
    _initialized: bool = False

    @classmethod
    def register(cls, name: str, source: CameraSource) -> None:
        """Register a camera source with a given name."""
        cls._registry[name] = source

    @classmethod
    def get_camera(cls, name: str) -> Optional[CameraSource]:
        """Get a registered camera by name. Initializes cameras on first call."""
        if not cls._initialized:
            cls._initialize_cameras()
        return cls._registry.get(name)

    @classmethod
    def _detect_local_camera_index(cls) -> int:
        """Auto-detect the local camera index (MacOS: find MacBook Pro Camera)."""
        import platform
        cam_idx = 0
        
        if platform.system() == "Darwin":
            try:
                result = subprocess.run(
                    ["ffmpeg", "-f", "avfoundation", "-list_devices", "true", "-i", ""],
                    stderr=subprocess.PIPE,
                    stdout=subprocess.PIPE,
                    text=True
                )
                import re
                for line in result.stderr.split('\n'):
                    if "MacBook Pro Camera" in line and "Desk View" not in line:
                        match = re.search(r'\[(\d+)\]', line)
                        if match:
                            cam_idx = int(match.group(1))
                            print(f"[CameraManager] Found MacBook Pro Camera at index {cam_idx}")
                            break
            except Exception as e:
                print(f"[CameraManager] Error auto-detecting camera: {e}")
                cam_idx = 1  # Fallback
        
        return cam_idx

    @classmethod
    def _initialize_cameras(cls) -> None:
        """Initialize all cameras based on CAMERA_CONFIG."""
        if cls._initialized:
            return
        cls._initialized = True
        
        if cv2 is None:
            print("[CameraManager] OpenCV not available, skipping camera initialization")
            return
        
        # Import config here to avoid circular imports
        try:
            from ..config import CAMERA_CONFIG, CAMERA_NAMES
        except ImportError:
            print("[CameraManager] Could not import CAMERA_CONFIG, using defaults")
            return
        
        for cam_key, cfg in CAMERA_CONFIG.items():
            transport = cfg.get("transport", "disabled")
            cam_name = CAMERA_NAMES.get(cam_key, cam_key)
            
            if transport == "disabled":
                continue
            
            try:
                if transport == "local":
                    # Local camera (webcam)
                    device_index = cfg.get("device_index", 0)
                    if device_index == "auto":
                        device_index = cls._detect_local_camera_index()
                    cam = LocalCamera(index=int(device_index))
                    cls.register("local", cam)  # Register as 'local' for backward compat
                    print(f"[CameraManager] Registered '{cam_name}' as local camera (index {device_index})")
                
                elif transport == "ros2":
                    # ROS2 subscriber
                    if HAS_ROS:
                        topic = cfg.get("topic", f"/rover/{cam_key.lower()}")
                        ros_cam = ROSCameraSubscriber(topic=topic)
                        cls.register(cam_name, ros_cam)
                        print(f"[CameraManager] Registered '{cam_name}' as ROS2 subscriber on {topic}")
                    else:
                        print(f"[CameraManager] ROS2 not available, skipping '{cam_name}'")
                
                elif transport == "udp":
                    # UDP network stream
                    opts = NetworkStreamOptions(
                        proto="udp",
                        host=cfg.get("host", "0.0.0.0"),
                        port=cfg.get("port", 5000),
                        buffer_size=cfg.get("buffer_size", 1)
                    )
                    net_cam = NetworkStreamCamera(options=opts)
                    cls.register(cam_name, net_cam)
                    print(f"[CameraManager] Registered '{cam_name}' as UDP stream (port {cfg.get('port')})")
                
                elif transport == "rtsp":
                    # RTSP stream
                    opts = NetworkStreamOptions(
                        proto="rtsp",
                        host=cfg.get("host", "127.0.0.1"),
                        port=cfg.get("port", 8554),
                        path=cfg.get("path", ""),
                        rtsp_transport=cfg.get("rtsp_transport", "udp"),
                        buffer_size=cfg.get("buffer_size", 1)
                    )
                    net_cam = NetworkStreamCamera(options=opts)
                    cls.register(cam_name, net_cam)
                    print(f"[CameraManager] Registered '{cam_name}' as RTSP stream")
                
                else:
                    print(f"[CameraManager] Unknown transport '{transport}' for '{cam_name}'")
            
            except Exception as e:
                print(f"[CameraManager] Error registering '{cam_name}': {e}")


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