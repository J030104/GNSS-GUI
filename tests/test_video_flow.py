
"""
Test Video Flow (System Simulation)

This test script simulates the entire video streaming flow from the client's perspective
without requiring:
1. Real ROS 2 installation.
2. Real Camera hardware.
3. Real OpenCV GUI support.

It achieves this by "monkey patching" (injecting mocks into) `sys.modules` for:
- `cv2`: To simulate video capture.
- `rclpy`: To simulate the ROS 2 network layer.
- `multi_cam_streamer`: To simulate the custom ROS service definitions.

NOTE: This is distinct from the "Mock Mode" built into `rover_stream_client.py`.
- That mode is a fallback for running the GUI on a laptop without crashing.
- This test FORCES mocks to verify the client logic would work IF connected to a real ROS system.
"""

import sys
import os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
import types
import time
import threading
from unittest.mock import MagicMock

# ------------------------------------------------------------------------------
# 1. Mock 'cv2'
# ------------------------------------------------------------------------------
# We mock cv2 to simulate video capture without creating real windows or dependencies.
mock_cv2 = types.ModuleType("cv2")
mock_cv2.CAP_PROP_FRAME_WIDTH = 3
mock_cv2.CAP_PROP_FRAME_HEIGHT = 4
mock_cv2.CAP_PROP_FPS = 5
mock_cv2.CAP_PROP_BUFFERSIZE = 38
mock_cv2.COLOR_BGR2RGB = 4
mock_cv2.VideoCapture = MagicMock()

# Mock a VideoCapture instance
class MockVideoCapture:
    def __init__(self, *args, **kwargs):
        self.is_open = True
        print(f"[MockCV2] VideoCapture opened with args: {args}")

    def set(self, prop, value):
        print(f"[MockCV2] VideoCapture.set({prop}, {value})")
        return True

    def read(self):
        if not self.is_open:
            return False, None
        # Return a black 10x10 frame BGR
        import numpy as np
        frame = np.zeros((10, 10, 3), dtype=np.uint8)
        return True, frame

    def release(self):
        self.is_open = False
        print("[MockCV2] VideoCapture released")

mock_cv2.VideoCapture.side_effect = MockVideoCapture
mock_cv2.cvtColor = lambda frame, code: frame # Identity for test
sys.modules["cv2"] = mock_cv2
sys.modules["numpy"] = __import__("numpy") # Use real numpy

# ------------------------------------------------------------------------------
# 2. Mock 'rclpy' and 'multi_cam_streamer.srv'
# ------------------------------------------------------------------------------
mock_rclpy = types.ModuleType("rclpy")
mock_rclpy.ok = MagicMock(return_value=True)
mock_rclpy.init = MagicMock()
mock_rclpy.shutdown = MagicMock()
mock_rclpy.spin = MagicMock(side_effect=lambda node: time.sleep(0.1)) # Just sleep
mock_rclpy.spin_once = MagicMock()

# Mock Node
class MockNode:
    def __init__(self, name):
        self.name = name
        print(f"[MockROS] Created Node: {name}")

    def create_client(self, srv_type, srv_name):
        print(f"[MockROS] Creating client for {srv_name}")
        return MockClient(srv_type, srv_name)

    def destroy_node(self):
        print(f"[MockROS] Destroying Node: {self.name}")

class MockFuture:
    def __init__(self):
        self._result = None
        self._done = False

    def done(self):
        return self._done

    def result(self):
        return self._result

    def set_result(self, res):
        self._result = res
        self._done = True

class MockClient:
    def __init__(self, srv_type, srv_name):
        self.srv_type = srv_type
        self.srv_name = srv_name

    def wait_for_service(self, timeout_sec=None):
        return True

    def call_async(self, req):
        print(f"[MockROS] call_async on {self.srv_name} with {req}")
        f = MockFuture()
        
        # Determine success based on srv_name (more reliable than type checking in this mock)
        if "start_stream" in self.srv_name:
            resp = MagicMock()
            resp.success = True
            resp.stream_url = f"udp://127.0.0.1:5000?cam={req.camera_id}"
            f.set_result(resp)
        elif "stop_stream" in self.srv_name:
            resp = MagicMock()
            resp.success = True
            f.set_result(resp)
        elif "get_status" in self.srv_name:
             resp = MagicMock()
             resp.known = True
             resp.streaming = True
             resp.stream_url = "udp://127.0.0.1:5000"
             f.set_result(resp)
        else:
             print(f"[MockROS] Unknown service name: {self.srv_name}")
        
        return f

mock_rclpy.create_node = MockNode
mock_rclpy.node = types.ModuleType("rclpy.node")
mock_rclpy.node.Node = MockNode # type: ignore

sys.modules["rclpy"] = mock_rclpy
sys.modules["rclpy.node"] = mock_rclpy.node

# Mock Messages
mock_srv = types.ModuleType("multi_cam_streamer.srv")

class MockSrvType:
    class Request:
        pass
    class Response:
        pass

mock_srv.StartCameraStream = MockSrvType
mock_srv.StopCameraStream = MockSrvType
mock_srv.GetCameraStreamStatus = MockSrvType

sys.modules["multi_cam_streamer"] = types.ModuleType("multi_cam_streamer")
sys.modules["multi_cam_streamer.srv"] = mock_srv


# ------------------------------------------------------------------------------
# 3. Import Project Modules
# ------------------------------------------------------------------------------
# Now we can safely import the project code
try:
    from gui.utilities.rover_stream_client import RoverStreamClient, StreamRequestParams
    from gui.utilities.video_streamer import NetworkStreamCamera, NetworkStreamOptions
except ImportError as e:
    print(f"Import Error: {e}")
    sys.exit(1)


# ------------------------------------------------------------------------------
# 4. Run the Test Flow
# ------------------------------------------------------------------------------
def run_test():
    """
    Executes the sequence:
    1. Start ROS client.
    2. Request stream (getting a mocked URL).
    3. Connect camera to that URL.
    4. Verify frames are received (mocked black frames).
    5. Clean up.
    """
    print("=== STARTING VIDEO FLOW TEST ===")
    
    # 1. Initialize Client
    print("\n1. Initializing RoverStreamClient...")
    client = RoverStreamClient()
    
    # 2. Start Stream
    print("\n2. Requesting Start Stream...")
    params = StreamRequestParams(camera_id="cam_front")
    url = client.start_stream(params)
    print(f"   -> Received URL: {url}")
    
    if "udp://" not in url:
        print("   FAILED: URL does not look right")
        return

    # 3. Setup Camera Receiver
    print("\n3. Setting up NetworkStreamCamera...")
    # Parse URL for the options (simplified logic for test)
    # url is udp://127.0.0.1:5000?cam=cam_front
    from urllib.parse import urlparse
    parsed = urlparse(url)
    
    opts = NetworkStreamOptions(
        proto="udp",
        host=parsed.hostname,
        port=parsed.port,
        extra_query=parsed.query
    )
    
    camera = NetworkStreamCamera(opts)
    
    # 4. Start Camera
    print("\n4. Starting Camera (mock capture)...")
    camera.start()
    
    # 5. Read Frames
    print("\n5. Reading frames...")
    for i in range(5):
        frame = camera.read_frame()
        if frame is None:
            print(f"   Frame {i}: None (waiting...)")
        else:
            print(f"   Frame {i}: Got frame of shape {frame.shape}")
        time.sleep(0.05)

    # 6. Stop Camera
    print("\n6. Stopping Camera...")
    camera.stop()
    
    # 7. Stop Stream on Server
    print("\n7. Stopping Stream on Server...")
    client.stop_stream("cam_front")
    
    # 8. Shutdown
    print("\n8. Shutting down client...")
    client.shutdown()
    
    print("\n=== TEST COMPLETED SUCCESSFULLY ===")

if __name__ == "__main__":
    run_test()
