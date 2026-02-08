"""
RoverStreamClient: ROS2 client utilities for controlling rover video streams.

This module uses direct ROS 2 command line calls (via subprocess) to interact
with the rover's camera manager service.

Usage pattern (typical):

    client = RoverStreamClient()
    url = client.start_stream("cam_front", bitrate_kbps=2000, codec="libx264")
"""

from __future__ import annotations

import re
import threading
import time
from dataclasses import dataclass
from typing import Optional, Tuple

import subprocess

@dataclass
class StreamRequestParams:
    camera_id: str
    bitrate_kbps: int = 2000
    codec: str = "libx264"
    resolution: str = "1280x720"
    framerate: int = 30
    transport: str = "udp"   # "rtsp" / "udp" / "srt"


class RoverStreamClient:
    """
    Wrapper to call ROS2 services via local subprocess.

    This client assumes the 'ros2' command is available in the environment
    (or via sourced setup scripts) and can communicate with the rover.
    """

    _instance_lock = threading.Lock()
    _instance: Optional["RoverStreamClient"] = None

    @classmethod
    def instance(cls) -> "RoverStreamClient":
        """Singleton accessor."""
        with cls._instance_lock:
            if cls._instance is None:
                cls._instance = cls()
            return cls._instance

    def __init__(self) -> None:
        self._lock = threading.Lock()

    def _exec_command(self, cmd: str) -> str:
        """Execute a command via local subprocess and return stdout."""
        
        # Prepare command with sourcing to ensure ROS 2 tools are available.
        # This attempts to source common locations.
        setup_cmd = (
             "source /opt/ros/humble/setup.zsh 2>/dev/null || source /opt/ros/humble/setup.bash 2>/dev/null || true; "
             "source /Users/egg/miniforge3/etc/profile.d/conda.sh 2>/dev/null && conda activate ros-humble 2>/dev/null || export PATH=/Users/egg/miniforge3/envs/ros-humble/bin:$PATH; " 
             "source ~/rover_software/install/setup.zsh 2>/dev/null || source ~/rover_software/install/setup.bash 2>/dev/null || "
             "source /Users/egg/Documents/VSCodeProjects/GNSS-GUI/rover_software/install/setup.zsh 2>/dev/null || true; "
             "export PATH=$PATH:/usr/local/bin:/usr/bin; "
        )
        full_cmd = f"{setup_cmd} {cmd}"

        try:
            # Run locally via subprocess
            # print(f"[RoverStreamClient] Executing: {cmd}")
            result = subprocess.run(
                full_cmd, 
                shell=True, 
                executable='/bin/zsh', # Use zsh for MacOS
                stdout=subprocess.PIPE, 
                stderr=subprocess.PIPE,
                timeout=5.0
            )
            out = result.stdout.decode('utf-8', errors='replace')
            err = result.stderr.decode('utf-8', errors='replace')
            
            if result.returncode != 0:
                print(f"[RoverStreamClient] Command Error: {err}")
                if not out:
                    raise RuntimeError(f"Command failed: {err}")
            
            return out
        except Exception as e:
            print(f"[RoverStreamClient] Execution failed: {e}")
            raise

    # ------------------------------------------------------------------ API

    def start_stream(self, params: StreamRequestParams, timeout: float = 10.0) -> str:
        """
        Start the stream for the given camera via ROS2 service call.
        """
        # Construct ROS2 service call command
        # Service type: multi_cam_streamer/srv/StartCameraStream
        yaml_args = (
            f"{{camera_id: '{params.camera_id}', "
            f"codec: '{params.codec}', "
            f"resolution: '{params.resolution}', "
            f"bitrate_kbps: {params.bitrate_kbps}, "
            f"framerate: {params.framerate}, "
            f"transport: '{params.transport}'}}"
        )
        
        cmd = f"ros2 service call /camera_manager/start_stream multi_cam_streamer/srv/StartCameraStream \"{yaml_args}\""
        
        try:
            output = self._exec_command(cmd)
            # Parse output looking for stream_url='...' or stream_url="..."
            # Example response:
            # response: 
            # multi_cam_streamer.srv.StartCameraStream_Response(success=True, message='OK', stream_url='udp://...')
            
            # Simple regex to find stream_url
            match = re.search(r"stream_url=['\"](.*?)['\"]", output)
            if match:
                url = match.group(1)
                if url:
                    return url
            
            # Fallback parsing if formatting differs
            if "success=True" in output:
                # If success but regex failed, maybe try to find the url differently 
                # or it might be empty.
                # Let's check for simple url pattern
                url_match = re.search(r"(udp|rtsp|srt)://[^\s'\"]+", output)
                if url_match:
                    return url_match.group(0)
            
            if "success=False" in output:
                raise RuntimeError(f"Service returned failure: {output}")

        except Exception as e:
            print(f"[RoverStreamClient] start_stream failed: {e}")
            raise RuntimeError(f"Failed to start stream: {e}")
            
        return "" # Should hopefully have returned by now if successful

    def restart_stream(self, params: StreamRequestParams, timeout: float = 15.0) -> str:
        """Stop and then Start the stream with new params."""
        try:
            self.stop_stream(params.camera_id)
            time.sleep(1.0) # Give it a moment to release resources
        except Exception:
            pass 
        return self.start_stream(params, timeout=timeout)

    def stop_stream(self, camera_id: str, timeout: float = 5.0) -> None:
        """Stop streaming for a given camera."""
        yaml_args = f"{{camera_id: '{camera_id}'}}"
        cmd = f"ros2 service call /camera_manager/stop_stream multi_cam_streamer/srv/StopCameraStream \"{yaml_args}\""
        
        try:
            self._exec_command(cmd)
        except Exception as e:
            print(f"[RoverStreamClient] stop_stream warning: {e}")

    def get_status(self, camera_id: str) -> Tuple[bool, bool, str, str]:
        """Return status tuple (known, streaming, url, last_error)."""
        yaml_args = f"{{camera_id: '{camera_id}'}}"
        cmd = f"ros2 service call /camera_manager/get_status multi_cam_streamer/srv/GetCameraStreamStatus \"{yaml_args}\""
        
        try:
            output = self._exec_command(cmd)
            # Parse response components
            known = "known=True" in output
            streaming = "streaming=True" in output
            
            url = ""
            u_match = re.search(r"stream_url=['\"](.*?)['\"]", output)
            if u_match:
                url = u_match.group(1)
                
            last_err = ""
            e_match = re.search(r"last_error=['\"](.*?)['\"]", output)
            if e_match:
                last_err = e_match.group(1)
                
            return (known, streaming, url, last_err)
        except Exception:
            return (False, False, "", "Connection Error")

    def shutdown(self) -> None:
        """Cleanup resources."""
        pass
