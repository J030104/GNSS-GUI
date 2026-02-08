from __future__ import annotations

import os
import re
import shlex
import subprocess
import textwrap
import threading
import time
from dataclasses import dataclass
from typing import Optional, Tuple


@dataclass
class StreamRequestParams:
    camera_id: str
    bitrate_kbps: int = 2000
    codec: str = "libx264"
    resolution: str = "1280x720"
    framerate: int = 30
    transport: str = "udp"   # "rtsp" / "udp" / "srt"


class RoverStreamClient:
    _instance_lock = threading.Lock()
    _instance: Optional["RoverStreamClient"] = None

    @classmethod
    def instance(cls) -> "RoverStreamClient":
        with cls._instance_lock:
            if cls._instance is None:
                cls._instance = cls()
            return cls._instance

    def __init__(self) -> None:
        self._lock = threading.Lock()

    def _exec_command(self, cmd: str, timeout: float = 15.0) -> str:
        import os, shlex, subprocess, textwrap

        conda_bin = "/Users/egg/miniforge3/envs/ros-humble/bin"
        overlay = "/Users/egg/Documents/VSCodeProjects/GNSS-GUI/rover_software/install/setup.zsh"

        script = textwrap.dedent(f"""
            set -eo pipefail

            # Use the ros-humble conda env binaries directly (no conda activate)
            export PATH={shlex.quote(conda_bin)}:$PATH

            # Force middleware + domain (your Fix A)
            export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
            export ROS_DOMAIN_ID=0

            # Source overlay to populate AMENT_PREFIX_PATH, PYTHONPATH, etc.
            if [ ! -f {shlex.quote(overlay)} ]; then
            echo "ERROR: overlay not found: {overlay}" 1>&2
            exit 2
            fi
            source {shlex.quote(overlay)}

            # Hard sanity checks (now should succeed)
            command -v ros2
            test -n "${{AMENT_PREFIX_PATH:-}}"
            ros2 interface show multi_cam_streamer/srv/StartCameraStream >/dev/null

            {cmd}
        """).strip()

        try:
            result = subprocess.run(
                ["/bin/zsh", "-lc", script],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                timeout=timeout,
                text=True
            )
        except subprocess.TimeoutExpired as e:
            raise RuntimeError(
                f"Command timed out after {timeout}s.\nCMD: {cmd}\n"
                f"STDOUT:\n{e.stdout or ''}\nSTDERR:\n{e.stderr or ''}"
            ) from e

        if result.returncode != 0:
            raise RuntimeError(
                f"Command failed (rc={result.returncode}).\nCMD: {cmd}\n"
                f"STDOUT:\n{result.stdout}\nSTDERR:\n{result.stderr}"
            )

        return result.stdout


    # ------------------------------------------------------------------ API

    def start_stream(self, params: StreamRequestParams, timeout: float = 10.0) -> str:
        yaml_args = (
            f"{{camera_id: '{params.camera_id}', "
            f"codec: '{params.codec}', "
            f"resolution: '{params.resolution}', "
            f"bitrate_kbps: {params.bitrate_kbps}, "
            f"framerate: {params.framerate}, "
            f"transport: '{params.transport}'}}"
        )

        # IMPORTANT: quote YAML safely (no nested shell quoting issues)
        req = shlex.quote(yaml_args)
        cmd = (
            "ros2 service call "
            "/camera_manager/start_stream "
            "multi_cam_streamer/srv/StartCameraStream "
            f"{req}"
        )

        output = self._exec_command(cmd, timeout=timeout)  # <-- pass timeout through

        m = re.search(r"stream_url=['\"](.*?)['\"]", output)
        if m and m.group(1):
            return m.group(1)

        if "success=True" in output:
            u = re.search(r"(udp|rtsp|srt)://[^\s'\"]+", output)
            if u:
                return u.group(0)

        if "success=False" in output:
            raise RuntimeError(f"Service returned failure:\n{output}")

        raise RuntimeError(f"Unexpected service output:\n{output}")

    def restart_stream(self, params: StreamRequestParams, timeout: float = 15.0) -> str:
        try:
            self.stop_stream(params.camera_id)
            time.sleep(1.0)
        except Exception:
            pass
        return self.start_stream(params, timeout=timeout)

    def stop_stream(self, camera_id: str, timeout: float = 15.0) -> None:
        yaml_args = f"{{camera_id: '{camera_id}'}}"
        req = shlex.quote(yaml_args)
        cmd = (
            "ros2 service call "
            "/camera_manager/stop_stream "
            "multi_cam_streamer/srv/StopCameraStream "
            f"{req}"
        )
        try:
            self._exec_command(cmd, timeout=timeout)
        except Exception as e:
            print(f"[RoverStreamClient] stop_stream warning: {e}")

    def get_status(self, camera_id: str) -> Tuple[bool, bool, str, str]:
        yaml_args = f"{{camera_id: '{camera_id}'}}"
        req = shlex.quote(yaml_args)
        cmd = (
            "ros2 service call "
            "/camera_manager/get_status "
            "multi_cam_streamer/srv/GetCameraStreamStatus "
            f"{req}"
        )
        try:
            output = self._exec_command(cmd, timeout=10.0)
            known = "known=True" in output
            streaming = "streaming=True" in output

            url = ""
            u = re.search(r"stream_url=['\"](.*?)['\"]", output)
            if u:
                url = u.group(1)

            last_err = ""
            e = re.search(r"last_error=['\"](.*?)['\"]", output)
            if e:
                last_err = e.group(1)

            return (known, streaming, url, last_err)
        except Exception as ex:
            return (False, False, "", f"Connection Error: {ex}")

    def shutdown(self) -> None:
        pass
