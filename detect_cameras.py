"""Detect all available cameras and probe supported resolutions."""
import cv2

# Common resolutions to test (width, height)
RESOLUTIONS = [
    (320, 240),    # QVGA
    (640, 480),    # VGA (4:3)
    (800, 600),    # SVGA (4:3)
    (1280, 720),   # 720p (16:9)
    (1280, 960),   # (4:3)
    (1920, 1080),  # 1080p (16:9)
    (2560, 1440),  # 1440p (16:9)
    (3840, 2160),  # 4K (16:9)
]

# Backends to try on Windows
BACKENDS = [
    (cv2.CAP_MSMF, "MSMF"),      # Media Foundation (Windows default)
    (cv2.CAP_DSHOW, "DSHOW"),    # DirectShow
    (cv2.CAP_ANY, "ANY"),        # Auto-detect
]

lines = []
lines.append("=== Camera Detection Results ===\n")

found_cameras = {}  # Track which cameras we found and with which backend

print("Scanning for cameras...", end="", flush=True)

for i in range(10):  # Scan more indices
    print(f" {i}", end="", flush=True)
    for backend_id, backend_name in BACKENDS:
        if i in found_cameras:
            break  # Already found this camera with another backend

        try:
            cap = cv2.VideoCapture(i, backend_id)
            if cap.isOpened():
                ok, frame = cap.read()
                if ok and frame is not None:
                    found_cameras[i] = backend_name
                    default_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
                    default_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
                    lines.append(f"Camera {i}: FOUND  (default {default_w}x{default_h})  backend={backend_name}")

                    # Test each resolution
                    supported = []
                    for w, h in RESOLUTIONS:
                        cap.set(cv2.CAP_PROP_FRAME_WIDTH, w)
                        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, h)
                        actual_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
                        actual_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
                        if actual_w == w and actual_h == h:
                            supported.append(f"    {w}x{h} ✓")
                        else:
                            supported.append(f"    {w}x{h} ✗ (got {actual_w}x{actual_h})")
                    lines.append("  Supported resolutions:")
                    lines.extend(supported)
                    lines.append("")
                cap.release()
        except Exception:
            pass

print(" done!\n")

# Report cameras not found
for i in range(10):
    if i not in found_cameras:
        lines.append(f"Camera {i}: NOT FOUND")

result = "\n".join(lines)
print(result)

print(f"\nSummary: Found {len(found_cameras)} camera(s)")
if found_cameras:
    for idx, backend in found_cameras.items():
        print(f"  - Camera {idx} ({backend})")

with open("camera_results.txt", "w", encoding="utf-8") as f:
    f.write(result)
print("\nResults saved to camera_results.txt")
