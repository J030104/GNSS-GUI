"""Detect all available cameras one at a time, write results to file."""
import cv2

lines = []
lines.append("=== Camera Detection Results ===\n")

for i in range(6):
    cap = cv2.VideoCapture(i)
    if cap.isOpened():
        ok, frame = cap.read()
        w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        backend = cap.getBackendName() if hasattr(cap, 'getBackendName') else "?"
        lines.append(f"Camera {i}: FOUND  {w}x{h}  backend={backend}  frame={'YES' if ok else 'NO'}")
    else:
        lines.append(f"Camera {i}: NOT FOUND")
    cap.release()

result = "\n".join(lines)
print(result)

with open("camera_results.txt", "w") as f:
    f.write(result)
print("\nResults also saved to camera_results.txt")
