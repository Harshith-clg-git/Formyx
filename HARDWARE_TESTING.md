# Formyx Drone — Raspberry Pi 5 Hardware Testing Guide

> **Platform:** Raspberry Pi 5 · **OS:** Raspberry Pi OS 64-bit (Bookworm) · **Python:** 3.11+  
> **Flight Controller:** Radiolink PIX6 (ArduPilot) via USB  
> **Depth Camera:** Intel RealSense D435i (USB 3.0)  
> **Detection:** YOLOv8 — dual-class (`balloon` class 0 / `drone` class 1)

---

## Table of Contents

1. [Phase 0 — OS & System Prerequisites](#phase-0--os--system-prerequisites)
2. [Phase 1 — Clone the Repository](#phase-1--clone-the-repository)
3. [Phase 2 — Python Package Installation](#phase-2--python-package-installation)
4. [Phase 3 — YOLO Model Setup](#phase-3--yolo-model-setup)
5. [Phase 4 — Package Smoke Test](#phase-4--package-smoke-test)
6. [Phase 5 — Run Unit Tests (No Hardware)](#phase-5--run-unit-tests-no-hardware)
7. [Phase 6 — Milestone Hardware Tests](#phase-6--milestone-hardware-tests)
   - [Milestone 3 — Navigation Controller](#milestone-3--navigation-controller)
   - [Milestone 4 — Search Patterns](#milestone-4--search-patterns)
   - [Milestone 5 — Object Detection (Balloon & Drone)](#milestone-5--object-detection-balloon--drone)
   - [Milestone 6 — RealSense Depth](#milestone-6--realsense-depth)
   - [Milestone 7 — Kalman Tracker](#milestone-7--kalman-tracker)
   - [Milestone 8 — Target Loss & Reacquisition](#milestone-8--target-loss--reacquisition)
   - [Milestone 9 — Safety & Failsafes](#milestone-9--safety--failsafes)
   - [Milestone 10 — Logging](#milestone-10--logging)
8. [Summary Table](#summary-table)
9. [Safety Checklist](#safety-checklist)
10. [Useful Diagnostics](#useful-diagnostics)

---

## Phase 0 — OS & System Prerequisites

Run immediately after first boot.

### 0.1 System Update

```bash
sudo apt update && sudo apt full-upgrade -y
sudo reboot
```

### 0.2 Build Tools & Media Libraries

```bash
sudo apt install -y python3 python3-pip python3-venv python3-dev \
    build-essential cmake git \
    libatlas-base-dev libhdf5-dev libhdf5-serial-dev \
    libjpeg-dev libpng-dev libtiff-dev \
    libavcodec-dev libavformat-dev libswscale-dev \
    libv4l-dev libxvidcore-dev libx264-dev \
    libopenblas-dev gfortran \
    libusb-1.0-0-dev udev curl wget
```

### 0.3 Python Virtual Environment

```bash
cd ~
python3 -m venv formyx_env
source formyx_env/bin/activate
pip install --upgrade pip
```

> **Tip:** Add `source ~/formyx_env/bin/activate` to `~/.bashrc` so it activates on every SSH login.

---

## Phase 1 — Clone the Repository

```bash
cd ~
git clone https://github.com/Harshith-clg-git/Formyx.git
cd Formyx/formyx_backend
```

---

## Phase 2 — Python Package Installation

### 2.1 Core Packages

```bash
pip install \
    "opencv-python>=4.8.0" \
    "numpy>=1.23.0" \
    "pymavlink>=2.4.41" \
    "pyyaml>=6.0.1" \
    "pytest>=8.0.0" \
    "pytest-mock>=3.12.0"
```

> If `opencv-python` fails on ARM64, fall back to the system package:
> ```bash
> sudo apt install -y python3-opencv
> ```

### 2.2 Ultralytics / YOLOv8 (Milestone 5)

```bash
pip install "ultralytics>=8.3.0"
```

> ⏳ Downloads the PyTorch CPU build — allow 5–15 minutes. Requires ~3 GB free space.

### 2.3 Intel RealSense SDK — `pyrealsense2` (Milestone 6)

The standard pip wheel does **not** support ARM64. Use Intel's APT repository:

```bash
# Add Intel RealSense APT repo
sudo mkdir -p /etc/apt/keyrings
curl -sSf https://librealsense.intel.com/Debian/librealsense.pgp \
    | sudo tee /etc/apt/keyrings/librealsense.pgp > /dev/null

echo "deb [signed-by=/etc/apt/keyrings/librealsense.pgp] \
https://librealsense.intel.com/Debian/apt-repo \
$(lsb_release -cs) main" \
    | sudo tee /etc/apt/sources.list.d/librealsense.list

sudo apt update
sudo apt install -y librealsense2-dkms librealsense2-utils \
                    librealsense2-dev librealsense2-dbg

pip install pyrealsense2
```

**Verify the camera SDK:**

```bash
rs-enumerate-devices -s
# Expected: "Intel RealSense D435i" with "USB 3.2" or "USB 3.1"
```

> [!IMPORTANT]
> Always plug the D435i into the **blue USB 3.0 port** on the Pi 5.  
> A black USB 2.0 port will restrict bandwidth and cause frame failures.

### 2.4 Serial Port Permissions (PIX6 Flight Controller)

```bash
sudo usermod -aG dialout $USER
sudo usermod -aG tty $USER
sudo apt install -y minicom   # optional serial diagnostic tool
sudo reboot
```

After reboot, re-activate the venv:

```bash
source ~/formyx_env/bin/activate
cd ~/Formyx/formyx_backend
```

---

## Phase 3 — YOLO Model Setup

The detector uses a **dual-class** YOLOv8 model trained to detect:

| Class ID | Label    |
|----------|----------|
| `0`      | balloon  |
| `1`      | drone    |

Place the trained weights in the `models/` directory:

```bash
mkdir -p ~/Formyx/formyx_backend/models

# Copy the dual-class weights from your dev machine:
# scp drone_balloon_detector.pt pi@<raspi-ip>:~/Formyx/formyx_backend/models/

# Verify the file is present:
ls -lh ~/Formyx/formyx_backend/models/drone_balloon_detector.pt
```

If you also have a test video for the FPS benchmark (Milestone 5, Test Case 5.3):

```bash
mkdir -p ~/Formyx/formyx_backend/tests/assets
# scp test_balloon.mp4 pi@<raspi-ip>:~/Formyx/formyx_backend/tests/assets/
```

---

## Phase 4 — Package Smoke Test

Verify all packages are correctly installed before touching any hardware:

```bash
cd ~/Formyx/formyx_backend
python3 - <<'EOF'
import cv2;            print(f"OpenCV:       {cv2.__version__}")
import numpy;          print(f"NumPy:        {numpy.__version__}")
import pymavlink;      print(f"PyMAVLink:    {pymavlink.__version__}")
import yaml;           print(f"PyYAML:       {yaml.__version__}")
import ultralytics;    print(f"Ultralytics:  {ultralytics.__version__}")
import pyrealsense2 as rs; print(f"pyrealsense2: {rs.__version__}")
import pytest;         print(f"pytest:       {pytest.__version__}")
print("\n✅ All packages OK!")
EOF
```

---

## Phase 5 — Run Unit Tests (No Hardware)

Run the complete test suite using mocks (no physical hardware required):

```bash
cd ~/Formyx/formyx_backend
python3 -m pytest tests/ -v --tb=short 2>&1 | tee pytest_results.txt
```

> [!IMPORTANT]
> **All unit tests must pass before hardware testing begins.**  
> This confirms the codebase executes correctly in the Pi 5's Python environment.

---

## Phase 6 — Milestone Hardware Tests

Run each milestone in order. Detailed test case documents are in
[`tests/hardware/`](tests/hardware/).

---

### Milestone 3 — Navigation Controller

> **Hardware:** PIX6 via USB | **Props:** OFF for bench, ON (tethered) for flight  
> **Plan:** [`tests/hardware/test_plan_milestone_3.md`](tests/hardware/test_plan_milestone_3.md)

**Test Case 3.1 — Bench: Velocity Command Validation (Props OFF)**

```bash
cd ~/Formyx/formyx_backend
python3 - <<'EOF'
from navigation.follow_controller import FollowController

fc = FollowController(desired_follow_dist_m=3.0)
targets = [
    (5.0,   0.0,  0.0),   # Expect vx > 0  (~+1.0 m/s forward)
    (1.0,   0.0,  0.0),   # Expect vx < 0  (~-1.0 m/s backward)
    (3.0,   0.0,  0.0),   # Expect vx = 0  (at follow distance)
    (3.0,   4.0, -2.0),   # Expect vy > 0, vz < 0
    (103.0, 0.0, 10.0),   # Expect vx clamped to 3.0, vz clamped to -1.5
]
for t in targets:
    vx, vy, vz = fc.compute_velocity_command(t[0], t[1], t[2])
    print(f"Target {t}  →  vx={vx:.2f}  vy={vy:.2f}  vz={vz:.2f}")
EOF
```

**Test Case 3.2 — Tethered Flight:** See full procedure in the hardware test plan.

---

### Milestone 4 — Search Patterns

> **Hardware:** PIX6 via USB | **Props:** OFF for bench, ON (tethered) for flight  
> **Plan:** [`tests/hardware/test_plan_milestone_4.md`](tests/hardware/test_plan_milestone_4.md)

**Test Case 4.1 — Bench: Waypoint Boundary Validation (Props OFF)**

```bash
cd ~/Formyx/formyx_backend
python3 - <<'EOF'
from navigation.search_patterns import generate_expanding_square, generate_lawnmower

print("=== Expanding Square (step=3m, max_radius=10m) ===")
wps = generate_expanding_square(step_m=3.0, max_radius_m=10.0)
bad = [(x, y) for (x, y, *_) in wps if abs(x) > 10.0 or abs(y) > 10.0]
print(f"Waypoints: {len(wps)}  |  Violations: {bad}")
print("PASS ✅" if not bad else "FAIL ❌")

print("\n=== Lawnmower (width=6m, length=10m, step=3m) ===")
wps = generate_lawnmower(width_m=6.0, length_m=10.0, step_m=3.0)
bad = [(x, y) for (x, y, *_) in wps if not (0.0 <= x <= 10.0 and 0.0 <= y <= 6.0)]
print(f"Waypoints: {len(wps)}  |  Violations: {bad}")
print("PASS ✅" if not bad else "FAIL ❌")
EOF
```

**Test Case 4.2 — Tethered Flight:** See full procedure in the hardware test plan.

---

### Milestone 5 — Object Detection (Balloon & Drone)

> **Hardware:** Camera or D435i (no props, no flight required)  
> **Plan:** [`tests/hardware/test_plan_milestone_5.md`](tests/hardware/test_plan_milestone_5.md)

The detector uses a **single inference pass** to detect both balloons (class 0) and
drones (class 1). The configuration in `config/settings.yaml` controls active classes:

```yaml
perception:
  model_path: "models/drone_balloon_detector.pt"
  target_class_ids: [0, 1]   # 0 = balloon | 1 = drone
```

**Test Case 5.1 — Model Loading**

```bash
cd ~/Formyx/formyx_backend
python3 - <<'EOF'
from perception.detector import ObjectDetector
detector = ObjectDetector()
print("Model path    :", detector.model_path)
print("Active classes:", detector.target_class_ids)
# Expected: {0, 1}
EOF
```

**Test Case 5.2 — Live Detection (Balloon + Drone)**

```bash
cd ~/Formyx/formyx_backend
python3 - <<'EOF'
import cv2
from perception.detector import ObjectDetector

detector = ObjectDetector()
cap = cv2.VideoCapture(0)  # use 0 for webcam; wire up RealSense if preferred

ret, frame = cap.read()
if ret:
    all_dets  = detector.detect(frame)
    balloons  = detector.detect_balloons(frame)
    drones    = detector.detect_drones(frame)
    print("All detections :", all_dets)
    print("Balloons only  :", balloons)
    print("Drones only    :", drones)
else:
    print("FAIL ❌ — could not capture frame")
cap.release()
EOF
```

Hold a **balloon** in front of the lens → expect `label == "balloon"`, `class_id == 0`.  
Replace with a **drone** (or printed drone image) → expect `label == "drone"`, `class_id == 1`.

**Test Case 5.3 — FPS Benchmark (>10 FPS target)**

```bash
cd ~/Formyx/formyx_backend
python3 - <<'EOF'
import time, cv2
from perception.detector import ObjectDetector

detector = ObjectDetector()
cap = cv2.VideoCapture("tests/assets/test_balloon.mp4")

count, start = 0, time.monotonic()
while cap.isOpened():
    ret, frame = cap.read()
    if not ret: break
    detector.detect(frame)   # detects all classes in one pass
    count += 1

fps = count / (time.monotonic() - start)
cap.release()
print(f"Processed {count} frames  →  FPS = {fps:.2f}")
print("PASS ✅" if fps >= 10.0 else f"FAIL ❌  (target ≥ 10.0 FPS)")
EOF
```

---

### Milestone 6 — RealSense Depth

> **Hardware:** Intel RealSense D435i (blue USB 3.0 port!) | **No flight required**  
> **Plan:** [`tests/hardware/test_plan_milestone_6.md`](tests/hardware/test_plan_milestone_6.md)

**Test Case 6.1 — Device Enumeration**

```bash
rs-enumerate-devices -s
# Expected: "Intel RealSense D435i"  with  "USB 3.2" or "USB 3.1"
```

**Test Case 6.2 — Frame Capture & Alignment**

```bash
cd ~/Formyx/formyx_backend
python3 - <<'EOF'
from depth.realsense_interface import RealSenseInterface

iface = RealSenseInterface(use_mock=False)
iface.start()
frames = iface.get_frames()
assert frames is not None, "FAIL ❌ — get_frames() returned None"
color, depth = frames
print(f"Color shape: {color.shape}")   # expect (480, 640, 3)
print(f"Depth shape: {depth.shape}")   # expect (480, 640)
print(f"Mock mode  : {iface.is_mock}") # expect False
iface.stop()
print("PASS ✅")
EOF
```

**Test Case 6.3 — Depth Accuracy at 1 m / 2 m / 3 m**

```bash
cd ~/Formyx/formyx_backend
python3 - <<'EOF'
from depth.realsense_interface import RealSenseInterface

iface = RealSenseInterface(use_mock=False)
iface.start()
_, depth = iface.get_frames()
dist = iface.get_distance_at_pixel(depth, 320, 240)
print(f"Measured distance: {dist:.3f} m")
print("Expected tolerances: 1.0m→[0.95–1.05] | 2.0m→[1.90–2.10] | 3.0m→[2.85–3.15]")
iface.stop()
EOF
```

Run three times, placing the camera at 1 m, 2 m, then 3 m from a flat wall.

---

### Milestone 7 — Kalman Tracker

> **Hardware:** None required  
> **Plan:** [`tests/hardware/test_plan_milestone_7.md`](tests/hardware/test_plan_milestone_7.md)

**Test Case 7.1 — Reset & Lifecycle**

```bash
cd ~/Formyx/formyx_backend
python3 - <<'EOF'
from tracking.target_tracker import TargetTracker

tracker = TargetTracker()
print("Initialized              :", tracker.is_initialized)  # False
tracker.update((1.0, 2.0, 3.0))
print("After update             :", tracker.is_initialized)  # True
print("State                    :", tracker.get_state())     # (1,2,3,0,0,0)
tracker.reset()
print("After reset              :", tracker.is_initialized)  # False
print("State after reset        :", tracker.get_state())     # None
EOF
```

**Test Case 7.2 — Velocity Estimation & Occlusion Prediction**

```bash
cd ~/Formyx/formyx_backend
python3 - <<'EOF'
from tracking.target_tracker import TargetTracker

tracker = TargetTracker()
dt = 0.1
for step in range(21):
    tracker.predict(dt)
    tracker.update((5.0 * step * dt, 0.0, 0.0))

print("State at t=2.0s:", tracker.get_state())
print("Expected: x≈10.0m, vx≈4.8–5.2 m/s")

print("\n--- Simulating 0.5s dropout ---")
for _ in range(5):
    tracker.predict(dt)
print("Predicted after dropout:", tracker.get_state())
print("Expected: x≈12.2–12.5m (damped)")
EOF
```

**Test Case 7.3 — Outlier Rejection**

```bash
cd ~/Formyx/formyx_backend
python3 - <<'EOF'
from tracking.target_tracker import TargetTracker

tracker = TargetTracker()
tracker.update((2.0, 0.0, 0.0))
tracker.predict(0.1)

ok = tracker.update((2.2, 0.0, 0.0))
print(f"True update accepted : {ok}")          # True

bad = tracker.update((2.3, 10.0, 0.0))
print(f"Outlier accepted     : {bad}")         # False
print(f"State after outlier  : {tracker.get_state()}")
# x should stay near 2.2, NOT jump to 10.0
EOF
```

---

### Milestone 8 — Target Loss & Reacquisition

> **Hardware:** PIX6 + Camera | **Props:** OFF for bench, ON (tethered) for flight  
> **Plan:** [`tests/hardware/test_plan_milestone_8.md`](tests/hardware/test_plan_milestone_8.md)

**Test Case 8.1 — Bench: Visual Sweep Trigger (Props OFF)**

```bash
cd ~/Formyx/formyx_backend
# Run main.py in bench-test mode, inject fake target detections,
# then stop feeding them and wait 2.0 seconds.
# Expected log output:
#   [StateMachine] TRACKING → TARGET_LOST_RECOVERY
#   [Navigation]   MAV_CMD_CONDITION_YAW commands streamed (+15°, +30°, ...)
python3 main.py --bench-mode
```

**Test Case 8.2 — Tethered Reacquisition Flight:** See full procedure in the hardware test plan.

---

### Milestone 9 — Safety & Failsafes

> **Hardware:** PIX6 via USB | **Props: ALWAYS OFF** for all M9 tests  
> **Plan:** [`tests/hardware/test_plan_milestone_9.md`](tests/hardware/test_plan_milestone_9.md)

**Test Case 9.1 — Heartbeat Loss Failsafe**

```bash
# 1. Start backend in one terminal:
cd ~/Formyx/formyx_backend && python3 main.py

# 2. Wait for telemetry to be confirmed in logs.
# 3. Physically UNPLUG the USB cable from the PIX6.
# 4. Wait 3.0 seconds.
# Expected:
#   [Failsafe] HEARTBEAT_LOST detected
#   [StateMachine] → EMERGENCY
```

**Test Case 9.2 — Battery Thresholds**

```bash
# Inject mock battery levels to verify thresholds:
# 24% → BATTERY_WARNING (log only, mission continues)
# 14% → BATTERY_CRITICAL → RTL command issued
# See test_plan_milestone_9.md for the power supply / injection procedure
```

**Test Case 9.3 — Geofencing**

```bash
# Inject GPS 55m from home → GEOFENCE_BREACH → RTL
# Inject altitude 16m AGL  → GEOFENCE_BREACH → RTL
# See test_plan_milestone_9.md for exact telemetry injection steps
```

---

### Milestone 10 — Logging

> **Hardware:** None required  
> **Plan:** [`tests/hardware/test_plan_milestone_10.md`](tests/hardware/test_plan_milestone_10.md)

**Test Case 10.1 — Log File Creation**

```bash
cd ~/Formyx/formyx_backend
python3 - <<'EOF'
import threading
from logging_system.logger import BlackBoxLogger

logger = BlackBoxLogger()
logger.start()
threads = [t.name for t in threading.enumerate()]
print("Active threads:", threads)
assert "blackbox-writer" in threads, "FAIL ❌ — worker thread missing!"
logger.stop()
print("PASS ✅ — log file created and worker thread verified")
EOF
```

**Test Case 10.2 — 10 Hz Write Rate**

```bash
cd ~/Formyx/formyx_backend
python3 - <<'EOF'
import time
from logging_system.logger import BlackBoxLogger
from mavlink_interface.connection import TelemetrySnapshot

logger = BlackBoxLogger()
logger.start()
snap = TelemetrySnapshot(armed=True, flight_mode="GUIDED", lat_deg=12.9, lon_deg=77.5)

print("Logging at 10 Hz for 10 seconds...")
for _ in range(100):
    logger.log("SEARCHING", snap,
               target_vector=(1.0, 2.0, 3.0, 0.0, 0.0, 0.0),
               cmd_vector=(0.5, 0.0, 0.0))
    time.sleep(0.1)

logger.stop()
print("Check logs/ — expect ~100 rows:")
EOF
ls -lh logs/*.csv
wc -l logs/*.csv
```

**Test Case 10.3 — Log Rotation**

```bash
cd ~/Formyx/formyx_backend
python3 - <<'EOF'
import time, glob
from logging_system.logger import BlackBoxLogger

for i in range(7):
    lg = BlackBoxLogger()
    lg.start()
    time.sleep(0.5)
    lg.stop()
    time.sleep(0.2)

files = glob.glob("logs/*.csv")
print(f"Log files found: {len(files)}")
print("PASS ✅" if len(files) <= 5 else f"FAIL ❌ — expected ≤5, got {len(files)}")
EOF
```

---

## Summary Table

| Milestone | Hardware Needed | Props | Flight | Key Test |
|---|---|---|---|---|
| M3 — Navigation Controller | PIX6 USB | OFF / ON (tethered) | Optional | Velocity command clamping |
| M4 — Search Patterns | PIX6 USB | OFF / ON (tethered) | Optional | Waypoint boundary validation |
| M5 — Object Detection | Camera / D435i | OFF | No | Balloon + Drone detection; FPS ≥ 10 |
| M6 — RealSense Depth | D435i (USB 3.0!) | OFF | No | Depth accuracy at 1 m / 2 m / 3 m |
| M7 — Kalman Tracker | None | OFF | No | Outlier rejection, velocity estimation |
| M8 — Reacquisition | PIX6 + Camera | OFF / ON (tethered) | Optional | FSM TRACKING → LOST → RECOVERY → TRACKING |
| M9 — Safety & Failsafes | PIX6 USB | **ALWAYS OFF** | No | Heartbeat loss, battery, geofence |
| M10 — Logging | None | OFF | No | File creation, 10 Hz write rate, rotation |

---

## Safety Checklist

Before any test session, verify:

- [ ] Propellers **physically removed** for all bench tests (M3, M4, M8 bench; all of M9)
- [ ] PIX6 detected: `ls /dev/ttyACM* /dev/ttyUSB*`
- [ ] D435i plugged into **blue USB 3.0** port only
- [ ] For any flight test: 5 m tether secured, safety pilot on RC ready to override
- [ ] Geofence set to ≤ 10 m in ArduPilot params before any flight
- [ ] YOLO model file present: `ls -lh models/drone_balloon_detector.pt`

---

## Useful Diagnostics

```bash
# List serial ports (PIX6 shows as ttyACM0 or ttyUSB0)
ls /dev/ttyACM* /dev/ttyUSB*

# Quick MAVLink link check
mavproxy.py --master=/dev/ttyACM0 --baudrate=57600

# Verify RealSense USB speed (look for "5000M" = USB 3.x)
lsusb -t

# Monitor CPU / RAM during inference
htop

# Check free disk space (important before logging tests)
df -h

# Re-activate virtualenv after reboot
source ~/formyx_env/bin/activate && cd ~/Formyx/formyx_backend
```
