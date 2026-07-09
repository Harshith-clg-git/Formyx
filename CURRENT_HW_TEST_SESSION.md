# Formyx — Hardware Test Session
## Setup: Raspberry Pi 5 · Radiolink PIX6 (USB) · Intel RealSense D435i

> **Date:** ___________  
> **Tester:** ___________  
> **Pi IP:** ___________  
> **Props attached?** NO — propellers must be physically absent for all tests in this guide.  
> **GPS attached?** NO — tethered flight and GUIDED-mode flight tests are excluded.

---

## Pre-Session Checklist

Before running any test, confirm the following:

- [ ] Pi 5 is powered and SSH is reachable
- [ ] PIX6 connected via USB (`ls /dev/ttyACM* /dev/ttyUSB*` shows a device)
- [ ] RealSense D435i plugged into the **blue USB 3.0 port** on the Pi 5
- [ ] Virtual environment activated (`source ~/formyx_env/bin/activate`)
- [ ] Repository is up to date (`git -C ~/Formyx pull origin master`)
- [ ] Working directory set (`cd ~/Formyx/formyx_backend`)

---

## Stage 0 — System & Port Discovery

### 0.1 Find the PIX6 Serial Port

```bash
ls /dev/ttyACM* /dev/ttyUSB*
```

> **Expected:** at least one device, e.g. `/dev/ttyACM0`  
> **Record port:** ___________

### 0.2 Verify RealSense USB Speed

```bash
rs-enumerate-devices -s
lsusb -t | grep -i "Intel"
```

> **Expected:** `Intel RealSense D435i` with `5000M` (USB 3.x)  
> ⚠️ If you see `480M` (USB 2.0), move the cable to the blue port and rerun.

**Result:** ☐ PASS  ☐ FAIL  
**Notes:** ___________

---

## Stage 1 — Package Smoke Test (No Hardware Interaction)

Verifies all Python packages imported correctly on the Pi's ARM64 environment.

```bash
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

**Result:** ☐ PASS  ☐ FAIL  
**Notes:** ___________

---

## Stage 2 — Full Unit Test Suite (No Hardware)

All mocked unit tests must pass before hardware testing begins.

```bash
python3 -m pytest tests/ -v --tb=short 2>&1 | tee /tmp/pytest_results.txt
echo "Exit code: $?"
```

> **Expected:** All tests `PASSED`. Zero failures.

**Total tests run:** ___________  
**Passed:** ___________  **Failed:** ___________  
**Result:** ☐ PASS  ☐ FAIL  
**Failed test names (if any):** ___________

---

## Stage 3 — Milestone 1: MAVLink Connection & Telemetry

> **Hardware:** PIX6 via USB  
> **Props:** OFF  

### Test 1.1 — MAVProxy Quick Link Check

Replace `/dev/ttyACM0` with your actual port from Stage 0.1.

```bash
mavproxy.py --master=/dev/ttyACM0 --baudrate=57600
```

> **Expected:** Heartbeat messages streaming every ~1 second.  
> Type `status` to see telemetry. Press `Ctrl+C` to exit.

**Result:** ☐ PASS  ☐ FAIL  
**Heartbeat rate observed:** ___________ Hz  
**Notes:** ___________

---

### Test 1.2 — Python MAVLink Connection & Telemetry Snapshot

```bash
python3 - <<'EOF'
import time
from mavlink_interface.connection import MAVLinkConnection

conn = MAVLinkConnection()
conn.connect()

print("Waiting 5 s for telemetry...")
time.sleep(5)

snap = conn.get_telemetry()
print(f"  connected       : {snap.connected}")
print(f"  armed           : {snap.armed}")
print(f"  flight_mode     : {snap.flight_mode}")
print(f"  battery_voltage : {snap.battery_voltage_v:.2f} V")
print(f"  battery_pct     : {snap.battery_remaining_pct} %")
print(f"  lat / lon       : {snap.lat_deg} / {snap.lon_deg}")
print(f"  alt_rel_m       : {snap.alt_rel_m:.2f} m")
print(f"  heading_deg     : {snap.heading_deg:.1f} °")
print(f"  groundspeed_ms  : {snap.groundspeed_ms:.2f} m/s")

assert snap.connected, "FAIL ❌ — not connected"
print("\nPASS ✅ — telemetry snapshot received")

conn.disconnect()
EOF
```

**Result:** ☐ PASS  ☐ FAIL  
**Battery voltage:** ___________ V  **Battery %:** ___________ %  
**Notes:** ___________

---

### Test 1.3 — 60-Second Heartbeat Stability (0% Packet Loss)

```bash
python3 - <<'EOF'
import time
from mavlink_interface.connection import MAVLinkConnection

conn = MAVLinkConnection()
conn.connect()

print("Monitoring heartbeat for 60 seconds...")
start    = time.monotonic()
last_hb  = None
received = 0
missed   = 0

while time.monotonic() - start < 60:
    snap = conn.get_telemetry()
    if snap.connected:
        if last_hb != snap.last_heartbeat_ts:
            last_hb = snap.last_heartbeat_ts
            received += 1
    else:
        missed += 1
    time.sleep(1)

total = received + missed
loss  = (missed / total * 100) if total else 100
print(f"\nHeartbeats received : {received} / {total}")
print(f"Packet loss         : {loss:.1f}%")
print("PASS ✅" if loss == 0 else f"FAIL ❌ ({loss:.1f}% loss)")
conn.disconnect()
EOF
```

**Result:** ☐ PASS  ☐ FAIL  
**Packet loss %:** ___________  
**Notes:** ___________

---

## Stage 4 — Milestone 3: Navigation Controller (Bench)

> **Hardware:** None (pure math, no MAVLink commands sent)  
> **Props:** OFF  

### Test 3.1 — Velocity Command Clamping

```bash
python3 - <<'EOF'
from navigation.follow_controller import FollowController

fc = FollowController(desired_follow_dist_m=3.0)

cases = [
    ((5.0,   0.0,  0.0), "vx > 0 (target far)"),
    ((1.0,   0.0,  0.0), "vx < 0 (target too close)"),
    ((3.0,   0.0,  0.0), "vx = 0 (at follow distance)"),
    ((3.0,   4.0, -2.0), "vy > 0, vz < 0"),
    ((103.0, 0.0, 10.0), "vx and vz must be clamped"),
]

for (tx, ty, tz), label in cases:
    vx, vy, vz = fc.compute_velocity_command(tx, ty, tz)
    print(f"  [{label}]  →  vx={vx:.2f}  vy={vy:.2f}  vz={vz:.2f}")

vx, vy, vz = fc.compute_velocity_command(103.0, 0.0, 10.0)
if abs(vx) <= 3.0 and abs(vz) <= 1.5:
    print("\nPASS ✅ — velocity correctly clamped")
else:
    print("\nFAIL ❌ — velocity exceeded limits")
EOF
```

**Result:** ☐ PASS  ☐ FAIL  
**Notes:** ___________

---

## Stage 5 — Milestone 4: Search Patterns (Bench)

> **Hardware:** None required  

### Test 4.1 — Expanding Square Boundary

```bash
python3 - <<'EOF'
from navigation.search_patterns import generate_expanding_square

wps = generate_expanding_square(step_m=3.0, max_radius_m=10.0)
bad = [(x, y) for (x, y, *_) in wps if abs(x) > 10.0 or abs(y) > 10.0]
print(f"Waypoints generated : {len(wps)}")
print(f"Boundary violations : {bad}")
print("PASS ✅" if not bad else "FAIL ❌")
EOF
```

**Waypoints:** ___________  **Result:** ☐ PASS  ☐ FAIL

---

### Test 4.2 — Lawnmower Boundary

```bash
python3 - <<'EOF'
from navigation.search_patterns import generate_lawnmower

wps = generate_lawnmower(width_m=6.0, length_m=10.0, step_m=3.0)
bad = [(x, y) for (x, y, *_) in wps
       if not (0.0 <= x <= 10.0 and 0.0 <= y <= 6.0)]
print(f"Waypoints generated : {len(wps)}")
print(f"Boundary violations : {bad}")
print("PASS ✅" if not bad else "FAIL ❌")
EOF
```

**Waypoints:** ___________  **Result:** ☐ PASS  ☐ FAIL

---

## Stage 6 — Milestone 6: Intel RealSense Depth

> **Hardware:** RealSense D435i (blue USB 3.0 port)  
> **Props:** OFF  

### Test 6.1 — Frame Capture & Alignment

```bash
python3 - <<'EOF'
from depth.realsense_interface import RealSenseInterface

iface = RealSenseInterface(use_mock=False)
iface.start()
frames = iface.get_frames()
assert frames is not None, "FAIL ❌ — get_frames() returned None"
color, depth = frames
print(f"Color shape : {color.shape}")    # expect (480, 640, 3)
print(f"Depth shape : {depth.shape}")    # expect (480, 640)
print(f"Mock mode   : {iface.is_mock}")  # expect False
iface.stop()
print("PASS ✅")
EOF
```

**Color shape:** ___________  **Depth shape:** ___________  
**Result:** ☐ PASS  ☐ FAIL

---

### Test 6.2 — Depth Accuracy at 1 m

Point the camera at a flat wall exactly **1.0 m** away:

```bash
python3 - <<'EOF'
from depth.realsense_interface import RealSenseInterface
iface = RealSenseInterface(use_mock=False)
iface.start()
_, depth = iface.get_frames()
dist = iface.get_distance_at_pixel(depth, 320, 240)
print(f"Measured : {dist:.3f} m  |  Expected : 0.95 – 1.05 m")
print("PASS ✅" if 0.95 <= dist <= 1.05 else "FAIL ❌")
iface.stop()
EOF
```

**Measured:** ___________ m  **Result:** ☐ PASS  ☐ FAIL

### Test 6.3 — Depth Accuracy at 2 m

```bash
python3 - <<'EOF'
from depth.realsense_interface import RealSenseInterface
iface = RealSenseInterface(use_mock=False)
iface.start()
_, depth = iface.get_frames()
dist = iface.get_distance_at_pixel(depth, 320, 240)
print(f"Measured : {dist:.3f} m  |  Expected : 1.90 – 2.10 m")
print("PASS ✅" if 1.90 <= dist <= 2.10 else "FAIL ❌")
iface.stop()
EOF
```

**Measured:** ___________ m  **Result:** ☐ PASS  ☐ FAIL

### Test 6.4 — Depth Accuracy at 3 m

```bash
python3 - <<'EOF'
from depth.realsense_interface import RealSenseInterface
iface = RealSenseInterface(use_mock=False)
iface.start()
_, depth = iface.get_frames()
dist = iface.get_distance_at_pixel(depth, 320, 240)
print(f"Measured : {dist:.3f} m  |  Expected : 2.85 – 3.15 m")
print("PASS ✅" if 2.85 <= dist <= 3.15 else "FAIL ❌")
iface.stop()
EOF
```

**Measured:** ___________ m  **Result:** ☐ PASS  ☐ FAIL

---

## Stage 7 — Milestone 5: Balloon Detection via RealSense Color Stream

> **Hardware:** RealSense D435i  
> **Requires:** `models/drone_balloon_detector.pt` present on the Pi  

### Test 5.1 — Model Loading

```bash
python3 - <<'EOF'
from perception.detector import ObjectDetector
detector = ObjectDetector()
print("Model path     :", detector.model_path)
print("Active classes :", detector.target_class_ids)
print("PASS ✅" if {0, 1}.issubset(detector.target_class_ids) else "FAIL ❌")
EOF
```

**Result:** ☐ PASS  ☐ FAIL  ☐ SKIP (no model weights)

---

### Test 5.2 — Live Balloon Detection via D435i

Hold a **balloon** in front of the camera and run:

```bash
python3 - <<'EOF'
from depth.realsense_interface import RealSenseInterface
from perception.detector import ObjectDetector

iface    = RealSenseInterface(use_mock=False)
iface.start()
detector = ObjectDetector()

color, _ = iface.get_frames()
balloons = detector.detect_balloons(color)
drones   = detector.detect_drones(color)

print("Balloons :", balloons)
print("Drones   :", drones)
iface.stop()
EOF
```

> Hold balloon → expect `label == "balloon"`, `class_id == 0`

**Balloon detected?** ☐ YES  ☐ NO  **Confidence:** ___________  
**Result:** ☐ PASS  ☐ FAIL  ☐ SKIP

---

### Test 5.3 — FPS Benchmark (≥ 10 FPS)

```bash
python3 - <<'EOF'
import time
from depth.realsense_interface import RealSenseInterface
from perception.detector import ObjectDetector

iface    = RealSenseInterface(use_mock=False)
iface.start()
detector = ObjectDetector()

print("Benchmarking 60 frames...")
start = time.monotonic()
for _ in range(60):
    color, _ = iface.get_frames()
    detector.detect(color)
fps = 60 / (time.monotonic() - start)
iface.stop()

print(f"FPS = {fps:.2f}")
print("PASS ✅" if fps >= 10.0 else f"FAIL ❌ (target ≥ 10 FPS)")
EOF
```

**FPS measured:** ___________  **Result:** ☐ PASS  ☐ FAIL  ☐ SKIP

---

## Stage 8 — Milestone 7: 3D Kalman Target Tracker (No Hardware)

### Test 7.1 — Reset & Lifecycle

```bash
python3 - <<'EOF'
from tracking.target_tracker import TargetTracker

tracker = TargetTracker()
assert not tracker.is_initialized
tracker.update((1.0, 2.0, 3.0))
assert tracker.is_initialized
print(f"State after update : {tracker.get_state()}")  # (1,2,3,0,0,0)
tracker.reset()
assert not tracker.is_initialized
assert tracker.get_state() is None
print("PASS ✅")
EOF
```

**Result:** ☐ PASS  ☐ FAIL

---

### Test 7.2 — Velocity Estimation & Dropout Prediction

```bash
python3 - <<'EOF'
from tracking.target_tracker import TargetTracker

tracker = TargetTracker()
dt = 0.1
for step in range(21):
    tracker.predict(dt)
    tracker.update((5.0 * step * dt, 0.0, 0.0))

state = tracker.get_state()
print(f"State at t=2.0s : {state}")
print(f"Expected        : x≈10.0 m, vx≈4.8–5.2 m/s")

for _ in range(5):
    tracker.predict(dt)
after = tracker.get_state()
print(f"\nPredicted (0.5s dropout) : {after}")
print(f"Expected                 : x≈12.2–12.5 m (damped)")
print("PASS ✅")
EOF
```

**x at t=2.0 s:** ___________  **vx:** ___________  **x after dropout:** ___________  
**Result:** ☐ PASS  ☐ FAIL

---

### Test 7.3 — Outlier Rejection (Mahalanobis Gate)

```bash
python3 - <<'EOF'
from tracking.target_tracker import TargetTracker

tracker = TargetTracker()
tracker.update((2.0, 0.0, 0.0))
tracker.predict(0.1)

ok  = tracker.update((2.2, 0.0,  0.0))   # valid
bad = tracker.update((2.3, 10.0, 0.0))   # outlier

print(f"Valid update accepted : {ok}")    # True
print(f"Outlier accepted      : {bad}")   # False
state = tracker.get_state()
print(f"State after outlier   : {state}")
print("PASS ✅" if (ok and not bad and state[1] < 1.0) else "FAIL ❌")
EOF
```

**Outlier rejected?** ☐ YES  ☐ NO  **Result:** ☐ PASS  ☐ FAIL

---

## Stage 9 — Milestone 9: Safety & Heartbeat-Loss Failsafe

> **Hardware:** PIX6 via USB  
> **Props: ALWAYS OFF — zero exceptions**  

### Test 9.1 — Heartbeat-Loss Failsafe (USB Unplug)

**Two terminals required.**

**Terminal 1 — Start the backend:**
```bash
cd ~/Formyx/formyx_backend && python3 main.py
```

**Terminal 2 — Monitor logs:**
```bash
tail -f logs/*.csv
```

**Steps:**
1. Wait for telemetry rows appearing in log output.
2. **Physically unplug the USB cable** from the PIX6.
3. Wait 3–5 seconds.

> **Expected Terminal 1:**
> ```
> [Failsafe] HEARTBEAT_LOST detected
> [StateMachine] → EMERGENCY
> ```

**Heartbeat loss detected within 5 s?** ☐ YES  ☐ NO  
**State machine → EMERGENCY?** ☐ YES  ☐ NO  
**Result:** ☐ PASS  ☐ FAIL

---

### Test 9.2 — Reconnection Recovery

After Test 9.1, **plug the USB cable back in** (without restarting the backend).

**Reconnection detected in logs?** ☐ YES  ☐ NO  
**Result:** ☐ PASS  ☐ FAIL  
**Notes:** ___________

---

## Stage 10 — Milestone 10: Logging & Black-Box Recording

> **Hardware:** None required  

### Test 10.1 — Log File Creation & Worker Thread

```bash
python3 - <<'EOF'
import threading
from logging_system.logger import BlackBoxLogger

logger = BlackBoxLogger()
logger.start()
threads = [t.name for t in threading.enumerate()]
print("Active threads:", threads)
ok = "blackbox-writer" in threads
logger.stop()
print("PASS ✅" if ok else "FAIL ❌ — blackbox-writer thread missing")
EOF
```

**Result:** ☐ PASS  ☐ FAIL

---

### Test 10.2 — 10 Hz Write Rate (100 rows in 10 s)

```bash
python3 - <<'EOF'
import time
from logging_system.logger import BlackBoxLogger
from mavlink_interface.connection import TelemetrySnapshot

logger = BlackBoxLogger()
logger.start()
snap = TelemetrySnapshot(armed=True, flight_mode="GUIDED", lat_deg=12.9, lon_deg=77.5)

print("Logging 100 rows at 10 Hz...")
for _ in range(100):
    logger.log("SEARCHING", snap,
               target_vector=(1.0, 2.0, 3.0, 0.0, 0.0, 0.0),
               cmd_vector=(0.5, 0.0, 0.0))
    time.sleep(0.1)

logger.stop()
print("Done.")
EOF

ls -lh logs/*.csv
wc -l logs/*.csv
```

> **Expected:** ≥ 101 lines (100 data rows + header)

**Row count:** ___________  **Result:** ☐ PASS  ☐ FAIL

---

### Test 10.3 — Log Rotation (≤ 5 files kept)

```bash
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
print(f"Log files found : {len(files)}")
print("PASS ✅" if len(files) <= 5 else f"FAIL ❌ (expected ≤ 5, got {len(files)})")
EOF
```

**Files found:** ___________  **Result:** ☐ PASS  ☐ FAIL

---

## Session Summary

| Stage | Test | Result |
|-------|------|:------:|
| 0.1 | PIX6 port discovery | ☐ P  ☐ F |
| 0.2 | RealSense USB 3.x speed | ☐ P  ☐ F |
| 1 | Package smoke test | ☐ P  ☐ F |
| 2 | Full unit test suite | ☐ P  ☐ F |
| 3.1 | MAVProxy heartbeat | ☐ P  ☐ F |
| 3.2 | Python telemetry snapshot | ☐ P  ☐ F |
| 3.3 | 60-s heartbeat stability | ☐ P  ☐ F |
| 4.1 | Velocity clamping | ☐ P  ☐ F |
| 5.1 | Expanding square boundaries | ☐ P  ☐ F |
| 5.2 | Lawnmower boundaries | ☐ P  ☐ F |
| 6.1 | Frame capture & alignment | ☐ P  ☐ F |
| 6.2 | Depth accuracy 1 m | ☐ P  ☐ F |
| 6.3 | Depth accuracy 2 m | ☐ P  ☐ F |
| 6.4 | Depth accuracy 3 m | ☐ P  ☐ F |
| 7.1 | Model loading | ☐ P  ☐ F  ☐ Skip |
| 7.2 | Live balloon detection | ☐ P  ☐ F  ☐ Skip |
| 7.3 | FPS benchmark ≥ 10 | ☐ P  ☐ F  ☐ Skip |
| 8.1 | Tracker reset & lifecycle | ☐ P  ☐ F |
| 8.2 | Velocity estimation & dropout | ☐ P  ☐ F |
| 8.3 | Outlier rejection | ☐ P  ☐ F |
| 9.1 | Heartbeat-loss failsafe | ☐ P  ☐ F |
| 9.2 | Reconnection recovery | ☐ P  ☐ F |
| 10.1 | Log file creation & thread | ☐ P  ☐ F |
| 10.2 | 10 Hz write rate | ☐ P  ☐ F |
| 10.3 | Log rotation ≤ 5 files | ☐ P  ☐ F |

**Overall session result:** ☐ ALL PASS  ☐ FAILURES

---

*Tests excluded (require GPS / propellers / tethered flight):*  
*M3 tethered flight · M4 tethered flight · M8 reacquisition flight · GUIDED-mode position holds*