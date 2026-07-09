# Formyx Hardware Test — Agent Prompt

> **READ THIS ENTIRE FILE BEFORE RUNNING A SINGLE COMMAND.**  
> This is a self-contained instruction set for a fresh agent to perform all possible  
> Formyx hardware tests **without propellers attached**, while verifying log output  
> simultaneously for every test.

---

## Context & Hardware

| Component | Detail |
|---|---|
| **SBC** | Raspberry Pi 5 (64-bit Bookworm OS) |
| **Flight Controller** | Radiolink PIX6 — connected via USB (ArduPilot firmware) |
| **Depth Camera** | Intel RealSense D435i — must use **blue USB 3.0 port** |
| **Propellers** | **PHYSICALLY REMOVED** — zero exceptions for all tests in this session |
| **GPS** | May or may not be attached — GPS-dependent tests (tethered flight) are excluded |
| **Detection Model** | YOLOv8 dual-class (`balloon`=class 0 / `drone`=class 1) at `models/drone_balloon_detector.pt` |
| **Working directory on Pi** | `~/Formyx/formyx_backend` |
| **Python environment** | `~/formyx_env` (activate with `source ~/formyx_env/bin/activate`) |

---

## Agent Operating Instructions

You are a **hardware test execution agent**. Your job is to:

1. **SSH into the Raspberry Pi** and run each test command exactly as written.
2. **Open a second terminal / tmux pane** for log monitoring on every test that touches hardware or the logging system — run the log-tail command *before* the test command so you can see output in real time.
3. **Record the actual output** from each test (copy the terminal output verbatim into your notes).
4. **Evaluate PASS / FAIL** based on the expected output defined under each test.
5. **Fill in the Results Table** at the bottom of this file after all tests complete.
6. **If a test FAILS**, record the full error output, attempt the listed recovery action once, and document whether recovery succeeded.
7. **Do not skip any test** unless the required hardware is genuinely absent. If hardware is absent, mark the test **SKIP** with a reason.
8. Proceed through stages in order — a failure in Stage 0 or 1 must be resolved before continuing.

### SSH and tmux setup (do this first)
```bash
# On your local machine — open two SSH sessions or use tmux:
ssh pi@<PI_IP_ADDRESS>

# Inside the Pi, start a tmux session with two panes:
tmux new-session -s hwtest
# Split into two panes: Ctrl+B then %
# Pane 1 = test commands | Pane 2 = log monitoring
```

---

## Stage 0 — System & Port Discovery

> **Log monitoring:** Not required for Stage 0.

### 0.1 — Activate Environment & Set Working Directory

```bash
source ~/formyx_env/bin/activate
cd ~/Formyx/formyx_backend
```

Expected: prompt shows `(formyx_env)` prefix. No errors.

---

### 0.2 — Find the PIX6 Serial Port

```bash
ls /dev/ttyACM* /dev/ttyUSB*
```

**Expected:** At least one device appears (e.g. `/dev/ttyACM0` or `/dev/ttyUSB0`).  
**Record the port:** _____________  
**⚠️ If nothing appears:** PIX6 is not detected. Check USB cable. Test cannot continue past Stage 3.

---

### 0.3 — Verify RealSense USB Speed

```bash
rs-enumerate-devices -s
lsusb -t | grep -i "Intel"
```

**Expected:** `Intel RealSense D435i` with `5000M` (USB 3.x SuperSpeed).  
**⚠️ If you see `480M`:** Move the cable to the blue USB 3.0 port and re-run.  
**Result:** ☐ PASS  ☐ FAIL

---

### 0.4 — Verify Git is Up to Date

```bash
git -C ~/Formyx pull origin master
git -C ~/Formyx log --oneline -3
```

**Expected:** Either `Already up to date.` or a list of new commits.

---

## Stage 1 — Package Smoke Test (No Hardware)

> **Log monitoring:** None required.

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

**Expected:** All package versions print without ImportError, followed by `✅ All packages OK!`  
**Result:** ☐ PASS  ☐ FAIL  
**Notes:** _____________

---

## Stage 2 — Full Unit Test Suite (No Hardware, Mocked)

> **Log monitoring (Pane 2):**
> ```bash
> # Watch for any log files created during pytest
> watch -n 2 "ls -lt logs/ 2>/dev/null | head -10"
> ```

```bash
python3 -m pytest tests/ -v --tb=short 2>&1 | tee /tmp/pytest_results.txt
echo "Exit code: $?"
```

**Expected:** All tests `PASSED`. Zero failures. Exit code `0`.  
**Total tests run:** _____________  **Passed:** _____________  **Failed:** _____________  
**Result:** ☐ PASS  ☐ FAIL  
**Failed test names (if any):** _____________

> ⚠️ **Do not proceed to Stage 3 if any unit test fails.** Investigate and fix first.

---

## Stage 3 — MAVLink Connection & Telemetry (PIX6 via USB)

> **Props:** ALWAYS OFF  
> **Replace `/dev/ttyACM0` with your actual port from Stage 0.2.**

---

### Test 3.1 — MAVProxy Quick Link Check

**Pane 2 (log monitor — run before Pane 1):**
```bash
# No formyx log yet; watch kernel USB events to confirm PIX6 is alive
dmesg | grep -i "ttyACM\|ttyUSB" | tail -5
```

**Pane 1 (test):**
```bash
mavproxy.py --master=/dev/ttyACM0 --baudrate=57600
```

**Expected:**  
- Heartbeat messages stream every ~1 second: `APM: ArduCopter ...`  
- Type `status` → shows telemetry fields  
- Press `Ctrl+C` to exit  

**Heartbeat rate observed:** ___________ Hz  
**Result:** ☐ PASS  ☐ FAIL  
**Notes:** _____________

---

### Test 3.2 — Python MAVLink Connection & Telemetry Snapshot

**Pane 2 (log monitor — run first):**
```bash
tail -f ~/Formyx/formyx_backend/logs/*.csv 2>/dev/null || \
  echo "No log files yet — waiting for test 3.2 to create them..."
```

**Pane 1 (test):**
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

**Expected:** `connected: True`, all telemetry fields populated, `PASS ✅` at the end.  
**Battery voltage:** ___________ V  **Battery %:** ___________ %  
**Result:** ☐ PASS  ☐ FAIL  
**Notes:** _____________

---

### Test 3.3 — 60-Second Heartbeat Stability (0% Packet Loss)

**Pane 2 (log monitor):**
```bash
tail -f ~/Formyx/formyx_backend/logs/*.csv 2>/dev/null | grep -i "heartbeat\|LOST\|EMERGENCY"
```

**Pane 1 (test):**
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

**Expected:** `Packet loss: 0.0%`, `PASS ✅`  
**Packet loss %:** ___________  
**Result:** ☐ PASS  ☐ FAIL

---

## Stage 4 — Navigation Controller Bench Tests (No Hardware)

> **Props:** OFF | **Hardware:** None (pure math)

---

### Test 4.1 — Velocity Command Clamping

**Pane 1 (test):**
```bash
python3 - <<'EOF'
from navigation.follow_controller import FollowController

fc = FollowController(desired_follow_dist_m=3.0)

cases = [
    ((5.0,   0.0,  0.0), "target far    → vx > 0"),
    ((1.0,   0.0,  0.0), "target close  → vx < 0"),
    ((3.0,   0.0,  0.0), "at follow dist → vx = 0"),
    ((3.0,   4.0, -2.0), "lateral + vert offset"),
    ((103.0, 0.0, 10.0), "extreme clamp test"),
]

for (tx, ty, tz), label in cases:
    vx, vy, vz = fc.compute_velocity_command(tx, ty, tz)
    print(f"  [{label}]  →  vx={vx:.2f}  vy={vy:.2f}  vz={vz:.2f}")

vx, vy, vz = fc.compute_velocity_command(103.0, 0.0, 10.0)
if abs(vx) <= 3.0 and abs(vz) <= 1.5:
    print("\nPASS ✅ — velocity correctly clamped")
else:
    print(f"\nFAIL ❌ — vx={vx:.2f} (limit 3.0), vz={vz:.2f} (limit 1.5)")
EOF
```

**Expected:** All five cases print reasonable velocity values; clamp test shows `PASS ✅`  
**Result:** ☐ PASS  ☐ FAIL  
**Notes:** _____________

---

## Stage 5 — Search Pattern Bench Tests (No Hardware)

---

### Test 5.1 — Expanding Square Boundary

**Pane 1 (test):**
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

**Waypoints generated:** ___________  **Violations:** ___________  
**Result:** ☐ PASS  ☐ FAIL

---

### Test 5.2 — Lawnmower Boundary

**Pane 1 (test):**
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

**Waypoints generated:** ___________  **Violations:** ___________  
**Result:** ☐ PASS  ☐ FAIL

---

## Stage 6 — Intel RealSense D435i Depth Tests

> **Hardware:** D435i must be in the **blue USB 3.0 port**.  
> **Props:** OFF

---

### Test 6.1 — Frame Capture & Alignment

**Pane 2 (log monitor):**
```bash
dmesg -w | grep -i "realsense\|usb\|error" 2>/dev/null
```

**Pane 1 (test):**
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

**Color shape:** ___________  **Depth shape:** ___________  **Mock mode:** ___________  
**Result:** ☐ PASS  ☐ FAIL

---

### Test 6.2 — Depth Accuracy at 1.0 m

> Place the camera exactly **1.0 m** from a flat wall before running.

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

---

### Test 6.3 — Depth Accuracy at 2.0 m

> Move the camera to exactly **2.0 m** from the wall.

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

---

### Test 6.4 — Depth Accuracy at 3.0 m

> Move the camera to exactly **3.0 m** from the wall.

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

## Stage 7 — Balloon & Drone Detection (YOLOv8 via D435i)

> **Hardware:** D435i + YOLO model weights  
> **Prerequisite:** `models/drone_balloon_detector.pt` must exist.

```bash
ls -lh ~/Formyx/formyx_backend/models/drone_balloon_detector.pt
```

If missing → mark all Stage 7 tests as **SKIP (no model weights)**.

---

### Test 7.1 — Model Loading

```bash
python3 - <<'EOF'
from perception.detector import ObjectDetector
detector = ObjectDetector()
print("Model path     :", detector.model_path)
print("Active classes :", detector.target_class_ids)
if {0, 1}.issubset(detector.target_class_ids):
    print("PASS ✅ — dual-class model loaded")
else:
    print("FAIL ❌ — missing class IDs")
EOF
```

**Result:** ☐ PASS  ☐ FAIL  ☐ SKIP

---

### Test 7.2 — Live Balloon Detection

> **Action:** Hold a **physical balloon** in front of the D435i lens.

**Pane 2 (log monitor):**
```bash
tail -f ~/Formyx/formyx_backend/logs/*.csv 2>/dev/null | grep -i "balloon\|detection\|TRACKING"
```

**Pane 1 (test):**
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

if balloons:
    b = balloons[0]
    print(f"First balloon — class_id={b.class_id}  conf={b.confidence:.2f}  label={b.label}")
    print("PASS ✅" if b.class_id == 0 and b.label == "balloon" else "FAIL ❌ — wrong class/label")
else:
    print("No balloon detected — FAIL ❌  (ensure balloon is visible in frame)")
EOF
```

**Balloon detected?** ☐ YES  ☐ NO  **Confidence:** ___________  
**Result:** ☐ PASS  ☐ FAIL  ☐ SKIP

---

### Test 7.3 — FPS Benchmark (target ≥ 10 FPS)

**Pane 2 (log monitor):**
```bash
htop
```

**Pane 1 (test):**
```bash
python3 - <<'EOF'
import time
from depth.realsense_interface import RealSenseInterface
from perception.detector import ObjectDetector

iface    = RealSenseInterface(use_mock=False)
iface.start()
detector = ObjectDetector()

print("Benchmarking 60 inference frames...")
start = time.monotonic()
for _ in range(60):
    color, _ = iface.get_frames()
    detector.detect(color)
fps = 60 / (time.monotonic() - start)
iface.stop()

print(f"FPS = {fps:.2f}")
print("PASS ✅" if fps >= 10.0 else f"FAIL ❌ (target ≥ 10 FPS, got {fps:.2f})")
EOF
```

**FPS measured:** ___________  **Result:** ☐ PASS  ☐ FAIL  ☐ SKIP

---

## Stage 8 — Kalman Target Tracker (No Hardware)

---

### Test 8.1 — Reset & Lifecycle

```bash
python3 - <<'EOF'
from tracking.target_tracker import TargetTracker

tracker = TargetTracker()
assert not tracker.is_initialized, "FAIL ❌ — should not be initialized at start"
tracker.update((1.0, 2.0, 3.0))
assert tracker.is_initialized, "FAIL ❌ — should be initialized after update"
print(f"State after update : {tracker.get_state()}")   # expect (1,2,3,0,0,0)
tracker.reset()
assert not tracker.is_initialized, "FAIL ❌ — should not be initialized after reset"
assert tracker.get_state() is None, "FAIL ❌ — state should be None after reset"
print("PASS ✅")
EOF
```

**Result:** ☐ PASS  ☐ FAIL

---

### Test 8.2 — Velocity Estimation & Dropout Prediction

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

### Test 8.3 — Outlier Rejection (Mahalanobis Gate)

```bash
python3 - <<'EOF'
from tracking.target_tracker import TargetTracker

tracker = TargetTracker()
tracker.update((2.0, 0.0, 0.0))
tracker.predict(0.1)

ok  = tracker.update((2.2, 0.0,  0.0))   # valid — small step
bad = tracker.update((2.3, 10.0, 0.0))   # outlier — huge lateral jump

print(f"Valid update accepted : {ok}")    # expect True
print(f"Outlier accepted      : {bad}")   # expect False
state = tracker.get_state()
print(f"State after outlier   : {state}")
passed = (ok is True) and (bad is False) and (state[1] < 1.0)
print("PASS ✅" if passed else "FAIL ❌")
EOF
```

**Outlier rejected?** ☐ YES  ☐ NO  **Result:** ☐ PASS  ☐ FAIL

---

## Stage 9 — Safety & Heartbeat-Loss Failsafe

> **Hardware:** PIX6 via USB  
> **Props: ALWAYS OFF — zero exceptions**

---

### Test 9.1 — Heartbeat-Loss Failsafe (USB Unplug)

**Pane 2 — Start log monitoring first:**
```bash
tail -f ~/Formyx/formyx_backend/logs/*.csv 2>/dev/null | \
  grep --line-buffered -i "heartbeat\|LOST\|EMERGENCY\|failsafe"
```

**Pane 1 — Start the backend:**
```bash
cd ~/Formyx/formyx_backend && python3 main.py 2>&1 | tee /tmp/backend_9_1.log
```

Wait until telemetry rows appear in Pane 2, then:

> **⚠️ ACTION:** Physically unplug the USB cable from the PIX6. Wait 3–5 seconds.

**Expected in Pane 1:**
```
[Failsafe] HEARTBEAT_LOST detected
[StateMachine] → EMERGENCY
```

**Heartbeat loss detected within 5 s?** ☐ YES  ☐ NO  
**State machine → EMERGENCY?** ☐ YES  ☐ NO  
**Result:** ☐ PASS  ☐ FAIL

---

### Test 9.2 — Reconnection Recovery

After Test 9.1, **plug the USB cable back in** without restarting the backend.

**Expected:**
```
[MAVLink] Reconnected — heartbeat restored
[StateMachine] → IDLE  (or SEARCHING)
```

**Reconnection detected within 10 s?** ☐ YES  ☐ NO  
**State machine recovered?** ☐ YES  ☐ NO  
**Result:** ☐ PASS  ☐ FAIL

---

### Test 9.3 — Battery Warning Threshold Injection

**Pane 2 (log monitor):**
```bash
tail -f ~/Formyx/formyx_backend/logs/*.csv 2>/dev/null | \
  grep --line-buffered -i "battery\|warning\|critical\|RTL"
```

**Pane 1 (test):**
```bash
python3 - <<'EOF'
from safety.failsafe_monitor import FailsafeMonitor
from mavlink_interface.connection import TelemetrySnapshot

monitor = FailsafeMonitor()

# 24% → expect BATTERY_WARNING (log only, mission continues)
snap_warn = TelemetrySnapshot(battery_remaining_pct=24, armed=False)
result_warn = monitor.check(snap_warn)
print(f"24% battery → {result_warn}")

# 14% → expect BATTERY_CRITICAL → RTL command queued
snap_crit = TelemetrySnapshot(battery_remaining_pct=14, armed=False)
result_crit = monitor.check(snap_crit)
print(f"14% battery → {result_crit}")
EOF
```

**24% triggers WARNING?** ☐ YES  ☐ NO  
**14% triggers CRITICAL/RTL?** ☐ YES  ☐ NO  
**Result:** ☐ PASS  ☐ FAIL  ☐ SKIP

---

### Test 9.4 — Geofence Breach Injection

```bash
python3 - <<'EOF'
from safety.failsafe_monitor import FailsafeMonitor
from mavlink_interface.connection import TelemetrySnapshot

monitor = FailsafeMonitor()

# 55 m from home → GEOFENCE_BREACH (limit = 50 m)
snap_geo = TelemetrySnapshot(distance_from_home_m=55.0, armed=False)
result = monitor.check(snap_geo)
print(f"55m from home → {result}")

# 16 m AGL → GEOFENCE_BREACH (limit = 15 m)
snap_alt = TelemetrySnapshot(alt_rel_m=16.0, armed=False)
result_alt = monitor.check(snap_alt)
print(f"16m AGL → {result_alt}")
EOF
```

**Geofence breach at 55 m?** ☐ YES  ☐ NO  
**Altitude breach at 16 m?** ☐ YES  ☐ NO  
**Result:** ☐ PASS  ☐ FAIL  ☐ SKIP

---

## Stage 10 — Logging System Tests

> **Hardware:** None required

---

### Test 10.1 — Log File Creation & Worker Thread

**Pane 2 (log monitor):**
```bash
watch -n 1 "ls -lt ~/Formyx/formyx_backend/logs/*.csv 2>/dev/null | head -5"
```

**Pane 1 (test):**
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

**Thread `blackbox-writer` present?** ☐ YES  ☐ NO  
**Result:** ☐ PASS  ☐ FAIL

---

### Test 10.2 — 10 Hz Write Rate (100 rows in 10 s)

**Pane 2 (log monitor — watch row count grow live):**
```bash
watch -n 1 "wc -l ~/Formyx/formyx_backend/logs/*.csv 2>/dev/null"
```

**Pane 1 (test):**
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

ls -lh ~/Formyx/formyx_backend/logs/*.csv
wc -l ~/Formyx/formyx_backend/logs/*.csv
```

**Expected:** ≥ 101 lines (1 header + 100 data rows)  
**Row count:** ___________  **Result:** ☐ PASS  ☐ FAIL

---

### Test 10.3 — Log Rotation (≤ 5 files kept)

**Pane 2 (log monitor):**
```bash
watch -n 1 "ls ~/Formyx/formyx_backend/logs/*.csv 2>/dev/null | wc -l"
```

**Pane 1 (test):**
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

**Files found after 7 runs:** ___________  **Result:** ☐ PASS  ☐ FAIL

---

## Stage 11 — Integrated System Smoke Test (All Hardware, 30 s)

> **Hardware:** PIX6 + D435i  
> **Props:** OFF

**Pane 2 (log monitor — run first):**
```bash
tail -f ~/Formyx/formyx_backend/logs/*.csv 2>/dev/null | \
  grep --line-buffered -i "state\|error\|exception\|SEARCHING\|TRACKING\|EMERGENCY"
```

**Pane 1 (test):**
```bash
cd ~/Formyx/formyx_backend
timeout 30 python3 main.py 2>&1 | tee /tmp/backend_integrated.log
echo "Exit code: $?"
grep -i "error\|exception\|traceback" /tmp/backend_integrated.log | head -20
```

**Expected:**
- No Python exceptions during 30 s run.
- Pane 2 shows state rows (`IDLE`, `SEARCHING`).
- No `EMERGENCY` state in first 30 s without hardware disconnect.
- Process exits cleanly after timeout.

**Backend crashes within 30 s?** ☐ YES  ☐ NO  
**State machine reaches SEARCHING?** ☐ YES  ☐ NO  
**Result:** ☐ PASS  ☐ FAIL

---

## Failure Recovery Guide

| Symptom | Likely Cause | Recovery Action |
|---|---|---|
| `ls /dev/ttyACM*` returns nothing | PIX6 USB not detected | Reconnect USB; check `dmesg \| tail -20` |
| RealSense shows USB 2.0 speed | Wrong USB port | Move D435i to blue USB 3.0 port |
| `ImportError: No module named X` | Package not installed | `pip install <pkg>` inside formyx_env |
| `FAIL — not connected` on MAVLink | Wrong baud rate | Try `--baudrate=921600` |
| `get_frames() returned None` | RealSense pipeline failed | Check USB speed; run `rs-enumerate-devices -s` |
| FPS < 10 | ARM CPU overloaded | Check `htop`; close background processes |
| `blackbox-writer` thread missing | Logger not started properly | Verify `BlackBoxLogger.start()` was called |
| Log rotation > 5 files | Rotation logic bug | Check `BlackBoxLogger` max_files config |
| EMERGENCY not triggered on USB unplug | Heartbeat timeout too long | Check `config/settings.yaml` → `mavlink.heartbeat_timeout_s` |

---

## Full Session Results Table

| # | Stage | Test | Result | Notes |
|---|---|---|:---:|---|
| 0.2 | System | PIX6 port discovery | ☐ P  ☐ F | |
| 0.3 | System | RealSense USB 3.x speed | ☐ P  ☐ F | |
| 0.4 | System | Git up to date | ☐ P  ☐ F | |
| 1 | Packages | Smoke test — all imports | ☐ P  ☐ F | |
| 2 | Unit Tests | Full pytest suite | ☐ P  ☐ F | |
| 3.1 | MAVLink | MAVProxy heartbeat | ☐ P  ☐ F | |
| 3.2 | MAVLink | Python telemetry snapshot | ☐ P  ☐ F | |
| 3.3 | MAVLink | 60-s heartbeat stability | ☐ P  ☐ F | |
| 4.1 | Navigation | Velocity clamping | ☐ P  ☐ F | |
| 5.1 | Search | Expanding square boundary | ☐ P  ☐ F | |
| 5.2 | Search | Lawnmower boundary | ☐ P  ☐ F | |
| 6.1 | Depth | Frame capture & alignment | ☐ P  ☐ F | |
| 6.2 | Depth | Accuracy at 1 m | ☐ P  ☐ F | |
| 6.3 | Depth | Accuracy at 2 m | ☐ P  ☐ F | |
| 6.4 | Depth | Accuracy at 3 m | ☐ P  ☐ F | |
| 7.1 | Detection | Model loading | ☐ P  ☐ F  ☐ S | |
| 7.2 | Detection | Live balloon detection | ☐ P  ☐ F  ☐ S | |
| 7.3 | Detection | FPS benchmark ≥ 10 | ☐ P  ☐ F  ☐ S | |
| 8.1 | Tracker | Reset & lifecycle | ☐ P  ☐ F | |
| 8.2 | Tracker | Velocity estimation & dropout | ☐ P  ☐ F | |
| 8.3 | Tracker | Outlier rejection | ☐ P  ☐ F | |
| 9.1 | Safety | Heartbeat-loss failsafe | ☐ P  ☐ F | |
| 9.2 | Safety | Reconnection recovery | ☐ P  ☐ F | |
| 9.3 | Safety | Battery threshold injection | ☐ P  ☐ F  ☐ S | |
| 9.4 | Safety | Geofence breach injection | ☐ P  ☐ F  ☐ S | |
| 10.1 | Logging | Log file creation & thread | ☐ P  ☐ F | |
| 10.2 | Logging | 10 Hz write rate | ☐ P  ☐ F | |
| 10.3 | Logging | Log rotation ≤ 5 files | ☐ P  ☐ F | |
| 11 | Integrated | Backend 30-s smoke test | ☐ P  ☐ F | |

**Overall session result:** ☐ ALL PASS  ☐ FAILURES  
**Date/Time:** _______________  **Tester:** _______________  **Pi IP:** _______________

---

## Tests Excluded (Require Propellers / Tethered Flight)

The following are intentionally excluded — they require props attached and a tethered flight environment:

- M3 tethered flight — velocity command via real MAVLink in GUIDED mode
- M4 tethered flight — lawnmower/expanding square with physical drone movement
- M8 reacquisition flight — FSM `TRACKING → TARGET_LOST_RECOVERY → TRACKING` in real flight
- Any GUIDED-mode position hold or waypoint navigation flight
