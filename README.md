# Formyx Autonomous Drone Backend

> Modular, production-ready Python backend for autonomous balloon-tracking drone operations on Raspberry Pi 5 + Radiolink PIX6 autopilot.

---

## Overview

This repository contains the companion-computer software stack for the **Formyx autonomous drone system**. It handles:

- 📡 **MAVLink communication** with the Radiolink PIX6 flight controller
- 🧠 **Mission state machine** — deterministic 11-state FSM governing the full flight lifecycle
- 🎯 **Balloon detection** — YOLOv8 real-time inference (Milestone 5)
- 📐 **3D Kalman tracking** — position + velocity estimation with Mahalanobis gating (Milestone 7)
- 🔍 **Autonomous search patterns** — lawnmower / expanding square (Milestone 4)
- 🛡 **Safety & failsafes** — battery, GPS, geofence, heartbeat monitoring (Milestone 9)
- 📊 **Black-box logging** — high-rate CSV/JSONL flight recorder (Milestone 10)

---

## Hardware

| Component | Model |
|---|---|
| Companion Computer | Raspberry Pi 5 (8GB) |
| Flight Controller | Radiolink PIX6 (ArduCopter firmware) |
| Depth Camera | Intel RealSense D435i |
| Connection | USB (dev/ttyACM0) or TELEM2 UART |

---

## Project Status

| Milestone | Description | Status |
|---|---|---|
| 1 | MAVLink Interface — connection, telemetry, commands | ✅ Complete |
| 2 | Mission State Machine — 11-state deterministic FSM | ✅ Complete |
| 3 | Navigation Controller — follow controller | 🔲 Planned |
| 4 | Search Patterns — lawnmower / expanding square | 🔲 Planned |
| 5 | Balloon Detection — YOLOv8 inference pipeline | 🔲 Planned |
| 6 | RealSense Depth Integration | 🔲 Planned |
| 7 | 3D Kalman Target Tracker | 🔲 Planned |
| 8 | Target Loss & Reacquisition | 🔲 Planned |
| 9 | Safety & Failsafe Monitor | 🔲 Planned |
| 10 | Black-Box Logging | 🔲 Planned |

---

## Repository Structure

```
formyx_backend/
├── main.py                      # Entry point
├── requirements.txt
├── config/
│   ├── settings.yaml            # All tunable parameters
│   └── loader.py                # Thread-safe config loader
├── mavlink_interface/
│   ├── connection.py            # MAVLink link + telemetry snapshot
│   └── commands.py              # arm/disarm/takeoff/RTL/mode/position
├── mission_manager/
│   └── state_machine.py         # Deterministic FSM (11 states, 40+ transitions)
├── navigation/                  # Milestone 3 & 4
├── perception/                  # Milestone 5
├── depth/                       # Milestone 6
├── tracking/                    # Milestone 7
├── safety/                      # Milestone 9
├── logging_system/              # Milestone 10
├── tools/
│   ├── hardware_test.py         # Interactive Pi + PIX6 validation script
│   └── HARDWARE_TEST_GUIDE.md   # Wiring + setup guide
├── models/
│   └── README.md                # Model weights instructions
└── tests/
    ├── test_mavlink_interface.py # 25 unit tests (Milestone 1)
    └── test_state_machine.py    # 91 unit tests (Milestone 2)
```

---

## Quick Start

### Dependencies
```bash
pip install pymavlink pyyaml pytest pytest-mock
# For full stack (Milestones 5-6, on Pi only):
# pip install opencv-python ultralytics pyrealsense2
```

### Run on SITL (no hardware)
```bash
# Start ArduPilot SITL first, then:
python main.py --connection udpin:localhost:14550
```

### Run on Raspberry Pi (USB)
```bash
python main.py --connection serial:/dev/ttyACM0:57600
```

### Run Hardware Validation
```bash
# Safe test (no arm):
python tools/hardware_test.py --connection /dev/ttyACM0 --baud 57600

# With arm/disarm (⚠ REMOVE PROPS FIRST):
python tools/hardware_test.py --connection /dev/ttyACM0 --baud 57600 --arm-test
```

### Run Tests
```bash
python -m pytest tests/ -v
# Expected: 116 passed
```

---

## Configuration

All parameters live in [`config/settings.yaml`](formyx_backend/config/settings.yaml). Key sections:

```yaml
mavlink:
  connection_string: "serial:/dev/ttyACM0:57600"  # USB
  telemetry_rate_hz: 10

safety:
  battery_critical_pct: 15      # RTL threshold
  gps_min_satellites: 6
  max_geofence_radius_m: 50.0

perception:
  confidence_threshold: 0.60
  inference_resolution: [320, 320]
```

---

## Safety

> [!CAUTION]
> **NEVER arm the vehicle with propellers attached during software testing.**
> The `hardware_test.py --arm-test` flag requires typing a full confirmation phrase before arming.

> [!WARNING]
> Always test on SITL before connecting to real hardware.
> The state machine's failsafe paths have been unit-tested but should be verified in SITL before flight.

---

## License

Private — Formyx Project © 2026
