# Formyx Hardware Test Guide
## Raspberry Pi 5 + Radiolink PIX6

---

## Step 1 — Physical Wiring

### Option A: USB (recommended for first test — easiest)
```
PIX6 Micro-USB  →  Raspberry Pi 5 USB-A port
```
- Port will appear as `/dev/ttyACM0` or `/dev/ttyUSB0`
- Baud rate: **57600** (USB auto-negotiates)
- ✅ No voltage level concerns, plug-and-play

### Option B: TELEM2 UART (for flight — lower latency)
```
PIX6 TELEM2 (6-pin JST-GH)   →   Pi 5 GPIO Header
─────────────────────────────────────────────────────
Pin 1: VCC (5V)               →   Pin 2  (5V) [optional, if powering Pi from PIX6]
Pin 2: TX  (3.3V logic)       →   Pin 10 (GPIO15 / RXD0)
Pin 3: RX  (3.3V logic)       →   Pin 8  (GPIO14 / TXD0)
Pin 6: GND                    →   Pin 6  (GND)
```
> ⚠️ **PIX6 TELEM2 is 3.3V logic** — safe to connect directly to Pi 5 GPIO.
> Port: `/dev/ttyAMA0`, Baud: **921600**

> ⚠️ **Enable UART on Pi 5** — run `sudo raspi-config` → Interface Options → Serial Port → Disable login shell, Enable serial hardware.

---

## Step 2 — Set Mission Planner / QGC to stream MAVLink

In **Mission Planner** (on Windows) or **QGroundControl**:
1. Connect to the PIX6 via USB from your PC first
2. Go to: `CONFIG → Full Parameter List`
3. Set `SERIAL2_BAUD = 921600` (for TELEM2)
4. Set `SERIAL2_PROTOCOL = 2` (MAVLink 2)
5. Write params → Reboot autopilot

---

## Step 3 — Copy code to the Raspberry Pi

### Via SCP from Windows (run in PowerShell):
```powershell
# Replace <PI_IP> with your Pi's IP address (find it via `hostname -I` on Pi)
scp -r "F:\Harshith\formyx_software\formyx_backend" pi@<PI_IP>:~/formyx_backend
```

### Or via USB drive:
Copy the `formyx_backend/` folder to a USB drive, then on the Pi:
```bash
cp -r /media/pi/USBDRIVE/formyx_backend ~/formyx_backend
```

---

## Step 4 — Install dependencies on Pi

```bash
cd ~/formyx_backend
pip install pymavlink pyyaml pytest pytest-mock
```

> Note: `opencv`, `ultralytics`, `pyrealsense2` can be installed later (Milestones 5 & 6)

---

## Step 5 — Run the hardware test

### USB connection (easiest):
```bash
# Find your port first:
ls /dev/ttyACM* /dev/ttyUSB*

# Run the test (10 second soak):
python tools/hardware_test.py --connection /dev/ttyACM0 --baud 57600

# Full 60 second soak:
python tools/hardware_test.py --connection /dev/ttyACM0 --baud 57600 --soak-duration 60
```

### TELEM2 UART:
```bash
python tools/hardware_test.py --connection /dev/ttyAMA0 --baud 921600
```

### With arm/disarm test (⚠ PROPS MUST BE PHYSICALLY REMOVED):
```bash
python tools/hardware_test.py --connection /dev/ttyACM0 --baud 57600 --arm-test
```

---

## Step 6 — Expected Output (healthy system)

```
══════════════════════════════════════════════════════════════
  FORMYX HARDWARE VALIDATION TOOL
  Raspberry Pi 5 + Radiolink PIX6
══════════════════════════════════════════════════════════════

──────────────────────────────────────────────────────────────
  TEST 1 — MAVLink Connection & Heartbeat
──────────────────────────────────────────────────────────────
  ℹ INFO  Connecting to: serial:/dev/ttyACM0:57600
  ✓ PASS  Heartbeat received on serial:/dev/ttyACM0:57600

──────────────────────────────────────────────────────────────
  TEST 2 — Telemetry Stream Quality (10s soak)
──────────────────────────────────────────────────────────────
  ✓ PASS  HEARTBEAT: 10 messages received (1.0 Hz)
  ✓ PASS  GLOBAL_POSITION_INT: 100 messages received (10.0 Hz)
  ✓ PASS  ATTITUDE: 100 messages received (10.0 Hz)
  ✓ PASS  SYS_STATUS: 100 messages received (10.0 Hz)

──────────────────────────────────────────────────────────────
  TEST 3 — GPS Fix & Satellite Count
──────────────────────────────────────────────────────────────
  ✓ PASS  3D GPS fix acquired (3D FIX)
  ✓ PASS  Satellite count OK: 9 ≥ 6

... (all tests pass) ...

  ✓ All tests passed — hardware link is healthy.
```

---

## Common Failures & Fixes

| Error | Cause | Fix |
|---|---|---|
| `No heartbeat received` | Wrong port or baud | Run `ls /dev/ttyACM*` to find port; try baud 57600 first |
| `Permission denied /dev/ttyACM0` | User not in `dialout` group | `sudo usermod -a -G dialout pi` then reboot |
| `GLOBAL_POSITION_INT not received` | Stream not configured | Set `SR2_POSITION=10` in Mission Planner params |
| `ARM rejected` | Pre-arm checks failing | Open QGC/Mission Planner → check pre-arm message |
| `Battery voltage = 0.0` | Voltage sensor not calibrated | Calibrate in Mission Planner → `BATT_VOLT_MULT` |
| GPS fix type = 0 | Indoors or antenna blocked | Move outdoors, wait 2–3 minutes for GPS lock |
