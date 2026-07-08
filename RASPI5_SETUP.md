# Formyx Drone — Raspberry Pi 5 Setup & Requirements

> **Fresh-boot setup guide for the Formyx autonomous drone companion computer.**  
> Follow every step in order before running any milestone hardware tests.

---

## Hardware Requirements

| Component | Specification |
|---|---|
| Companion Computer | Raspberry Pi 5 (4 GB or 8 GB RAM) |
| OS | Raspberry Pi OS **64-bit** (Bookworm) |
| Python | 3.11 or newer |
| Flight Controller | Radiolink PIX6 — ArduPilot firmware, connected via **USB** |
| Depth Camera | Intel RealSense D435i — connected via **USB 3.0 (blue port)** |
| Storage | ≥ 32 GB microSD (Class 10 / A2) |
| Power | Official Raspberry Pi 5 USB-C PD 5A supply |

---

## Step 1 — Update the OS

```bash
sudo apt update && sudo apt full-upgrade -y
sudo reboot
```

---

## Step 2 — Install System Build Dependencies

```bash
sudo apt install -y \
    python3 python3-pip python3-venv python3-dev \
    build-essential cmake git \
    libatlas-base-dev libhdf5-dev libhdf5-serial-dev \
    libjpeg-dev libpng-dev libtiff-dev \
    libavcodec-dev libavformat-dev libswscale-dev \
    libv4l-dev libxvidcore-dev libx264-dev \
    libopenblas-dev gfortran \
    libusb-1.0-0-dev udev curl wget minicom
```

---

## Step 3 — Create Python Virtual Environment

```bash
python3 -m venv ~/formyx_env
source ~/formyx_env/bin/activate
pip install --upgrade pip
```

> **Persist across reboots** — add this line to `~/.bashrc`:
> ```bash
> source ~/formyx_env/bin/activate
> ```

---

## Step 4 — Clone the Repository

```bash
cd ~
git clone https://github.com/Harshith-clg-git/Formyx.git
cd Formyx/formyx_backend
```

---

## Step 5 — Install Python Packages

### 5.1 Core packages

```bash
pip install \
    "opencv-python>=4.8.0" \
    "numpy>=1.23.0" \
    "pymavlink>=2.4.41" \
    "pyyaml>=6.0.1" \
    "pytest>=8.0.0" \
    "pytest-mock>=3.12.0"
```

> If `opencv-python` fails to build on ARM64, use the system package instead:
> ```bash
> sudo apt install -y python3-opencv
> ```

### 5.2 YOLOv8 / Ultralytics (for Milestone 5 — Object Detection)

```bash
pip install "ultralytics>=8.3.0"
```

> ⏳ This downloads the PyTorch CPU build. Allow **10–20 minutes** and ensure at least **3 GB** of free disk space.

### 5.3 Intel RealSense SDK — `pyrealsense2` (for Milestone 6 — Depth Camera)

The standard pip wheel does **not** support ARM64. Use Intel's official APT repository:

```bash
# 1. Add Intel's package signing key
sudo mkdir -p /etc/apt/keyrings
curl -sSf https://librealsense.intel.com/Debian/librealsense.pgp \
    | sudo tee /etc/apt/keyrings/librealsense.pgp > /dev/null

# 2. Add the Intel RealSense APT repo
echo "deb [signed-by=/etc/apt/keyrings/librealsense.pgp] \
https://librealsense.intel.com/Debian/apt-repo \
$(lsb_release -cs) main" \
    | sudo tee /etc/apt/sources.list.d/librealsense.list

# 3. Install the SDK
sudo apt update
sudo apt install -y librealsense2-dkms librealsense2-utils \
                    librealsense2-dev librealsense2-dbg

# 4. Install the Python bindings
pip install pyrealsense2
```

**Verify the D435i is detected correctly:**

```bash
rs-enumerate-devices -s
```

Expected output:
```
Intel RealSense D435i
    USB: USB 3.2  ← must be 3.x, NOT 2.x
```

> [!IMPORTANT]
> Always plug the D435i into the **blue USB 3.0 port** on the Pi 5.  
> Plugging into a black USB 2.0 port will fail silently — the camera will not stream full-resolution depth data.

---

## Step 6 — Serial Port Permissions (for PIX6 Flight Controller)

```bash
sudo usermod -aG dialout $USER
sudo usermod -aG tty $USER
sudo reboot
```

After reboot, re-activate the virtualenv:

```bash
source ~/formyx_env/bin/activate
cd ~/Formyx/formyx_backend
```

Verify the PIX6 is detected:

```bash
ls /dev/ttyACM*   # Usually /dev/ttyACM0
# or
ls /dev/ttyUSB*   # If using an FTDI adapter
```

---

## Step 7 — Place the YOLO Model Weights

The detector uses a **dual-class YOLOv8 model** trained to detect:

| Class ID | Object |
|---|---|
| `0` | Balloon |
| `1` | Drone |

```bash
mkdir -p ~/Formyx/formyx_backend/models

# Copy the weights file from your development machine via SCP:
# (run this on your dev machine, not the Pi)
# scp drone_balloon_detector.pt pi@<raspi-ip>:~/Formyx/formyx_backend/models/

# On the Pi — verify the file is in place:
ls -lh ~/Formyx/formyx_backend/models/drone_balloon_detector.pt
```

---

## Step 8 — Verify All Packages

Run this smoke test to confirm every package is correctly installed:

```bash
cd ~/Formyx/formyx_backend
python3 - <<'EOF'
import sys
print(f"Python: {sys.version}\n")

import cv2;             print(f"✅ OpenCV:        {cv2.__version__}")
import numpy;           print(f"✅ NumPy:         {numpy.__version__}")
import pymavlink;       print(f"✅ PyMAVLink:     {pymavlink.__version__}")
import yaml;            print(f"✅ PyYAML:        {yaml.__version__}")
import ultralytics;     print(f"✅ Ultralytics:   {ultralytics.__version__}")
import pyrealsense2 as rs; print(f"✅ pyrealsense2:  {rs.__version__}")
import pytest;          print(f"✅ pytest:        {pytest.__version__}")

print("\n🚀 All packages installed correctly. Ready for testing!")
EOF
```

All 7 lines must print with `✅`. If any line errors, re-run the relevant install step above.

---

## Step 9 — Run Unit Tests (Confirm Code is Correct)

Before touching any physical hardware, run the full pytest suite (uses mocks — no hardware needed):

```bash
cd ~/Formyx/formyx_backend
python3 -m pytest tests/ -v --tb=short
```

**All tests must pass** before proceeding to hardware tests.

---

## Quick Reference — Package Summary

| Package | Version | Purpose |
|---|---|---|
| `opencv-python` | ≥ 4.8.0 | Image capture & processing |
| `numpy` | ≥ 1.23.0 | Numerical / linear algebra |
| `ultralytics` | ≥ 8.3.0 | YOLOv8 balloon + drone detection |
| `pyrealsense2` | ≥ 2.50.0 | Intel RealSense D435i depth camera |
| `pymavlink` | ≥ 2.4.41 | MAVLink protocol (PIX6 flight controller) |
| `pyyaml` | ≥ 6.0.1 | YAML config file parsing |
| `pytest` | ≥ 8.0.0 | Unit test runner |
| `pytest-mock` | ≥ 3.12.0 | Hardware mocking for tests |

---

## What's Next

Once all 9 steps above are complete and unit tests pass, proceed to the full milestone hardware test procedures:

📄 **[HARDWARE_TESTING.md](HARDWARE_TESTING.md)** — Milestone-by-milestone hardware test commands (M3 → M10)
