# Hardware Test Plan — Milestone 6: Intel RealSense Depth Integration

This document outlines the hardware validation cases for the **RealSense Depth Camera Interface** (`depth/realsense_interface.py`) on the Raspberry Pi 5 with the physical Intel RealSense D435i camera.

---

## Safety Requirements & Pre-conditions

> [!NOTE]
> All RealSense depth camera verification is done on a **stationary bench setup** (no flight required).
> Ensure the vehicle remains disarmed with propellers **physically removed**.

> [!IMPORTANT]
> The Intel RealSense D435i camera **must be plugged into one of the blue USB 3.0 ports** on the Raspberry Pi 5.
> Connecting to a black USB 2.0 port will restrict the camera's bandwidth, resulting in failure to stream high-resolution depth data.

---

## Test Cases

### Test Case 6.1: Camera Physical Connection & Device Enumeration

*   **Objective**: Verify that the companion computer operating system successfully detects the Intel RealSense D435i camera over USB 3.0.
*   **Pre-conditions**:
    *   Camera: Physically connected to the Raspberry Pi 5 USB 3.0 port via a USB-C to USB-A cable.
*   **Execution Steps**:
    1. SSH into the Raspberry Pi 5.
    2. Run the Intel RealSense SDK query tool:
        ```bash
        rs-enumerate-devices -s
        ```
*   **Expected Results**:
    *   The output lists the device name `Intel RealSense D435i`.
    *   The connection type is explicitly reported as `USB 3.2` or `USB 3.1` (not `USB 2.1`).
*   **Pass/Fail Criteria**:
    *   **PASS**: Camera is detected and operates on USB 3.x.
    *   **FAIL**: Camera is not detected, or connects only on USB 2.x.

---

### Test Case 6.2: Frame Capture & Alignment Validation

*   **Objective**: Confirm that the camera streams initialize, align, and return correctly sized data structures without throwing exceptions.
*   **Pre-conditions**:
    *   RealSense camera successfully enumerated (Test Case 6.1 passes).
    *   `pyrealsense2` library installed on the Raspberry Pi 5.
*   **Execution Steps**:
    1. Open a Python shell in the root of the project directory.
    2. Run the initialization and frame capture test:
        ```python
        from depth.realsense_interface import RealSenseInterface
        interface = RealSenseInterface(use_mock=False)
        interface.start()
        
        frames = interface.get_frames()
        assert frames is not None, "Failed to capture frames!"
        
        color, depth = frames
        print("Color image shape :", color.shape)
        print("Depth image shape :", depth.shape)
        print("Is mock mode      :", interface.is_mock)
        
        interface.stop()
        ```
*   **Expected Results**:
    *   No warnings or error logs are generated during start.
    *   Console output confirms:
        *   `Color image shape : (480, 640, 3)`
        *   `Depth image shape : (480, 640)`
        *   `Is mock mode      : False`
*   **Pass/Fail Criteria**:
    *   **PASS**: Frame capture succeeds, returns correct dimensions, and `is_mock` is False.
    *   **FAIL**: Startup crashes, frames are None, shapes are incorrect, or it falls back to mock mode.

---

### Test Case 6.3: Depth Accuracy & Spatial Patch-Averaging

*   **Objective**: Validate depth measurement accuracy against static targets at known physical distances, and verify that the spatial patch-averaging algorithm successfully resolves edge shadows.
*   **Pre-conditions**:
    *   Mount the camera on a tripod or stable bench pointing directly at a flat wall.
    *   Keep a tape measure handy.
*   **Execution Steps**:
    1. Place the camera exactly **1.0 meter** away from the flat wall (measured from the front glass of the camera).
    2. Run a query script to fetch the distance at the center pixel `(320, 240)`:
        ```python
        from depth.realsense_interface import RealSenseInterface
        interface = RealSenseInterface(use_mock=False)
        interface.start()
        color, depth = interface.get_frames()
        dist = interface.get_distance_at_pixel(depth, 320, 240)
        print(f"Measured Distance at 1.0m: {dist:.3f} meters")
        ```
    3. Repeat the measurement at exactly **2.0 meters** and **3.0 meters**.
    4. Introduce a partial obstruction near the camera lens to create a local depth shadow (a zero-depth band) near the center, and query the distance again.
*   **Expected Results**:
    *   Calculated distance at 1.0m is between `0.95m` and `1.05m`.
    *   Calculated distance at 2.0m is between `1.90m` and `2.10m`.
    *   Calculated distance at 3.0m is between `2.85m` and `3.15m`.
    *   In the presence of local depth shadows (zeros), the spatial patch-averaging algorithm ignores the zero values and successfully computes the median of the remaining valid pixels, returning a correct distance.
*   **Pass/Fail Criteria**:
    *   **PASS**: Distance measurements are within the 5% error margin, and patch-averaging successfully mitigates shadows.
    *   **FAIL**: Measurements are inaccurate, or the distance query returns None in shadowed areas.
