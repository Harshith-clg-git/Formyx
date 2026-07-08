# Hardware Test Plan — Milestone 3: Navigation Controller

This document outlines the hardware validation cases for the **Follow Controller** (`navigation/follow_controller.py`) on the physical Raspberry Pi 5 + Radiolink PIX6 drone.

---

## Safety Requirements & Pre-conditions

> [!CAUTION]
> **PROPS MUST BE PHYSICALLY REMOVED** for all bench tests (Test Case 3.1).
> The motors can spin up instantly when the autopilot is armed and receives guided commands.

> [!WARNING]
> For any flight test (Test Case 3.2), the vehicle must be **securely tethered** to the ground, tested outdoors in an open area, with a safety pilot holding the RC controller ready to switch to `STABILIZE` or `LOITER` mode to override autonomous operations at any moment.

---

## Test Cases

### Test Case 3.1: Bench Validation of Velocity Commands (Props OFF)

*   **Objective**: Validate that the proportional feedback laws and speed limiters operate correctly on the physical companion computer and stream valid MAVLink packets.
*   **Pre-conditions**:
    *   Propellers: **Physically removed** from all motors.
    *   Connection: Raspberry Pi 5 connected to PIX6 via USB or TELEM2.
    *   GPS: Indoor lock or GPS simulation active.
    *   Safety Pilot: Ready to disarm via RC transmitter if needed.
*   **Execution Steps**:
    1. Turn on the drone and verify a healthy MAVLink connection (`hardware_test.py` passes).
    2. Start the follow controller in mock test mode by running a test script or injecting mock target detections into the perception loop:
        *   Inject Target at `(5.0, 0.0, 0.0)` — 5 meters directly in front of the drone.
        *   Inject Target at `(1.0, 0.0, 0.0)` — 1 meter directly in front of the drone.
        *   Inject Target at `(3.0, 0.0, 0.0)` — 3 meters directly in front of the drone (desired follow distance).
        *   Inject Target at `(3.0, 4.0, -2.0)` — 3m forward, 4m right, 2m above (z is negative in FRD).
        *   Inject Target at `(103.0, 0.0, 10.0)` — extreme distance (103m forward, 10m below).
    3. Observe the output logs and the transmitted MAVLink `SET_POSITION_TARGET_LOCAL_NED` messages.
*   **Expected Results**:
    *   For `(5.0, 0.0, 0.0)`: Computed velocity command is forward: `vx > 0` (approx `1.0 m/s`), `vy == 0`, `vz == 0`.
    *   For `(1.0, 0.0, 0.0)`: Computed velocity command is backward: `vx < 0` (approx `-1.0 m/s`), `vy == 0`, `vz == 0`.
    *   For `(3.0, 0.0, 0.0)`: Computed velocity command is zero: `vx == 0`, `vy == 0`, `vz == 0`.
    *   For `(3.0, 4.0, -2.0)`: Proportional response: `vx == 0`, `vy > 0` (rightward), `vz < 0` (upward, climbing).
    *   For `(103.0, 0.0, 10.0)`: The velocity commands are clamped exactly to the safety limits: `vx == 3.0 m/s` (max horizontal), `vy == 0.0 m/s`, `vz == -1.5 m/s` (max vertical).
*   **Pass/Fail Criteria**:
    *   **PASS**: All computed velocity commands match the sign and magnitude expected, clamp at safety bounds, and do not crash the system.
    *   **FAIL**: Velocity calculations are incorrect, clamping fails, or the loop crashes.

---

### Test Case 3.2: Tethered Flight Follow Check (Props ON, Tethered)

*   **Objective**: Verify the closed-loop physical tracking response of the autopilot guided flight controller in real space.
*   **Pre-conditions**:
    *   Propellers: **Attached**.
    *   Area: Open outdoor field, GPS 3D Fix lock, wind speed < 5 m/s.
    *   Tether: Securely anchored to the ground with a 5-meter heavy-duty line.
    *   Autopilot: Safety switch enabled, geofencing set to 10m.
    *   Safety Pilot: Finger on the mode switch, ready to toggle `LOITER` or `STABILIZE`.
*   **Execution Steps**:
    1. Secure the drone to the tether.
    2. Arm the vehicle, change mode to `GUIDED`, and command takeoff to 2.5 meters.
    3. Introduce a physical target (e.g. balloon or target board on a pole) 5 meters in front of the drone.
    4. Activate target tracking mode.
    5. Move the target closer to the drone (approx 2m). Observe drone response.
    6. Move the target away from the drone (approx 5m). Observe drone response.
    7. Shift the target horizontally (left/right) and vertically. Observe drone response.
    8. Toggle flight mode to `LOITER` via RC transmitter to terminate the autonomous tracking run.
*   **Expected Results**:
    *   The drone moves backward when the target is closer than 3.0m.
    *   The drone moves forward when the target is further than 3.0m.
    *   The drone yaws or moves sideways to align horizontally with the target.
    *   The drone adjusts altitude to match target altitude.
    *   RC transmitter manual override instantly aborts tracking and restores manual pilot control.
*   **Pass/Fail Criteria**:
    *   **PASS**: The drone maintains the follow distance (within ±0.5m tolerance), behaves stably, and manual override succeeds instantly.
    *   **FAIL**: Severe oscillations, failure to move in the correct direction, or failure to return control to the manual pilot.
