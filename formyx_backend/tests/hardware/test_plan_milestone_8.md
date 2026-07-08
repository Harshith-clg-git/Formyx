# Hardware Test Plan — Milestone 8: Target Loss & Reacquisition

This document outlines the hardware validation cases for the **Target Loss and Reacquisition** module (`navigation/search_patterns.py` additions and state transitions) on the physical Raspberry Pi 5 companion computer and drone.

---

## Safety Requirements & Pre-conditions

> [!CAUTION]
> **PROPS MUST BE PHYSICALLY REMOVED** for all bench checks (Test Case 8.1).
> The drone can command yaws and circular movements instantly when transitioning states.

> [!WARNING]
> For any flight test (Test Case 8.2), the vehicle must be **securely tethered** to the ground, tested outdoors in an open area, with a safety pilot holding the RC controller ready to switch to `STABILIZE` or `LOITER` mode to override autonomous operations at any moment.

---

## Test Cases

### Test Case 8.1: Bench Validation of Visual Sweep Sequence (Props OFF)

*   **Objective**: Confirm that when the target tracker reports lost frames, the FSM transitions to `TARGET_LOST_RECOVERY` and the backend commands correct yaw rotations and orbital sweeps.
*   **Pre-conditions**:
    *   Propellers: **Physically removed** from all motors.
    *   Connection: Raspberry Pi 5 connected.
*   **Execution Steps**:
    1. Start the main backend process in bench test mode.
    2. Feed fake target detections to initialize tracking and trigger `TRACKING` state.
    3. Stop feeding target detections (simulating target occlusion).
    4. Wait 2.0 seconds (`target_lost_timeout_s` parameter).
    5. Verify the FSM transitions to `TARGET_LOST_RECOVERY`.
    6. Observe output logs and outbound MAVLink commands.
*   **Expected Results**:
    *   State machine transitions: `TRACKING` → `TARGET_LOST_RECOVERY`.
    *   The drone commands a visual yaw sweep: outbound MAVLink `MAV_CMD_CONDITION_YAW` messages are sent containing sequential relative yaw target steps (e.g. +15°, +30°, +15°, 0°, -15°, -30°, -15°, 0°).
    *   Alternatively, circular relative position offsets are computed and streamed.
*   **Pass/Fail Criteria**:
    *   **PASS**: State transition is successful, and MAVLink visual sweep commands are streamed.
    *   **FAIL**: Tracker does not report target loss, FSM fails to transition, or sweep commands are not sent.

---

### Test Case 8.2: Closed-Loop Reacquisition (Props ON, Tethered)

*   **Objective**: Verify in flight that the drone switches to recovery when blocked, sweeps to search, and re-locks when unblocked.
*   **Pre-conditions**:
    *   Propellers: **Attached**.
    *   Area: Open outdoor field, GPS 3D Fix lock.
    *   Tether: Securely anchored.
    *   Target: Balloon on a stick held in front of the drone.
*   **Execution Steps**:
    1. Take off in `GUIDED` mode to 2.5m altitude.
    2. Trigger target tracking so the drone locks and holds position relative to the balloon.
    3. Block the camera lens completely with an opaque board.
    4. Verify that:
        *   The drone remains stable at its current position for 2.0 seconds while the tracker propagates states in prediction mode.
        *   At 2.0 seconds, the drone FSM transitions to `TARGET_LOST_RECOVERY`.
        *   The drone yaws slowly left/right to execute the search sweep.
    5. Remove the lens block so the camera sees the balloon again.
    6. Verify that:
        *   The drone immediately re-locks on the balloon.
        *   FSM transitions back to `TRACKING`.
        *   The yaw sweep stops, and the drone resumes following the target.
    7. Terminate the test by switching to `LAND` or `LOITER` on the RC transmitter.
*   **Expected Results**:
    *   Closed-loop state transitions and physical behavior operate seamlessly: tracking → lost (holds position on prediction) → recovery (yaw sweep) → reacquired (follow target).
    *   Manual override works instantly.
*   **Pass/Fail Criteria**:
    *   **PASS**: The drone performs all transitions correctly, executes the sweep, re-locks immediately when target is visible, and manual override succeeds.
    *   **FAIL**: Drone does not transition, behaves unstably during target loss, or fails to re-acquire.
