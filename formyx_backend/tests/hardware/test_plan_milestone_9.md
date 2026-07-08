# Hardware Test Plan — Milestone 9: Safety, Geofencing, and Failsafes

This document outlines the hardware validation cases for the **Failsafe Monitor** (`safety/failsafe_monitor.py`) on the Raspberry Pi 5 + Radiolink PIX6 drone.

---

## Safety Requirements & Pre-conditions

> [!CAUTION]
> **PROPS MUST BE PHYSICALLY REMOVED** for all bench tests (Test Cases 9.1, 9.2, and 9.3).
> Failsafe actions command emergency flight mode changes (such as Return-to-Launch) which can spin up the motors immediately.

---

## Test Cases

### Test Case 9.1: Heartbeat Loss Failsafe (Props OFF)

*   **Objective**: Confirm that physical loss of the MAVLink connection triggers an immediate emergency shutdown transition.
*   **Pre-conditions**:
    *   Propellers: **Physically removed** from all motors.
    *   Connection: Pi 5 connected to PIX6 via USB or TELEM2.
*   **Execution Steps**:
    1. Start the main backend process.
    2. Wait for telemetry stream validation and arming on bench.
    3. Physically disconnect the USB serial cable (or UART lines) between the Pi 5 and the PIX6.
    4. Wait 3.0 seconds (`heartbeat_loss_timeout_s`).
    5. Observe the backend console logs.
*   **Expected Results**:
    *   Failsafe monitor detects age of last heartbeat exceeds 3.0s.
    *   `HEARTBEAT_LOST` event is posted.
    *   State machine transitions to `EMERGENCY` state.
*   **Pass/Fail Criteria**:
    *   **PASS**: State machine transitions to `EMERGENCY` state within 3.5 seconds of disconnection.
    *   **FAIL**: Heartbeat loss goes undetected, or FSM fails to transition.

---

### Test Case 9.2: Battery Level Failsafes (Props OFF)

*   **Objective**: Verify that critical low battery level triggers Return-to-Launch (`RTL`) transition.
*   **Pre-conditions**:
    *   Propellers: **Physically removed** from all motors.
*   **Execution Steps**:
    1. Start the backend. Arm the drone mock-armed on bench.
    2. Transition the FSM to a flying state (e.g. `SEARCHING`).
    3. Use a variable power supply to lower the battery voltage input to the PIX6, or simulate/inject a battery percentage of 24%, and then 14% into the telemetry receiver.
    4. Observe FSM transition logs.
*   **Expected Results**:
    *   At 24% (Warning threshold): Failsafe monitor triggers `BATTERY_WARNING`. A warning is logged but search continues.
    *   At 14% (Critical threshold): Failsafe monitor triggers `BATTERY_CRITICAL`. FSM transitions to `RTL` and commands Return-to-Launch.
*   **Pass/Fail Criteria**:
    *   **PASS**: Warning logged at warning limit, and RTL transition triggered instantly at critical limit.
    *   **FAIL**: Battery thresholds fail to trigger correct FSM events.

---

### Test Case 9.3: Geofencing & Altitude Ceiling breaches (Props OFF)

*   **Objective**: Verify that exceeding horizontal radius limits or vertical altitude ceilings commands immediate Return-to-Launch (`RTL`).
*   **Pre-conditions**:
    *   Propellers: **Physically removed** from all motors.
*   **Execution Steps**:
    1. Start the backend. Lock home position at the drone's current GPS position on arming.
    2. Inject telemetry coordinates representing a position 55 meters away horizontally from the locked home position.
    3. Observe FSM transition logs.
    4. Restart/reset, lock home position, and inject an altitude AGL of 16.0 meters.
    5. Observe FSM transition logs.
*   **Expected Results**:
    *   Exceeding 50m geofence radius triggers `GEOFENCE_BREACH` event and transitions FSM to `RTL`.
    *   Exceeding 15m altitude ceiling triggers `GEOFENCE_BREACH` event and transitions FSM to `RTL`.
*   **Pass/Fail Criteria**:
    *   **PASS**: Geofence breaches trigger `RTL` transitions immediately.
    *   **FAIL**: Drone fails to detect boundaries, or FSM does not command RTL.
