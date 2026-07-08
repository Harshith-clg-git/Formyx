# Hardware Test Plan — Milestone 4: Search Pattern Implementation

This document outlines the hardware validation cases for the **Search Pattern Generator** (`navigation/search_patterns.py`) on the physical companion computer and drone autopilot.

---

## Safety Requirements & Pre-conditions

> [!CAUTION]
> **PROPS MUST BE PHYSICALLY REMOVED** for all bench checks (Test Case 4.1).
> Generating search waypoints can cause the autopilot to command thrust and rotation instantly if armed.

> [!WARNING]
> For any flight test (Test Case 4.2), the vehicle must be **securely tethered** to the ground, tested outdoors in an open area, with a safety pilot holding the RC controller ready to switch to `STABILIZE` or `LOITER` mode to override autonomous operations at any moment.

---

## Test Cases

### Test Case 4.1: Bench Validation of Bounded Waypoint Generation (Props OFF)

*   **Objective**: Verify that both the expanding square and lawnmower pattern generators correctly compute coordinate coordinates within specified boundaries and do not trigger infinite loops or invalid structures.
*   **Pre-conditions**:
    *   Propellers: **Physically removed** from all motors.
    *   Connection: Raspberry Pi 5 setup complete.
*   **Execution Steps**:
    1. Start a python interactive shell or run a verification script on the Raspberry Pi 5.
    2. Run the expanding square generator:
        ```python
        from navigation.search_patterns import generate_expanding_square
        wps = generate_expanding_square(step_m=3.0, max_radius_m=10.0)
        print("Waypoints:", wps)
        ```
    3. Check the maximum absolute values of all generated X and Y coordinates.
    4. Run the lawnmower generator:
        ```python
        from navigation.search_patterns import generate_lawnmower
        wps = generate_lawnmower(width_m=6.0, length_m=10.0, step_m=3.0)
        print("Waypoints:", wps)
        ```
    5. Check the range of X and Y coordinates.
*   **Expected Results**:
    *   Expanding square coordinates must never have `abs(x) > 10.0` or `abs(y) > 10.0`. Waypoints should form a clockwise expanding spiral.
    *   Lawnmower coordinates must remain within `x` in `[0.0, 10.0]` and `y` in `[0.0, 6.0]`. The path must snake back and forth (e.g. from y=0 to y=6, step X, then y=6 to y=0).
*   **Pass/Fail Criteria**:
    *   **PASS**: All waypoints strictly adhere to boundaries, follow the correct geometry, and generate cleanly.
    *   **FAIL**: Waypoints breach configured boundaries, generate empty list, or cause infinite execution loops.

---

### Test Case 4.2: Tethered Search Flight Execution (Props ON, Tethered)

*   **Objective**: Verify that the autopilot successfully receives and navigates through the computed relative waypoints in order during flight.
*   **Pre-conditions**:
    *   Propellers: **Attached**.
    *   Area: Open outdoor field, GPS 3D Fix lock.
    *   Tether: Securely anchored to the ground with a 5-meter line.
    *   Autopilot: Safety switch active, geofencing set to 10m.
    *   Safety Pilot: Ready to abort and switch to manual flight mode (`STABILIZE` or `LOITER`).
*   **Execution Steps**:
    1. Arm the vehicle and take off to 2.5m altitude in `GUIDED` mode.
    2. Inject the `WAYPOINT_REACHED` or dummy event to transition the mission state machine to `SEARCHING`.
    3. Monitor the drone as it navigates relative waypoints sequentially.
    4. Verify that:
        *   It transitions cleanly from waypoint to waypoint.
        *   It slows down or pauses slightly at corners (due to autopilot acceleration limiting).
        *   It stays within the horizontal bounds of the search space.
    5. After completing the search waypoints, verify that the drone transitions to a safe state (e.g., loiters at the final waypoint or returns to search origin).
    6. Switch mode back to `LOITER`/`LAND` to complete the test.
*   **Expected Results**:
    *   The drone flies the designated square spiral or lawnmower pattern smoothly.
    *   It remains inside the horizontal geofence boundary at all times.
    *   RC transmitter manual override takes control instantly when triggered.
*   **Pass/Fail Criteria**:
    *   **PASS**: The drone navigates the pattern accurately, transitions between points correctly, and manual override works.
    *   **FAIL**: Drone flies erratically, breaches geofence boundaries, or fails to transition between waypoints.
