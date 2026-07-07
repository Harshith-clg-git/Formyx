# Hardware Test Plan — Milestone 7: 3D Kalman Target Tracker

This document outlines the hardware validation cases for the **3D Kalman Target Tracker** (`tracking/target_tracker.py`) running on the Raspberry Pi 5.

---

## Safety Requirements & Pre-conditions

> [!NOTE]
> All target tracker verification is done on a **stationary bench setup** (no flight required).
> Ensure the vehicle remains disarmed with propellers **physically removed**.

---

## Test Cases

### Test Case 7.1: Tracker Reset & Lifecycle

*   **Objective**: Confirm the Kalman filter correctly resets and updates initialization flags on demand.
*   **Pre-conditions**:
    *   No physical camera is required; this can be executed via an interactive python script.
*   **Execution Steps**:
    1. Open Python in the project root on Pi 5.
    2. Initialize the tracker:
        ```python
        from tracking.target_tracker import TargetTracker
        tracker = TargetTracker()
        print("Initialized:", tracker.is_initialized)
        
        # Feed first mock detection
        tracker.update((1.0, 2.0, 3.0))
        print("Initialized after update:", tracker.is_initialized)
        print("Estimated State:", tracker.get_state())
        
        # Reset tracker
        tracker.reset()
        print("Initialized after reset:", tracker.is_initialized)
        print("Estimated State after reset:", tracker.get_state())
        ```
*   **Expected Results**:
    *   Initial `is_initialized` is `False`.
    *   After update, `is_initialized` is `True`, and state matches `(1.0, 2.0, 3.0, 0.0, 0.0, 0.0)`.
    *   After reset, `is_initialized` is `False`, and state is `None`.
*   **Pass/Fail Criteria**:
    *   **PASS**: State changes and reset flags match the expected output.
    *   **FAIL**: Reset fails to clear the state or initialization flag remains true.

---

### Test Case 7.2: Velocity Estimation & Occlusion Prediction

*   **Objective**: Verify the Kalman filter correctly estimates target velocity for a moving target, and accurately projects coordinates during simulated sensor dropouts.
*   **Pre-conditions**:
    *   Mock measurements representing a target moving along the +X axis (Forward) at a constant velocity of `5.0 m/s` (fed at 10 Hz).
*   **Execution Steps**:
    1. Run a test script to feed position measurements to the tracker:
        ```python
        import time
        from tracking.target_tracker import TargetTracker
        
        tracker = TargetTracker()
        dt = 0.1  # 10 Hz
        
        # Feed constant speed coordinates: x = 5.0 * t
        for step in range(21):  # 2.0 seconds
            t = step * dt
            tracker.predict(dt)
            tracker.update((5.0 * t, 0.0, 0.0))
            
        state = tracker.get_state()
        print("State at t=2.0s:", state)
        
        # Simulate target occlusion (lost frames) for 0.5s
        print("\n--- Simulating 0.5s visual dropout ---")
        for _ in range(5):
            tracker.predict(dt)
            
        print("Predicted State after dropout:", tracker.get_state())
        ```
*   **Expected Results**:
    *   At `t=2.0s`, the estimated position X is approximately `10.0m`, and the estimated velocity `vx` is close to `5.0 m/s` (typically `4.8` to `5.2 m/s`).
    *   During the dropout, the predicted position X propagates forward based on velocity.
    *   After the `0.5s` dropout (5 prediction steps), the predicted position X is close to `12.2m` to `12.5m` (slightly damped). Estimated `vx` decays slightly towards 0 due to velocity damping.
*   **Pass/Fail Criteria**:
    *   **PASS**: Velocity estimation matches actual target speed, and predicted positions are physically reasonable.
    *   **FAIL**: Velocity estimation is wrong or the prediction diverges.

---

### Test Case 7.3: Outlier Rejection & Crossing Path Gating

*   **Objective**: Confirm the Mahalanobis gate successfully filters out sudden clutter/noise spikes and handles crossing object paths.
*   **Pre-conditions**:
    *   Target tracker initialized and locked on a target moving along +X axis.
*   **Execution Steps**:
    1. Run a script that initializes a track and then introduces a sudden high-distance coordinate spike:
        ```python
        from tracking.target_tracker import TargetTracker
        tracker = TargetTracker()
        
        # Target locked at (2.0, 0.0, 0.0)
        tracker.update((2.0, 0.0, 0.0))
        tracker.predict(0.1)
        
        # Update with true continuation: (2.2, 0.0, 0.0)
        true_ok = tracker.update((2.2, 0.0, 0.0))
        print("True update accepted:", true_ok)
        
        # Update with outlier clutter measurement: (2.3, 10.0, 0.0)
        outlier_ok = tracker.update((2.3, 10.0, 0.0))
        print("Outlier update accepted:", outlier_ok)
        print("State after outlier attempt:", tracker.get_state())
        ```
*   **Expected Results**:
    *   `True update accepted` is `True`.
    *   `Outlier update accepted` is `False`.
    *   The state estimate position remains close to `(2.2, 0.0, 0.0)` and does not jump towards the outlier.
*   **Pass/Fail Criteria**:
    *   **PASS**: Outlier is successfully gated, and track state is unaffected.
    *   **FAIL**: Outlier is accepted, or tracker crashes.
