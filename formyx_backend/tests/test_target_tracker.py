"""
tests/test_target_tracker.py
----------------------------
Unit tests for the TargetTracker class.
"""

from __future__ import annotations

import sys
import pathlib
import numpy as np
import pytest

_BACKEND = pathlib.Path(__file__).parent.parent
if str(_BACKEND) not in sys.path:
    sys.path.insert(0, str(_BACKEND))

from tracking.target_tracker import TargetTracker


def test_tracker_initialization():
    """Verify tracker starts uninitialized and resets correctly."""
    tracker = TargetTracker()
    assert tracker.is_initialized is False
    assert tracker.lost_frames == 0
    assert tracker.get_state() is None


def test_first_measurement_initializes_state():
    """Verify that the first measurement sets position state directly and sets is_initialized."""
    tracker = TargetTracker()
    success = tracker.update((1.2, 3.4, -5.6))
    
    assert success is True
    assert tracker.is_initialized is True
    
    state = tracker.get_state()
    assert state is not None
    x, y, z, vx, vy, vz = state
    assert pytest.approx(x) == 1.2
    assert pytest.approx(y) == 3.4
    assert pytest.approx(z) == -5.6
    assert pytest.approx(vx) == 0.0
    assert pytest.approx(vy) == 0.0
    assert pytest.approx(vz) == 0.0


def test_prediction_moves_state():
    """Verify that prediction propagates state based on velocity."""
    tracker = TargetTracker()
    # Initialize at (0, 0, 0)
    tracker.update((0.0, 0.0, 0.0))
    
    # Inject some velocity manually
    tracker._x[3:6] = [2.0, -1.0, 0.5]  # vx, vy, vz
    
    # Predict forward by 2 seconds
    tracker.predict(dt=2.0)
    
    state = tracker.get_state()
    assert state is not None
    x, y, z, vx, vy, vz = state
    # Expected position: pos + vel * dt
    # x = 0 + 2.0 * 2.0 = 4.0
    # y = 0 - 1.0 * 2.0 = -2.0
    # z = 0 + 0.5 * 2.0 = 1.0
    # Note: velocity is damped: vel = vel * (1 - damping * dt) = vel * (1 - 0.1 * 2) = vel * 0.8
    assert pytest.approx(x) == 4.0
    assert pytest.approx(y) == -2.0
    assert pytest.approx(z) == 1.0
    assert pytest.approx(vx) == 1.6
    assert pytest.approx(vy) == -0.8
    assert pytest.approx(vz) == 0.4


def test_velocity_damping_decays():
    """Verify that without updates, prediction velocities decay to zero."""
    tracker = TargetTracker()
    tracker.update((0.0, 0.0, 0.0))
    tracker._x[3:6] = [10.0, 10.0, 10.0]
    
    # Predict repeatedly
    for _ in range(50):
        tracker.predict(dt=0.1)
        
    state = tracker.get_state()
    assert state is not None
    _, _, _, vx, vy, vz = state
    # Velocity should have decayed significantly
    assert abs(vx) < 7.0
    assert abs(vy) < 7.0
    assert abs(vz) < 7.0


def test_mahalanobis_gating_rejects_outliers():
    """Verify that updates far from prediction are rejected by the Mahalanobis gate."""
    tracker = TargetTracker()
    # Initialize at (0, 0, 0)
    tracker.update((0.0, 0.0, 0.0))
    
    # Run a prediction step
    tracker.predict(dt=0.1)
    
    # Try updating with a massive outlier measurement
    success = tracker.update((100.0, -100.0, 50.0))
    
    assert success is False
    assert tracker.lost_frames == 1
    
    # Position should not have jumped to the outlier
    state = tracker.get_state()
    assert state is not None
    x, y, z, _, _, _ = state
    assert abs(x) < 5.0
    assert abs(y) < 5.0
    assert abs(z) < 5.0


def test_target_loss_drops_track():
    """Verify that after max_lost_frames, the tracker drops tracking."""
    tracker = TargetTracker()
    tracker.update((0.0, 0.0, 0.0))
    
    # Max lost frames is 30. Force 30 failures.
    for _ in range(30):
        tracker.update((100.0, 100.0, 100.0))  # rejected as outlier
        
    assert tracker.lost_frames == 30
    assert tracker.get_state() is None  # track dropped


def test_intersecting_paths_separation():
    """
    Verify that tracker distinguishes between the tracked target and a nearby intersecting path.
    Tests data association by feeding a correct continuation and a crossing clutter path.
    """
    tracker = TargetTracker()
    
    # Target 1 starts at (0, 0, 0) moving with vx=10 m/s
    tracker.update((0.0, 0.0, 0.0))
    
    # Track target for a few steps
    t = 0.0
    dt = 0.1
    for i in range(5):
        tracker.predict(dt)
        t += dt
        # Actual target position: x = 10 * t
        tracker.update((10.0 * t, 0.0, 0.0))
        
    # At t=0.5s, target 1 is at (5.0, 0.0, 0.0) with vx estimated around 10 m/s
    # Now we predict to t=0.6s. Expected target position is (6.0, 0.0, 0.0)
    tracker.predict(dt)
    
    # Scenario: two detections appear at t=0.6s:
    # A) Target 1 continued path: (6.0, 0.0, 0.0)
    # B) Intersecting path crossing from side: (5.9, 3.0, 0.0) (3m off Y axis)
    
    # Verify that the tracker accepts the true path (A) and rejects the cross path (B)
    success_b = tracker.update((5.9, 3.0, 0.0))
    assert success_b is False, "Tracker incorrectly associated crossing clutter path!"
    
    success_a = tracker.update((6.0, 0.0, 0.0))
    assert success_a is True, "Tracker failed to associate true continuation path!"
