"""
tests/test_reacquisition.py
----------------------------
Unit tests for target loss and reacquisition logic, including sweep generators
and FSM target loss state transitions.
"""

from __future__ import annotations

import sys
import pathlib
import pytest

_BACKEND = pathlib.Path(__file__).parent.parent
if str(_BACKEND) not in sys.path:
    sys.path.insert(0, str(_BACKEND))

from navigation.search_patterns import (
    generate_orbital_sweep,
    generate_yaw_sweep_pattern,
)
from mission_manager.state_machine import (
    MissionStateMachine,
    MissionState,
    MissionEvent,
)


def test_orbital_sweep_geometry():
    """Verify that orbital sweep correctly generates coordinates on a circle."""
    # Radius = 2m, 4 points (should be N, E, S, W i.e. theta = 0, pi/2, pi, 3pi/2)
    waypoints = generate_orbital_sweep(radius_m=2.0, num_points=4)
    
    assert len(waypoints) == 4
    # (2, 0)
    assert pytest.approx(waypoints[0][0]) == 2.0
    assert pytest.approx(waypoints[0][1]) == 0.0
    
    # (0, 2) (using cos(pi/2)=0, sin(pi/2)=1)
    assert pytest.approx(waypoints[1][0]) == 0.0
    assert pytest.approx(waypoints[1][1]) == 2.0
    
    # (-2, 0)
    assert pytest.approx(waypoints[2][0]) == -2.0
    assert pytest.approx(waypoints[2][1]) == 0.0
    
    # (0, -2)
    assert pytest.approx(waypoints[3][0]) == 0.0
    assert pytest.approx(waypoints[3][1]) == -2.0


def test_yaw_sweep_pattern_generation():
    """Verify that yaw sweep pattern outputs correct left/right angles sequence."""
    angles = generate_yaw_sweep_pattern(sweep_range_deg=30.0, step_deg=15.0)
    
    # Expected sequence:
    # 1. Sweep right (pos yaw): 15.0, 30.0
    # 2. Sweep back: 15.0, 0.0
    # 3. Sweep left (neg yaw): -15.0, -30.0
    # 4. Sweep back: -15.0, 0.0
    expected = [15.0, 30.0, 15.0, 0.0, -15.0, -30.0, -15.0, 0.0]
    assert angles == expected


def test_fsm_target_loss_transitions():
    """Verify FSM transitions from TRACKING to TARGET_LOST_RECOVERY and back or timeout."""
    sm = MissionStateMachine(initial_state=MissionState.TRACKING)
    
    # Post target lost event
    sm.post_event(MissionEvent.TARGET_LOST, source="tracker")
    sm.step()
    assert sm.state == MissionState.TARGET_LOST_RECOVERY
    
    # Scenario A: Target reacquired -> goes back to tracking
    sm.post_event(MissionEvent.TARGET_REACQUIRED, source="tracker")
    sm.step()
    assert sm.state == MissionState.TRACKING
    
    # Scenario B: Target lost again -> recovery -> timeout -> searching
    sm.post_event(MissionEvent.TARGET_LOST, source="tracker")
    sm.step()
    assert sm.state == MissionState.TARGET_LOST_RECOVERY
    
    sm.post_event(MissionEvent.RECOVERY_TIMEOUT, source="timer")
    sm.step()
    assert sm.state == MissionState.SEARCHING
