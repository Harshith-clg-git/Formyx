"""
tests/test_follow_controller.py
--------------------------------
Unit tests for the FollowController class.
"""

from __future__ import annotations

import math
import sys
import pathlib
from unittest.mock import patch

import pytest

_BACKEND = pathlib.Path(__file__).parent.parent
if str(_BACKEND) not in sys.path:
    sys.path.insert(0, str(_BACKEND))

from navigation.follow_controller import FollowController


def test_follow_controller_init():
    """Test that controller loads configurations correctly."""
    controller = FollowController()
    assert controller.follow_distance == 3.0
    assert controller.max_horiz_speed == 3.0
    assert controller.max_vert_speed == 1.5


def test_target_at_perfect_distance():
    """If target is exactly at follow distance, horizontal speed should be 0."""
    controller = FollowController()
    # Desired distance is 3m. Let's place it at x=3, y=0, z=0.
    vx, vy, vz = controller.get_velocity_command((3.0, 0.0, 0.0))
    assert pytest.approx(vx) == 0.0
    assert pytest.approx(vy) == 0.0
    assert pytest.approx(vz) == 0.0


def test_target_too_far():
    """If target is further than follow distance, drone should approach it."""
    controller = FollowController()
    # Place target at x=5m, y=0m, z=0m. Standoff is 3m. Error is +2m.
    # Expected vx = kp_xy * error * ux = 0.5 * 2.0 * 1.0 = 1.0 m/s.
    vx, vy, vz = controller.get_velocity_command((5.0, 0.0, 0.0))
    assert pytest.approx(vx) == 1.0
    assert pytest.approx(vy) == 0.0
    assert pytest.approx(vz) == 0.0


def test_target_too_close():
    """If target is closer than follow distance, drone should back up."""
    controller = FollowController()
    # Place target at x=1m, y=0m, z=0m. Standoff is 3m. Error is -2m.
    # Expected vx = 0.5 * -2.0 * 1.0 = -1.0 m/s.
    vx, vy, vz = controller.get_velocity_command((1.0, 0.0, 0.0))
    assert pytest.approx(vx) == -1.0
    assert pytest.approx(vy) == 0.0
    assert pytest.approx(vz) == 0.0


def test_horizontal_speed_clamping():
    """Horizontal speed should be clamped to max_horizontal_speed_ms."""
    controller = FollowController()
    # Standoff is 3m. Let's place the target at x=103m, y=0m, z=0m.
    # Error = 100m. Proportional speed = 0.5 * 100 = 50 m/s.
    # Max speed is 3.0 m/s.
    vx, vy, vz = controller.get_velocity_command((103.0, 0.0, 0.0))
    assert pytest.approx(vx) == 3.0
    assert pytest.approx(vy) == 0.0
    
    # Check multi-axis clamping
    # Place target at x=103m, y=103m, z=0m.
    vx, vy, vz = controller.get_velocity_command((103.0, 103.0, 0.0))
    speed = math.sqrt(vx**2 + vy**2)
    assert pytest.approx(speed) == 3.0


def test_vertical_speed_clamping():
    """Vertical speed should be clamped to max_vertical_speed_ms."""
    controller = FollowController()
    # Target is below by 10m (z=10). Proportional speed = 0.5 * 10 = 5.0 m/s.
    # Max speed is 1.5 m/s.
    vx, vy, vz = controller.get_velocity_command((3.0, 0.0, 10.0))
    assert pytest.approx(vz) == 1.5

    # Target is above by 10m (z=-10).
    vx, vy, vz = controller.get_velocity_command((3.0, 0.0, -10.0))
    assert pytest.approx(vz) == -1.5


def test_near_zero_distance():
    """Verify that near-zero distance is handled without crash/division-by-zero."""
    controller = FollowController()
    vx, vy, vz = controller.get_velocity_command((0.005, 0.005, 0.0))
    assert vx == 0.0
    assert vy == 0.0
    assert vz == 0.0


def test_get_position_command():
    """Verify position offset calculation matches geometry."""
    controller = FollowController()
    # Target at 5m in front. Standoff 3m. Expected offset to travel = 2m.
    dx, dy, dz = controller.get_position_command((5.0, 0.0, 2.0))
    assert pytest.approx(dx) == 2.0
    assert pytest.approx(dy) == 0.0
    assert pytest.approx(dz) == 2.0
