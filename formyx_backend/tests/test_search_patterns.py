"""
tests/test_search_patterns.py
-----------------------------
Unit tests for search pattern generators in navigation/search_patterns.py.
"""

from __future__ import annotations

import sys
import pathlib
import pytest

_BACKEND = pathlib.Path(__file__).parent.parent
if str(_BACKEND) not in sys.path:
    sys.path.insert(0, str(_BACKEND))

from navigation.search_patterns import (
    generate_expanding_square,
    generate_lawnmower,
)


def test_expanding_square_geometry():
    """Verify that waypoints follow the clockwise expanding spiral correctly."""
    # Step = 2.0, max_radius = 5.0
    waypoints = generate_expanding_square(step_m=2.0, max_radius_m=5.0)
    
    # Expected legs:
    # 1: North (2.0) -> (2.0, 0.0)
    # 2: East (2.0)  -> (2.0, 2.0)
    # 3: South (4.0) -> (-2.0, 2.0)
    # 4: West (4.0)  -> (-2.0, -2.0)
    # 5: North (6.0) -> (4.0, -2.0)  (Wait, X absolute is 4.0 <= 5.0, so yes)
    # 6: East (6.0)  -> (4.0, 4.0)   (Wait, Y absolute is 4.0 <= 5.0, so yes)
    # 7: South (8.0) -> (-4.0, 4.0)  (X absolute is 4.0 <= 5.0, so yes)
    # 8: West (8.0)  -> (-4.0, -4.0) (Y absolute is 4.0 <= 5.0, so yes)
    # 9: North (10.0) -> (6.0, -4.0) (X absolute is 6.0 > 5.0 -> breaches boundary!)
    
    assert len(waypoints) == 8
    assert waypoints[0] == (2.0, 0.0)
    assert waypoints[1] == (2.0, 2.0)
    assert waypoints[2] == (-2.0, 2.0)
    assert waypoints[3] == (-2.0, -2.0)
    assert waypoints[4] == (4.0, -2.0)
    assert waypoints[5] == (4.0, 4.0)
    assert waypoints[6] == (-4.0, 4.0)
    assert waypoints[7] == (-4.0, -4.0)


def test_expanding_square_respects_boundary():
    """Verify that no waypoints in expanding square breach the maximum radius boundary."""
    max_radius = 10.0
    waypoints = generate_expanding_square(step_m=3.0, max_radius_m=max_radius)
    
    for x, y in waypoints:
        assert abs(x) <= max_radius
        assert abs(y) <= max_radius


def test_expanding_square_default_radius():
    """Verify that expanding square uses the search_radius_m config by default."""
    # settings.yaml default is 10.0
    waypoints = generate_expanding_square(step_m=4.0)
    for x, y in waypoints:
        assert abs(x) <= 10.0
        assert abs(y) <= 10.0


def test_lawnmower_geometry():
    """Verify that lawnmower sweeps back and forth correctly."""
    # width = 10m (Y axis), length = 8m (X axis), step = 4m (spacing)
    waypoints = generate_lawnmower(width_m=10.0, length_m=8.0, step_m=4.0)
    
    # Expected flow:
    # 1. Sweep positive Y: (0.0, 10.0)
    # 2. Advance to next track: (4.0, 10.0)
    # 3. Sweep negative Y: (4.0, 0.0)
    # 4. Advance to next track: (8.0, 0.0)
    # 5. Sweep positive Y: (8.0, 10.0)
    # 6. Advance to next track: 12.0 > 8.0 -> terminate.
    
    assert len(waypoints) == 5
    assert waypoints[0] == (0.0, 10.0)
    assert waypoints[1] == (4.0, 10.0)
    assert waypoints[2] == (4.0, 0.0)
    assert waypoints[3] == (8.0, 0.0)
    assert waypoints[4] == (8.0, 10.0)


def test_lawnmower_empty_if_length_zero():
    """If length is less than 0, it should be empty."""
    waypoints = generate_lawnmower(width_m=10.0, length_m=-1.0, step_m=4.0)
    assert len(waypoints) == 0
