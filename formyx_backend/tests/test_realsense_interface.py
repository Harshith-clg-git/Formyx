"""
tests/test_realsense_interface.py
----------------------------------
Unit tests for the RealSenseInterface class.
"""

from __future__ import annotations

import sys
import pathlib
import numpy as np
import pytest

_BACKEND = pathlib.Path(__file__).parent.parent
if str(_BACKEND) not in sys.path:
    sys.path.insert(0, str(_BACKEND))

from depth.realsense_interface import RealSenseInterface


def test_realsense_init_and_mock():
    """Verify that interface correctly initializes and identifies mock state."""
    interface = RealSenseInterface(use_mock=True)
    assert interface.is_mock is True
    assert interface.patch_size == 15
    assert interface.min_depth == 0.3
    assert interface.max_depth == 10.0


def test_mock_get_frames():
    """Verify mock frame retrieval returns correct shapes and types."""
    interface = RealSenseInterface(use_mock=True)
    interface.start()
    
    frames = interface.get_frames()
    assert frames is not None
    color, depth = frames
    
    assert color.shape == (480, 640, 3)
    assert depth.shape == (480, 640)
    assert color.dtype == np.uint8
    assert depth.dtype == np.float32
    
    interface.stop()


def test_get_distance_at_pixel_float():
    """Verify distance calculation on float depth frames (mock format)."""
    interface = RealSenseInterface(use_mock=True)
    
    # 640x480 depth frame filled with 3.5m
    depth_frame = np.full((480, 640), 3.5, dtype=np.float32)
    
    dist = interface.get_distance_at_pixel(depth_frame, x=320, y=240)
    assert pytest.approx(dist) == 3.5


def test_get_distance_at_pixel_uint16():
    """Verify distance calculation and scaling on integer depth frames (camera format)."""
    interface = RealSenseInterface(use_mock=True)
    # Set depth scale manually for testing
    interface._depth_scale = 0.001  # 1mm = 0.001m
    
    # uint16 depth frame filled with 2500 (2.5 meters)
    depth_frame = np.full((480, 640), 2500, dtype=np.uint16)
    
    dist = interface.get_distance_at_pixel(depth_frame, x=320, y=240)
    assert pytest.approx(dist) == 2.5


def test_patch_averaging_ignores_shadows():
    """Verify patch-averaging ignores zeros (shadows) and out-of-range values."""
    interface = RealSenseInterface(use_mock=True)
    interface.patch_size = 5  # smaller patch for precise test
    
    # Create patch centered at (2, 2)
    # 5x5 area with values:
    # 0 0 3.0 3.0 3.0
    # 0 0 3.0 3.0 3.0
    # 0 0 3.0 3.0 3.0
    # 0 0 3.0 3.0 3.0
    # 0 0 3.0 3.0 12.0  (12m is out of max range 10m)
    depth_frame = np.full((5, 5), 3.0, dtype=np.float32)
    depth_frame[:, 0:2] = 0.0      # shadow values (ignored)
    depth_frame[4, 4] = 12.0       # out-of-range value (ignored)
    
    # Center pixel is 3.0, surrounding has some shadows
    dist = interface.get_distance_at_pixel(depth_frame, x=2, y=2)
    # Should average only the valid 3.0 values (median of 3.0s is 3.0)
    assert pytest.approx(dist) == 3.0


def test_all_shadows_returns_none():
    """Verify that if all values in the patch are invalid, None is returned."""
    interface = RealSenseInterface(use_mock=True)
    interface.patch_size = 5
    
    # Entire patch is filled with zero or out-of-range values
    depth_frame = np.full((5, 5), 0.0, dtype=np.float32)
    depth_frame[0, 0] = 0.1  # too close (min is 0.3)
    depth_frame[4, 4] = 15.0 # too far (max is 10.0)
    
    dist = interface.get_distance_at_pixel(depth_frame, x=2, y=2)
    assert dist is None
