"""
formyx_backend/depth/realsense_interface.py
-------------------------------------------
Interfaces with the Intel RealSense D435i depth camera.
Acquires aligned RGB and Depth frames, and implements spatial patch-averaging
to handle depth shadows (invalid depth measurements).
"""

from __future__ import annotations

import logging
from typing import Tuple

import numpy as np

from config import get

log = logging.getLogger(__name__)

# Try to import pyrealsense2, falling back to mock mode if not installed (e.g. on Windows)
try:
    import pyrealsense2 as rs
    HAS_REALSENSE = True
except ImportError:
    HAS_REALSENSE = False
    log.warning("pyrealsense2 library not found. RealSenseInterface will run in MOCK mode.")


class RealSenseInterface:
    """
    Manages the Intel RealSense camera lifecycle and frame acquisition.
    Optimized to align depth maps with RGB colorspace.
    """

    def __init__(self, use_mock: bool = False) -> None:
        self.is_mock = use_mock or (not HAS_REALSENSE)
        
        # Load settings
        self.patch_size: int = get("depth", "depth_patch_size", 15)
        self.min_depth: float = get("depth", "min_valid_depth_m", 0.3)
        self.max_depth: float = get("depth", "max_valid_depth_m", 10.0)

        self._pipeline = None
        self._align = None
        self._depth_scale = 0.001  # Default fallback scale (1mm = 0.001m)

    def start(self) -> None:
        """
        Start the camera pipeline. Falls back to mock mode if no device is connected.
        """
        if self.is_mock:
            log.info("Starting Mock RealSense Interface (640x480 resolution).")
            return

        try:
            self._pipeline = rs.pipeline()
            config = rs.config()
            
            # Enable standard streams
            config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
            config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
            
            pipeline_profile = self._pipeline.start(config)
            
            # Retrieve depth scale for physical unit conversion
            depth_sensor = pipeline_profile.get_device().first_depth_sensor()
            self._depth_scale = depth_sensor.get_depth_scale()
            
            # Align depth stream to color stream
            self._align = rs.align(rs.stream.color)
            log.info("RealSense D435i camera pipeline started successfully.")

        except Exception as exc:
            log.warning(
                "Failed to start RealSense camera pipeline: %s. "
                "Switching to MOCK mode.",
                exc,
            )
            self.is_mock = True

    def get_frames(self) -> Tuple[np.ndarray, np.ndarray] | None:
        """
        Retrieve aligned color and depth frames.

        Returns
        -------
        Tuple[np.ndarray, np.ndarray] or None
            (color_image, depth_image) as numpy arrays.
            * color_image: shape (480, 640, 3) uint8 BGR
            * depth_image: shape (480, 640) uint16 (millimeters/sensor units) or float (meters in mock)
        """
        if self.is_mock:
            # Generate synthetic frames
            color = np.zeros((480, 640, 3), dtype=np.uint8)
            # Create a mock target at 3.5m in the center
            depth = np.full((480, 640), 3.5, dtype=np.float32)
            # Add some simulated depth shadows (0.0 values) to test averaging
            depth[230:250, 310:330] = 0.0
            # Restore center object
            depth[240:245, 320:325] = 3.5
            return color, depth

        if self._pipeline is None:
            log.error("RealSense pipeline is not started.")
            return None

        try:
            # Block until frames are available (timeout 2 seconds)
            frames = self._pipeline.wait_for_frames(timeout_ms=2000)
            aligned_frames = self._align.process(frames)
            
            color_frame = aligned_frames.get_color_frame()
            depth_frame = aligned_frames.get_depth_frame()
            
            if not color_frame or not depth_frame:
                return None
                
            color_image = np.asanyarray(color_frame.get_data())
            depth_image = np.asanyarray(depth_frame.get_data())
            
            return color_image, depth_image

        except Exception as exc:
            log.error("Failed to retrieve frames from RealSense: %s", exc)
            return None

    def get_distance_at_pixel(
        self, depth_frame: np.ndarray, x: int, y: int
    ) -> float | None:
        """
        Calculate robust distance at pixel coordinates (x, y) using spatial patch-averaging.

        Parameters
        ----------
        depth_frame : np.ndarray
            The depth image array.
        x : int
            Pixel column coordinate.
        y : int
            Pixel row coordinate.

        Returns
        -------
        float or None
            Distance in meters, or None if no valid depth values are found.
        """
        h, w = depth_frame.shape[:2]
        
        # Calculate half patch size
        half_p = self.patch_size // 2
        
        # Determine crop boundaries, ensuring they remain inside the image bounds
        x_min = max(0, x - half_p)
        x_max = min(w - 1, x + half_p)
        y_min = max(0, y - half_p)
        y_max = min(h - 1, y + half_p)

        # Extract patch
        patch = depth_frame[y_min : y_max + 1, x_min : x_max + 1]

        # Convert patch values to meters
        if np.issubdtype(depth_frame.dtype, np.integer):
            # Scale uint16 values to float meters
            patch_meters = patch * self._depth_scale
        else:
            patch_meters = patch.astype(np.float32)

        # Filter values to keep only those within the valid depth range
        valid_mask = (patch_meters >= self.min_depth) & (patch_meters <= self.max_depth)
        valid_values = patch_meters[valid_mask]

        if len(valid_values) == 0:
            log.warning("No valid depth values in patch at (%d, %d)", x, y)
            return None

        # Return median value to avoid outlier skew (e.g. from edge bleeding)
        return float(np.median(valid_values))

    def stop(self) -> None:
        """Stop the camera pipeline."""
        if self.is_mock:
            log.info("Mock RealSense Interface stopped.")
            return

        if self._pipeline:
            try:
                self._pipeline.stop()
                log.info("RealSense D435i camera pipeline stopped.")
            except Exception as exc:
                log.error("Error stopping RealSense pipeline: %s", exc)
            finally:
                self._pipeline = None
