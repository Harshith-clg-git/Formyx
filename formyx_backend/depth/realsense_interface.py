"""
formyx_backend/depth/realsense_interface.py
-------------------------------------------
Interfaces with the Intel RealSense D435i depth camera.
Acquires aligned RGB and Depth frames, and implements spatial patch-averaging
to handle depth shadows (invalid depth measurements).
"""

from __future__ import annotations

import logging
from typing import Any, Tuple

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
        self._rs_depth_frame = None  # Retained raw RS frame for get_distance() access

        # Camera intrinsics — populated after start() on real hardware
        self.fx: float = 606.8
        self.fy: float = 607.1
        self.ppx: float = 320.0
        self.ppy: float = 240.0

    @property
    def pipeline(self) -> Any:
        """Access the underlying pyrealsense2 pipeline."""
        return self._pipeline

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
            
            # Retrieve camera intrinsics for 3D coordinate projection
            color_stream = pipeline_profile.get_stream(rs.stream.color)
            intrinsics = color_stream.as_video_stream_profile().get_intrinsics()
            self.fx = intrinsics.fx
            self.fy = intrinsics.fy
            self.ppx = intrinsics.ppx
            self.ppy = intrinsics.ppy
            log.info(
                "Camera intrinsics — fx=%.1f fy=%.1f cx=%.1f cy=%.1f",
                self.fx, self.fy, self.ppx, self.ppy,
            )

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
            * depth_image: shape (480, 640) uint16 — raw sensor units (multiply by
              self._depth_scale to get metres). In mock mode this is float32 metres.

        The raw pyrealsense2 DepthFrame is also retained internally and can be
        queried with get_distance_at_pixel() for per-pixel metric distances.
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
            self._rs_depth_frame = None  # No native frame in mock mode
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
            
            # Retain the native RS frame so get_distance_at_pixel() can use
            # depth_frame.get_distance() — the most accurate shadow-free path.
            self._rs_depth_frame = depth_frame
                
            color_image = np.asanyarray(color_frame.get_data())
            depth_image = np.asanyarray(depth_frame.get_data())
            
            return color_image, depth_image

        except Exception as exc:
            log.error("Failed to retrieve frames from RealSense: %s", exc)
            return None

    def get_distance_at_pixel(self, depth_frame: np.ndarray, x: int, y: int) -> float | None:
        """
        Robust metric distance at pixel (x, y) with depth-shadow handling.

        Strategy (mirrors Formyxcv's get_robust_distance):
        1. If a retained native RS DepthFrame is available, use the fast
           depth_frame.get_distance() API which handles the depth scale
           internally — no manual uint16 conversion needed.
        2. Fall back to numpy patch-averaging on the raw uint16 array,
           filtering pixels that are exactly 0 (sensor shadow) BEFORE
           applying the depth scale, then applying the valid-range filter.

        Parameters
        ----------
        depth_frame : np.ndarray
            The uint16 depth image returned by get_frames().
        x : int
            Pixel column coordinate.
        y : int
            Pixel row coordinate.

        Returns
        -------
        float or None
            Distance in metres, or None if the whole patch is a depth shadow.
        """
        # --- Path 1: native RS frame API (real hardware only) ---
        if self._rs_depth_frame is not None:
            try:
                dist = self._rs_depth_frame.get_distance(int(x), int(y))
                if dist > 0.0 and self.min_depth <= dist <= self.max_depth:
                    return dist
                # Centre pixel is a shadow — search a surrounding patch
                half_p = self.patch_size // 2
                h, w = 480, 640  # known stream dimensions
                distances = []
                for dy in range(-half_p, half_p + 1):
                    for dx in range(-half_p, half_p + 1):
                        px, py = int(x) + dx, int(y) + dy
                        if 0 <= px < w and 0 <= py < h:
                            d = self._rs_depth_frame.get_distance(px, py)
                            if self.min_depth <= d <= self.max_depth:
                                distances.append(d)
                if distances:
                    return float(np.median(distances))
                return None
            except Exception:
                pass  # Fall through to numpy path

        # --- Path 2: numpy patch on uint16 array (mock or fallback) ---
        h, w = depth_frame.shape[:2]
        half_p = self.patch_size // 2
        x_min = max(0, x - half_p)
        x_max = min(w - 1, x + half_p)
        y_min = max(0, y - half_p)
        y_max = min(h - 1, y + half_p)
        patch = depth_frame[y_min : y_max + 1, x_min : x_max + 1]

        if np.issubdtype(depth_frame.dtype, np.integer):
            # BUG FIX: filter out sensor-shadow pixels (raw value == 0) BEFORE
            # scaling — previously, 0 * depth_scale = 0.0 which then failed the
            # >= min_depth gate and discarded all valid neighbours in the patch.
            valid_raw = patch[patch > 0]
            if len(valid_raw) == 0:
                return None
            patch_meters = valid_raw.astype(np.float32) * self._depth_scale
        else:
            patch_meters = patch.astype(np.float32)

        valid_mask = (patch_meters >= self.min_depth) & (patch_meters <= self.max_depth)
        valid_values = patch_meters[valid_mask]

        if len(valid_values) == 0:
            return None

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
