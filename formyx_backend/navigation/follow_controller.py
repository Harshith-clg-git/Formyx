"""
formyx_backend/navigation/follow_controller.py
----------------------------------------------
Calculates body-frame velocity commands to maintain a stable follow
distance from a moving target (balloon).
"""

from __future__ import annotations

import logging
import math
from typing import Tuple

from config import get

log = logging.getLogger(__name__)


class FollowController:
    """
    Computes smooth approach vectors and body-relative velocity targets
    to track and follow a target in 3D space.
    """

    def __init__(self, desired_follow_dist_m: float | None = None) -> None:
        # Load constraints from global settings
        self.max_horiz_speed: float = get("navigation", "max_horizontal_speed_ms", 3.0)
        self.max_vert_speed: float = get("navigation", "max_vertical_speed_ms", 1.5)
        self.follow_distance: float = (
            desired_follow_dist_m if desired_follow_dist_m is not None
            else get("navigation", "follow_distance_m", 3.0)
        )
        
        # Proportional controller gains
        self.kp_xy: float = get("navigation", "kp_xy", 0.5)
        self.kp_z: float = get("navigation", "kp_z", 0.5)

        log.info(
            "FollowController initialized: follow_dist=%.1fm, "
            "max_horiz=%.1fm/s, max_vert=%.1fm/s",
            self.follow_distance,
            self.max_horiz_speed,
            self.max_vert_speed,
        )

    def compute_velocity_command(
        self, x: float, y: float, z: float
    ) -> Tuple[float, float, float]:
        """Compatibility wrapper for hardware test guide."""
        return self.get_velocity_command((x, y, z))

    def get_velocity_command(
        self, target_rel_pos: Tuple[float, float, float]
    ) -> Tuple[float, float, float]:
        """
        Calculate body-frame velocity targets (vx, vy, vz) in m/s to follow the target.

        Parameters
        ----------
        target_rel_pos : Tuple[float, float, float]
            Relative position of the target (x, y, z) in the drone's FRD body frame:
            * x: Forward (positive = target in front, negative = target behind)
            * y: Right (positive = target to the right, negative = target to the left)
            * z: Down (positive = target below drone, negative = target above drone)

        Returns
        -------
        Tuple[float, float, float]
            Velocity commands (vx, vy, vz) in FRD body frame (m/s).
        """
        x_rel, y_rel, z_rel = target_rel_pos
        d_horiz = math.sqrt(x_rel**2 + y_rel**2)

        # 1. Horizontal velocity calculation
        if d_horiz < 0.01:
            vx = 0.0
            vy = 0.0
        else:
            # Distance error relative to desired stand-off distance
            err_horiz = d_horiz - self.follow_distance
            
            # Unit vector pointing from drone to target (horizontal)
            ux = x_rel / d_horiz
            uy = y_rel / d_horiz
            
            # Proportional velocity commands
            vx = err_horiz * self.kp_xy * ux
            vy = err_horiz * self.kp_xy * uy

        # 2. Vertical velocity calculation
        # We want target relative z to be 0 (meaning same altitude).
        # Since +Z is down, if target is below (z_rel > 0), we descend (+vz).
        vz = z_rel * self.kp_z

        # 3. Limit/Clamp Velocities to safety bounds
        speed_horiz = math.sqrt(vx**2 + vy**2)
        if speed_horiz > self.max_horiz_speed:
            scale = self.max_horiz_speed / speed_horiz
            vx *= scale
            vy *= scale

        if abs(vz) > self.max_vert_speed:
            vz = math.copysign(self.max_vert_speed, vz)

        log.debug(
            "Follow command: target_offset=(%.2f, %.2f, %.2f) -> "
            "vel_cmd=(%.2f, %.2f, %.2f) m/s",
            x_rel, y_rel, z_rel,
            vx, vy, vz,
        )
        return vx, vy, vz

    def get_position_command(
        self, target_rel_pos: Tuple[float, float, float]
    ) -> Tuple[float, float, float]:
        """
        Calculate body-frame relative position offsets (dx, dy, dz) to maintain stand-off.

        Parameters
        ----------
        target_rel_pos : Tuple[float, float, float]
            Relative position of the target (x, y, z) in FRD body frame.

        Returns
        -------
        Tuple[float, float, float]
            Body-relative position offsets (dx, dy, dz) in meters.
        """
        x_rel, y_rel, z_rel = target_rel_pos
        d_horiz = math.sqrt(x_rel**2 + y_rel**2)

        if d_horiz < 0.01:
            dx = 0.0
            dy = 0.0
        else:
            err_horiz = d_horiz - self.follow_distance
            ux = x_rel / d_horiz
            uy = y_rel / d_horiz
            dx = err_horiz * ux
            dy = err_horiz * uy

        dz = z_rel
        return dx, dy, dz
