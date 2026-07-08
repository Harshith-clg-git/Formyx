"""
formyx_backend/navigation/search_patterns.py
--------------------------------------------
Implements autonomous search patterns (expanding square and lawnmower)
used to find the balloon target when its location is unknown.
"""

from __future__ import annotations

import logging
import math
from typing import List, Tuple

from config import get

log = logging.getLogger(__name__)


def generate_expanding_square(
    step_m: float = 4.0,
    max_radius_m: float | None = None,
) -> List[Tuple[float, float]]:
    """
    Generate an expanding square spiral pattern of (x, y) relative offsets (in meters)
    in the FRD body frame starting at (0.0, 0.0).

    The pattern grows by `step_m` every two turns (legs) and is bounded by `max_radius_m`.

    Parameters
    ----------
    step_m : float
        Spacing between tracks of the square spiral.
    max_radius_m : float, optional
        Maximum bounding box radius (limits horizontal distance from origin).
        If None, reads `navigation.search_radius_m` from settings.yaml.

    Returns
    -------
    List[Tuple[float, float]]
        List of relative waypoint coordinates (x, y) in meters.
    """
    if max_radius_m is None:
        max_radius_m = get("navigation", "search_radius_m", 10.0)

    waypoints: List[Tuple[float, float]] = []
    x, y = 0.0, 0.0
    i = 1

    log.info(
        "Generating expanding square pattern: step=%.1fm, max_radius=%.1fm",
        step_m,
        max_radius_m,
    )

    while True:
        # Segment length increases every 2 legs
        L = math.ceil(i / 2) * step_m
        direction = i % 4

        # Compute next waypoint
        next_x, next_y = x, y
        if direction == 1:    # North / Forward (+X)
            next_x += L
        elif direction == 2:  # East / Right (+Y)
            next_y += L
        elif direction == 3:  # South / Backward (-X)
            next_x -= L
        else:                 # West / Left (-Y)
            next_y -= L

        # Check if the next segment endpoint breaches the boundary box.
        # If it does, we clip the waypoint generation.
        if abs(next_x) > max_radius_m or abs(next_y) > max_radius_m:
            log.debug("Expanding square hit boundary at (x=%.2f, y=%.2f)", next_x, next_y)
            break

        x, y = next_x, next_y
        waypoints.append((x, y))
        i += 1

    log.info("Generated %d waypoints for expanding square.", len(waypoints))
    return waypoints


def generate_lawnmower(
    width_m: float,
    length_m: float,
    step_m: float = 4.0,
) -> List[Tuple[float, float]]:
    """
    Generate a lawnmower (creeping line) search pattern of (x, y) relative offsets (in meters)
    sweeping a rectangular area of width x length.

    The rectangle is situated in the first quadrant of the local frame:
    * X extends from 0.0 to length_m
    * Y extends from 0.0 to width_m

    Parameters
    ----------
    width_m : float
        Width of the search area (along the Y axis).
    length_m : float
        Length of the search area (along the X axis).
    step_m : float
        Spacing between tracks of the search.

    Returns
    -------
    List[Tuple[float, float]]
        List of relative waypoint coordinates (x, y) in meters.
    """
    waypoints: List[Tuple[float, float]] = []
    x = 0.0
    direction = 1  # 1 = sweep in positive Y, -1 = sweep in negative Y

    log.info(
        "Generating lawnmower pattern: width=%.1fm, length=%.1fm, step=%.1fm",
        width_m,
        length_m,
        step_m,
    )

    while x <= length_m:
        # Move along Y axis to the end of the current track
        if direction == 1:
            waypoints.append((x, width_m))
        else:
            waypoints.append((x, 0.0))

        # Advance along X axis to the next track
        x += step_m
        if x > length_m:
            break

        # Mark the turn corner at the start of the next track
        if direction == 1:
            waypoints.append((x, width_m))
        else:
            waypoints.append((x, 0.0))

        # Flip the direction for the next sweep
        direction *= -1

    log.info("Generated %d waypoints for lawnmower.", len(waypoints))
    return waypoints
