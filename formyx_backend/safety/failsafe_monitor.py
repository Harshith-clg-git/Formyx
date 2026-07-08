"""
formyx_backend/safety/failsafe_monitor.py
-----------------------------------------
Monitors vehicle telemetry and safety boundaries (geofences, battery limits,
GPS lock status, heartbeat health) and posts emergency FSM events when breaches occur.
"""

from __future__ import annotations

import logging
import math
import time
from typing import TYPE_CHECKING

from config import get
from mission_manager.state_machine import MissionEvent

if TYPE_CHECKING:
    from mavlink_interface.connection import TelemetrySnapshot
    from mission_manager.state_machine import MissionStateMachine

log = logging.getLogger(__name__)


class FailsafeMonitor:
    """
    Actively checks vehicle health metrics against safety thresholds and
    triggers appropriate failsafes in the mission state machine.
    """

    def __init__(self, state_machine: MissionStateMachine) -> None:
        self.state_machine = state_machine

        # Load parameters
        self.battery_critical_pct: int = get("safety", "battery_critical_pct", 15)
        self.battery_warning_pct: int = get("safety", "battery_warning_pct", 25)
        self.gps_min_satellites: int = get("safety", "gps_min_satellites", 6)
        self.max_geofence_radius: float = get("safety", "max_geofence_radius_m", 50.0)
        self.max_altitude: float = get("safety", "max_altitude_m", 15.0)
        self.heartbeat_timeout: float = get("safety", "heartbeat_loss_timeout_s", 3.0)

        # Home position coordinates for geofence checks (locked on arming)
        self.home_lat: float | None = None
        self.home_lon: float | None = None

        # Triggers to prevent duplicate event spamming
        self.battery_warning_triggered = False
        self.battery_critical_triggered = False
        self.gps_degraded_triggered = False
        self.geofence_breach_triggered = False
        self.heartbeat_lost_triggered = False

        log.info(
            "FailsafeMonitor initialized: batt_warn=%d%%, batt_crit=%d%%, "
            "gps_sats=%d, geofence=%.1fm, max_alt=%.1fm, hb_timeout=%.1fs",
            self.battery_warning_pct,
            self.battery_critical_pct,
            self.gps_min_satellites,
            self.max_geofence_radius,
            self.max_altitude,
            self.heartbeat_timeout,
        )

    def set_home_position(self, lat: float, lon: float) -> None:
        """Manually lock the geofence home position."""
        self.home_lat = lat
        self.home_lon = lon
        log.info("Geofence home position locked manually: (%.6f, %.6f)", lat, lon)

    def reset_triggers(self) -> None:
        """Reset all triggered flags (useful if vehicle state recovers)."""
        self.battery_warning_triggered = False
        self.battery_critical_triggered = False
        self.gps_degraded_triggered = False
        self.geofence_breach_triggered = False
        self.heartbeat_lost_triggered = False
        log.info("Failsafe monitor triggers reset.")

    def monitor(
        self, telemetry: TelemetrySnapshot, current_time: float | None = None
    ) -> None:
        """
        Scan latest telemetry and trigger events if any safety constraints are violated.

        Parameters
        ----------
        telemetry : TelemetrySnapshot
            The latest vehicle telemetry data.
        current_time : float, optional
            Reference monotonic time. If None, uses time.monotonic().
        """
        now = current_time if current_time is not None else time.monotonic()

        # ------------------------------------------------------------------
        # 1. Heartbeat Link Loss Check
        # ------------------------------------------------------------------
        if not telemetry.connected:
            hb_age = float("inf")
        else:
            hb_age = now - telemetry.last_heartbeat_ts

        if hb_age > self.heartbeat_timeout:
            if not self.heartbeat_lost_triggered:
                log.critical("MAVLink heartbeat lost! Age: %.2fs", hb_age)
                self.state_machine.post_event(
                    MissionEvent.HEARTBEAT_LOST, source="failsafe_monitor"
                )
                self.heartbeat_lost_triggered = True
        else:
            self.heartbeat_lost_triggered = False

        # ------------------------------------------------------------------
        # 2. Battery Percentage Checks
        # ------------------------------------------------------------------
        if telemetry.battery_remaining_pct != -1:
            # Critical low battery -> triggers RTL
            if telemetry.battery_remaining_pct < self.battery_critical_pct:
                if not self.battery_critical_triggered:
                    log.critical(
                        "Battery critically low: %d%%! Triggering RTL.",
                        telemetry.battery_remaining_pct,
                    )
                    self.state_machine.post_event(
                        MissionEvent.BATTERY_CRITICAL, source="failsafe_monitor"
                    )
                    self.battery_critical_triggered = True
            # Warning low battery -> logs warning event
            elif telemetry.battery_remaining_pct < self.battery_warning_pct:
                if not self.battery_warning_triggered:
                    log.warning(
                        "Battery warning threshold breached: %d%%",
                        telemetry.battery_remaining_pct,
                    )
                    self.state_machine.post_event(
                        MissionEvent.BATTERY_WARNING, source="failsafe_monitor"
                    )
                    self.battery_warning_triggered = True
            else:
                self.battery_warning_triggered = False
                self.battery_critical_triggered = False

        # ------------------------------------------------------------------
        # 3. GPS Quality Checks (Checked only when flying/navigating)
        # ------------------------------------------------------------------
        if self.state_machine.is_flying():
            if (
                telemetry.satellites_visible < self.gps_min_satellites
                or telemetry.gps_fix_type < 3
            ):
                if not self.gps_degraded_triggered:
                    log.warning(
                        "GPS Signal Degraded! Sats=%d, Fix=%d",
                        telemetry.satellites_visible,
                        telemetry.gps_fix_type,
                    )
                    self.state_machine.post_event(
                        MissionEvent.GPS_DEGRADED, source="failsafe_monitor"
                    )
                    self.gps_degraded_triggered = True
            else:
                self.gps_degraded_triggered = False

        # ------------------------------------------------------------------
        # 4. Geofencing (Horizontal Radius & Altitude Ceilings)
        # ------------------------------------------------------------------
        # Auto-initialize home coordinate when vehicle arms
        if telemetry.armed and telemetry.gps_fix_type >= 3:
            if self.home_lat is None or self.home_lon is None:
                self.home_lat = telemetry.lat_deg
                self.home_lon = telemetry.lon_deg
                log.info(
                    "Vehicle ARMED — auto-locked home coordinates at (%.6f, %.6f)",
                    self.home_lat,
                    self.home_lon,
                )

        # Check horizontal distance from home if home is set
        if self.home_lat is not None and self.home_lon is not None:
            # Lat/Lon flat-Earth distance approximation in meters
            lat_mid_rad = math.radians((telemetry.lat_deg + self.home_lat) / 2.0)
            dx = (telemetry.lat_deg - self.home_lat) * 111319.9
            dy = (
                (telemetry.lon_deg - self.home_lon)
                * 111319.9
                * math.cos(lat_mid_rad)
            )
            dist_from_home = math.sqrt(dx**2 + dy**2)

            if dist_from_home > self.max_geofence_radius:
                if not self.geofence_breach_triggered:
                    log.critical(
                        "GEOFENCE BREACH: %.1fm from home (limit=%.1fm)!",
                        dist_from_home,
                        self.max_geofence_radius,
                    )
                    self.state_machine.post_event(
                        MissionEvent.GEOFENCE_BREACH, source="failsafe_monitor"
                    )
                    self.geofence_breach_triggered = True
            else:
                # Clear horizontal geofence breach flag
                if telemetry.alt_agl_m <= self.max_altitude:
                    self.geofence_breach_triggered = False

        # Check vertical ceiling
        if telemetry.alt_agl_m > self.max_altitude:
            if not self.geofence_breach_triggered:
                log.critical(
                    "ALTITUDE CEILING BREACH: Alt=%.1fm (limit=%.1fm)!",
                    telemetry.alt_agl_m,
                    self.max_altitude,
                )
                self.state_machine.post_event(
                    MissionEvent.GEOFENCE_BREACH, source="failsafe_monitor"
                )
                self.geofence_breach_triggered = True
