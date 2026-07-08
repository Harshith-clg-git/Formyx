"""
tests/test_failsafe_monitor.py
------------------------------
Unit tests for the FailsafeMonitor class.
"""

from __future__ import annotations

import sys
import pathlib
import pytest
from unittest.mock import MagicMock

_BACKEND = pathlib.Path(__file__).parent.parent
if str(_BACKEND) not in sys.path:
    sys.path.insert(0, str(_BACKEND))

from safety.failsafe_monitor import FailsafeMonitor
from mission_manager.state_machine import (
    MissionStateMachine,
    MissionState,
    MissionEvent,
)
from mavlink_interface.connection import TelemetrySnapshot


import time

def _mock_telem(
    connected=True,
    last_heartbeat_ts=None,
    battery_remaining_pct=100,
    satellites_visible=10,
    gps_fix_type=3,
    armed=False,
    lat_deg=12.9716,
    lon_deg=77.5946,
    alt_agl_m=2.0
):
    """Helper to construct a TelemetrySnapshot."""
    if last_heartbeat_ts is None:
        last_heartbeat_ts = time.monotonic()
    return TelemetrySnapshot(
        connected=connected,
        last_heartbeat_ts=last_heartbeat_ts,
        battery_remaining_pct=battery_remaining_pct,
        satellites_visible=satellites_visible,
        gps_fix_type=gps_fix_type,
        armed=armed,
        lat_deg=lat_deg,
        lon_deg=lon_deg,
        alt_agl_m=alt_agl_m,
    )


def test_failsafe_monitor_init():
    sm = MissionStateMachine()
    monitor = FailsafeMonitor(sm)
    
    assert monitor.battery_critical_pct == 15
    assert monitor.battery_warning_pct == 25
    assert monitor.gps_min_satellites == 6
    assert monitor.max_geofence_radius == 50.0
    assert monitor.max_altitude == 15.0
    assert monitor.heartbeat_timeout == 3.0


def test_heartbeat_loss_failsafe():
    sm = MagicMock()
    monitor = FailsafeMonitor(sm)
    
    # Healthy telemetry at t=100s
    telem = _mock_telem(last_heartbeat_ts=100.0)
    monitor.monitor(telem, current_time=101.0)
    sm.post_event.assert_not_called()
    
    # Heartbeat timeout: last heartbeat 95s, current time 100s -> age 5s (limit 3s)
    telem_lost = _mock_telem(last_heartbeat_ts=95.0)
    monitor.monitor(telem_lost, current_time=100.0)
    sm.post_event.assert_called_once_with(MissionEvent.HEARTBEAT_LOST, source="failsafe_monitor")
    
    # Check trigger limits spam: call again, shouldn't post again
    sm.post_event.reset_mock()
    monitor.monitor(telem_lost, current_time=101.0)
    sm.post_event.assert_not_called()


def test_battery_failsafes():
    sm = MagicMock()
    monitor = FailsafeMonitor(sm)
    
    # Battery at 24% (Warning threshold)
    telem = _mock_telem(battery_remaining_pct=24)
    monitor.monitor(telem)
    sm.post_event.assert_called_once_with(MissionEvent.BATTERY_WARNING, source="failsafe_monitor")
    
    # Battery at 14% (Critical threshold)
    sm.post_event.reset_mock()
    telem_crit = _mock_telem(battery_remaining_pct=14)
    monitor.monitor(telem_crit)
    sm.post_event.assert_called_once_with(MissionEvent.BATTERY_CRITICAL, source="failsafe_monitor")


def test_gps_degraded_when_flying():
    # Set initial state to flying state (e.g. SEARCHING)
    sm = MissionStateMachine(initial_state=MissionState.SEARCHING)
    sm.post_event = MagicMock()
    monitor = FailsafeMonitor(sm)
    
    # Sats degraded: 5 sats (limit 6)
    telem = _mock_telem(satellites_visible=5)
    monitor.monitor(telem)
    sm.post_event.assert_called_once_with(MissionEvent.GPS_DEGRADED, source="failsafe_monitor")


def test_geofence_horizontal_breach():
    sm = MissionStateMachine(initial_state=MissionState.NAVIGATING_TO_GPS)
    sm.post_event = MagicMock()
    monitor = FailsafeMonitor(sm)
    
    # Arm the vehicle -> auto locks home position at (12.9716, 77.5946)
    telem_arm = _mock_telem(armed=True, lat_deg=12.9716, lon_deg=77.5946)
    monitor.monitor(telem_arm)
    assert monitor.home_lat == 12.9716
    assert monitor.home_lon == 77.5946
    
    # Fly close: (12.9717, 77.5947) -> within 50m
    telem_close = _mock_telem(armed=True, lat_deg=12.9717, lon_deg=77.5947)
    monitor.monitor(telem_close)
    sm.post_event.assert_not_called()
    
    # Fly far: (12.9730, 77.5960) -> ~210 meters away (breach!)
    telem_far = _mock_telem(armed=True, lat_deg=12.9730, lon_deg=77.5960)
    monitor.monitor(telem_far)
    sm.post_event.assert_called_once_with(MissionEvent.GEOFENCE_BREACH, source="failsafe_monitor")


def test_geofence_altitude_breach():
    sm = MissionStateMachine(initial_state=MissionState.NAVIGATING_TO_GPS)
    sm.post_event = MagicMock()
    monitor = FailsafeMonitor(sm)
    
    # Altitude 16m (ceiling is 15.0m)
    telem = _mock_telem(alt_agl_m=16.0)
    monitor.monitor(telem)
    sm.post_event.assert_called_once_with(MissionEvent.GEOFENCE_BREACH, source="failsafe_monitor")
