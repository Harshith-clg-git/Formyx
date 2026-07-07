"""
tests/test_mavlink_interface.py
--------------------------------
Unit tests for ``mavlink_interface/connection.py`` and
``mavlink_interface/commands.py``.

All tests use mock objects to replace real MAVLink serial/UDP
connections so they can run offline, without an autopilot or SITL.

Test coverage
-------------
* MAVLinkConnection.connect() — heartbeat success & timeout
* TelemetrySnapshot update from every supported message type
* commands.arm / disarm — ACK accepted & rejected paths
* commands.set_flight_mode — valid / invalid mode name
* commands.takeoff — ACK success
* commands.send_position_target_local_ned — message content
* commands.send_condition_yaw — CW / CCW / absolute
"""

from __future__ import annotations

import time
import threading
from types import SimpleNamespace
from unittest.mock import MagicMock, patch, PropertyMock
import pytest

# ---------------------------------------------------------------------------
# Path shim: when running tests from repo root we need the backend on sys.path
# ---------------------------------------------------------------------------
import sys
import pathlib

_BACKEND = pathlib.Path(__file__).parent.parent  # formyx_backend/
sys.path.insert(0, str(_BACKEND))

from mavlink_interface.connection import MAVLinkConnection, TelemetrySnapshot
from mavlink_interface.commands import (
    arm,
    disarm,
    set_flight_mode,
    takeoff,
    send_position_target_local_ned,
    send_condition_yaw,
    CommandRejectedError,
    CommandTimeoutError,
)


# ===========================================================================
# Helpers / fixtures
# ===========================================================================

def _fake_heartbeat(armed: bool = False, mode: int = 4) -> SimpleNamespace:
    """Return a mock HEARTBEAT message namespace."""
    import pymavlink.mavutil as mu
    msg = SimpleNamespace()
    msg.get_type = lambda: "HEARTBEAT"
    msg.base_mode = (
        mu.mavlink.MAV_MODE_FLAG_SAFETY_ARMED if armed else 0
    )
    msg.custom_mode = mode
    return msg


def _fake_msg(msg_type: str, **kwargs) -> SimpleNamespace:
    """Generic fake MAVLink message."""
    msg = SimpleNamespace(**kwargs)
    msg.get_type = lambda: msg_type
    return msg


def _make_ack(command_id: int, result: int) -> SimpleNamespace:
    """Produce a fake COMMAND_ACK."""
    ack = SimpleNamespace()
    ack.get_type = lambda: "COMMAND_ACK"
    ack.command = command_id
    ack.result = result
    return ack


@pytest.fixture
def mock_mav_conn():
    """
    A MAVLinkConnection whose internal _mav is replaced by a MagicMock.
    The connect() method is NOT called — tests manipulate _snapshot directly.
    """
    conn = MAVLinkConnection.__new__(MAVLinkConnection)
    conn._conn_str = "udpin:localhost:14550"
    conn._heartbeat_timeout = 5
    conn._telem_rate_hz = 10
    conn._reconnect_attempts = 5
    conn._reconnect_delay = 2.0
    conn._snapshot = TelemetrySnapshot()
    conn._lock = threading.Lock()
    conn._reader_thread = None
    conn._stop_event = threading.Event()

    mav_mock = MagicMock()
    mav_mock.target_system = 1
    mav_mock.target_component = 1
    conn._mav = mav_mock

    return conn, mav_mock


# ===========================================================================
# MAVLinkConnection.connect() tests
# ===========================================================================

class TestConnect:
    @patch("mavlink_interface.connection.mavutil.mavlink_connection")
    def test_connect_success(self, mock_mav_class):
        """connect() starts reader thread when heartbeat received."""
        mock_mav_inst = MagicMock()
        mock_mav_inst.target_system = 1
        mock_mav_inst.target_component = 1
        mock_mav_inst.recv_match.return_value = _fake_heartbeat()
        mock_mav_class.return_value = mock_mav_inst

        conn = MAVLinkConnection("udpin:localhost:14550")
        conn.connect()

        assert conn._reader_thread is not None
        assert conn._reader_thread.is_alive()
        conn.close()

    @patch("mavlink_interface.connection.mavutil.mavlink_connection")
    def test_connect_heartbeat_timeout(self, mock_mav_class):
        """connect() raises ConnectionError when no heartbeat arrives."""
        mock_mav_inst = MagicMock()
        mock_mav_inst.recv_match.return_value = None   # timeout
        mock_mav_class.return_value = mock_mav_inst

        conn = MAVLinkConnection("udpin:localhost:14550")
        with pytest.raises(ConnectionError, match="No heartbeat"):
            conn.connect()

    @patch("mavlink_interface.connection.mavutil.mavlink_connection")
    def test_connect_sets_armed_state(self, mock_mav_class):
        """Heartbeat with armed flag updates snapshot.armed."""
        mock_mav_inst = MagicMock()
        mock_mav_inst.target_system = 1
        mock_mav_inst.target_component = 1
        mock_mav_inst.recv_match.return_value = _fake_heartbeat(armed=True)
        mock_mav_class.return_value = mock_mav_inst

        conn = MAVLinkConnection("udpin:localhost:14550")
        conn.connect()
        telem = conn.get_telemetry()
        assert telem.armed is True
        conn.close()


# ===========================================================================
# TelemetrySnapshot update tests
# ===========================================================================

class TestTelemetryUpdates:
    def test_update_global_position(self, mock_mav_conn):
        conn, _ = mock_mav_conn
        msg = _fake_msg(
            "GLOBAL_POSITION_INT",
            lat=175432000,   # 17.5432°
            lon=783456000,   # 78.3456°
            alt=50000,       # 50 m AMSL (mm)
            relative_alt=30000,  # 30 m AGL (mm)
            vx=100, vy=200, vz=-50,
        )
        with conn._lock:
            conn._update_global_pos(msg)

        t = conn.get_telemetry()
        assert t.lat_deg == pytest.approx(17.5432, abs=1e-4)
        assert t.lon_deg == pytest.approx(78.3456, abs=1e-4)
        assert t.alt_agl_m == pytest.approx(30.0, abs=0.01)
        assert t.alt_amsl_m == pytest.approx(50.0, abs=0.01)
        assert t.vx_ms == pytest.approx(1.0, abs=0.01)

    def test_update_attitude(self, mock_mav_conn):
        conn, _ = mock_mav_conn
        msg = _fake_msg("ATTITUDE", roll=0.1, pitch=0.05, yaw=1.57)
        with conn._lock:
            conn._update_attitude(msg)

        t = conn.get_telemetry()
        assert t.roll_rad == pytest.approx(0.1)
        assert t.yaw_rad == pytest.approx(1.57)

    def test_update_battery(self, mock_mav_conn):
        conn, _ = mock_mav_conn
        msg = _fake_msg(
            "SYS_STATUS",
            voltage_battery=11800,  # 11.8 V
            current_battery=2500,   # 25.0 A
            battery_remaining=72,
        )
        with conn._lock:
            conn._update_sys_status(msg)

        t = conn.get_telemetry()
        assert t.battery_voltage_v == pytest.approx(11.8)
        assert t.battery_current_a == pytest.approx(25.0)
        assert t.battery_remaining_pct == 72

    def test_update_gps_raw(self, mock_mav_conn):
        conn, _ = mock_mav_conn
        msg = _fake_msg("GPS_RAW_INT", fix_type=3, satellites_visible=9)
        with conn._lock:
            conn._update_gps_raw(msg)

        t = conn.get_telemetry()
        assert t.gps_fix_type == 3
        assert t.satellites_visible == 9

    def test_update_vfr_hud(self, mock_mav_conn):
        conn, _ = mock_mav_conn
        msg = _fake_msg(
            "VFR_HUD",
            groundspeed=5.2,
            airspeed=5.0,
            heading=180,
            throttle=45,
        )
        with conn._lock:
            conn._update_vfr_hud(msg)

        t = conn.get_telemetry()
        assert t.groundspeed_ms == pytest.approx(5.2)
        assert t.heading_deg == 180
        assert t.throttle_pct == 45

    def test_heartbeat_mode_name(self, mock_mav_conn):
        conn, _ = mock_mav_conn
        hb = _fake_heartbeat(armed=False, mode=5)  # 5 = LOITER
        with conn._lock:
            conn._update_snapshot_from_heartbeat(hb)

        t = conn.get_telemetry()
        assert t.flight_mode == "LOITER"
        assert t.armed is False

    def test_unknown_mode_fallback(self, mock_mav_conn):
        conn, _ = mock_mav_conn
        hb = _fake_heartbeat(armed=False, mode=99)
        with conn._lock:
            conn._update_snapshot_from_heartbeat(hb)

        t = conn.get_telemetry()
        assert t.flight_mode == "MODE_99"


# ===========================================================================
# commands.arm / disarm
# ===========================================================================

class TestArmDisarm:
    def test_arm_sends_correct_command(self, mock_mav_conn):
        """arm() calls command_long_send with param1=1."""
        from pymavlink import mavutil as mu

        conn, mav_mock = mock_mav_conn
        ack = _make_ack(
            mu.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            mu.mavlink.MAV_RESULT_ACCEPTED,
        )
        mav_mock.recv_match.return_value = ack

        arm(conn, timeout=2.0)

        call_args = mav_mock.mav.command_long_send.call_args[0]
        # param1 is index 4 (system, component, command, confirmation, param1…)
        assert call_args[4] == 1.0, "arm() must send param1=1"

    def test_disarm_sends_correct_command(self, mock_mav_conn):
        from pymavlink import mavutil as mu

        conn, mav_mock = mock_mav_conn
        ack = _make_ack(
            mu.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            mu.mavlink.MAV_RESULT_ACCEPTED,
        )
        mav_mock.recv_match.return_value = ack

        disarm(conn, timeout=2.0)

        call_args = mav_mock.mav.command_long_send.call_args[0]
        assert call_args[4] == 0.0, "disarm() must send param1=0"

    def test_arm_raises_on_rejected_ack(self, mock_mav_conn):
        from pymavlink import mavutil as mu

        conn, mav_mock = mock_mav_conn
        ack = _make_ack(
            mu.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            mu.mavlink.MAV_RESULT_DENIED,
        )
        mav_mock.recv_match.return_value = ack

        with pytest.raises(CommandRejectedError):
            arm(conn, timeout=2.0)

    def test_arm_raises_on_timeout(self, mock_mav_conn):
        conn, mav_mock = mock_mav_conn
        mav_mock.recv_match.return_value = None   # no ACK ever arrives

        with pytest.raises(CommandTimeoutError):
            arm(conn, timeout=0.3)


# ===========================================================================
# commands.set_flight_mode
# ===========================================================================

class TestSetFlightMode:
    def test_set_guided_mode(self, mock_mav_conn):
        from pymavlink import mavutil as mu

        conn, mav_mock = mock_mav_conn
        ack = _make_ack(
            mu.mavlink.MAV_CMD_DO_SET_MODE,
            mu.mavlink.MAV_RESULT_ACCEPTED,
        )
        mav_mock.recv_match.return_value = ack

        set_flight_mode(conn, "GUIDED", timeout=2.0)

        call_args = mav_mock.mav.command_long_send.call_args[0]
        # param2 = custom_mode; GUIDED = 4
        assert call_args[5] == 4, "GUIDED should map to custom_mode=4"

    def test_invalid_mode_raises_value_error(self, mock_mav_conn):
        conn, _ = mock_mav_conn
        with pytest.raises(ValueError, match="Unknown flight mode"):
            set_flight_mode(conn, "HYPERSPACE", timeout=2.0)

    def test_mode_is_case_insensitive(self, mock_mav_conn):
        from pymavlink import mavutil as mu

        conn, mav_mock = mock_mav_conn
        ack = _make_ack(
            mu.mavlink.MAV_CMD_DO_SET_MODE,
            mu.mavlink.MAV_RESULT_ACCEPTED,
        )
        mav_mock.recv_match.return_value = ack
        # Should not raise
        set_flight_mode(conn, "loiter", timeout=2.0)


# ===========================================================================
# commands.takeoff
# ===========================================================================

class TestTakeoff:
    def test_takeoff_sends_altitude(self, mock_mav_conn):
        from pymavlink import mavutil as mu

        conn, mav_mock = mock_mav_conn
        ack = _make_ack(
            mu.mavlink.MAV_CMD_NAV_TAKEOFF,
            mu.mavlink.MAV_RESULT_ACCEPTED,
        )
        mav_mock.recv_match.return_value = ack

        takeoff(conn, altitude_m=5.0, timeout=3.0)

        call_args = mav_mock.mav.command_long_send.call_args[0]
        # param7 = altitude_m (index 10: sys, comp, cmd, conf, p1-p6, p7)
        assert call_args[10] == 5.0


# ===========================================================================
# commands.send_position_target_local_ned
# ===========================================================================

class TestPositionTarget:
    def test_sends_correct_offsets(self, mock_mav_conn):
        conn, mav_mock = mock_mav_conn
        send_position_target_local_ned(conn, x_m=1.0, y_m=2.0, z_m=-1.5)
        assert mav_mock.mav.set_position_target_local_ned_send.called

        call_args = mav_mock.mav.set_position_target_local_ned_send.call_args[0]
        # Signature: (time_boot_ms, target_sys, target_comp, frame, type_mask, x, y, z, ...)
        # Positional indices after stripping 'self': x=5, y=6, z=7
        assert call_args[5] == pytest.approx(1.0)
        assert call_args[6] == pytest.approx(2.0)
        assert call_args[7] == pytest.approx(-1.5)

    def test_raises_without_connection(self):
        conn = MAVLinkConnection.__new__(MAVLinkConnection)
        conn._mav = None
        with pytest.raises(ConnectionError):
            send_position_target_local_ned(conn, 0, 0, 0)


# ===========================================================================
# commands.send_condition_yaw
# ===========================================================================

class TestConditionYaw:
    def test_cw_yaw(self, mock_mav_conn):
        conn, mav_mock = mock_mav_conn
        send_condition_yaw(conn, target_yaw_deg=90.0, relative=True)
        assert mav_mock.mav.command_long_send.called
        call_args = mav_mock.mav.command_long_send.call_args[0]
        # param3 (direction) should be +1 for positive angle
        assert call_args[6] == 1

    def test_ccw_yaw(self, mock_mav_conn):
        conn, mav_mock = mock_mav_conn
        send_condition_yaw(conn, target_yaw_deg=-45.0, relative=True)
        call_args = mav_mock.mav.command_long_send.call_args[0]
        assert call_args[6] == -1


# ===========================================================================
# is_connected() / close()
# ===========================================================================

class TestConnectionHealth:
    def test_is_connected_false_when_no_recent_heartbeat(self, mock_mav_conn):
        conn, _ = mock_mav_conn
        conn._snapshot.connected = True
        # Simulate stale heartbeat (100 s ago)
        conn._snapshot.last_heartbeat_ts = time.monotonic() - 100
        assert conn.is_connected() is False

    def test_is_connected_true_with_fresh_heartbeat(self, mock_mav_conn):
        conn, _ = mock_mav_conn
        conn._snapshot.connected = True
        conn._snapshot.last_heartbeat_ts = time.monotonic()
        assert conn.is_connected() is True

    def test_get_telemetry_returns_copy(self, mock_mav_conn):
        """Verify mutations to returned snapshot don't affect internal state."""
        conn, _ = mock_mav_conn
        t1 = conn.get_telemetry()
        t1.battery_remaining_pct = 99
        t2 = conn.get_telemetry()
        assert t2.battery_remaining_pct != 99
