"""
formyx_backend/mavlink_interface/commands.py
--------------------------------------------
Sends MAVLink commands to the autopilot.  All functions accept a
``MAVLinkConnection`` instance and block until ACK is received (or
``timeout`` expires), allowing callers to handle failures explicitly.

Design decisions
-----------------
* Every command waits for a COMMAND_ACK to detect rejection.
* Functions raise ``CommandRejectedError`` on non-zero ACK result so
  the mission state machine can handle them cleanly.
* ``set_flight_mode()`` translates human-readable mode names to the
  ArduPilot mode integer via the heartbeat's base_mode + custom_mode.
"""

from __future__ import annotations

import logging
import time
from typing import Optional

from pymavlink import mavutil

from .connection import MAVLinkConnection

log = logging.getLogger(__name__)


class CommandRejectedError(Exception):
    """Raised when the autopilot returns a non-zero COMMAND_ACK result."""
    pass


class CommandTimeoutError(Exception):
    """Raised when an ACK is not received within the timeout window."""
    pass


# ---------------------------------------------------------------------------
# Internal helpers
# ---------------------------------------------------------------------------

def _wait_for_ack(
    conn: MAVLinkConnection,
    command_id: int,
    timeout: float = 5.0,
) -> None:
    """
    Block until a COMMAND_ACK for *command_id* is received.

    Raises
    ------
    CommandRejectedError
        If the ACK result is not MAV_RESULT_ACCEPTED (0).
    CommandTimeoutError
        If no ACK arrives within *timeout* seconds.
    """
    mav = conn.get_raw_connection()
    if mav is None:
        raise ConnectionError("No active MAVLink connection.")

    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        ack = mav.recv_match(type="COMMAND_ACK", blocking=True, timeout=0.5)
        if ack is None:
            continue
        if ack.command == command_id:
            if ack.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                log.debug("ACK accepted for command %d.", command_id)
                return
            else:
                result_name = mavutil.mavlink.enums["MAV_RESULT"].get(
                    ack.result, mavutil.mavlink.EnumEntry("?", "UNKNOWN")
                ).name
                raise CommandRejectedError(
                    f"Command {command_id} rejected: {result_name} (code={ack.result})"
                )

    raise CommandTimeoutError(
        f"No COMMAND_ACK for command {command_id} within {timeout}s."
    )


# ---------------------------------------------------------------------------
# ArduPilot mode table
# (mode_name -> custom_mode int)
# ---------------------------------------------------------------------------

_ARDU_MODE_MAP: dict[str, int] = {
    "STABILIZE": 0, "ACRO": 1, "ALT_HOLD": 2, "AUTO": 3,
    "GUIDED":    4, "LOITER": 5, "RTL": 6,    "CIRCLE": 7,
    "LAND":      9, "DRIFT": 11, "SPORT": 13, "FLIP": 14,
    "AUTOTUNE": 15, "POSHOLD": 16, "BRAKE": 17, "THROW": 18,
    "AVOID_ADSB": 19, "GUIDED_NOGPS": 20,
}


# ---------------------------------------------------------------------------
# Public command functions
# ---------------------------------------------------------------------------

def arm(conn: MAVLinkConnection, timeout: float = 10.0) -> None:
    """
    Arm the vehicle motors.

    Parameters
    ----------
    conn    : Active ``MAVLinkConnection``.
    timeout : Seconds to wait for ACK.

    Raises
    ------
    CommandRejectedError, CommandTimeoutError, ConnectionError
    """
    mav = conn.get_raw_connection()
    if mav is None:
        raise ConnectionError("No active MAVLink connection.")

    log.info("Sending ARM command…")
    mav.mav.command_long_send(
        mav.target_system,
        mav.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,        # confirmation
        1.0,      # param1: 1 = arm
        0, 0, 0, 0, 0, 0,
    )
    _wait_for_ack(conn, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, timeout)
    log.info("Vehicle ARMED.")


def disarm(conn: MAVLinkConnection, timeout: float = 10.0) -> None:
    """
    Disarm the vehicle motors.

    Parameters
    ----------
    conn    : Active ``MAVLinkConnection``.
    timeout : Seconds to wait for ACK.

    Raises
    ------
    CommandRejectedError, CommandTimeoutError, ConnectionError
    """
    mav = conn.get_raw_connection()
    if mav is None:
        raise ConnectionError("No active MAVLink connection.")

    log.info("Sending DISARM command…")
    mav.mav.command_long_send(
        mav.target_system,
        mav.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        0.0,  # param1: 0 = disarm
        0, 0, 0, 0, 0, 0,
    )
    _wait_for_ack(conn, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, timeout)
    log.info("Vehicle DISARMED.")


def set_flight_mode(
    conn: MAVLinkConnection,
    mode_name: str,
    timeout: float = 5.0,
) -> None:
    """
    Switch the autopilot flight mode.

    Parameters
    ----------
    conn      : Active ``MAVLinkConnection``.
    mode_name : Human-readable mode name, e.g. ``"GUIDED"``, ``"LOITER"``.
    timeout   : Seconds to wait for ACK.

    Raises
    ------
    ValueError
        If *mode_name* is not in the known mode table.
    CommandRejectedError, CommandTimeoutError, ConnectionError
    """
    mav = conn.get_raw_connection()
    if mav is None:
        raise ConnectionError("No active MAVLink connection.")

    mode_name_upper = mode_name.upper()
    if mode_name_upper not in _ARDU_MODE_MAP:
        raise ValueError(
            f"Unknown flight mode '{mode_name}'. "
            f"Known modes: {list(_ARDU_MODE_MAP.keys())}"
        )

    custom_mode = _ARDU_MODE_MAP[mode_name_upper]
    log.info("Switching flight mode → %s (custom_mode=%d)", mode_name_upper, custom_mode)

    mav.mav.command_long_send(
        mav.target_system,
        mav.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_MODE,
        0,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        custom_mode,
        0, 0, 0, 0, 0,
    )
    _wait_for_ack(conn, mavutil.mavlink.MAV_CMD_DO_SET_MODE, timeout)
    log.info("Flight mode set to %s.", mode_name_upper)


def takeoff(
    conn: MAVLinkConnection,
    altitude_m: float,
    timeout: float = 15.0,
) -> None:
    """
    Command a GUIDED-mode takeoff to the specified altitude AGL.

    .. note::
        Vehicle must be armed and in GUIDED mode before calling this.

    Parameters
    ----------
    conn       : Active ``MAVLinkConnection``.
    altitude_m : Target altitude in metres above ground level.
    timeout    : Seconds to wait for ACK.
    """
    mav = conn.get_raw_connection()
    if mav is None:
        raise ConnectionError("No active MAVLink connection.")

    log.info("Commanding takeoff to %.1f m AGL…", altitude_m)
    mav.mav.command_long_send(
        mav.target_system,
        mav.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0,
        0, 0, 0, 0, 0, 0,  # params 1-6 unused for copter
        altitude_m,         # param7: altitude AGL
    )
    _wait_for_ack(conn, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, timeout)
    log.info("Takeoff command accepted (target=%.1f m).", altitude_m)


def land(conn: MAVLinkConnection, timeout: float = 5.0) -> None:
    """
    Command the vehicle to land at the current location.
    """
    log.info("Commanding LAND…")
    set_flight_mode(conn, "LAND", timeout=timeout)


def return_to_launch(conn: MAVLinkConnection, timeout: float = 5.0) -> None:
    """
    Command Return-to-Launch (RTL).
    """
    log.info("Commanding RTL…")
    set_flight_mode(conn, "RTL", timeout=timeout)


def send_position_target_local_ned(
    conn: MAVLinkConnection,
    x_m: float,
    y_m: float,
    z_m: float,
    vx_ms: float = 0.0,
    vy_ms: float = 0.0,
    vz_ms: float = 0.0,
    yaw_rad: Optional[float] = None,
    coordinate_frame: int = mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
) -> None:
    """
    Send a SET_POSITION_TARGET_LOCAL_NED message to move the drone in
    body-offset NED coordinates (relative movement from current position).

    Parameters
    ----------
    conn             : Active ``MAVLinkConnection``.
    x_m, y_m, z_m   : Desired position offset in metres (NED, +Z = down).
    vx_ms … vz_ms   : Optional velocity components (m/s).
    yaw_rad          : Optional desired yaw (radians).  None = ignore.
    coordinate_frame : MAVLink coordinate frame.  Default is body-offset NED.
    """
    mav = conn.get_raw_connection()
    if mav is None:
        raise ConnectionError("No active MAVLink connection.")

    # Type mask: ignore acceleration fields; include pos + vel
    type_mask = (
        0b0000_111_111_000_111  # ignore acc x,y,z + force + yaw_rate
    )
    if yaw_rad is None:
        type_mask |= 0b0000_000_000_100_000  # also ignore yaw
        yaw_rad = 0.0

    mav.mav.set_position_target_local_ned_send(
        0,                          # time_boot_ms (ignored)
        mav.target_system,
        mav.target_component,
        coordinate_frame,
        type_mask,
        x_m, y_m, z_m,
        vx_ms, vy_ms, vz_ms,
        0, 0, 0,                    # afx, afy, afz (ignored)
        yaw_rad,
        0,                          # yaw_rate (ignored)
    )
    log.debug(
        "SET_POSITION_TARGET_LOCAL_NED → x=%.2f y=%.2f z=%.2f", x_m, y_m, z_m
    )


def send_condition_yaw(
    conn: MAVLinkConnection,
    target_yaw_deg: float,
    yaw_rate_dps: float = 20.0,
    relative: bool = True,
    timeout: float = 3.0,
) -> None:
    """
    Command a yaw rotation (used for visual scanning).

    Parameters
    ----------
    target_yaw_deg : Desired yaw angle in degrees.
    yaw_rate_dps   : Rotation rate in degrees/second.
    relative       : If True, yaw is relative to current heading.
    """
    mav = conn.get_raw_connection()
    if mav is None:
        raise ConnectionError("No active MAVLink connection.")

    log.info(
        "Yaw → %.1f° (%s) at %.1f°/s",
        target_yaw_deg,
        "relative" if relative else "absolute",
        yaw_rate_dps,
    )
    mav.mav.command_long_send(
        mav.target_system,
        mav.target_component,
        mavutil.mavlink.MAV_CMD_CONDITION_YAW,
        0,
        abs(target_yaw_deg),         # param1: target yaw magnitude
        yaw_rate_dps,                # param2: yaw rate
        1 if target_yaw_deg >= 0 else -1,  # param3: direction (1=CW, -1=CCW)
        1 if relative else 0,        # param4: relative flag
        0, 0, 0,
    )
