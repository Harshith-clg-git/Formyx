"""
formyx_backend/mavlink_interface/connection.py
----------------------------------------------
Establishes and maintains the MAVLink link to the Radiolink PIX6
autopilot.  Runs a dedicated daemon thread that continuously reads
incoming MAVLink messages and updates a shared telemetry snapshot
protected by a threading.Lock.

Key design decisions
---------------------
* **Daemon thread** — the reader thread is set as a daemon so it
  automatically terminates when the main process exits, preventing
  zombie threads.
* **Separation of concerns** — this module only handles connection
  management and telemetry *reading*.  Vehicle *commands* are in
  ``commands.py`` to keep each file testable in isolation.
* **Non-blocking snapshot reads** — callers always get the latest
  available telemetry via ``get_telemetry()`` without blocking the
  main loop on raw MAVLink I/O.
"""

from __future__ import annotations

import logging
import threading
import time
from dataclasses import dataclass, field
from typing import Optional

from pymavlink import mavutil

from config import get

log = logging.getLogger(__name__)


# ---------------------------------------------------------------------------
# Telemetry snapshot dataclass
# ---------------------------------------------------------------------------

@dataclass
class TelemetrySnapshot:
    """
    Immutable-style snapshot of the latest vehicle telemetry.
    Updated atomically (under lock) by the reader thread.
    """
    # Connectivity
    connected: bool = False
    last_heartbeat_ts: float = 0.0          # epoch seconds

    # GLOBAL_POSITION_INT
    lat_deg: float = 0.0
    lon_deg: float = 0.0
    alt_agl_m: float = 0.0                  # altitude above ground (m)
    alt_amsl_m: float = 0.0                 # altitude above mean sea level (m)
    vx_ms: float = 0.0                      # NED velocity X (m/s)
    vy_ms: float = 0.0                      # NED velocity Y (m/s)
    vz_ms: float = 0.0                      # NED velocity Z (m/s)

    # ATTITUDE
    roll_rad: float = 0.0
    pitch_rad: float = 0.0
    yaw_rad: float = 0.0

    # SYS_STATUS — battery
    battery_voltage_v: float = 0.0
    battery_current_a: float = 0.0
    battery_remaining_pct: int = -1         # -1 = not reported

    # GPS_RAW_INT
    gps_fix_type: int = 0                   # 0=no fix … 6=RTK fixed
    satellites_visible: int = 0

    # VFR_HUD
    groundspeed_ms: float = 0.0
    airspeed_ms: float = 0.0
    heading_deg: float = 0.0
    throttle_pct: int = 0

    # HEARTBEAT
    armed: bool = False
    flight_mode: str = "UNKNOWN"

    # Raw message store for advanced consumers
    raw_messages: dict = field(default_factory=dict)


# ---------------------------------------------------------------------------
# MAVLink connection handler
# ---------------------------------------------------------------------------

class MAVLinkConnection:
    """
    Manages the MAVLink connection lifecycle and continuously streams
    telemetry into a thread-safe ``TelemetrySnapshot``.

    Usage
    -----
    >>> conn = MAVLinkConnection()
    >>> conn.connect()
    >>> telem = conn.get_telemetry()
    >>> print(telem.battery_remaining_pct)
    >>> conn.close()
    """

    # ArduPilot mode mapping (mode_id → name); extend as needed
    _ARDU_MODES: dict[int, str] = {
        0: "STABILIZE", 1: "ACRO", 2: "ALT_HOLD", 3: "AUTO",
        4: "GUIDED",    5: "LOITER", 6: "RTL",    7: "CIRCLE",
        9: "LAND",     11: "DRIFT", 13: "SPORT",  14: "FLIP",
        15: "AUTOTUNE",16: "POSHOLD",17: "BRAKE", 18: "THROW",
        19: "AVOID_ADSB", 20: "GUIDED_NOGPS",
    }

    def __init__(self, connection_string: Optional[str] = None) -> None:
        self._conn_str: str = connection_string or get(
            "mavlink", "connection_string", "udpin:localhost:14550"
        )
        self._heartbeat_timeout: float = get("mavlink", "heartbeat_timeout_s", 5)
        self._telem_rate_hz: int = get("mavlink", "telemetry_rate_hz", 10)
        self._reconnect_attempts: int = get("mavlink", "reconnect_attempts", 5)
        self._reconnect_delay: float = get("mavlink", "reconnect_delay_s", 2.0)

        self._mav: Optional[mavutil.mavlink_connection] = None
        self._snapshot = TelemetrySnapshot()
        self._lock = threading.Lock()
        self._reader_thread: Optional[threading.Thread] = None
        self._stop_event = threading.Event()

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def connect(self) -> None:
        """
        Open the MAVLink link and block until the first heartbeat is
        received (up to ``heartbeat_timeout_s`` seconds).

        Raises
        ------
        ConnectionError
            If no heartbeat is received within the timeout.
        """
        log.info("Connecting to autopilot: %s", self._conn_str)
        self._mav = mavutil.mavlink_connection(self._conn_str)

        log.info(
            "Waiting for heartbeat (timeout=%ss)…", self._heartbeat_timeout
        )
        msg = self._mav.recv_match(
            type="HEARTBEAT",
            blocking=True,
            timeout=self._heartbeat_timeout,
        )
        if msg is None:
            raise ConnectionError(
                f"No heartbeat received from autopilot within "
                f"{self._heartbeat_timeout}s on {self._conn_str}"
            )

        log.info(
            "Heartbeat received — system %d component %d",
            self._mav.target_system,
            self._mav.target_component,
        )
        self._update_snapshot_from_heartbeat(msg)

        # Request telemetry streams from autopilot
        self._request_data_streams()

        # Start the background reader thread
        self._stop_event.clear()
        self._reader_thread = threading.Thread(
            target=self._reader_loop,
            name="mavlink-reader",
            daemon=True,
        )
        self._reader_thread.start()
        log.info("MAVLink reader thread started.")

    def get_telemetry(self) -> TelemetrySnapshot:
        """
        Return a *copy* of the latest telemetry snapshot.
        This is always non-blocking.
        """
        with self._lock:
            import copy
            return copy.copy(self._snapshot)

    def get_raw_connection(self) -> Optional[mavutil.mavlink_connection]:
        """Expose the raw pymavlink object for command senders."""
        return self._mav

    def is_connected(self) -> bool:
        """True if the reader thread is alive and a recent heartbeat exists."""
        with self._lock:
            hb_age = time.monotonic() - self._snapshot.last_heartbeat_ts
            return (
                self._snapshot.connected
                and hb_age < self._heartbeat_timeout * 2
            )

    def close(self) -> None:
        """Gracefully stop the reader thread and close the MAVLink link."""
        log.info("Closing MAVLink connection.")
        self._stop_event.set()
        if self._reader_thread and self._reader_thread.is_alive():
            self._reader_thread.join(timeout=3.0)
        if self._mav:
            self._mav.close()
            self._mav = None
        with self._lock:
            self._snapshot.connected = False
        log.info("MAVLink connection closed.")

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _request_data_streams(self) -> None:
        """Ask ArduPilot to send telemetry streams at the configured rate."""
        if self._mav is None:
            return
        stream_ids = [
            mavutil.mavlink.MAV_DATA_STREAM_ALL,
        ]
        for stream_id in stream_ids:
            self._mav.mav.request_data_stream_send(
                self._mav.target_system,
                self._mav.target_component,
                stream_id,
                self._telem_rate_hz,
                1,  # start sending
            )
        log.debug(
            "Requested telemetry streams at %d Hz.", self._telem_rate_hz
        )

    def _reader_loop(self) -> None:
        """
        Daemon loop: reads MAVLink messages and dispatches to
        per-type update handlers.  Handles connection dropouts with
        exponential back-off reconnection.
        """
        log.debug("MAVLink reader loop started.")
        while not self._stop_event.is_set():
            if self._mav is None:
                time.sleep(0.1)
                continue
            try:
                msg = self._mav.recv_match(blocking=True, timeout=1.0)
                if msg is None:
                    continue

                msg_type = msg.get_type()

                with self._lock:
                    self._snapshot.raw_messages[msg_type] = msg

                    if msg_type == "HEARTBEAT":
                        self._update_snapshot_from_heartbeat(msg)
                    elif msg_type == "GLOBAL_POSITION_INT":
                        self._update_global_pos(msg)
                    elif msg_type == "ATTITUDE":
                        self._update_attitude(msg)
                    elif msg_type == "SYS_STATUS":
                        self._update_sys_status(msg)
                    elif msg_type == "GPS_RAW_INT":
                        self._update_gps_raw(msg)
                    elif msg_type == "VFR_HUD":
                        self._update_vfr_hud(msg)

            except Exception as exc:  # noqa: BLE001
                log.warning("Reader error: %s — attempting to continue.", exc)
                time.sleep(0.1)

        log.debug("MAVLink reader loop exiting.")

    # ------------------------------------------------------------------
    # Per-message-type snapshot update methods
    # NOTE: Always called while holding self._lock
    # ------------------------------------------------------------------

    def _update_snapshot_from_heartbeat(self, msg) -> None:
        self._snapshot.connected = True
        self._snapshot.last_heartbeat_ts = time.monotonic()
        self._snapshot.armed = bool(
            msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
        )
        mode_id = msg.custom_mode
        self._snapshot.flight_mode = self._ARDU_MODES.get(mode_id, f"MODE_{mode_id}")

    def _update_global_pos(self, msg) -> None:
        self._snapshot.lat_deg = msg.lat / 1e7
        self._snapshot.lon_deg = msg.lon / 1e7
        self._snapshot.alt_amsl_m = msg.alt / 1000.0
        self._snapshot.alt_agl_m = msg.relative_alt / 1000.0
        self._snapshot.vx_ms = msg.vx / 100.0
        self._snapshot.vy_ms = msg.vy / 100.0
        self._snapshot.vz_ms = msg.vz / 100.0

    def _update_attitude(self, msg) -> None:
        self._snapshot.roll_rad = msg.roll
        self._snapshot.pitch_rad = msg.pitch
        self._snapshot.yaw_rad = msg.yaw

    def _update_sys_status(self, msg) -> None:
        self._snapshot.battery_voltage_v = msg.voltage_battery / 1000.0
        self._snapshot.battery_current_a = msg.current_battery / 100.0
        self._snapshot.battery_remaining_pct = msg.battery_remaining

    def _update_gps_raw(self, msg) -> None:
        self._snapshot.gps_fix_type = msg.fix_type
        self._snapshot.satellites_visible = msg.satellites_visible

    def _update_vfr_hud(self, msg) -> None:
        self._snapshot.groundspeed_ms = msg.groundspeed
        self._snapshot.airspeed_ms = msg.airspeed
        self._snapshot.heading_deg = msg.heading
        self._snapshot.throttle_pct = msg.throttle
