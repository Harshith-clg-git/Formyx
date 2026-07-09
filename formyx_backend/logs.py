"""
formyx_backend/logs.py
-----------------------
Passive MAVLink flight-log recorder for the Radiolink PIX6.

Connects to the PIX6 via serial (or UDP in SITL), listens to ALL incoming
MAVLink messages, and writes two files per session inside the logs/ folder:

  logs/
  ├── session_YYYYMMDD_HHMMSS.csv   ← human-readable, one row per second
  └── session_YYYYMMDD_HHMMSS.tlog  ← raw binary MAVLink stream (open in
                                        Mission Planner / QGroundControl)

NO commands are ever sent to the flight controller — this script is purely
a passive observer.  Safe to run during any manual flight.

Usage
-----
    cd formyx_backend
    python3 logs.py

Options
-------
    --connection STR    MAVLink connection string (overrides settings.yaml)
                        Default: /dev/ttyACM0,57600
    --log-dir PATH      Output folder (default: logs)
    --rate-hz INT       Telemetry polling rate in Hz (default: 10)
    --csv-interval SEC  How often a new CSV row is written (default: 0.5)
    --segment-mins INT  Rotate log files every N minutes and start fresh ones.
                        (default: 3, 0 = disabled — run one log until stopped)
    --raw               Also save a raw .tlog binary stream
    --no-raw            Skip .tlog (CSV only)
    --quiet             Suppress console heartbeat prints

Keys
----
    Ctrl-C              Stop recording and flush files cleanly.
"""

from __future__ import annotations

import argparse
import csv
import glob
import logging
import math
import os
import signal
import subprocess
import sys
import time
from datetime import datetime
from pathlib import Path

from pymavlink import mavutil

from config import get


# ---------------------------------------------------------------------------
# Port auto-detection
# ---------------------------------------------------------------------------

def _detect_pix6_port() -> str:
    """
    Try to find the RadiolinkPIX6 serial port automatically by scanning
    /dev/ttyACM* and checking the USB vendor:product ID via udev/sysfs.

    Radiolink PIX6 USB ID: 1209:5740

    Returns the first matching port, or falls back to /dev/ttyACM0.
    """
    candidates = sorted(glob.glob("/dev/ttyACM*"))
    if not candidates:
        return "/dev/ttyACM0"  # default; let pymavlink raise on open failure

    for port in candidates:
        # Derive the sysfs path for this ttyACM device
        dev_name = Path(port).name          # e.g. ttyACM0
        sysfs_paths = glob.glob(
            f"/sys/bus/usb/drivers/cdc_acm/*/{dev_name}"
        )
        if not sysfs_paths:
            # Alternate sysfs layout
            sysfs_paths = glob.glob(
                f"/sys/class/tty/{dev_name}/device/**", recursive=False
            )

        for sp in sysfs_paths:
            try:
                # Walk up to the USB device directory that has idVendor / idProduct
                usb_dev = Path(sp)
                for _ in range(6):
                    usb_dev = usb_dev.parent
                    vid_file = usb_dev / "idVendor"
                    pid_file = usb_dev / "idProduct"
                    if vid_file.exists() and pid_file.exists():
                        vid = vid_file.read_text().strip()
                        pid = pid_file.read_text().strip()
                        if vid == "1209" and pid == "5740":
                            return port  # Found the PIX6!
                        break  # Wrong device — stop climbing
            except Exception:
                continue

    # Could not match by ID — return the first available ACM port
    return candidates[0]

# ---------------------------------------------------------------------------
# Logging
# ---------------------------------------------------------------------------
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)-8s] %(name)s — %(message)s",
    datefmt="%H:%M:%S",
)
log = logging.getLogger("logs")

# ---------------------------------------------------------------------------
# ArduPilot flight mode table (custom_mode → name)
# ---------------------------------------------------------------------------
_ARDU_MODES: dict[int, str] = {
    0: "STABILIZE", 1: "ACRO",    2: "ALT_HOLD", 3: "AUTO",
    4: "GUIDED",    5: "LOITER",  6: "RTL",       7: "CIRCLE",
    9: "LAND",     11: "DRIFT",  13: "SPORT",    14: "FLIP",
    15: "AUTOTUNE",16: "POSHOLD",17: "BRAKE",    18: "THROW",
    19: "AVOID_ADSB", 20: "GUIDED_NOGPS",
}

# ---------------------------------------------------------------------------
# CSV columns — one human-readable row written every --csv-interval seconds
# ---------------------------------------------------------------------------
CSV_COLUMNS = [
    "timestamp_utc",
    "elapsed_s",
    # HEARTBEAT
    "armed",
    "flight_mode",
    # GLOBAL_POSITION_INT
    "lat_deg",
    "lon_deg",
    "alt_agl_m",
    "alt_amsl_m",
    "vx_ms",
    "vy_ms",
    "vz_ms",
    # ATTITUDE (degrees)
    "roll_deg",
    "pitch_deg",
    "yaw_deg",
    # VFR_HUD
    "groundspeed_ms",
    "airspeed_ms",
    "heading_deg",
    "throttle_pct",
    # SYS_STATUS / battery
    "battery_v",
    "battery_a",
    "battery_pct",
    # GPS_RAW_INT
    "gps_fix_type",
    "satellites_visible",
    # RC_CHANNELS (channels 1-8)
    "rc1", "rc2", "rc3", "rc4",
    "rc5", "rc6", "rc7", "rc8",
    # STATUSTEXT messages
    "last_status_text",
]

# ---------------------------------------------------------------------------
# Telemetry state — updated as messages arrive
# ---------------------------------------------------------------------------
class _State:
    def __init__(self) -> None:
        self.armed           = False
        self.flight_mode     = "UNKNOWN"
        self.lat_deg         = 0.0
        self.lon_deg         = 0.0
        self.alt_agl_m       = 0.0
        self.alt_amsl_m      = 0.0
        self.vx_ms           = 0.0
        self.vy_ms           = 0.0
        self.vz_ms           = 0.0
        self.roll_rad        = 0.0
        self.pitch_rad       = 0.0
        self.yaw_rad         = 0.0
        self.groundspeed_ms  = 0.0
        self.airspeed_ms     = 0.0
        self.heading_deg     = 0.0
        self.throttle_pct    = 0
        self.battery_v       = 0.0
        self.battery_a       = 0.0
        self.battery_pct     = -1
        self.gps_fix_type    = 0
        self.satellites      = 0
        self.rc              = [0] * 8
        self.last_status_txt = ""

    def as_csv_row(self, elapsed_s: float) -> list:
        return [
            datetime.utcnow().strftime("%Y-%m-%dT%H:%M:%S.%f")[:-3] + "Z",
            f"{elapsed_s:.3f}",
            self.armed,
            self.flight_mode,
            f"{self.lat_deg:.7f}",
            f"{self.lon_deg:.7f}",
            f"{self.alt_agl_m:.2f}",
            f"{self.alt_amsl_m:.2f}",
            f"{self.vx_ms:.2f}",
            f"{self.vy_ms:.2f}",
            f"{self.vz_ms:.2f}",
            f"{math.degrees(self.roll_rad):.2f}",
            f"{math.degrees(self.pitch_rad):.2f}",
            f"{math.degrees(self.yaw_rad):.2f}",
            f"{self.groundspeed_ms:.2f}",
            f"{self.airspeed_ms:.2f}",
            f"{self.heading_deg:.1f}",
            self.throttle_pct,
            f"{self.battery_v:.3f}",
            f"{self.battery_a:.2f}",
            self.battery_pct,
            self.gps_fix_type,
            self.satellites,
            *self.rc,
            self.last_status_txt,
        ]


# ---------------------------------------------------------------------------
# Dispatch incoming messages into state
# ---------------------------------------------------------------------------
def _dispatch(msg, state: _State) -> None:
    t = msg.get_type()

    if t == "HEARTBEAT":
        state.armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
        state.flight_mode = _ARDU_MODES.get(msg.custom_mode, f"MODE_{msg.custom_mode}")

    elif t == "GLOBAL_POSITION_INT":
        state.lat_deg    = msg.lat / 1e7
        state.lon_deg    = msg.lon / 1e7
        state.alt_amsl_m = msg.alt / 1000.0
        state.alt_agl_m  = msg.relative_alt / 1000.0
        state.vx_ms      = msg.vx / 100.0
        state.vy_ms      = msg.vy / 100.0
        state.vz_ms      = msg.vz / 100.0

    elif t == "ATTITUDE":
        state.roll_rad  = msg.roll
        state.pitch_rad = msg.pitch
        state.yaw_rad   = msg.yaw

    elif t == "VFR_HUD":
        state.groundspeed_ms = msg.groundspeed
        state.airspeed_ms    = msg.airspeed
        state.heading_deg    = msg.heading
        state.throttle_pct   = msg.throttle

    elif t == "SYS_STATUS":
        state.battery_v   = msg.voltage_battery / 1000.0
        state.battery_a   = msg.current_battery / 100.0
        state.battery_pct = msg.battery_remaining

    elif t == "GPS_RAW_INT":
        state.gps_fix_type = msg.fix_type
        state.satellites   = msg.satellites_visible

    elif t == "RC_CHANNELS":
        state.rc = [
            msg.chan1_raw, msg.chan2_raw, msg.chan3_raw, msg.chan4_raw,
            msg.chan5_raw, msg.chan6_raw, msg.chan7_raw, msg.chan8_raw,
        ]

    elif t == "STATUSTEXT":
        severity = getattr(msg, "severity", 6)
        text     = msg.text.strip()
        state.last_status_txt = text
        level = {0: "EMERGENCY", 1: "ALERT", 2: "CRITICAL",
                 3: "ERROR",     4: "WARNING", 5: "NOTICE",
                 6: "INFO",      7: "DEBUG"}.get(severity, "INFO")
        log.info("[PIX6 STATUSTEXT / %s] %s", level, text)


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------
def _parse() -> argparse.Namespace:
    p = argparse.ArgumentParser(
        prog="logs",
        description="Passive MAVLink flight-log recorder (Radiolink PIX6)",
    )
    p.add_argument(
        "--connection", default=None,
        help="MAVLink connection string — e.g. /dev/ttyACM0,57600 or "
             "udpin:localhost:14550 (default: read from settings.yaml)",
    )
    p.add_argument("--log-dir",      default="logs",  help="Output folder (default: logs)")
    p.add_argument("--rate-hz",      type=int,   default=10,  help="Telemetry rate Hz")
    p.add_argument("--csv-interval", type=float, default=0.5, help="CSV row interval (s)")
    p.add_argument("--segment-mins", type=int,   default=3,
                   help="Rotate log files every N minutes (default 3, 0 = disabled)")
    p.add_argument("--no-raw",       action="store_true", help="Skip raw .tlog binary")
    p.add_argument("--quiet",        action="store_true", help="Suppress heartbeat console prints")
    return p.parse_args()


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
_stop = False

def _handle_signal(signum, frame):  # noqa: ANN001
    global _stop
    _stop = True

def main() -> int:
    global _stop

    signal.signal(signal.SIGINT,  _handle_signal)
    signal.signal(signal.SIGTERM, _handle_signal)

    args = _parse()

    # Determine connection string: CLI arg > settings.yaml > auto-detect
    if args.connection:
        conn_s = args.connection
        port_source = "CLI --connection flag"
    else:
        cfg_conn = get("mavlink", "connection_string", None)
        if cfg_conn:
            conn_s = cfg_conn
            port_source = "settings.yaml"
        else:
            detected_port = _detect_pix6_port()
            conn_s = f"{detected_port},57600"
            port_source = f"auto-detected ({detected_port})"

    Path(args.log_dir).mkdir(parents=True, exist_ok=True)

    log.info("=" * 60)
    log.info("Formyx Flight Log Recorder")
    log.info("Connection : %s  [source: %s]", conn_s, port_source)
    log.info("Log dir    : %s", os.path.abspath(args.log_dir))
    log.info("CSV rate   : every %.2f s", args.csv_interval)
    log.info("Raw tlog   : %s", "disabled" if args.no_raw else "enabled")
    log.info("Press Ctrl-C to stop.")
    log.info("=" * 60)

    while not _stop:
        stamp    = datetime.now().strftime("%Y%m%d_%H%M%S")
        csv_path = os.path.join(args.log_dir, f"session_{stamp}.csv")
        tlog_path = os.path.join(args.log_dir, f"session_{stamp}.tlog")

        log.info("Connecting to Radiolink PIX6...")
        mav = None
        try:
            mav = mavutil.mavlink_connection(conn_s)
        except Exception as exc:
            log.warning("Failed to open connection %s: %s. Retrying in 3s...", conn_s, exc)
            time.sleep(3.0)
            continue

        log.info("Waiting for heartbeat (up to 15 s)...")
        hb = mav.recv_match(type="HEARTBEAT", blocking=True, timeout=15)
        if hb is None:
            log.warning("No heartbeat received on %s. Retrying...", conn_s)
            try:
                mav.close()
            except Exception:
                pass
            time.sleep(2.0)
            continue

        log.info(
            "PIX6 detected — system_id=%d component_id=%d",
            mav.target_system, mav.target_component,
        )

        # Request all data streams
        mav.mav.request_data_stream_send(
            mav.target_system, mav.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_ALL,
            args.rate_hz, 1,
        )
        log.info("Data streams requested at %d Hz.", args.rate_hz)

        # Open files
        csv_file = None
        tlog_file = None
        try:
            csv_file   = open(csv_path, "w", newline="", encoding="utf-8")
            csv_writer = csv.writer(csv_file)
            csv_writer.writerow(CSV_COLUMNS)

            if not args.no_raw:
                tlog_file = open(tlog_path, "wb")

            state        = _State()
            start_t      = time.monotonic()
            last_csv_t   = start_t
            last_hb_t    = start_t
            total_msg    = 0
            total_rows   = 0
            segment_secs = args.segment_mins * 60 if args.segment_mins > 0 else None
            seg_start_t  = start_t

            log.info("Recording to %s — press Ctrl-C to stop.", csv_path)

            while not _stop:
                try:
                    msg = mav.recv_match(blocking=True, timeout=1.0)
                except Exception as exc:
                    log.error("Read error: %s. Reconnecting...", exc)
                    break

                if msg is None:
                    now = time.monotonic()
                    if now - last_hb_t > 5.0:
                        log.warning("No messages for >5 s — link may be lost.")
                    continue

                total_msg += 1

                if tlog_file:
                    tlog_file.write(msg.get_msgbuf())

                _dispatch(msg, state)

                if msg.get_type() == "HEARTBEAT":
                    last_hb_t = time.monotonic()
                    if not args.quiet:
                        elapsed = time.monotonic() - start_t
                        log.info(
                            "[+%.1fs] armed=%-5s mode=%-12s alt=%.1fm "
                            "bat=%d%%  gps_fix=%d sats=%d",
                            elapsed,
                            state.armed,
                            state.flight_mode,
                            state.alt_agl_m,
                            state.battery_pct,
                            state.gps_fix_type,
                            state.satellites,
                        )

                now = time.monotonic()
                if now - last_csv_t >= args.csv_interval:
                    row = state.as_csv_row(now - start_t)
                    csv_writer.writerow(row)
                    csv_file.flush()
                    last_csv_t = now
                    total_rows += 1

                # --------------------------------------------------
                # Segment rotation — break inner loop to trigger
                # finally block, then outer loop reopens fresh files
                # --------------------------------------------------
                if segment_secs and (time.monotonic() - seg_start_t >= segment_secs):
                    log.info("Segment limit reached — rotating log files…")
                    break

        except Exception as exc:
            log.error("Unexpected error during session: %s", exc)
        finally:
            log.info("-" * 50)
            log.info("Session recording stopped.")
            if csv_file:
                csv_file.close()
                log.info("  CSV saved: %s", os.path.abspath(csv_path))
            if tlog_file:
                tlog_file.close()
                log.info("  tlog saved: %s", os.path.abspath(tlog_path))
            try:
                mav.close()
            except Exception:
                pass
            log.info("-" * 50)

        if not _stop:
            time.sleep(2.0)

    log.info("Logs recorder exiting cleanly.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
