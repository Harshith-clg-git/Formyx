"""
formyx_backend/main.py
-----------------------
Top-level entry point for the Formyx Autonomous Drone backend.

Orchestrates all subsystems in the correct startup order and provides
clean shutdown handling (Ctrl-C / SIGTERM).

Current status: **Phase 1 stub** — connects MAVLink, streams telemetry,
and logs to console. Future milestones will integrate the state machine,
perception pipeline, and navigation controller here.

Usage
-----
    cd formyx_backend
    python main.py [--connection <conn_str>] [--log-level DEBUG|INFO|WARNING]

SITL example:
    python main.py --connection udpin:localhost:14550

Hardware (Raspberry Pi 5):
    python main.py --connection serial:/dev/ttyAMA0:921600
"""

from __future__ import annotations

import argparse
import logging
import signal
import sys
import time

from config import load_config
from mavlink_interface.connection import MAVLinkConnection
from mavlink_interface.commands import (
    set_flight_mode,
    CommandRejectedError,
    CommandTimeoutError,
)


# ---------------------------------------------------------------------------
# Logging setup
# ---------------------------------------------------------------------------

def _setup_logging(level_name: str) -> None:
    level = getattr(logging, level_name.upper(), logging.INFO)
    logging.basicConfig(
        level=level,
        format="%(asctime)s [%(levelname)-8s] %(name)s — %(message)s",
        datefmt="%Y-%m-%d %H:%M:%S",
    )


# ---------------------------------------------------------------------------
# Graceful shutdown
# ---------------------------------------------------------------------------

_shutdown_requested = False


def _handle_signal(signum, frame):  # noqa: ANN001
    global _shutdown_requested
    logging.getLogger(__name__).warning(
        "Signal %d received — initiating clean shutdown.", signum
    )
    _shutdown_requested = True


# ---------------------------------------------------------------------------
# CLI argument parser
# ---------------------------------------------------------------------------

def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        prog="formyx_backend",
        description="Formyx Autonomous Balloon-Tracking Drone Backend",
    )
    parser.add_argument(
        "--connection",
        default=None,
        help="MAVLink connection string (overrides settings.yaml). "
             "E.g. udpin:localhost:14550 or serial:/dev/ttyAMA0:921600",
    )
    parser.add_argument(
        "--log-level",
        default="INFO",
        choices=["DEBUG", "INFO", "WARNING", "ERROR"],
        help="Console log verbosity (default: INFO)",
    )
    return parser.parse_args()


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main() -> int:
    args = _parse_args()
    _setup_logging(args.log_level)
    log = logging.getLogger(__name__)

    signal.signal(signal.SIGINT, _handle_signal)
    signal.signal(signal.SIGTERM, _handle_signal)

    cfg = load_config()
    log.info("Formyx Backend starting — loading configuration.")

    # ------------------------------------------------------------------
    # 1. MAVLink connection
    # ------------------------------------------------------------------
    conn_string = args.connection or cfg["mavlink"]["connection_string"]
    log.info("Connecting to autopilot at: %s", conn_string)

    conn = MAVLinkConnection(connection_string=conn_string)

    try:
        conn.connect()
    except ConnectionError as exc:
        log.critical("Failed to connect to autopilot: %s", exc)
        return 1

    log.info("Connected — beginning telemetry loop. Press Ctrl-C to exit.")

    # ------------------------------------------------------------------
    # 2. Telemetry monitoring loop (Phase 1 stub)
    #    Future phases: inject state machine, perception, navigation here.
    # ------------------------------------------------------------------
    try:
        while not _shutdown_requested:
            telem = conn.get_telemetry()

            log.info(
                "| Armed=%-5s | Mode=%-15s | Alt=%.1fm | Bat=%d%% | "
                "GPS_Fix=%d Sats=%d | GndSpd=%.1fm/s |",
                telem.armed,
                telem.flight_mode,
                telem.alt_agl_m,
                telem.battery_remaining_pct,
                telem.gps_fix_type,
                telem.satellites_visible,
                telem.groundspeed_ms,
            )

            if not conn.is_connected():
                log.error("MAVLink heartbeat lost! Waiting to reconnect…")

            time.sleep(1.0 / cfg["mavlink"]["telemetry_rate_hz"])

    finally:
        log.info("Shutting down MAVLink connection.")
        conn.close()
        log.info("Formyx Backend stopped cleanly.")

    return 0


if __name__ == "__main__":
    sys.exit(main())
