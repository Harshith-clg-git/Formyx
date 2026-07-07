"""
formyx_backend/tools/hardware_test.py
--------------------------------------
Interactive hardware validation script for the Raspberry Pi 5 +
Radiolink PIX6 autopilot combination.

What this script tests (in order)
-----------------------------------
[TEST 1]  MAVLink connection & heartbeat reception
[TEST 2]  Telemetry stream quality (60 second soak test)
[TEST 3]  GPS fix & satellite count check
[TEST 4]  Battery voltage & percentage check
[TEST 5]  Flight mode read-back
[TEST 6]  (OPTIONAL) Arm → verify armed flag → Disarm  [PROPS OFF ONLY]
[TEST 7]  Pre-flight check validation (all safety thresholds)

Usage
------
    # On Raspberry Pi — USB connection (easiest for first test):
    python tools/hardware_test.py --connection /dev/ttyUSB0 --baud 57600

    # TELEM2 UART on Pi 5:
    python tools/hardware_test.py --connection /dev/ttyAMA0 --baud 921600

    # Windows (USB):
    python tools/hardware_test.py --connection COM3 --baud 57600

    # Skip the arm/disarm test (default — SAFE):
    python tools/hardware_test.py --connection /dev/ttyUSB0 --no-arm-test

    # Enable arm/disarm test (PROPS MUST BE REMOVED):
    python tools/hardware_test.py --connection /dev/ttyUSB0 --arm-test

WARNING
-------
    NEVER run --arm-test with propellers attached.
    The motors WILL spin immediately on arming.
"""

from __future__ import annotations

import argparse
import logging
import sys
import time
import pathlib

# --- make sure formyx_backend/ is on the path ---
sys.path.insert(0, str(pathlib.Path(__file__).parent.parent))

from mavlink_interface.connection import MAVLinkConnection
from mavlink_interface.commands import (
    arm,
    disarm,
    CommandRejectedError,
    CommandTimeoutError,
)

# ---------------------------------------------------------------------------
# ANSI colour helpers (degrade gracefully on Windows without ANSI support)
# ---------------------------------------------------------------------------
try:
    import colorama
    colorama.init()
    GREEN  = "\033[92m"
    RED    = "\033[91m"
    YELLOW = "\033[93m"
    CYAN   = "\033[96m"
    BOLD   = "\033[1m"
    RESET  = "\033[0m"
except ImportError:
    GREEN = RED = YELLOW = CYAN = BOLD = RESET = ""


def _pass(msg: str) -> None:
    print(f"  {GREEN}✓ PASS{RESET}  {msg}")


def _fail(msg: str) -> None:
    print(f"  {RED}✗ FAIL{RESET}  {msg}")


def _warn(msg: str) -> None:
    print(f"  {YELLOW}⚠ WARN{RESET}  {msg}")


def _info(msg: str) -> None:
    print(f"  {CYAN}ℹ INFO{RESET}  {msg}")


def _header(title: str) -> None:
    print(f"\n{BOLD}{'─'*60}{RESET}")
    print(f"{BOLD}  {title}{RESET}")
    print(f"{BOLD}{'─'*60}{RESET}")


# ---------------------------------------------------------------------------
# Test results accumulator
# ---------------------------------------------------------------------------

class Results:
    def __init__(self):
        self.passed = 0
        self.failed = 0
        self.warnings = 0

    def ok(self, msg: str):
        _pass(msg)
        self.passed += 1

    def fail(self, msg: str):
        _fail(msg)
        self.failed += 1

    def warn(self, msg: str):
        _warn(msg)
        self.warnings += 1

    def summary(self):
        total = self.passed + self.failed
        _header("TEST SUMMARY")
        print(f"  Tests run   : {total}")
        print(f"  {GREEN}Passed{RESET}      : {self.passed}")
        print(f"  {RED}Failed{RESET}      : {self.failed}")
        print(f"  {YELLOW}Warnings{RESET}    : {self.warnings}")
        print()
        if self.failed == 0:
            print(f"  {GREEN}{BOLD}✓ All tests passed — hardware link is healthy.{RESET}")
        else:
            print(f"  {RED}{BOLD}✗ {self.failed} test(s) failed — review output above.{RESET}")
        print()


# ---------------------------------------------------------------------------
# Individual tests
# ---------------------------------------------------------------------------

def test_connection(conn_str: str, baud: int, r: Results) -> MAVLinkConnection | None:
    """TEST 1 — MAVLink connection & heartbeat."""
    _header("TEST 1 — MAVLink Connection & Heartbeat")

    # Build connection string
    if conn_str.startswith("/dev/") or conn_str.startswith("COM"):
        full_conn = f"serial:{conn_str}:{baud}"
    else:
        full_conn  = conn_str  # already full (e.g. udpin:...)

    _info(f"Connecting to: {full_conn}")
    conn = MAVLinkConnection(connection_string=full_conn)

    try:
        conn.connect()
        r.ok(f"Heartbeat received on {full_conn}")
        return conn
    except ConnectionError as exc:
        r.fail(f"Connection failed: {exc}")
        return None


def test_telemetry_soak(conn: MAVLinkConnection, r: Results, duration_s: int = 60) -> None:
    """TEST 2 — Telemetry stream soak (counts messages over N seconds)."""
    _header(f"TEST 2 — Telemetry Stream Quality ({duration_s}s soak)")

    _info(f"Collecting telemetry for {duration_s} seconds…")
    start = time.monotonic()
    message_counts: dict[str, int] = {}
    samples = 0

    while time.monotonic() - start < duration_s:
        t = conn.get_telemetry()
        samples += 1

        # Count message types received
        for msg_type in t.raw_messages:
            message_counts[msg_type] = message_counts.get(msg_type, 0) + 1

        time.sleep(0.1)  # 10 Hz polling

    elapsed = time.monotonic() - start
    _info(f"Soak complete — {samples} polls in {elapsed:.1f}s")
    _info(f"Message types seen: {sorted(message_counts.keys())}")

    required_types = ["HEARTBEAT", "GLOBAL_POSITION_INT", "ATTITUDE", "SYS_STATUS"]
    for msg_type in required_types:
        count = message_counts.get(msg_type, 0)
        rate = count / elapsed
        if count > 0:
            r.ok(f"{msg_type}: {count} messages received ({rate:.1f} Hz)")
        else:
            r.fail(f"{msg_type}: NOT received — check MAVLink stream configuration")


def test_gps(conn: MAVLinkConnection, r: Results, min_sats: int = 6) -> None:
    """TEST 3 — GPS fix quality."""
    _header("TEST 3 — GPS Fix & Satellite Count")

    t = conn.get_telemetry()
    fix_names = {0: "NO FIX", 1: "NO FIX", 2: "2D FIX", 3: "3D FIX",
                 4: "DGPS", 5: "RTK FLOAT", 6: "RTK FIXED"}
    fix_name = fix_names.get(t.gps_fix_type, f"UNKNOWN({t.gps_fix_type})")

    _info(f"GPS fix type   : {t.gps_fix_type} ({fix_name})")
    _info(f"Satellites     : {t.satellites_visible}")
    _info(f"Lat / Lon      : {t.lat_deg:.6f}°, {t.lon_deg:.6f}°")

    if t.gps_fix_type >= 3:
        r.ok(f"3D GPS fix acquired ({fix_name})")
    elif t.gps_fix_type == 2:
        r.warn("Only 2D GPS fix — GUIDED mode will not be reliable outdoors")
    else:
        r.fail(f"No GPS fix ({fix_name}) — cannot fly GUIDED mode safely")

    if t.satellites_visible >= min_sats:
        r.ok(f"Satellite count OK: {t.satellites_visible} ≥ {min_sats}")
    else:
        r.warn(
            f"Low satellite count: {t.satellites_visible} < {min_sats} "
            f"— wait for better GPS lock before flying"
        )


def test_battery(conn: MAVLinkConnection, r: Results) -> None:
    """TEST 4 — Battery voltage & state of charge."""
    _header("TEST 4 — Battery Status")

    t = conn.get_telemetry()
    _info(f"Voltage        : {t.battery_voltage_v:.2f} V")
    _info(f"Current        : {t.battery_current_a:.1f} A")
    _info(f"Remaining      : {t.battery_remaining_pct}%")

    if t.battery_voltage_v <= 0:
        r.warn("Battery voltage not reported — check SYS_STATUS stream")
    elif t.battery_voltage_v >= 11.0:
        r.ok(f"Battery voltage healthy: {t.battery_voltage_v:.2f} V")
    elif t.battery_voltage_v >= 10.5:
        r.warn(f"Battery voltage low: {t.battery_voltage_v:.2f} V — consider charging")
    else:
        r.fail(f"Battery critically low: {t.battery_voltage_v:.2f} V — DO NOT FLY")

    if t.battery_remaining_pct == -1:
        r.warn("Battery percentage not reported by autopilot")
    elif t.battery_remaining_pct >= 50:
        r.ok(f"Battery charge OK: {t.battery_remaining_pct}%")
    elif t.battery_remaining_pct >= 25:
        r.warn(f"Battery at {t.battery_remaining_pct}% — sufficient for short test")
    else:
        r.fail(f"Battery at {t.battery_remaining_pct}% — charge before flying")


def test_flight_mode(conn: MAVLinkConnection, r: Results) -> None:
    """TEST 5 — Flight mode read-back."""
    _header("TEST 5 — Flight Mode & Armed State")

    t = conn.get_telemetry()
    _info(f"Flight mode    : {t.flight_mode}")
    _info(f"Armed          : {t.armed}")

    if t.flight_mode != "UNKNOWN":
        r.ok(f"Flight mode readable: {t.flight_mode}")
    else:
        r.fail("Flight mode reported as UNKNOWN — heartbeat parsing issue")

    if not t.armed:
        r.ok("Vehicle is currently DISARMED (safe)")
    else:
        r.warn("Vehicle is currently ARMED — ensure props are removed for testing")


def test_arm_disarm(conn: MAVLinkConnection, r: Results) -> None:
    """TEST 6 — Arm / Disarm cycle (PROPS MUST BE REMOVED)."""
    _header("TEST 6 — ARM / DISARM Cycle (⚠ PROPS OFF)")

    print(f"\n  {RED}{BOLD}SAFETY CHECK:{RESET}")
    print(f"  {BOLD}  Are all propellers physically removed from the motors?{RESET}")
    print(f"  Type  '{BOLD}YES I CONFIRM PROPS ARE OFF{RESET}'  to proceed,")
    print( "  or press Enter to skip this test.\n")

    try:
        response = input("  > ").strip()
    except EOFError:
        response = ""

    if response != "YES I CONFIRM PROPS ARE OFF":
        _warn("Arm test skipped by user.")
        return

    _info("Attempting to ARM…")
    try:
        arm(conn, timeout=10.0)
        t = conn.get_telemetry()
        time.sleep(1.0)   # Let telemetry update
        t = conn.get_telemetry()

        if t.armed:
            r.ok("ARM command accepted — armed flag confirmed in telemetry")
        else:
            r.fail("ARM command ACK'd but armed flag NOT set in telemetry")

    except CommandRejectedError as exc:
        r.fail(f"ARM rejected by autopilot: {exc}")
        _info("Common causes: pre-arm check failures, compass not calibrated,")
        _info("safety switch not pressed, or not in STABILIZE/GUIDED mode.")
        return
    except CommandTimeoutError as exc:
        r.fail(f"ARM timed out (no ACK): {exc}")
        return

    _info("Waiting 3 seconds before disarm…")
    time.sleep(3.0)

    _info("Attempting to DISARM…")
    try:
        disarm(conn, timeout=10.0)
        time.sleep(1.0)
        t = conn.get_telemetry()
        if not t.armed:
            r.ok("DISARM confirmed — armed flag cleared in telemetry")
        else:
            r.warn("DISARM ACK'd but armed flag still set — may need a moment")
    except (CommandRejectedError, CommandTimeoutError) as exc:
        r.fail(f"DISARM failed: {exc}")


def test_preflight_checks(conn: MAVLinkConnection, r: Results) -> None:
    """TEST 7 — Validate all safety thresholds as the state machine would."""
    _header("TEST 7 — Pre-flight Safety Threshold Check")

    from config import get
    t = conn.get_telemetry()

    min_sats     = get("safety", "gps_min_satellites", 6)
    min_bat_pct  = get("safety", "battery_warning_pct", 25)
    min_bat_volt = 10.5   # practical minimum for 3S LiPo

    checks = [
        ("GPS satellites",   t.satellites_visible >= min_sats,
         f"{t.satellites_visible} sats (need ≥{min_sats})"),
        ("GPS 3D fix",       t.gps_fix_type >= 3,
         f"fix_type={t.gps_fix_type} (need ≥3)"),
        ("Battery %",        t.battery_remaining_pct == -1 or
                             t.battery_remaining_pct >= min_bat_pct,
         f"{t.battery_remaining_pct}% (need ≥{min_bat_pct}%)"),
        ("Battery voltage",  t.battery_voltage_v <= 0 or
                             t.battery_voltage_v >= min_bat_volt,
         f"{t.battery_voltage_v:.2f}V (need ≥{min_bat_volt}V)"),
        ("MAVLink connected", conn.is_connected(),
         "heartbeat age check"),
        ("Vehicle disarmed", not t.armed,
         "must be disarmed for pre-flight"),
    ]

    all_pass = True
    for name, passed, detail in checks:
        if passed:
            r.ok(f"{name}: {detail}")
        else:
            r.fail(f"{name}: {detail}")
            all_pass = False

    if all_pass:
        print(f"\n  {GREEN}{BOLD}✓ PRE-FLIGHT CHECKS PASS — Safe to proceed.{RESET}\n")
    else:
        print(f"\n  {RED}{BOLD}✗ PRE-FLIGHT CHECKS FAILED — Do NOT arm the vehicle.{RESET}\n")


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main() -> int:
    parser = argparse.ArgumentParser(
        description="Formyx Hardware Validation Tool — Pi 5 + PIX6",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument(
        "--connection", "-c",
        default="/dev/ttyAMA0",
        help="Serial port or connection string (default: /dev/ttyAMA0)",
    )
    parser.add_argument(
        "--baud", "-b",
        type=int,
        default=921600,
        help="Serial baud rate (default: 921600)",
    )
    parser.add_argument(
        "--soak-duration",
        type=int,
        default=10,
        help="Telemetry soak duration in seconds (default: 10, use 60 for full test)",
    )
    parser.add_argument(
        "--arm-test",
        action="store_true",
        default=False,
        help="Enable the arm/disarm test (⚠ REMOVE PROPS FIRST)",
    )
    parser.add_argument(
        "--no-arm-test",
        dest="arm_test",
        action="store_false",
        help="Skip the arm/disarm test (default)",
    )
    args = parser.parse_args()

    logging.basicConfig(level=logging.WARNING)  # suppress MAVLink noise

    r = Results()

    print(f"\n{BOLD}{'═'*60}{RESET}")
    print(f"{BOLD}  FORMYX HARDWARE VALIDATION TOOL{RESET}")
    print(f"{BOLD}  Raspberry Pi 5 + Radiolink PIX6{RESET}")
    print(f"{BOLD}{'═'*60}{RESET}")
    print(f"  Connection : {args.connection}")
    print(f"  Baud rate  : {args.baud}")
    print(f"  Soak       : {args.soak_duration}s")
    print(f"  Arm test   : {'ENABLED ⚠' if args.arm_test else 'DISABLED (safe)'}")

    # --- TEST 1: Connect ---
    conn = test_connection(args.connection, args.baud, r)
    if conn is None:
        r.summary()
        return 1

    # --- TEST 2: Telemetry soak ---
    test_telemetry_soak(conn, r, duration_s=args.soak_duration)

    # --- TEST 3: GPS ---
    test_gps(conn, r)

    # --- TEST 4: Battery ---
    test_battery(conn, r)

    # --- TEST 5: Flight mode ---
    test_flight_mode(conn, r)

    # --- TEST 6: Arm/disarm (optional) ---
    if args.arm_test:
        test_arm_disarm(conn, r)
    else:
        _header("TEST 6 — ARM/DISARM")
        _warn("Skipped (run with --arm-test to enable, props MUST be off)")

    # --- TEST 7: Pre-flight summary ---
    test_preflight_checks(conn, r)

    conn.close()
    r.summary()
    return 0 if r.failed == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
