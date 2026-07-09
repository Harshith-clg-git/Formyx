"""
tests/test_logger.py
--------------------
Unit tests for the BlackBoxLogger class.
"""

from __future__ import annotations

import csv
import os
import sys
import pathlib
import time
import pytest
from unittest.mock import MagicMock

_BACKEND = pathlib.Path(__file__).parent.parent
if str(_BACKEND) not in sys.path:
    sys.path.insert(0, str(_BACKEND))

from logging_system.logger import BlackBoxLogger
from mavlink_interface.connection import TelemetrySnapshot


def _mock_telem():
    """Helper to construct a TelemetrySnapshot."""
    return TelemetrySnapshot(
        connected=True,
        last_heartbeat_ts=time.monotonic(),
        battery_remaining_pct=90,
        battery_voltage_v=12.2,
        satellites_visible=9,
        gps_fix_type=3,
        armed=True,
        lat_deg=12.9,
        lon_deg=77.5,
        alt_agl_m=4.5,
        alt_amsl_m=900.0,
        vx_ms=1.1,
        vy_ms=-0.2,
        vz_ms=0.3,
        flight_mode="GUIDED",
    )


def test_logger_initialization(tmp_path):
    """Verify logger parameter loading."""
    logger = BlackBoxLogger(log_dir=str(tmp_path))
    assert logger.log_dir == tmp_path
    assert logger.max_log_files == 5
    assert logger.log_rate_hz == 10


def test_logger_creates_file_with_headers(tmp_path):
    """Verify start() creates the log file and writes the correct CSV header."""
    logger = BlackBoxLogger(log_dir=str(tmp_path))
    logger.start()
    
    # Check that a file was created
    log_files = list(tmp_path.glob("flight_log_*.csv"))
    assert len(log_files) == 1
    
    # Stop logger to close file
    logger.stop()
    
    # Read headers
    with open(log_files[0], "r", encoding="utf-8") as f:
        reader = csv.reader(f)
        headers = next(reader)
        
    assert headers == BlackBoxLogger.HEADERS


def test_asynchronous_logging_and_flush(tmp_path):
    """Verify that logging pushes to the queue and stop() flushes it to disk."""
    logger = BlackBoxLogger(log_dir=str(tmp_path))
    logger.start()
    
    telem = _mock_telem()
    # Log 3 separate rows
    logger.log("SEARCHING", telem, target_vector=(1.0, 2.0, 3.0, 0.1, 0.2, 0.3), cmd_vector=(0.5, 0.5, 0.0))
    logger.log("TRACKING", telem, target_vector=(2.0, 3.0, 4.0, 0.2, 0.3, 0.4), cmd_vector=(1.0, 1.0, 0.0))
    logger.log("LANDING", telem, target_vector=None, cmd_vector=None)
    
    # Stop logger (triggers queue flushing)
    logger.stop()
    
    # Read file rows
    log_files = list(tmp_path.glob("flight_log_*.csv"))
    with open(log_files[0], "r", encoding="utf-8") as f:
        reader = csv.reader(f)
        headers = next(reader)
        rows = list(reader)
        
    assert len(rows) == 3
    assert rows[0][1] == "SEARCHING"
    assert rows[0][2] == "1"  # armed = True (int 1)
    assert rows[0][3] == "GUIDED"  # flight mode
    
    # Verify floats parsed
    assert pytest.approx(float(rows[0][4])) == 12.9  # lat
    assert pytest.approx(float(rows[0][15])) == 1.0  # target_x
    assert pytest.approx(float(rows[0][22])) == 0.5  # cmd_vx
    
    # Second row checks
    assert rows[1][1] == "TRACKING"
    assert pytest.approx(float(rows[1][16])) == 3.0  # target_y
    
    # Third row checks (None target vector defaults to zeros)
    assert rows[2][1] == "LANDING"
    assert pytest.approx(float(rows[2][15])) == 0.0  # target_x


def test_log_rotation_deletes_oldest(tmp_path):
    """Verify log rotation keeps log file count within the limit."""
    logger = BlackBoxLogger(log_dir=str(tmp_path))
    logger.max_log_files = 3
    
    # Create 3 pre-existing mock logs
    for i in range(3):
        mock_file = tmp_path / f"flight_log_20260707_12000{i}.csv"
        mock_file.write_text("dummy")
        # Give them staggered modification times
        os.utime(mock_file, (time.time() + i*10, time.time() + i*10))
        
    log_files_before = sorted(tmp_path.glob("flight_log_*.csv"), key=os.path.getmtime)
    assert len(log_files_before) == 3
    oldest_filename = log_files_before[0].name
    
    # Start logger, which should trigger rotation (max 3, current 3, adding 1 -> deletes 1)
    logger.start()
    logger.stop()
    
    log_files_after = sorted(tmp_path.glob("flight_log_*.csv"), key=os.path.getmtime)
    assert len(log_files_after) == 3  # still capped at 3
    
    # Check that the oldest file was deleted
    remaining_names = [f.name for f in log_files_after]
    assert oldest_filename not in remaining_names
