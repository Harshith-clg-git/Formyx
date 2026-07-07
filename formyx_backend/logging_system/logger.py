"""
formyx_backend/logging_system/logger.py
----------------------------------------
High-rate black-box logger that saves flight logs to disk for post-flight analysis.
Writes asynchronously using a background thread and enforces log rotation limits.
"""

from __future__ import annotations

import csv
import logging
import os
import queue
import threading
import time
from datetime import datetime
from pathlib import Path
from typing import TYPE_CHECKING, Any, Dict, List, Tuple

from config import get

if TYPE_CHECKING:
    from mavlink_interface.connection import TelemetrySnapshot

log = logging.getLogger(__name__)


class BlackBoxLogger:
    """
    Asynchronous CSV logger that buffers logs in a queue and writes them to disk
    using a background worker thread. Keeps log count within limits via rotation.
    """

    # CSV headers
    HEADERS = [
        "timestamp",
        "state",
        "armed",
        "flight_mode",
        "lat",
        "lon",
        "alt_agl",
        "alt_amsl",
        "vx",
        "vy",
        "vz",
        "batt_v",
        "batt_pct",
        "gps_fix",
        "gps_sats",
        "target_x",
        "target_y",
        "target_z",
        "target_vx",
        "target_vy",
        "target_vz",
        "cmd_vx",
        "cmd_vy",
        "cmd_vz",
    ]

    def __init__(self, log_dir: str | None = None) -> None:
        self.log_dir = Path(log_dir or get("logging", "log_dir", "logs"))
        self.max_log_files: int = get("logging", "max_log_files", 50)
        self.log_rate_hz: int = get("logging", "log_rate_hz", 10)

        # Asynchronous write buffer
        self._queue: queue.Queue = queue.Queue()
        self._stop_event = threading.Event()
        self._worker_thread: threading.Thread | None = None
        self._file_handle = None
        self._csv_writer = None

        log.info(
            "BlackBoxLogger initialized: log_dir=%s, max_logs=%d, log_rate=%dHz",
            self.log_dir,
            self.max_log_files,
            self.log_rate_hz,
        )

    def start(self) -> None:
        """Create log directory, clean up old logs, open the new log file, and start worker."""
        # Ensure log directory exists
        self.log_dir.mkdir(parents=True, exist_ok=True)

        # Enforce log rotation (delete oldest files)
        self._rotate_logs()

        # Generate timestamp-based log filename
        timestamp_str = datetime.now().strftime("%Y%m%d_%H%M%S")
        log_file_path = self.log_dir / f"flight_log_{timestamp_str}.csv"

        try:
            # Open file with explicit UTF-8 encoding
            self._file_handle = open(log_file_path, "w", newline="", encoding="utf-8")
            self._csv_writer = csv.writer(self._file_handle)
            # Write header row
            self._csv_writer.writerow(self.HEADERS)
            self._file_handle.flush()
            log.info("Opened new flight log file: %s", log_file_path)
        except Exception as exc:
            log.error("Failed to open flight log file %s: %s", log_file_path, exc)
            raise

        # Start the background worker thread
        self._stop_event.clear()
        self._worker_thread = threading.Thread(
            target=self._writer_loop,
            name="blackbox-writer",
            daemon=True,
        )
        self._worker_thread.start()
        log.debug("BlackBox logger worker thread started.")

    def log(
        self,
        state_name: str,
        telemetry: TelemetrySnapshot,
        target_vector: Tuple[float, float, float, float, float, float] | None = None,
        cmd_vector: Tuple[float, float, float] | None = None,
    ) -> None:
        """
        Queue a telemetry snapshot, state, target vector, and command vector to be logged.
        This is non-blocking and thread-safe.
        """
        if self._stop_event.is_set():
            return

        target_x, target_y, target_z, target_vx, target_vy, target_vz = (
            target_vector if target_vector is not None else (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        )
        cmd_vx, cmd_vy, cmd_vz = cmd_vector if cmd_vector is not None else (0.0, 0.0, 0.0)

        row = [
            time.time(),
            state_name,
            int(telemetry.armed),
            telemetry.flight_mode,
            telemetry.lat_deg,
            telemetry.lon_deg,
            telemetry.alt_agl_m,
            telemetry.alt_amsl_m,
            telemetry.vx_ms,
            telemetry.vy_ms,
            telemetry.vz_ms,
            telemetry.battery_voltage_v,
            telemetry.battery_remaining_pct,
            telemetry.gps_fix_type,
            telemetry.satellites_visible,
            target_x,
            target_y,
            target_z,
            target_vx,
            target_vy,
            target_vz,
            cmd_vx,
            cmd_vy,
            cmd_vz,
        ]

        # Put in the queue
        self._queue.put(row)

    def stop(self) -> None:
        """Stop background worker, flush remaining queues, and close files."""
        log.info("Stopping BlackBoxLogger.")
        self._stop_event.set()

        if self._worker_thread and self._worker_thread.is_alive():
            self._worker_thread.join(timeout=3.0)

        # Process any leftover items in the queue
        self._flush_queue()

        if self._file_handle:
            try:
                self._file_handle.close()
                log.info("Flight log file closed.")
            except Exception as exc:
                log.error("Error closing flight log file: %s", exc)
            finally:
                self._file_handle = None
                self._csv_writer = None

    def _rotate_logs(self) -> None:
        """Delete oldest log files if total log count exceeds max_log_files limit."""
        try:
            # List all flight log CSV files
            log_files = sorted(
                self.log_dir.glob("flight_log_*.csv"),
                key=os.path.getmtime,
            )
            
            # If we exceed the limit, delete oldest files
            if len(log_files) >= self.max_log_files:
                excess_count = len(log_files) - self.max_log_files + 1
                log.warning(
                    "Log limit reached (%d >= %d). Rotating oldest %d files.",
                    len(log_files),
                    self.max_log_files,
                    excess_count,
                )
                for i in range(excess_count):
                    try:
                        log_files[i].unlink()
                        log.info("Deleted rotated log file: %s", log_files[i].name)
                    except Exception as exc:
                        log.error("Failed to delete rotated log file %s: %s", log_files[i].name, exc)
        except Exception as exc:
            log.error("Error running log rotation: %s", exc)

    def _writer_loop(self) -> None:
        """Background worker thread consuming log items from queue and writing to disk."""
        while not self._stop_event.is_set():
            try:
                # Wait for items to arrive in queue (100ms timeout)
                row = self._queue.get(timeout=0.1)
                self._write_row(row)
                self._queue.task_done()
            except queue.Empty:
                continue
            except Exception as exc:
                log.error("Error in blackbox logger writer thread: %s", exc)

    def _write_row(self, row: List[Any]) -> None:
        if self._csv_writer and self._file_handle:
            self._csv_writer.writerow(row)
            self._file_handle.flush()

    def _flush_queue(self) -> None:
        """Synchronously write any remaining items in the queue to disk."""
        log.debug("Flushing logger queue (%d items remaining).", self._queue.qsize())
        while not self._queue.empty():
            try:
                row = self._queue.get_nowait()
                self._write_row(row)
                self._queue.task_done()
            except queue.Empty:
                break
            except Exception as exc:
                log.error("Error flushing logger queue: %s", exc)
