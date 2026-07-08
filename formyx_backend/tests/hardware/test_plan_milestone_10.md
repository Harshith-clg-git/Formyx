# Hardware Test Plan — Milestone 10: Logging & Black-Box Recording

This document outlines the hardware validation cases for the **Black-Box Logger** (`logging_system/logger.py`) running on the companion computer (Raspberry Pi 5).

---

## Safety Requirements & Pre-conditions

> [!NOTE]
> All flight logging verification is done on a **stationary bench setup** (no flight required).
> Ensure the vehicle remains disarmed with propellers **physically removed**.

---

## Test Cases

### Test Case 10.1: Logging Thread & File Creation

*   **Objective**: Verify that starting a logging session creates a CSV log file and initializes the worker thread.
*   **Pre-conditions**:
    *   No props.
    *   Log directory specified (default: `logs/`).
*   **Execution Steps**:
    1. Open a Python shell in the root of the project directory on Pi 5.
    2. Start the logger:
        ```python
        from logging_system.logger import BlackBoxLogger
        logger = BlackBoxLogger()
        logger.start()
        
        # Verify worker thread is alive
        import threading
        threads = [t.name for t in threading.enumerate()]
        print("Active Threads:", threads)
        
        logger.stop()
        ```
*   **Expected Results**:
    *   A new `.csv` file (named `flight_log_YYYYMMDD_HHMMSS.csv`) is created in the `logs/` directory.
    *   The list of active threads contains `'blackbox-writer'`.
    *   The log file contains the CSV header row.
*   **Pass/Fail Criteria**:
    *   **PASS**: Log file and worker thread created successfully.
    *   **FAIL**: Failed to create file, or background thread does not start.

---

### Test Case 10.2: Asynchronous Log Writing & Rate Accuracy

*   **Objective**: Validate high-frequency asynchronous logging under continuous data streams at 10 Hz.
*   **Pre-conditions**:
    *   `reconnect_delay_s = 2.0`, `log_rate_hz = 10` in settings.yaml.
*   **Execution Steps**:
    1. Run a logger test script:
        ```python
        import time
        from logging_system.logger import BlackBoxLogger
        from mavlink_interface.connection import TelemetrySnapshot
        
        logger = BlackBoxLogger()
        logger.start()
        
        # Create a mock telemetry snapshot
        t = TelemetrySnapshot(armed=True, flight_mode="GUIDED", lat_deg=12.9, lon_deg=77.5)
        
        print("Logging at 10 Hz for 10 seconds...")
        for _ in range(100):
            logger.log("SEARCHING", t, target_vector=(1.0, 2.0, 3.0, 0.0, 0.0, 0.0), cmd_vector=(0.5, 0.0, 0.0))
            time.sleep(0.1)
            
        logger.stop()
        ```
    2. Count the number of rows (excluding header) in the generated log file.
*   **Expected Results**:
    *   The log file contains approximately 100 rows.
    *   Data values are completely populated and contain exact float representations.
*   **Pass/Fail Criteria**:
    *   **PASS**: Approximately 100 rows written, data fields correct.
    *   **FAIL**: File is empty, missing rows, or has malformed formats.

---

### Test Case 10.3: Log Rotation & Drive Protection

*   **Objective**: Confirm that log rotation successfully deletes old files to protect disk space.
*   **Pre-conditions**:
    *   Delete all files in `logs/` directory.
*   **Execution Steps**:
    1. Set `max_log_files = 5` in settings.yaml (or programmatically).
    2. Start the logger.
    3. Run a loop to start and stop the logger 7 times sequentially.
    4. List the CSV files in `logs/`.
*   **Expected Results**:
    *   The total number of CSV log files in `logs/` is exactly 5.
    *   The oldest log files are deleted automatically.
*   **Pass/Fail Criteria**:
    *   **PASS**: Logger enforces log file limit correctly, deleting oldest files first.
    *   **FAIL**: Log count exceeds 5, or rotation deletes incorrect files.
