# Hardware Test Plan — Milestone 5: Balloon Detection Pipeline

This document outlines the hardware validation cases for the **Balloon Detector** (`perception/detector.py`) running on the companion computer (Raspberry Pi 5) with a connected camera.

---

## Safety Requirements & Pre-conditions

> [!NOTE]
> All balloon detection verification is done on a **stationary bench setup** (flight is not required).
> Ensure the vehicle remains disarmed with propellers **physically removed**.

---

## Test Cases

### Test Case 5.1: Model Loading & Initialization

*   **Objective**: Confirm that the YOLOv8 model weights file is successfully loaded and initialized on the Raspberry Pi 5 CPU without memory leaks or missing dependencies.
*   **Pre-conditions**:
    *   Place the trained weights file `balloon_detector.pt` in `formyx_backend/models/`.
    *   Verify PyYAML, OpenCV, and Ultralytics packages are installed.
*   **Execution Steps**:
    1. SSH into the Raspberry Pi 5.
    2. Open a Python shell in the root of the project directory.
    3. Execute the initialization sequence:
        ```python
        from perception.detector import BalloonDetector
        detector = BalloonDetector()
        print("Model path:", detector.model_path)
        ```
*   **Expected Results**:
    *   No exception is raised.
    *   Console log prints: `Loading YOLO model from models/balloon_detector.pt...` followed by `YOLO model loaded successfully on CPU.`
*   **Pass/Fail Criteria**:
    *   **PASS**: Model initializes successfully and log message is printed.
    *   **FAIL**: Model file is not found, out of memory error, import error, or initialization hangs.

---

### Test Case 5.2: Real-time Balloon Detection & Coordinate Accuracy

*   **Objective**: Verify the detector class detects a physical balloon with high confidence and produces expected bounding box center coordinate outputs.
*   **Pre-conditions**:
    *   Connect the camera (Intel RealSense D435i or USB webcam) to the Pi 5.
    *   Hold a physical balloon (or show an image of one) in front of the lens.
*   **Execution Steps**:
    1. Run a test pipeline script (or custom validation script) that queries frames from the camera and passes them to the detector:
        ```python
        import cv2
        from perception.detector import BalloonDetector
        
        detector = BalloonDetector()
        cap = cv2.VideoCapture(0) # or realsense stream
        
        ret, frame = cap.read()
        if ret:
            detections = detector.detect(frame)
            print("Detections found:", detections)
        cap.release()
        ```
    2. Read output detections structure.
    3. Move the balloon around the frame (left, right, close, far) and verify that the `center` and `bbox` coordinate fields change correspondingly.
*   **Expected Results**:
    *   A list containing at least one detection dict with keys `bbox`, `center`, `confidence`, and `class_id == 0`.
    *   Confidence score should be >= 0.60.
    *   Center X-coordinate increases as the balloon is moved to the right of the frame.
    *   Center Y-coordinate increases as the balloon is moved down.
*   **Pass/Fail Criteria**:
    *   **PASS**: The balloon is successfully detected, confidence matches or exceeds 0.60, and coordinate tracking logic is confirmed.
    *   **FAIL**: The balloon is not detected in the frame, confidence is low, or coordinate tracking is incorrect.

---

### Test Case 5.3: CPU Inference Framerate Benchmark

*   **Objective**: Ensure the Pi 5 CPU can execute inference on the YOLOv8 model at the required target frame rate (>10 FPS) at 320x320 resolution.
*   **Pre-conditions**:
    *   Record or copy a 10-second test video containing a moving balloon (30 FPS, e.g. `tests/assets/test_balloon.mp4`) onto the Pi 5.
*   **Execution Steps**:
    1. Run a benchmarker script that loops through the video frames and measures inference execution speed:
        ```python
        import time
        import cv2
        from perception.detector import BalloonDetector
        
        detector = BalloonDetector()
        cap = cv2.VideoCapture("tests/assets/test_balloon.mp4")
        
        frame_count = 0
        start_time = time.monotonic()
        
        while cap.isOpened():
            ret, frame = cap.read()
            if not ret:
                break
            _ = detector.detect(frame)
            frame_count += 1
            
        elapsed = time.monotonic() - start_time
        fps = frame_count / elapsed
        print(f"Processed {frame_count} frames in {elapsed:.2f}s (FPS = {fps:.2f})")
        cap.release()
        ```
*   **Expected Results**:
    *   The benchmark completes and outputs the total frames and average frame rate.
    *   The calculated FPS is greater than or equal to 10.0 FPS.
*   **Pass/Fail Criteria**:
    *   **PASS**: Average inference frame rate is >= 10.0 FPS.
    *   **FAIL**: Frame rate falls below 10.0 FPS, indicating the model or configuration is not sufficiently optimized for the Pi 5 CPU.
