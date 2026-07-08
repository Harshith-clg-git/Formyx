# Hardware Test Plan — Milestone 5: Object Detection Pipeline (Balloon & Drone)

This document outlines the hardware validation cases for the **Object Detector** (`perception/detector.py`) running on the companion computer (Raspberry Pi 5) with a connected camera. The detector targets two classes simultaneously:
- **Class 0** — `balloon`
- **Class 1** — `drone`

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
    *   Place the trained weights file `drone_balloon_detector.pt` (dual-class: balloon + drone) in `formyx_backend/models/`.
    *   Verify PyYAML, OpenCV, and Ultralytics packages are installed.
*   **Execution Steps**:
    1. SSH into the Raspberry Pi 5.
    2. Open a Python shell in the root of the project directory.
    3. Execute the initialization sequence:
        ```python
        from perception.detector import ObjectDetector
        detector = ObjectDetector()
        print("Model path:", detector.model_path)
        print("Active classes:", detector.target_class_ids)
        ```
*   **Expected Results**:
    *   No exception is raised.
    *   Console log prints: `Loading YOLO model from models/drone_balloon_detector.pt...` followed by `YOLO model loaded successfully on CPU.`
    *   `active classes` prints `{0, 1}`.
*   **Pass/Fail Criteria**:
    *   **PASS**: Model initializes successfully, log message is printed, and active class set is `{0, 1}`.
    *   **FAIL**: Model file not found, out of memory error, import error, or initialization hangs.

---

### Test Case 5.2: Real-time Balloon AND Drone Detection & Coordinate Accuracy

*   **Objective**: Verify the detector class detects both a physical **balloon** and a physical (or simulated) **drone** with high confidence, and produces correct bounding box center coordinate outputs.
*   **Pre-conditions**:
    *   Connect the camera (Intel RealSense D435i or USB webcam) to the Pi 5.
    *   Have a physical balloon and a small quadrotor (or printed drone image) available to hold in front of the lens.
*   **Execution Steps**:
    1. Run a test pipeline script that queries frames from the camera and passes them to the detector:
        ```python
        import cv2
        from perception.detector import ObjectDetector
        
        detector = ObjectDetector()
        cap = cv2.VideoCapture(0)  # or realsense stream
        
        ret, frame = cap.read()
        if ret:
            detections = detector.detect(frame)  # returns balloons + drones
            print("All detections:", detections)
            print("Balloons only:", detector.detect_balloons(frame))
            print("Drones only  :", detector.detect_drones(frame))
        cap.release()
        ```
    2. Read output detections structure.
    3. Hold the **balloon** in frame — verify `label == "balloon"` and `class_id == 0`.
    4. Replace with the **drone** (or drone image) — verify `label == "drone"` and `class_id == 1`.
    5. Move each target around the frame and verify that the `center` fields change correspondingly.
*   **Expected Results**:
    *   For balloon: detection dict with `label == "balloon"`, `class_id == 0`, `confidence >= 0.60`, correct center coordinates.
    *   For drone: detection dict with `label == "drone"`, `class_id == 1`, `confidence >= 0.60`, correct center coordinates.
    *   Center X-coordinate increases as the target is moved to the right.
    *   Center Y-coordinate increases as the target is moved downward.
*   **Pass/Fail Criteria**:
    *   **PASS**: Both target classes are detected with confidence ≥ 0.60, correct labels, and correct coordinate tracking.
    *   **FAIL**: Either class is not detected, confidence is low, label is wrong, or coordinate tracking is incorrect.

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
        from perception.detector import ObjectDetector
        
        detector = ObjectDetector()
        cap = cv2.VideoCapture("tests/assets/test_balloon.mp4")
        
        frame_count = 0
        start_time = time.monotonic()
        
        while cap.isOpened():
            ret, frame = cap.read()
            if not ret:
                break
            _ = detector.detect(frame)  # detects all classes in one pass
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
