"""
tests/test_detector.py
----------------------
Unit tests for ObjectDetector (formerly BalloonDetector).

Tests cover:
  - Model loading / missing weights
  - Multi-class detection: balloon (class 0) AND drone (class 1)
  - Class filtering via target_class_ids
  - Convenience wrappers: detect_balloons() / detect_drones()
  - Detection dict schema: label field is present and correct
  - Edge-case inputs: None frame, empty frame
"""

from __future__ import annotations

import sys
import pathlib
import numpy as np
import pytest
from unittest.mock import MagicMock, patch

_BACKEND = pathlib.Path(__file__).parent.parent
if str(_BACKEND) not in sys.path:
    sys.path.insert(0, str(_BACKEND))

from perception.detector import ObjectDetector, BalloonDetector, TargetClass


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _make_mock_yolo_result(bboxes, confs, clss):
    """Build a nested mock object mimicking a YOLO result.boxes structure."""
    mock_result = MagicMock()
    mock_boxes  = MagicMock()

    mock_xyxy = MagicMock()
    mock_xyxy.cpu.return_value.numpy.return_value = np.array(bboxes, dtype=np.float32)
    mock_boxes.xyxy = mock_xyxy

    mock_conf = MagicMock()
    mock_conf.cpu.return_value.numpy.return_value = np.array(confs, dtype=np.float32)
    mock_boxes.conf = mock_conf

    mock_cls = MagicMock()
    mock_cls.cpu.return_value.numpy.return_value = np.array(clss, dtype=np.float32)
    mock_boxes.cls = mock_cls

    mock_result.boxes = mock_boxes
    return mock_result


def _make_detector(tmp_path, mock_yolo):
    """Create an ObjectDetector with a dummy weights file."""
    dummy_model = tmp_path / "dummy_detector.pt"
    dummy_model.write_text("fake weights")
    mock_yolo.return_value = MagicMock()
    return ObjectDetector(model_path=str(dummy_model))


# ---------------------------------------------------------------------------
# 1. Model Loading
# ---------------------------------------------------------------------------

def test_detector_missing_weights_raises_file_not_found():
    """FileNotFoundError when model weights don't exist."""
    with pytest.raises(FileNotFoundError):
        ObjectDetector(model_path="models/does_not_exist_xyz.pt")


@patch("perception.detector.YOLO")
def test_detector_initialisation_stores_config(mock_yolo, tmp_path):
    """Verify default config values are stored and YOLO is called once."""
    detector = _make_detector(tmp_path, mock_yolo)

    assert detector.conf_threshold == 0.60
    assert detector.resolution == (320, 320)
    # Both classes active by default
    assert TargetClass.BALLOON in detector.target_class_ids
    assert TargetClass.DRONE   in detector.target_class_ids
    mock_yolo.assert_called_once()


@patch("perception.detector.YOLO")
def test_backwards_compat_alias(mock_yolo, tmp_path):
    """BalloonDetector is an alias for ObjectDetector — must not break imports."""
    dummy = tmp_path / "dummy.pt"
    dummy.write_text("fake")
    mock_yolo.return_value = MagicMock()
    detector = BalloonDetector(model_path=str(dummy))
    assert isinstance(detector, ObjectDetector)


# ---------------------------------------------------------------------------
# 2. Balloon detection (class 0)
# ---------------------------------------------------------------------------

@patch("perception.detector.YOLO")
def test_detect_balloon_returns_correct_dict(mock_yolo, tmp_path):
    """Balloon detection dict has correct keys, values, and label."""
    detector = _make_detector(tmp_path, mock_yolo)
    mock_result = _make_mock_yolo_result(
        bboxes=[[100.0, 100.0, 200.0, 200.0]],
        confs=[0.85],
        clss=[0],  # balloon
    )
    detector.model.return_value = [mock_result]

    frame = np.zeros((480, 640, 3), dtype=np.uint8)
    detections = detector.detect(frame)

    assert len(detections) == 1
    d = detections[0]
    assert d["bbox"]       == (100.0, 100.0, 200.0, 200.0)
    assert d["center"]     == (150.0, 150.0)
    assert pytest.approx(d["confidence"]) == 0.85
    assert d["class_id"]   == TargetClass.BALLOON
    assert d["label"]      == "balloon"


# ---------------------------------------------------------------------------
# 3. Drone detection (class 1)
# ---------------------------------------------------------------------------

@patch("perception.detector.YOLO")
def test_detect_drone_returns_correct_dict(mock_yolo, tmp_path):
    """Drone detection dict has correct keys, values, and label."""
    detector = _make_detector(tmp_path, mock_yolo)
    mock_result = _make_mock_yolo_result(
        bboxes=[[50.0, 60.0, 150.0, 160.0]],
        confs=[0.92],
        clss=[1],  # drone
    )
    detector.model.return_value = [mock_result]

    frame = np.zeros((480, 640, 3), dtype=np.uint8)
    detections = detector.detect(frame)

    assert len(detections) == 1
    d = detections[0]
    assert d["class_id"] == TargetClass.DRONE
    assert d["label"]    == "drone"
    assert pytest.approx(d["confidence"]) == 0.92


# ---------------------------------------------------------------------------
# 4. Multi-class — balloon + drone in same frame
# ---------------------------------------------------------------------------

@patch("perception.detector.YOLO")
def test_detect_returns_both_balloon_and_drone(mock_yolo, tmp_path):
    """Both class IDs returned when present in the same frame."""
    detector = _make_detector(tmp_path, mock_yolo)
    mock_result = _make_mock_yolo_result(
        bboxes=[
            [10.0, 10.0, 80.0, 80.0],    # balloon
            [200.0, 200.0, 300.0, 300.0], # drone
        ],
        confs=[0.88, 0.75],
        clss=[0, 1],
    )
    detector.model.return_value = [mock_result]

    frame = np.zeros((480, 640, 3), dtype=np.uint8)
    detections = detector.detect(frame)

    assert len(detections) == 2
    labels = {d["label"] for d in detections}
    assert labels == {"balloon", "drone"}


# ---------------------------------------------------------------------------
# 5. Class filtering — unknown class ignored
# ---------------------------------------------------------------------------

@patch("perception.detector.YOLO")
def test_detect_ignores_unknown_class_id(mock_yolo, tmp_path):
    """Class IDs not in target_class_ids are silently discarded."""
    detector = _make_detector(tmp_path, mock_yolo)
    mock_result = _make_mock_yolo_result(
        bboxes=[
            [50.0, 50.0, 150.0, 150.0],   # balloon  → keep
            [200.0, 200.0, 300.0, 300.0],  # class 99 → ignore
        ],
        confs=[0.90, 0.75],
        clss=[0, 99],
    )
    detector.model.return_value = [mock_result]

    frame = np.zeros((480, 640, 3), dtype=np.uint8)
    detections = detector.detect(frame)

    assert len(detections) == 1
    assert detections[0]["class_id"] == TargetClass.BALLOON


# ---------------------------------------------------------------------------
# 6. Convenience wrappers
# ---------------------------------------------------------------------------

@patch("perception.detector.YOLO")
def test_detect_balloons_filters_correctly(mock_yolo, tmp_path):
    """detect_balloons() returns only balloon detections."""
    detector = _make_detector(tmp_path, mock_yolo)
    mock_result = _make_mock_yolo_result(
        bboxes=[[10.0, 10.0, 80.0, 80.0], [200.0, 200.0, 300.0, 300.0]],
        confs=[0.88, 0.75],
        clss=[0, 1],
    )
    detector.model.return_value = [mock_result]

    frame = np.zeros((480, 640, 3), dtype=np.uint8)
    results = detector.detect_balloons(frame)

    assert all(d["class_id"] == TargetClass.BALLOON for d in results)
    assert len(results) == 1


@patch("perception.detector.YOLO")
def test_detect_drones_filters_correctly(mock_yolo, tmp_path):
    """detect_drones() returns only drone detections."""
    detector = _make_detector(tmp_path, mock_yolo)
    mock_result = _make_mock_yolo_result(
        bboxes=[[10.0, 10.0, 80.0, 80.0], [200.0, 200.0, 300.0, 300.0]],
        confs=[0.88, 0.75],
        clss=[0, 1],
    )
    detector.model.return_value = [mock_result]

    frame = np.zeros((480, 640, 3), dtype=np.uint8)
    results = detector.detect_drones(frame)

    assert all(d["class_id"] == TargetClass.DRONE for d in results)
    assert len(results) == 1


# ---------------------------------------------------------------------------
# 7. Edge cases
# ---------------------------------------------------------------------------

@patch("perception.detector.YOLO")
def test_detect_returns_empty_for_none_frame(mock_yolo, tmp_path):
    """detect() returns [] for a None input frame."""
    detector = _make_detector(tmp_path, mock_yolo)
    assert detector.detect(None) == []


@patch("perception.detector.YOLO")
def test_detect_returns_empty_for_empty_frame(mock_yolo, tmp_path):
    """detect() returns [] for a zero-size array."""
    detector = _make_detector(tmp_path, mock_yolo)
    assert detector.detect(np.array([])) == []
