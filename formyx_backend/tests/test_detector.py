"""
tests/test_detector.py
----------------------
Unit tests for the BalloonDetector class.
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

from perception.detector import BalloonDetector


def test_detector_missing_weights_raises_error():
    """Verify that FileNotFoundError is raised if model weights are missing."""
    with pytest.raises(FileNotFoundError):
        # Pass a path that definitely doesn't exist
        BalloonDetector(model_path="models/does_not_exist_xyz.pt")


@patch("perception.detector.YOLO")
def test_detector_initialization(mock_yolo, tmp_path):
    """Verify that detector loads config settings and initializes YOLO when file exists."""
    dummy_model = tmp_path / "dummy_detector.pt"
    dummy_model.write_text("fake weights")

    detector = BalloonDetector(model_path=str(dummy_model))

    assert detector.model_path == dummy_model
    assert detector.conf_threshold == 0.60
    assert detector.resolution == (320, 320)
    assert detector.class_id == 0
    mock_yolo.assert_called_once_with(str(dummy_model))


def _make_mock_yolo_result(bboxes, confs, clss):
    """Helper to build nested mock structure for YOLO result.boxes."""
    mock_result = MagicMock()
    mock_boxes = MagicMock()

    # Mock xyxy.cpu().numpy()
    mock_xyxy = MagicMock()
    mock_xyxy.cpu.return_value.numpy.return_value = np.array(bboxes, dtype=np.float32)
    mock_boxes.xyxy = mock_xyxy

    # Mock conf.cpu().numpy()
    mock_conf = MagicMock()
    mock_conf.cpu.return_value.numpy.return_value = np.array(confs, dtype=np.float32)
    mock_boxes.conf = mock_conf

    # Mock cls.cpu().numpy()
    mock_cls = MagicMock()
    mock_cls.cpu.return_value.numpy.return_value = np.array(clss, dtype=np.float32)
    mock_boxes.cls = mock_cls

    mock_result.boxes = mock_boxes
    return mock_result


@patch("perception.detector.YOLO")
def test_detect_parses_outputs_correctly(mock_yolo, tmp_path):
    """Verify that detection boxes are parsed, center is computed, and results mapped."""
    dummy_model = tmp_path / "dummy_detector.pt"
    dummy_model.write_text("fake weights")

    # Set up mock YOLO call output
    mock_yolo_inst = MagicMock()
    mock_yolo.return_value = mock_yolo_inst

    detector = BalloonDetector(model_path=str(dummy_model))

    # Mock frame
    frame = np.zeros((480, 640, 3), dtype=np.uint8)

    # 1 detection: class 0 (balloon), conf 0.85, bbox (100, 100, 200, 200)
    mock_result = _make_mock_yolo_result(
        bboxes=[[100.0, 100.0, 200.0, 200.0]],
        confs=[0.85],
        clss=[0]
    )
    mock_yolo_inst.return_value = [mock_result]

    detections = detector.detect(frame)

    assert len(detections) == 1
    det = detections[0]
    assert det["bbox"] == (100.0, 100.0, 200.0, 200.0)
    assert det["center"] == (150.0, 150.0)  # center of 100 and 200
    assert pytest.approx(det["confidence"]) == 0.85
    assert det["class_id"] == 0


@patch("perception.detector.YOLO")
def test_detect_filters_by_class_id(mock_yolo, tmp_path):
    """Verify that detections with class IDs other than the configured class_id are ignored."""
    dummy_model = tmp_path / "dummy_detector.pt"
    dummy_model.write_text("fake weights")

    mock_yolo_inst = MagicMock()
    mock_yolo.return_value = mock_yolo_inst

    detector = BalloonDetector(model_path=str(dummy_model))
    # Configured class_id is 0.

    # 2 detections: class 0 (match) and class 1 (ignore)
    mock_result = _make_mock_yolo_result(
        bboxes=[[50.0, 50.0, 150.0, 150.0], [200.0, 200.0, 300.0, 300.0]],
        confs=[0.90, 0.75],
        clss=[0, 1]
    )
    mock_yolo_inst.return_value = [mock_result]

    frame = np.zeros((480, 640, 3), dtype=np.uint8)
    detections = detector.detect(frame)

    # Should only return class 0 detection
    assert len(detections) == 1
    assert detections[0]["class_id"] == 0
    assert detections[0]["bbox"] == (50.0, 50.0, 150.0, 150.0)


@patch("perception.detector.YOLO")
def test_detect_handles_empty_or_invalid_inputs(mock_yolo, tmp_path):
    """Verify detect returns empty lists for None frame, empty frame, or no detections."""
    dummy_model = tmp_path / "dummy_detector.pt"
    dummy_model.write_text("fake weights")

    detector = BalloonDetector(model_path=str(dummy_model))

    # None frame
    assert detector.detect(None) == []

    # Empty frame (size 0)
    assert detector.detect(np.array([])) == []
