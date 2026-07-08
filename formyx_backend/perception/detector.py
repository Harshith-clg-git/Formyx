"""
formyx_backend/perception/detector.py
--------------------------------------
Loads the YOLOv8 model and performs real-time object detection
to locate target objects (balloons AND drones) in raw video frames.

Supported target classes (configured in config/settings.yaml):
    0 — balloon
    1 — drone

The detector returns all detections whose class_id is in the
configured target_class_ids list. Each detection dict includes
a human-readable 'label' field for logging and display.
"""

from __future__ import annotations

import logging
from enum import IntEnum
from pathlib import Path
from typing import Any, Dict, List, Optional, Set, Tuple

import cv2
import numpy as np

try:
    from ultralytics import YOLO
    HAS_ULTRALYTICS = True
except ImportError:
    HAS_ULTRALYTICS = False
    YOLO = None  # type: ignore[assignment,misc]

from config import get

log = logging.getLogger(__name__)


# ---------------------------------------------------------------------------
# Target class registry
# ---------------------------------------------------------------------------

class TargetClass(IntEnum):
    """YOLO class IDs for detectable mission-critical objects."""
    BALLOON = 0
    DRONE   = 1


# Human-readable labels keyed by class ID
_CLASS_LABELS: Dict[int, str] = {
    TargetClass.BALLOON: "balloon",
    TargetClass.DRONE:   "drone",
}

# Default set of class IDs to detect when config is absent
_DEFAULT_TARGET_CLASS_IDS: List[int] = [TargetClass.BALLOON, TargetClass.DRONE]


class ObjectDetector:
    """
    YOLOv8-based multi-class object detector optimised for companion computer
    (CPU) execution on Raspberry Pi 5.

    Detects both **balloons** (class 0) and **drones** (class 1) from a single
    inference pass.  The set of active target classes is controlled via
    ``config/settings.yaml`` under ``perception.target_class_ids``.

    Detection dict schema
    ---------------------
    Each element returned by :meth:`detect` is a ``dict`` with keys:

    * ``bbox``       – ``Tuple[float, float, float, float]``
                       (xmin, ymin, xmax, ymax) in pixels
    * ``center``     – ``Tuple[float, float]``
                       (x_center, y_center) in pixels
    * ``confidence`` – ``float``  detection confidence [0.0, 1.0]
    * ``class_id``   – ``int``    YOLO class index
    * ``label``      – ``str``    human-readable class name
                       (e.g. ``"balloon"`` or ``"drone"``)
    """

    def __init__(self, model_path: Optional[str] = None) -> None:
        if not HAS_ULTRALYTICS:
            raise ImportError(
                "The 'ultralytics' package is required for ObjectDetector. "
                "Install it with:  pip install ultralytics"
            )

        cfg_path = model_path or get("perception", "model_path", "models/drone_balloon_detector.pt")
        self.model_path = Path(cfg_path)

        self.conf_threshold: float = get("perception", "confidence_threshold", 0.60)
        self.resolution: Tuple[int, int] = tuple(
            get("perception", "inference_resolution", [320, 320])
        )

        # Build the active target class-ID set from config.
        # Config key ``target_class_ids`` is a list of ints, e.g. [0, 1].
        # Falls back to _DEFAULT_TARGET_CLASS_IDS if key is absent.
        cfg_ids = get("perception", "target_class_ids", _DEFAULT_TARGET_CLASS_IDS)
        self.target_class_ids: Set[int] = set(int(c) for c in cfg_ids)

        log.info(
            "ObjectDetector — active classes: %s",
            {_CLASS_LABELS.get(c, str(c)) for c in self.target_class_ids},
        )

        if not self.model_path.exists():
            raise FileNotFoundError(
                f"YOLO model weights not found: {self.model_path.absolute()}\n"
                "Place the trained weights file in the models/ directory.\n"
                "Expected filename: drone_balloon_detector.pt"
            )

        log.info("Loading YOLO model from %s...", self.model_path)
        self.model = YOLO(str(self.model_path))
        log.info("YOLO model loaded successfully on CPU.")

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def detect(self, frame: np.ndarray) -> List[Dict[str, Any]]:
        """
        Run inference on a single BGR frame and return all target detections.

        Parameters
        ----------
        frame : np.ndarray
            Input image in OpenCV BGR format.

        Returns
        -------
        List[Dict[str, Any]]
            Detections for **all** active target classes (balloons + drones).
            Empty list if frame is None/empty or no targets are found.
        """
        if frame is None or frame.size == 0:
            return []

        results = self.model(
            frame,
            imgsz=self.resolution,
            conf=self.conf_threshold,
            device="cpu",
            verbose=False,
        )

        detections: List[Dict[str, Any]] = []

        for result in results:
            if result.boxes is None:
                continue

            xyxys = result.boxes.xyxy.cpu().numpy()
            confs  = result.boxes.conf.cpu().numpy()
            clss   = result.boxes.cls.cpu().numpy()

            for bbox, conf, cls in zip(xyxys, confs, clss):
                cls_val = int(cls)

                # Only keep classes that are in the active target set
                if cls_val not in self.target_class_ids:
                    continue

                xmin, ymin, xmax, ymax = map(float, bbox)
                x_center = (xmin + xmax) / 2.0
                y_center = (ymin + ymax) / 2.0

                detections.append({
                    "bbox":       (xmin, ymin, xmax, ymax),
                    "center":     (x_center, y_center),
                    "confidence": float(conf),
                    "class_id":   cls_val,
                    "label":      _CLASS_LABELS.get(cls_val, f"class_{cls_val}"),
                })

        return detections

    def detect_balloons(self, frame: np.ndarray) -> List[Dict[str, Any]]:
        """
        Convenience wrapper — returns only balloon detections.

        Parameters
        ----------
        frame : np.ndarray
            Input BGR frame.

        Returns
        -------
        List[Dict[str, Any]]
            Subset of :meth:`detect` results filtered to ``class_id == BALLOON``.
        """
        return [d for d in self.detect(frame) if d["class_id"] == TargetClass.BALLOON]

    def detect_drones(self, frame: np.ndarray) -> List[Dict[str, Any]]:
        """
        Convenience wrapper — returns only drone detections.

        Parameters
        ----------
        frame : np.ndarray
            Input BGR frame.

        Returns
        -------
        List[Dict[str, Any]]
            Subset of :meth:`detect` results filtered to ``class_id == DRONE``.
        """
        return [d for d in self.detect(frame) if d["class_id"] == TargetClass.DRONE]


# ---------------------------------------------------------------------------
# Backwards-compatibility alias
# ---------------------------------------------------------------------------
# Earlier code imported BalloonDetector by name.  Keep the alias so existing
# imports don't break while the codebase is migrated to ObjectDetector.
BalloonDetector = ObjectDetector
