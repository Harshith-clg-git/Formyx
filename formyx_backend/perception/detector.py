"""
formyx_backend/perception/detector.py
-------------------------------------
Loads the YOLOv8 model and performs real-time object detection
to locate balloon targets in raw video frames.
"""

from __future__ import annotations

import logging
from pathlib import Path
from typing import Any, Dict, List, Tuple

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


class BalloonDetector:
    """
    YOLOv8-based balloon detector optimized for companion computer (CPU) execution.
    """

    def __init__(self, model_path: str | None = None) -> None:
        if not HAS_ULTRALYTICS:
            raise ImportError(
                "The 'ultralytics' package is required for BalloonDetector. "
                "Install it with: pip install ultralytics"
            )

        cfg_path = model_path or get("perception", "model_path", "models/balloon_detector.pt")
        self.model_path = Path(cfg_path)
        
        self.conf_threshold: float = get("perception", "confidence_threshold", 0.60)
        self.resolution: Tuple[int, int] = tuple(get("perception", "inference_resolution", [320, 320]))
        self.class_id: int = get("perception", "target_class_id", 0)
        
        if not self.model_path.exists():
            raise FileNotFoundError(
                f"YOLO model weights file not found: {self.model_path.absolute()}\n"
                "Please place the trained balloon_detector.pt file in the models/ directory."
            )
            
        log.info("Loading YOLO model from %s...", self.model_path)
        self.model = YOLO(str(self.model_path))
        log.info("YOLO model loaded successfully on CPU.")

    def detect(self, frame: np.ndarray) -> List[Dict[str, Any]]:
        """
        Run inference on a single BGR frame.

        Parameters
        ----------
        frame : np.ndarray
            Input image in OpenCV BGR format.

        Returns
        -------
        List[Dict[str, Any]]
            List of detected balloons. Each detection is a dict with keys:
            * 'bbox': Tuple[float, float, float, float] -> (xmin, ymin, xmax, ymax) in pixels
            * 'center': Tuple[float, float] -> (x_center, y_center) in pixels
            * 'confidence': float -> detection confidence [0.0, 1.0]
            * 'class_id': int -> class index
        """
        if frame is None or frame.size == 0:
            return []

        # Run inference using CPU-optimized parameters
        results = self.model(
            frame,
            imgsz=self.resolution,
            conf=self.conf_threshold,
            device="cpu",
            verbose=False,
        )

        detections = []
        for result in results:
            if result.boxes is None:
                continue
            
            # Move data to CPU numpy for quick iteration
            xyxys = result.boxes.xyxy.cpu().numpy()
            confs = result.boxes.conf.cpu().numpy()
            clss = result.boxes.cls.cpu().numpy()

            for bbox, conf, cls in zip(xyxys, confs, clss):
                cls_val = int(cls)
                if cls_val != self.class_id:
                    continue

                xmin, ymin, xmax, ymax = map(float, bbox)
                x_center = (xmin + xmax) / 2.0
                y_center = (ymin + ymax) / 2.0

                detections.append({
                    "bbox": (xmin, ymin, xmax, ymax),
                    "center": (x_center, y_center),
                    "confidence": float(conf),
                    "class_id": cls_val
                })

        return detections
