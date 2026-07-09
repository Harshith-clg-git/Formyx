"""
formyx_backend/camcv.py
-----------------------
Standalone dual-class detection viewer + recorder.

Displays a live OpenCV window with balloon (red) and drone (green) detection
overlays, FPS counter, and Kalman-tracker crosshair.
Every session is saved as a timestamped .mp4 in the camrec/ folder.

NOTE: This version uses a standard UVC webcam via OpenCV VideoCapture
      (USB 2.0 compatible, e.g. /dev/video0).  The Intel RealSense D435i
      is NOT required — depth estimates are therefore unavailable and will
      display as "N/A".

Usage
-----
    cd formyx_backend
    DISPLAY=:0 python3 camcv.py

Options
-------
    --device INT        OpenCV camera device index (default: 0  → /dev/video0)
    --width INT         Capture width  in pixels (default: 640)
    --height INT        Capture height in pixels (default: 480)
    --conf FLOAT        Detection confidence threshold (default: 0.25)
    --interval INT      Run YOLO every N frames (default: 2, raise to 3 for
                        slower Pi boards)
    --no-tracker        Disable the Kalman tracker overlay
    --no-record         Stream only, do not save video
    --no-display        Headless mode — record without showing a window
    --output-dir PATH   Override recording directory (default: camrec)
    --segment-mins INT  Split recording into segments of this length in minutes.
                        When the segment ends the current .mp4 is saved and a
                        new one starts automatically. (default: 3, 0 = disabled)
"""

from __future__ import annotations

import argparse
import logging
import os
import signal
import sys
import time
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np

from perception.detector import ObjectDetector
from tracking.target_tracker import TargetTracker

# ---------------------------------------------------------------------------
# Logging
# ---------------------------------------------------------------------------
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)-8s] %(name)s — %(message)s",
    datefmt="%H:%M:%S",
)
log = logging.getLogger("camcv")

# ---------------------------------------------------------------------------
# Colour palette (BGR)
# ---------------------------------------------------------------------------
CLR_BALLOON  = (0,   0, 255)   # Red    — balloon
CLR_DRONE    = (0, 220,   0)   # Green  — drone
CLR_KF       = (0, 255, 255)   # Yellow — Kalman projection
CLR_FPS      = (0, 255,   0)   # Green  — HUD text
CLR_WHITE    = (255, 255, 255)
CLR_BLACK    = (0,   0,   0)

# ---------------------------------------------------------------------------
# CLI args
# ---------------------------------------------------------------------------

def _parse() -> argparse.Namespace:
    p = argparse.ArgumentParser(
        prog="camcv",
        description="Formyx dual-class detection viewer + recorder (USB 2.0 UVC webcam)",
    )
    p.add_argument("--device",     type=int,   default=0,
                   help="OpenCV camera device index (default 0 → /dev/video0)")
    p.add_argument("--width",      type=int,   default=640,
                   help="Capture width in pixels (default 640)")
    p.add_argument("--height",     type=int,   default=480,
                   help="Capture height in pixels (default 480)")
    p.add_argument("--conf",       type=float, default=0.25,
                   help="Detection confidence threshold (default 0.25)")
    p.add_argument("--interval",   type=int,   default=2,
                   help="Run YOLO every N frames (default 2)")
    p.add_argument("--no-tracker", action="store_true",
                   help="Disable the Kalman tracker overlay")
    p.add_argument("--no-record",  action="store_true",
                   help="Do not save a video recording")
    p.add_argument("--no-display", action="store_true",
                   help="Disable the OpenCV camera window (headless mode)")
    p.add_argument("--output-dir", default="camrec",
                   help="Recording output directory (default: camrec/)")
    p.add_argument("--segment-mins", type=int, default=3,
                   help="Split recording every N minutes (default 3, 0 = disabled)")
    return p.parse_args()

# ---------------------------------------------------------------------------
# Drawing helpers
# ---------------------------------------------------------------------------

def _draw_box(img: np.ndarray, xmin: int, ymin: int, xmax: int, ymax: int,
              color: tuple, label: str) -> None:
    """Draw a bounding box with a filled label tab above it."""
    cv2.rectangle(img, (xmin, ymin), (xmax, ymax), color, 2)
    (tw, _), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.48, 1)
    cv2.rectangle(img, (xmin, ymin - 20), (xmin + tw + 4, ymin), color, -1)
    cv2.putText(img, label, (xmin + 2, ymin - 5),
                cv2.FONT_HERSHEY_SIMPLEX, 0.48, CLR_WHITE, 1, cv2.LINE_AA)


def _draw_hud(img: np.ndarray, fps: float, interval: int,
              n_balloon: int, n_drone: int, tracking: bool) -> None:
    """Render the on-screen HUD (top-left corner)."""
    lines = [
        f"FPS: {fps:.1f}  |  interval: {interval}x",
        f"Balloons: {n_balloon}   Drones: {n_drone}",
        f"Tracker: {'LOCKED' if tracking else 'SEARCHING'}",
    ]
    y = 28
    for line in lines:
        # Drop-shadow for readability
        cv2.putText(img, line, (11, y + 1),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.65, CLR_BLACK, 3, cv2.LINE_AA)
        cv2.putText(img, line, (10, y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.65, CLR_FPS, 2, cv2.LINE_AA)
        y += 26


def _draw_kalman_2d(img: np.ndarray, tracker: TargetTracker) -> None:
    """
    Overlay the Kalman tracker prediction.
    With a plain webcam we have no depth — we treat the first two state
    components (x, y) as pixel offsets from frame centre and render a cross.
    The state is in camera-frame metres so we use a fixed nominal depth to
    project back to pixels (intrinsics approximated from webcam field of view).
    """
    state = tracker.get_state()
    if state is None:
        return
    px, py, pz, vx, vy, vz = state

    # Approximate projection: assume ~60° HFOV at 640px → fx ≈ 554
    fx = fy = 554.0
    cx_px = img.shape[1] / 2
    cy_px = img.shape[0] / 2

    # Use pz if it looks like a plausible depth; else fall back to 1 m nominal
    depth = max(pz, 0.5) if abs(pz) > 0.01 else 1.0
    proj_x = int(px * fx / depth + cx_px)
    proj_y = int(py * fy / depth + cy_px)

    h, w = img.shape[:2]
    if not (0 <= proj_x < w and 0 <= proj_y < h):
        return

    cv2.drawMarker(img, (proj_x, proj_y), CLR_KF,
                   cv2.MARKER_CROSS, markerSize=22, thickness=2)
    label = f"KF  v=({vx:+.1f},{vy:+.1f})"
    (tw, _), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.45, 1)
    bx = min(proj_x + 12, w - tw - 4)
    cv2.rectangle(img, (bx - 2, proj_y - 14), (bx + tw + 2, proj_y + 2),
                  CLR_BLACK, -1)
    cv2.putText(img, label, (bx, proj_y),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, CLR_KF, 1, cv2.LINE_AA)


# ---------------------------------------------------------------------------
# Video writer setup
# ---------------------------------------------------------------------------

def _make_writer(output_dir: str, width: int, height: int) -> tuple:
    Path(output_dir).mkdir(parents=True, exist_ok=True)
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    out_path = os.path.join(output_dir, f"session_{stamp}.mp4")
    fourcc = cv2.VideoWriter_fourcc(*"mp4v")
    writer = cv2.VideoWriter(out_path, fourcc, 15.0, (width, height))
    if not writer.isOpened():
        log.warning("VideoWriter failed to open — recording disabled.")
        return None, out_path
    log.info("Recording to: %s", out_path)
    return writer, out_path


# ---------------------------------------------------------------------------
# Signal handler
# ---------------------------------------------------------------------------
_shutdown_requested = False

def _handle_signal(signum, frame):  # noqa: ANN001
    global _shutdown_requested
    logging.getLogger("camcv").warning(
        "Signal %d received — initiating clean shutdown.", signum)
    _shutdown_requested = True


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main() -> int:
    args = _parse()
    show_display = not args.no_display

    signal.signal(signal.SIGINT,  _handle_signal)
    signal.signal(signal.SIGTERM, _handle_signal)

    # ------------------------------------------------------------------
    # 1. Open UVC webcam via OpenCV
    # ------------------------------------------------------------------
    log.info("Opening USB 2.0 UVC webcam at /dev/video%d …", args.device)
    cap = cv2.VideoCapture(args.device, cv2.CAP_V4L2)
    if not cap.isOpened():
        log.error("Could not open /dev/video%d. Check that the camera is connected.", args.device)
        return 1

    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  args.width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
    cap.set(cv2.CAP_PROP_FPS, 30)
    # Prefer MJPG codec — allows full 30fps on USB 2.0 bandwidth
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))

    W = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    H = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    log.info("Camera opened — resolution %dx%d", W, H)

    # ------------------------------------------------------------------
    # 2. Detector
    # ------------------------------------------------------------------
    log.info("Loading dual-class YOLO detector (balloon + drone)…")
    detector = ObjectDetector()
    log.info("Detector ready (ONNX=%s).", detector.use_onnx)

    # ------------------------------------------------------------------
    # 3. Tracker (optional)
    # ------------------------------------------------------------------
    tracker = TargetTracker() if not args.no_tracker else None

    # ------------------------------------------------------------------
    # 4. Video writer
    # ------------------------------------------------------------------
    writer, rec_path = None, None
    if not args.no_record:
        writer, rec_path = _make_writer(args.output_dir, W, H)

    # ------------------------------------------------------------------
    # 5. Display window  (skipped when --no-display)
    # ------------------------------------------------------------------
    WINDOW = "Formyx Detection — Q / Esc to quit"
    if show_display:
        cv2.namedWindow(WINDOW, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(WINDOW, W, H)
        log.info("Press Q or Esc in the window to stop.")
    else:
        log.info("Running headless (no display). Press Ctrl-C to stop.")

    # ------------------------------------------------------------------
    # 6. Main loop
    # ------------------------------------------------------------------
    loop_idx     = 0
    fps_display  = 0.0
    fps_count    = 0
    fps_t0       = time.monotonic()
    last_dets: list = []
    segment_secs = args.segment_mins * 60 if args.segment_mins > 0 else None
    seg_start_t  = time.monotonic()   # tracks when current segment started

    try:
        while not _shutdown_requested:
            t_start = time.monotonic()
            loop_idx += 1
            run_yolo = (loop_idx % args.interval == 0)

            # --- grab frame -------------------------------------------
            ret, color = cap.read()
            if not ret or color is None:
                log.warning("Failed to grab frame — retrying…")
                time.sleep(0.05)
                continue

            # --- YOLO inference ---------------------------------------
            n_balloon = n_drone = 0
            if run_yolo:
                raw = detector.detect(color)
                last_dets = []
                for det in raw:
                    xmin, ymin, xmax, ymax = map(int, det["bbox"])
                    cx = int((xmin + xmax) / 2)
                    cy = int((ymin + ymax) / 2)
                    last_dets.append({
                        **det,
                        "cx": cx, "cy": cy,
                        "dist": None,  # No depth sensor available
                    })
                    # Feed tracker with normalised pixel coords (no depth)
                    # Map pixel to nominal camera-frame: assume 1 m depth
                    if tracker:
                        fx = fy = 554.0
                        cx_c = W / 2
                        cy_c = H / 2
                        rx = (cx - cx_c) / fx
                        ry = (cy - cy_c) / fy
                        rz = 1.0  # nominal 1 m (no depth)
                        tracker.predict(dt=1.0 / 30.0)
                        tracker.update((rx, ry, rz))
            elif tracker:
                tracker.predict(dt=1.0 / 30.0)

            # --- FPS counter ------------------------------------------
            fps_count += 1
            now = time.monotonic()
            if now - fps_t0 >= 1.0:
                fps_display = fps_count / (now - fps_t0)
                fps_count   = 0
                fps_t0      = now
                n_b = sum(1 for d in last_dets if d["class_id"] == 0)
                n_d = sum(1 for d in last_dets if d["class_id"] == 1)
                log.info(
                    "FPS=%.1f  Balloons=%d  Drones=%d  Tracker=%s",
                    fps_display, n_b, n_d,
                    "LOCKED" if (tracker and tracker.is_initialized) else "SEARCHING",
                )

            # --- Annotate frame ---------------------------------------
            need_annotate = (writer and writer.isOpened()) or show_display
            if need_annotate:
                vis = color.copy()
                n_b = n_d = 0
                for det in last_dets:
                    xmin, ymin, xmax, ymax = map(int, det["bbox"])
                    cx, cy  = det["cx"], det["cy"]
                    conf    = det["confidence"]
                    cls_id  = det["class_id"]
                    color_box = CLR_BALLOON if cls_id == 0 else CLR_DRONE
                    name      = "Balloon"   if cls_id == 0 else "Drone"
                    if cls_id == 0:
                        n_b += 1
                    else:
                        n_d += 1
                    _draw_box(vis, xmin, ymin, xmax, ymax, color_box,
                              f"{name}  {conf:.2f}  N/A")
                    cv2.circle(vis, (cx, cy), 4, CLR_KF, -1)

                if tracker:
                    _draw_kalman_2d(vis, tracker)

                _draw_hud(vis, fps_display, args.interval, n_b, n_d,
                          tracker.is_initialized if tracker else False)

                # --- Write to recording ---------------------------
                if writer and writer.isOpened():
                    writer.write(vis)

            # ----------------------------------------------------------
            # Segment rotation — close current file, open a new one
            # ----------------------------------------------------------
            if segment_secs and writer and (time.monotonic() - seg_start_t >= segment_secs):
                log.info("Segment limit reached — rotating video file…")
                writer.release()
                log.info("Segment saved → %s", rec_path)
                writer, rec_path = _make_writer(args.output_dir, W, H)
                seg_start_t = time.monotonic()

            # --- Show window (only when not headless) ---------
            if show_display:
                cv2.imshow(WINDOW, vis)
                key = cv2.waitKey(1) & 0xFF
                if key in (ord("q"), 27):
                    break
                try:
                    if cv2.getWindowProperty(WINDOW, cv2.WND_PROP_VISIBLE) < 1:
                        break
                except cv2.error:
                    break

    except KeyboardInterrupt:
        log.info("Interrupted.")
    finally:
        log.info("Shutting down…")
        if writer and writer.isOpened():
            writer.release()
            log.info("Video saved → %s", rec_path)
        cap.release()
        cv2.destroyAllWindows()
        for _ in range(5):
            cv2.waitKey(1)
        log.info("Done.")

    return 0


if __name__ == "__main__":
    sys.exit(main())
