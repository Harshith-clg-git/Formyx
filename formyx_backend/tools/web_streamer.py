import os
import sys
import time
import cv2
import numpy as np
from flask import Flask, Response, render_template_string

# Add formyx_backend to python path
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from depth.realsense_interface import RealSenseInterface
from perception.detector import ObjectDetector

app = Flask(__name__)

# Initialize interface and detector
iface = RealSenseInterface(use_mock=False)
iface.start()
detector = ObjectDetector()

def generate_frames():
    while True:
        try:
            frames = iface.get_frames()
            if frames is None:
                time.sleep(0.03)
                continue
            color_image, depth_image = frames
            
            # Run detection
            balloons = detector.detect_balloons(color_image)
            drones = detector.detect_drones(color_image)
            
            annotated = color_image.copy()
            
            # Draw balloons (class 0) - Red
            for det in balloons:
                xmin, ymin, xmax, ymax = map(int, det["bbox"])
                cx, cy = map(int, det["center"])
                conf = det["confidence"]
                
                # Query depth
                dist = iface.get_distance_at_pixel(depth_image, cx, cy)
                dist_str = f"{dist:.2f}m" if dist is not None else "No Depth"
                
                cv2.rectangle(annotated, (xmin, ymin), (xmax, ymax), (0, 0, 255), 2)
                label = f"Balloon {conf:.2f} {dist_str}"
                (w_label, h_label), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
                cv2.rectangle(annotated, (xmin, ymin - 20), (xmin + w_label, ymin), (0, 0, 255), -1)
                cv2.putText(annotated, label, (xmin, ymin - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
                cv2.circle(annotated, (cx, cy), 4, (0, 255, 255), -1)
                
            # Draw drones (class 1) - Green
            for det in drones:
                xmin, ymin, xmax, ymax = map(int, det["bbox"])
                cx, cy = map(int, det["center"])
                conf = det["confidence"]
                
                # Query depth
                dist = iface.get_distance_at_pixel(depth_image, cx, cy)
                dist_str = f"{dist:.2f}m" if dist is not None else "No Depth"
                
                cv2.rectangle(annotated, (xmin, ymin), (xmax, ymax), (0, 255, 0), 2)
                label = f"Drone {conf:.2f} {dist_str}"
                (w_label, h_label), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
                cv2.rectangle(annotated, (xmin, ymin - 20), (xmin + w_label, ymin), (0, 255, 0), -1)
                cv2.putText(annotated, label, (xmin, ymin - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
                cv2.circle(annotated, (cx, cy), 4, (0, 255, 255), -1)
                
            # Encode frame
            ret, buffer = cv2.imencode('.jpg', annotated)
            if not ret:
                continue
            frame_bytes = buffer.tobytes()
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')
            time.sleep(0.05)  # cap frame rate to prevent cpu overload
        except Exception as e:
            print("Error in generator:", e)
            time.sleep(0.1)

@app.route('/video_feed')
def video_feed():
    return Response(generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/')
def index():
    html = """
    <!DOCTYPE html>
    <html lang="en">
    <head>
        <meta charset="UTF-8">
        <meta name="viewport" content="width=device-width, initial-scale=1.0">
        <title>Formyx Drone Perception Pipeline</title>
        <link href="https://fonts.googleapis.com/css2?family=Outfit:wght@300;400;600;800&display=swap" rel="stylesheet">
        <style>
            body {
                margin: 0;
                padding: 0;
                background-color: #0b0f19;
                color: #f3f4f6;
                font-family: 'Outfit', sans-serif;
                display: flex;
                flex-direction: column;
                align-items: center;
                justify-content: center;
                min-height: 100vh;
                overflow: hidden;
            }
            .container {
                text-align: center;
                background: rgba(17, 24, 39, 0.8);
                backdrop-filter: blur(10px);
                border: 1px solid rgba(255, 255, 255, 0.1);
                border-radius: 24px;
                padding: 30px;
                box-shadow: 0 20px 50px rgba(0, 0, 0, 0.5);
                max-width: 90%;
            }
            h1 {
                font-size: 2.5rem;
                font-weight: 800;
                margin: 0 0 10px 0;
                background: linear-gradient(135deg, #10b981, #3b82f6);
                -webkit-background-clip: text;
                -webkit-text-fill-color: transparent;
                text-transform: uppercase;
                letter-spacing: 2px;
            }
            p.subtitle {
                font-size: 1rem;
                color: #9ca3af;
                margin-bottom: 25px;
                font-weight: 300;
            }
            .video-wrapper {
                position: relative;
                border-radius: 16px;
                overflow: hidden;
                border: 4px solid #1f2937;
                box-shadow: 0 10px 30px rgba(16, 185, 129, 0.2);
                background-color: #000;
                display: inline-block;
            }
            img {
                display: block;
                max-width: 100%;
                height: auto;
                border-radius: 12px;
            }
            .status-badge {
                display: inline-flex;
                align-items: center;
                gap: 8px;
                background: rgba(16, 185, 129, 0.1);
                border: 1px solid rgba(16, 185, 129, 0.3);
                color: #10b981;
                padding: 6px 16px;
                border-radius: 50px;
                font-size: 0.875rem;
                font-weight: 600;
                margin-top: 20px;
                text-transform: uppercase;
                letter-spacing: 1px;
            }
            .status-dot {
                width: 8px;
                height: 8px;
                background-color: #10b981;
                border-radius: 50%;
                animation: pulse 1.5s infinite;
            }
            @keyframes pulse {
                0% { transform: scale(0.9); opacity: 0.6; }
                50% { transform: scale(1.2); opacity: 1; }
                100% { transform: scale(0.9); opacity: 0.6; }
            }
        </style>
    </head>
    <body>
        <div class="container">
            <h1>Formyx RealSense Perception</h1>
            <p class="subtitle">Raspberry Pi 5 CPU-Optimized Dual-Class YOLO Stream</p>
            <div class="video-wrapper">
                <img src="/video_feed" alt="Video stream" width="640" height="480">
            </div>
            <div>
                <span class="status-badge">
                    <span class="status-dot"></span>
                    Live Feed Active
                </span>
            </div>
        </div>
    </body>
    </html>
    """
    return render_template_string(html)

if __name__ == '__main__':
    try:
        app.run(host='0.0.0.0', port=5000, threaded=True)
    finally:
        iface.stop()
