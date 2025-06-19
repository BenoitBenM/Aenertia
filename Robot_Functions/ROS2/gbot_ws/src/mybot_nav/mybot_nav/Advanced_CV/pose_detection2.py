
import os
import time
import cv2

from picamera2 import Picamera2
import mediapipe as mp
from mediapipe.tasks.python import vision
from mediapipe.tasks.python.vision import PoseLandmarker, PoseLandmarkerOptions
from mediapipe.tasks.python.core.base_options import BaseOptions

# 1) Load MediaPipe model bundle
MODEL_PATH = os.path.expanduser("~/pose_landmarker_lite.task")
if not os.path.exists(MODEL_PATH):
    raise FileNotFoundError(f"MediaPipe model not found at {MODEL_PATH}")

base_opts = BaseOptions(model_asset_path=MODEL_PATH)
pose_opts = PoseLandmarkerOptions(
    base_options=base_opts,
    running_mode=vision.RunningMode.VIDEO,
    min_pose_detection_confidence=0.5
)
landmarker = PoseLandmarker.create_from_options(pose_opts)

# 2) Initialize PiCamera2
cam = Picamera2()
# configure at 1280×960 (RGB888)
cam.configure(cam.create_video_configuration(
    main={"size": (1280, 960), "format": "RGB888"}
))
cam.start()
time.sleep(0.5)  # allow auto exposure/WB to settle

def gen_frames():
    """
    MJPEG generator for Flask:
     - capture frame
     - run pose detection
     - draw lower-body bounding box
     - JPEG-encode and yield as multipart
    """
    w, h = cam.camera_config["main"]["size"]
    mp_fmt = mp.ImageFormat.SRGB

    try:
        while True:
            # Capture a frame
            frame_bgr = cam.capture_array()  # BGR numpy array

            # Convert for MediaPipe
            frame_rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
            mp_image = mp.Image(image_format=mp_fmt, data=frame_rgb)
            timestamp = int(time.time() * 1000)

            # Run inference
            try:
                result = landmarker.detect_for_video(mp_image, timestamp)
            except Exception as e:
                print("Inference error:", e)
            else:
                # Draw lower-body box if found
                if result.pose_landmarks:
                    lm = result.pose_landmarks[0]
                    idxs = [11, 12, 13, 14, 15, 16]  # hips/knees/ankles
                    pts = [
                        (int(lm[i].x * w), int(lm[i].y * h))
                        for i in idxs
                        if 0 <= lm[i].x <= 1 and 0 <= lm[i].y <= 1
                    ]
                    if pts:
                        xs, ys = zip(*pts)
                        cv2.rectangle(
                            frame_bgr,
                            (min(xs), min(ys)),
                            (max(xs), max(ys)),
                            (0, 255, 0),
                            2
                        )

            # JPEG-encode
            ret, buf = cv2.imencode('.jpg', frame_bgr)
            if not ret:
                continue

            # Yield multipart frame
            yield (
                b'--frame\r\n'
                b'Content-Type: image/jpeg\r\n\r\n' +
                buf.tobytes() +
                b'\r\n'
            )
    finally:
        # Cleanup
        landmarker.close()
        cam.stop()

