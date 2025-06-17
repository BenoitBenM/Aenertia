import sys
import os
sys.path.append(os.path.dirname(os.path.dirname(__file__)))
import time
import threading

# from picamera2 import Picamera2
import cv2

# Tasks‐API imports
import mediapipe as mp
from mediapipe.tasks.python.core.base_options import BaseOptions
from mediapipe.tasks.python import vision
from mediapipe.tasks.python.vision import (
    PoseLandmarker,
    PoseLandmarkerOptions,
)

import global_var as gv


latest_frame = None
frame_lock = threading.Lock()


def pose_detection():
    global latest_frame

    # 1) Load your .task bundle
    model_path = os.path.expanduser("~/pose_landmarker_lite.task")
    if not os.path.exists(model_path):
        raise FileNotFoundError(f"Model file not found: {model_path}")
    base_options = BaseOptions(model_asset_path=model_path)

    options = PoseLandmarkerOptions(
        base_options=base_options,
        running_mode=vision.RunningMode.VIDEO,        # synchronous VIDEO mode
        min_pose_detection_confidence=0.5,
    )
    landmarker = PoseLandmarker.create_from_options(options)

    # 2) Start the PiCamera2 at 640×480 (full FOV)
#    n = 2 # prevents PiCamera zoom
#    cam = Picamera2()
#    cam.configure(cam.create_video_configuration(
#        main={"size": (640*n, 480*n), "format": "RGB888"}
#    ))
#    cam.start()
#    time.sleep(1)  # let auto-exposure / white balance settle

#    # Debug: confirm the actual configuration
#    cfg = cam.camera_config["main"]["size"]
#    # print(f"Camera configured at: {cfg[0]}×{cfg[1]}")

    cam = cv2.VideoCapture(0)  # 0 = /dev/video0, change if needed
    cam.set(cv2.CAP_PROP_FPS, 10)
    cam.set(cv2.CAP_PROP_FRAME_WIDTH, 1000)
    cam.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    time.sleep(2)
    
    print("Camera Started")


    mp_image_format = mp.ImageFormat.SRGB

    while True:
        # 3) Capture & wrap frame
        ret, frame_bgr = cam.read()
        if not ret:
            print("⚠️ Failed to grab frame from USB camera")
            continue
	
        frame_rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
        mp_image = mp.Image(
            image_format=mp_image_format,
            data=frame_rgb
        )
    
        timestamp_ms = int(time.time() * 1000)
        try:
            result = landmarker.detect_for_video(mp_image, timestamp_ms)
        except Exception as e:
            print("⚠️ Inference error:", e)
            continue

            # 5) Pull out only hips/knees/ankles from the FIRST pose detection ---
            # Indices in the 33-landmark list:
            # 11=LEFT_HIP, 12=RIGHT_HIP, 13=LEFT_KNEE, 14=RIGHT_KNEE,
            # 15=LEFT_ANKLE,16=RIGHT_ANKLE
        lower_idxs = [11, 12, 13, 14, 15, 16]
        pts = []
        h, w = frame_bgr.shape[:2]

        gv.HumanDetected
        gv.offset
        gv.offset = 0

        if result.pose_landmarks:
            gv.HumanDetected = True

            first_pose: list = result.pose_landmarks[0]
            for idx in lower_idxs:
                kp = first_pose[idx]
                if 0 <= kp.x <= 1 and 0 <= kp.y <= 1:
                    x_px = int(kp.x * w)
                    y_px = int(kp.y * h)
                    pts.append((x_px, y_px))
        else:
            # print("No results")
            gv.HumanDetected = False
        # 6) Draw bounding box & compute offset
        if pts:
            xs, ys = zip(*pts)
            x1, x2 = min(xs), max(xs)
            y1, y2 = min(ys), max(ys)
            cv2.rectangle(frame_bgr, (x1, y1), (x2, y2), (0, 255, 0), 2)
            gv.offset = ((x1 + x2) // 2) - (w // 2)
            if gv.offset == None:
                print("stop")
        # print(f"HumanDetected = {HumanDetected}, Offset = {offset}")

        with frame_lock:
            latest_frame = frame_bgr.copy()
            
#    #     Display for debugging
#        cv2.imshow("Lower-Body Follow", frame_bgr)
#        if cv2.waitKey(1) & 0xFF == ord("q"):
#            break


#        # 8) Cleanup 
#    landmarker.close()
#    cam.release()
#    cv2.destroyAllWindows()



def send_frame():
    global latest_frame

    while True:
        with frame_lock:
            frame = None if latest_frame is None else latest_frame.copy()
        
        if frame is None:
            time.sleep(0.2)
            continue

        # JPEG-encode
        ok, buf = cv2.imencode('.jpg', frame)
        if not ok:
            continue

        # Yield multipart frame
        yield (
            b'--frame\r\n'
            b'Content-Type: image/jpeg\r\n\r\n' +
            buf.tobytes() +
            b'\r\n'
        )
        # finally:
    #     # 8) Cleanup 
    #     landmarker.close()
    #     cam.stop()
    #     cv2.destroyAllWindows()


if __name__ == "__main__":
    pose_detection()
