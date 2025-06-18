import sys
import os
import time
import cv2
from ultralytics import YOLO
from picamera2 import Picamera2  # Picamera2 logic restored

# ensure our globals module is importable
sys.path.append(os.path.dirname(os.path.dirname(__file__)))
import global_var as gv

def yolo_person_tracker():
    # 1) Load a YOLOv8 model (pretrained on COCO person class)
    model = YOLO('yolov8n.pt')  # you can substitute yolov8n.pt with a larger model if needed

    # 2) Initialize PiCamera2
    n = 2  # prevents PiCamera digital zoom by using a higher resolution capture
    cam = Picamera2()
    cam.configure(cam.create_video_configuration(
        main={"size": (640*n, 480*n), "format": "RGB888"}
    ))
    cam.start()
    time.sleep(1)  # allow auto-exposure / white balance to stabilize

    try:
        while True:
            # 3) Capture frame
            frame = cam.capture_array()  # returns BGR by default
            h, w = frame.shape[:2]

            # 4) Run YOLO inference
            results = model(frame)

            # 5) Check for person detections and keep only the largest box
            largest_box = None
            largest_area = 0
            for bbox, cls in zip(results[0].boxes.xyxy, results[0].boxes.cls):
                if int(cls) == 0:  # COCO class 0 == person
                    x1, y1, x2, y2 = map(int, bbox)
                    area = (x2 - x1) * (y2 - y1)
                    if area > largest_area:
                        largest_area = area
                        largest_box = (x1, y1, x2, y2)

            if largest_box:
                gv.HumanDetected = True
                x1, y1, x2, y2 = largest_box

                # Draw bounding box
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)

                # 6) Compute horizontal offset from center
                center_x = (x1 + x2) // 2
                gv.offset = center_x - (w // 2)
            else:
                gv.HumanDetected = False
                gv.offset = 0

            # 7) Display
            cv2.imshow("YOLO Person Tracker", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    finally:
        # 8) Cleanup
        cam.stop()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    yolo_person_tracker()
