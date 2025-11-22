from picamera2 import Picamera2, Preview
from shared_state import picam2_state, vehicle_data
import time
import requests
import BytesIO
import threading
import json

DELAY = 0.25
GCS_URL = "http://192.168.1.64:80"
stop_camera_thread = threading.Event()

def continuously_capture_images():
    if picam2_state["picam2"] is None:
        cam = Picamera2()
        config = cam.create_still_configuration()
        cam.configure(config)
        cam.start_preview(Preview.NULL)
        time.sleep(1)
        picam2_state["picam2"] = cam
    else:
        cam = picam2_state["picam2"]

    cam.start()

    try:
        while picam2_state["is_camera_on"] and not stop_camera_thread.is_set():
            picam2_state["image_number"] += 1
            num = picam2_state["image_number"]

            delay_left = DELAY - take_picture(num, cam)
            if delay_left > 0:
                time.sleep(delay_left)

    finally:
        cam.stop()


def take_picture(number, cam):
    start = time.time()
    buf = BytesIO()
    meta = json.dumps(vehicle_data)
    img = cam.capture_image('main')
    img.save(buf, format='JPEG')
    buf.seek(0)

    requests.post(f"{GCS_URL}/submit",
        files={"file": (f"{number:05d}.jpg", buf, "image/jpg")}
    )

    json_buf = BytesIO(meta.encode("utf-8"))
    requests.post(f"{GCS_URL}/submit",
        files={"file": (f"{number:05d}.json", json_buf, "application/json")}
    )

    return time.time() - start
