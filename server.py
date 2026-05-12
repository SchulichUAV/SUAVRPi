# Buit to sync with GCS2025 in Schulich UAV repository/organization
# Built for Raspberry Pi 5 (Linux OS)

from flask import Flask, jsonify, request
from flask_cors import CORS
from adafruit_servokit import ServoKit
import cv2
import threading
import queue
import socket
import ctypes
import fcntl
import json
import sys
import time
import os

import modules.AutopilotDevelopment.General.Operations.initialize as initialize
import modules.AutopilotDevelopment.General.Operations.mode as autopilot_mode
import modules.AutopilotDevelopment.General.Operations.mission as mission
import modules.AutopilotDevelopment.Plane.Operations.altitude as autopilot_altitude
import modules.payload as payload


VEHICLE_PORT = "udp:127.0.0.1:5006"
PPS_DEVICE = "/dev/pps0"  # kernel PPS driver via dtoverlay=pps-gpio,gpiopin=4
CAMERA_DEVICE = "/dev/video0"

# ── Kernel PPSAPI (linux/pps.h) ──────────────────────────────────────────────
class _PPSKTime(ctypes.Structure):
    _fields_ = [("sec", ctypes.c_int64), ("nsec", ctypes.c_int32), ("flags", ctypes.c_uint32)]

class _PPSKInfo(ctypes.Structure):
    _fields_ = [
        ("assert_sequence", ctypes.c_uint32),
        ("clear_sequence",  ctypes.c_uint32),
        ("assert_tu",       _PPSKTime),
        ("clear_tu",        _PPSKTime),
        ("current_mode",    ctypes.c_int),
    ]

class _PPSFData(ctypes.Structure):
    _fields_ = [("info", _PPSKInfo), ("timeout", _PPSKTime)]

def _ioctl_nr(direction, nr, size):
    return (direction << 30) | (ord('p') << 8) | nr | (size << 16)

# PPS ioctl uses a pointer type in the UAPI header, so the encoded size is
# sizeof(pointer) = 8 on 64-bit, not sizeof(struct pps_fdata).
_PPS_FETCH = _ioctl_nr(3, 0xa4, ctypes.sizeof(ctypes.c_void_p))   # _IOWR

# Configured PPS rate (Hz). Used as a debounce floor against spurious extra
# wake-ups (e.g. line glitches, falling-edge captures when the driver is in
# capture-both mode).
PPS_HZ = 10
_PPS_MIN_INTERVAL_S = 0.5 / PPS_HZ  # accept pulses no more than 2x rate

_last_assert_seq = None
_last_pulse_time = 0.0


def _pps_open() -> int:
    """Open PPS_DEVICE and reset per-session debounce state."""
    global _last_assert_seq, _last_pulse_time
    _last_assert_seq = None
    _last_pulse_time = 0.0
    # Zero the shared ioctl buffer so a stale assert_sequence from a previous
    # session can never seed _last_assert_seq.
    ctypes.memset(ctypes.addressof(_pps_fdata), 0, ctypes.sizeof(_pps_fdata))
    return os.open(PPS_DEVICE, os.O_RDWR)
# ─────────────────────────────────────────────────────────────────────────────
IMAGE_SAVE_DIR = "/home/suavgeopi/images"

camera_connection = None
vehicle_connection = None
image_number = 0

kit = None

app = Flask(__name__)
CORS(app, supports_credentials=True)

vehicle_data = {
    "last_time": 0,
    "lat": 0,
    "lon": 0,
    "rel_alt": 0,
    "alt": 0,
    "roll": 0,
    "pitch": 0,
    "yaw": 0,
    "dlat": 0,
    "dlon": 0,
    "dalt": 0,
    "heading": 0,
    "groundspeed": 0,
    "throttle": 0,
    "climb": 0,
    "num_satellites": 0,
    "position_uncertainty": 0,
    "alt_uncertainty": 0,
    "flight_mode": 0,
    "battery_voltage": 0,
    "battery_current": 0,
    "battery_remaining": 0,
}
vehicle_data_lock = threading.Lock()
_vehicle_keys = list(vehicle_data.keys())[1:]

@app.route('/set_flight_mode', methods=["POST"])
def set_flight_mode():
# Ardupilot docs (for flight modes): https://ardupilot.org/copter/docs/parameters.html
    try:
        json_data = request.json
        mode_id = int(json_data['mode_id'])
        # TODO: Need to determine if we are plane or copter mode when starting the server
        selected_flight_mode = list(autopilot_mode.plane_modes.keys())[mode_id] # list of keys in dictionary, access the key with mode id as index
        print(f'We are in: {selected_flight_mode}')

        # Retrieve mode_id mapping and print the mode name (mode mappings stored in AutopilotDevelopment/General/Operations/mode.py)
        print(autopilot_mode.set_mode(vehicle_connection, mode_id)) # TODO: Need to use set_mode from plane.py or copter.py depending on current vehicle
    except Exception as e:
        return jsonify({'error': "Invalid operation."}), 400

    return jsonify({'message': 'Mode set successfully'}), 200

@app.route('/set_altitude_goto', methods=["POST"])
def set_altitude_goto():
    try:
        json_data = request.json
        altitude = int(json_data['altitude'])
        if altitude < 0:
            print("Error: setting altitude to less than 0")
            return jsonify({'error': "Altitude must be non-negative."}), 400
        autopilot_altitude.set_current_altitude(vehicle_connection, altitude)
        print(f'Setting altitude to: {altitude}')
    except Exception as e:
        return jsonify({'error': "Invalid operation."}), 400

    return jsonify({'message': 'Altitude set successfully'}), 200

@app.route('/payload_drop_mission', methods=["POST"])
def payload_drop_mission():
    try:
        json_data = request.json
        target_lat = float(json_data['latitude'])
        target_lon = float(json_data['longitude'])
        drop_altitude = 20 # 18m = 59ft - lowest allowed altitude is 50ft but want to be low for drops

        payload_object_coord = [target_lat, target_lon, drop_altitude]

        mission.upload_payload_drop_mission(vehicle_connection, payload_object_coord)
        print("Mission successfully uploaded.")
        return jsonify({'message': 'Mission uploaded successfully.'}), 200
            
    except Exception as e:
        print(f"Error uploading mission. Error: {e}")
        return jsonify({'error': "Invalid operation."}), 400
    
@app.route('/monitor_mission_and_drop', methods=["POST"])
def monitor_mission_and_drop():
    try:
        json_data = request.json
        bay = json_data['bay']
        print(f"Initiating background monitor and drop for bay {bay}...")

        def monitor_and_drop():
            try:
                # Wait for the vehicle to reach the target waypoint
                while True:
                    msg = vehicle_connection.recv_match(type='MISSION_CURRENT', blocking=True, timeout=5)
                    if msg is not None and msg.seq == 2:  # Assuming seq 2 is the payload drop waypoint
                        autopilot_mode.set_mode(vehicle_connection, 10)  # Set to AUTO mode
                        break

                # Drop the payload
                mission.check_distance_and_drop(vehicle_connection, bay - 1, kit, vehicle_data)
                print(f"Payload drop completed for bay {bay}")
            except Exception as drop_error:
                print(f"[Background Thread] Error in mission drop: {drop_error}")

        # Start thread
        thread = threading.Thread(target=monitor_and_drop)
        thread.start()

    except Exception as e:
        print(f"[Flask] Error starting monitor and drop thread: {e}")
        return jsonify({'error': "Failed to start background drop task."}), 400

    return jsonify({'message': 'Payload drop initiated in background'}), 200

@app.route('/payload_manual_control', methods=["POST"])
def payload_manual_control():
    json_data = request.get_json()
    payload_id = json_data.get('payload_id')
    payload_open = json_data.get('payload_open')

    if not isinstance(payload_id, int) or not isinstance(payload_open, bool):
        print("Invalid or missing parameters in API request.")
        return jsonify({'error': 'Invalid payload_id or payload_open.'}), 400

    if 1 <= payload_id <= 4:
        try:
            payload.set_servo_state(payload_id - 1, payload_open)
        except Exception as e:
            print("Could not set servo state:", e)
            return jsonify({'error': "Failed to set servo state."}), 400
    else:
        return jsonify({'error': 'Invalid payload_id (must be 1-4).'}), 400

    return jsonify({'servo_status': payload_open, 'message': 'Payload trigger successful'}), 200


@app.route('/payload_release', methods=["POST"])
def payload_release():
    json_data = request.get_json()
    payload_id = json_data.get('bay')

    if not isinstance(payload_id, int) or not (1 <= payload_id <= 4):
        print("Invalid or missing payload_id.")
        return jsonify({'error': 'Invalid bay (must be an integer from 1 to 4).'}), 400

    try:
        payload.payload_release(kit, payload_id - 1, vehicle_data)
    except Exception as e:
        print("Could not release payload:", e)
        return jsonify({'error': "Failed to release payload."}), 400

    return jsonify({'message': 'Payload release successful'}), 200

@app.route('/payload_release_all', methods=["POST"])
def payload_release_all():
    try:
        payload.release_all(kit, vehicle_data)
    except Exception as e:
        print("Could not release all payloads:", e)
        return jsonify({'error': "Failed to release all payloads."}), 400

    return jsonify({'message': 'All payloads released successfully'}), 200

@app.route('/payload_close_all', methods=["POST"])
def payload_close_all():
    try:
        payload.close_all_servos(kit)
    except Exception as e:
        print("Could not close all servos:", e)
        return jsonify({'error': "Failed to close all servos."}), 400

    return jsonify({'message': 'All servos closed successfully'}), 200

@app.route('/payload_open_all', methods=["POST"])
def payload_open_all():
    try:
        payload.open_all_servos(kit)
    except Exception as e:
        print("Could not open all servos:", e)
        return jsonify({'error': "Failed to open all servos."}), 400

    return jsonify({'message': 'All servos opened successfully'}), 200

@app.route('/payload_open', methods=["POST"])
def payload_open():
    json_data = request.get_json()
    payload_id = json_data.get('bay')

    if not isinstance(payload_id, int) or not (1 <= payload_id <= 4):
        print("Invalid or missing payload_id.")
        return jsonify({'error': 'Invalid bay (must be an integer from 1 to 4).'}), 400

    try:
        payload.open_servo(kit, payload_id - 1)
    except Exception as e:
        print("Could not open servo:", e)
        return jsonify({'error': "Failed to open servo."}), 400

    return jsonify({'message': 'Servo opened successfully'}), 200

@app.route('/payload_close', methods=["POST"])
def payload_close():
    json_data = request.get_json()
    payload_id = json_data.get('bay')

    if not isinstance(payload_id, int) or not (1 <= payload_id <= 4):
        print("Invalid or missing payload_id.")
        return jsonify({'error': 'Invalid bay (must be an integer from 1 to 4).'}), 400

    try:
        payload.close_servo(kit, payload_id - 1)
    except Exception as e:
        print("Could not close servo:", e)
        return jsonify({'error': "Failed to close servo."}), 400

    return jsonify({'message': 'Servo closed successfully'}), 200

# Pre-allocated PPS buffer — avoids allocation in the timing-critical path
_pps_buf = bytearray(ctypes.sizeof(_PPSFData))
_pps_fdata = _PPSFData.from_buffer(_pps_buf)

def wait_for_pulse(pps_fd: int) -> float:
    """Blocks until the next genuine PPS assert using the kernel PPSAPI.

    PPS_FETCH blocks internally (wait_event_interruptible_timeout) until a
    new hardware pulse increments assert_sequence, so unlike select() it
    will NOT return on a stale/already-seen event. We additionally:
      * Seed _last_assert_seq from a real pulse (not stale buffer bytes),
        skipping any pulses that occurred while the camera was off.
      * Reject wake-ups whose hardware timestamp is closer than
        _PPS_MIN_INTERVAL_S to the previous accepted pulse — defends
        against double-edge captures and GPIO glitches that would
        otherwise produce a higher capture rate than the configured
        PPS frequency.
      * Warn if the kernel sequence skips, indicating dropped pulses.

    Returns the kernel hardware timestamp of the pulse (seconds, float).
    """
    global _last_assert_seq, _last_pulse_time
    fdata = _pps_fdata
    buf = _pps_buf

    while True:
        # Honor a shutdown request even when PPS is silent. Without this
        # check the thread can be wedged in PPS_FETCH forever (kernel
        # ioctl re-entered every 2 s) and a second toggle ON would race a
        # zombie thread on the shared PPS / camera globals.
        if stop_camera_thread.is_set():
            return 0.0

        fdata.timeout.sec  = 2
        fdata.timeout.nsec = 0
        fdata.timeout.flags = 0
        try:
            fcntl.ioctl(pps_fd, _PPS_FETCH, buf)
        except TimeoutError:
            # No pulse in the last 2 s — likely lost the GPS signal. Loop.
            continue
        except OSError as e:
            print(f"WARN: PPS_FETCH failed: {e}")
            time.sleep(0.05)
            continue

        seq = fdata.info.assert_sequence
        pulse_time = fdata.info.assert_tu.sec + fdata.info.assert_tu.nsec * 1e-9

        # Seed on first real event so we only accept NEW pulses going forward.
        if _last_assert_seq is None:
            _last_assert_seq = seq
            _last_pulse_time = pulse_time
            return pulse_time

        # No new event since last fetch — keep waiting.
        if seq == _last_assert_seq:
            continue

        # Debounce: ignore pulses arriving suspiciously close to the previous one.
        # In capture-both mode this also rejects the falling edge of every legitimate
        # pulse (expected, not an error), so we don't log it.
        delta = pulse_time - _last_pulse_time
        if delta > 0 and delta < _PPS_MIN_INTERVAL_S:
            _last_assert_seq = seq
            continue

        # Detect skipped pulses (lost events between fetches).
        skipped = (seq - _last_assert_seq) & 0xFFFFFFFF
        if skipped > 1:
            print(f"WARN: PPS sequence skipped {skipped - 1} pulse(s) "
                  f"({_last_assert_seq} -> {seq})")

        _last_assert_seq = seq
        _last_pulse_time = pulse_time
        return pulse_time


camera_thread = None
camera_thread_lock = threading.Lock()
stop_camera_thread = threading.Event()

# Queue for offloading disk I/O from the timing-critical capture loop.
# A single, long-lived writer thread is started at process startup so we
# don't leak a new writer thread every time the camera is toggled.
_save_queue: "queue.Queue" = queue.Queue()
_writer_thread_started = False
_writer_thread_lock = threading.Lock()

def _image_writer():
    """Background thread that drains _save_queue and writes images + metadata to disk."""
    while True:
        item = _save_queue.get()
        try:
            if item is None:  # poison pill (only used at process shutdown)
                break
            file_name, frame, metadata = item
            try:
                cv2.imwrite(os.path.join(IMAGE_SAVE_DIR, f'{file_name}.jpg'), frame)
                with open(os.path.join(IMAGE_SAVE_DIR, f'{file_name}.json'), 'w') as f:
                    json.dump(metadata, f)
            except Exception as e:
                print(f"WARN: Failed to save {file_name}: {e}")
        finally:
            _save_queue.task_done()

def _ensure_writer_thread() -> None:
    """Start the singleton image-writer thread on first use."""
    global _writer_thread_started
    with _writer_thread_lock:
        if _writer_thread_started:
            return
        threading.Thread(target=_image_writer, daemon=True, name="image-writer").start()
        _writer_thread_started = True

@app.route("/toggle_camera", methods=["POST"])
def toggle_camera():
    global image_number
    global camera_thread

    try:
        json_data = request.json
        requested_state = bool(json_data["is_camera_on"])
        image_number = int(json_data["image_count"])
    except Exception as e:
        print("Could not interpret toggle_camera payload:", e)
        return jsonify({"error": "Invalid payload"}), 400

    # Serialise toggle handling so off/on bursts cannot leave the camera enabled
    # with no live capture thread (or vice versa).
    with camera_thread_lock:
        if requested_state:
            # Wait for any prior thread to finish before starting a new one.
            # If it refuses to exit, refuse to start a new one rather than
            # leaving two threads racing on the shared camera + PPS globals.
            if camera_thread is not None and camera_thread.is_alive():
                stop_camera_thread.set()
                camera_thread.join(timeout=5)
                if camera_thread.is_alive():
                    print("ERROR: previous camera thread did not exit; refusing to start a new one")
                    return jsonify({
                        "error": "Previous camera thread is still running. "
                                 "Check PPS signal / camera USB and try again.",
                    }), 503
            stop_camera_thread.clear()
            _ensure_writer_thread()
            camera_thread = threading.Thread(
                target=continuously_capture_images, name="camera-capture", daemon=True,
            )
            camera_thread.start()
            print("Starting camera")
        else:
            print("Stopping Camera")
            stop_camera_thread.set()

    return jsonify({"message": "Success!"}), 200

def continuously_capture_images():
    global camera_connection
    global image_number

    if camera_connection is None or not camera_connection.isOpened():
        print("Initializing camera...")
        camera_connection = cv2.VideoCapture(CAMERA_DEVICE, cv2.CAP_V4L2)

        camera_connection.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        camera_connection.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        camera_connection.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        camera_connection.set(cv2.CAP_PROP_FPS, 30)
        camera_connection.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        if not camera_connection.isOpened():
            print("ERROR: Could not open camera")
            return

        # Optional: throw away a few startup frames
        for _ in range(5):
            camera_connection.read()

        print("Camera opened with:")
        print("Width:", camera_connection.get(cv2.CAP_PROP_FRAME_WIDTH))
        print("Height:", camera_connection.get(cv2.CAP_PROP_FRAME_HEIGHT))
        print("FPS:", camera_connection.get(cv2.CAP_PROP_FPS))

    print("Camera ready, waiting for PPS pulses.")
    try:
        pps_fd = _pps_open()
    except OSError as e:
        print(f"ERROR: Could not open {PPS_DEVICE}: {e}")
        if camera_connection is not None:
            camera_connection.release()
            camera_connection = None
        return

    try:
        while not stop_camera_thread.is_set():
            pulse_time = wait_for_pulse(pps_fd)
            if stop_camera_thread.is_set():
                break
            # Snapshot vehicle state immediately at pulse time (before frame read latency)
            with vehicle_data_lock:
                vehicle_data_snapshot = dict(vehicle_data)
            vehicle_data_snapshot["pps_timestamp"] = pulse_time

            image_number += 1
            print(f"DEBUG: PPS pulse #{image_number} at t={pulse_time:.6f}")
            take_picture(image_number, camera_connection, vehicle_data_snapshot)
    finally:
        os.close(pps_fd)
        # Drain pending writes; the writer thread is shared and long-lived,
        # so we do NOT send a poison pill here.
        _save_queue.join()
        if camera_connection is not None:
            camera_connection.release()
            camera_connection = None

def take_picture(image_number, camera_connection, metadata):
    ret, frame = camera_connection.read()
    if not ret:
        print(f"WARN: Failed to capture frame {image_number}")
        return

    file_name = f'{image_number:05d}'
    print(f"DEBUG: Image {file_name} captured ({frame.shape[1]}x{frame.shape[0]}), queuing write")

    # Enqueue for async disk write — keeps the capture loop tight
    _save_queue.put((file_name, frame, metadata))

@app.route("/heartbeat-validate")
def heartbeat_validate():
    with vehicle_data_lock:
        return dict(vehicle_data)

def receive_vehicle_position():  # Actively runs and receives live vehicle data on a separate thread
    '''
    This function is ran on a second thread to actively retrieve and update the vehicle state. This vehicle state contains GPS and vehicle orientation.
    These are pulled at the time of taking an image in order to geotag images and support target localization.
    '''
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

    sock.bind(("127.0.0.1", 5005))
    while True:
        try:
            data = sock.recvfrom(1024)
            items = data[0].decode().strip()[1:-1].split(",")
            message_time = float(items[0])
        except (UnicodeDecodeError, ValueError, IndexError) as e:
            print(f"WARN: malformed vehicle position datagram: {e}")
            continue
        except OSError as e:
            print(f"WARN: vehicle position socket error: {e}")
            continue

        if len(items) != len(vehicle_data):
            print("Received data item does not match expected length...")
            continue

        with vehicle_data_lock:
            if message_time <= vehicle_data["last_time"]:
                continue
            try:
                vehicle_data["last_time"] = message_time
                for i, key in enumerate(_vehicle_keys, start=1):
                    vehicle_data[key] = float(items[i])
            except ValueError as e:
                print(f"WARN: failed to parse vehicle field: {e}")

if __name__ == "__main__":
    # Need to take a parameter off of the command line to determine if we are a plane or copter 
    kit = ServoKit(channels=16)

    os.makedirs(IMAGE_SAVE_DIR, exist_ok=True)

    position_thread = threading.Thread(target=receive_vehicle_position, daemon=True)
    position_thread.start()
    time.sleep(1)

    print(f"Attempting to connect to port: {VEHICLE_PORT}")
    vehicle_connection = initialize.connect_to_vehicle(VEHICLE_PORT)
    print("Vehicle connection established.")
    retVal = initialize.verify_connection(vehicle_connection)
    print("Vehicle connection verified.")

    if not retVal:
        print("Error. Could not connect and/or verify a valid connection to the vehicle.")
        sys.exit(1)

    app.run(debug=False, host='0.0.0.0', port=5000)
