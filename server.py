# Buit to sync with GCS2025 in Schulich UAV repository/organization
# Built for Raspberry Pi 5 (Linux OS)

from flask import Flask, jsonify, request
from flask_cors import CORS
from adafruit_servokit import ServoKit
from io import BytesIO
import cv2
import threading
import requests
import socket
import ctypes
import fcntl
import json
import sys
import time
import os
import subprocess

import modules.AutopilotDevelopment.General.Operations.initialize as initialize
import modules.AutopilotDevelopment.General.Operations.mode as autopilot_mode
import modules.AutopilotDevelopment.General.Operations.mission as mission
import modules.AutopilotDevelopment.Plane.Operations.altitude as autopilot_altitude
import modules.payload as payload


VEHICLE_PORT = "udp:127.0.0.1:5006"
PPS_DEVICE = "/dev/pps0"  # kernel PPS driver via dtoverlay=pps-gpio,gpiopin=4

# SETTINGS REQUIRE MANUAL TWEAK
'''
GCS_URL - Depends which IP the laptop appears as on the network. Use ifconfig/ipconfig to check.
CAMERA_DEVICE - Depends on which USB port the camera is plugged into. Check with `v4l2-ctl --list-devices` and look for the /dev/video* entry under the correct camera.
'''
GCS_URL = "http://192.168.1.66:80"
CAMERA_DEVICE = "/dev/video0"

# Manual exposure (shutter) settings. The camera is mounted on a moving
# aircraft, so leaving exposure on auto produces motion-blurred frames.
# UVC/V4L2 expresses exposure_absolute in units of 100 µs.
CAMERA_EXPOSURE = 1   # 1 * 100µs = 100 µs shutter

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

@app.route("/toggle_camera", methods=["POST"])
def toggle_camera():
    global image_number
    global camera_thread

    try:
        json_data = request.json
        requested_state = bool(json_data["is_camera_on"])
        amount_of_existing_images = int(json_data["image_count"])
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
            camera_thread = threading.Thread(
                target=continuously_capture_images, name="camera-capture", daemon=True,
            )
            camera_thread.start()
            print("Starting camera...")
        else:
            print("Stopping Camera")
            stop_camera_thread.set()

    return jsonify({"message": "Success!"}), 200

def _v4l2_set(device: str, control: str, value) -> bool:
    """Set a single V4L2 control via v4l2-ctl. Returns True on success."""
    try:
        result = subprocess.run(
            ["v4l2-ctl", "-d", device, "--set-ctrl", f"{control}={value}"],
            capture_output=True, text=True, timeout=2,
        )
    except (FileNotFoundError, subprocess.TimeoutExpired) as e:
        print(f"WARN: v4l2-ctl unavailable for {control}={value}: {e}")
        return False
    if result.returncode != 0:
        # Driver will complain on stderr if the control name isn't recognised.
        return False
    return True

def _apply_manual_exposure(device: str, exposure_units: int) -> None:
    """Force manual exposure on a UVC camera, trying both UAPI naming schemes."""
    # auto-exposure: 1 = Manual Mode, 3 = Aperture Priority Mode (auto).
    if not (_v4l2_set(device, "auto_exposure", 1)
            or _v4l2_set(device, "exposure_auto", 1)):
        print("WARN: could not disable auto-exposure via v4l2-ctl; "
              "manual shutter may not take effect")
        return
    if not (_v4l2_set(device, "exposure_time_absolute", exposure_units)
            or _v4l2_set(device, "exposure_absolute", exposure_units)):
        print(f"WARN: could not set manual exposure to {exposure_units} via v4l2-ctl")

def _print_exposure_state(device: str) -> None:
    """Log the camera's current exposure controls for verification.
    
    THIS FUNCTION IS CURRENTLY UNUSED. IF WE NEED TO DEBUG EXPOSURE SETTINGS ON THE SPOT, CALL THIS!
    """
    try:
        result = subprocess.run(
            ["v4l2-ctl", "-d", device, "--get-ctrl",
             "auto_exposure,exposure_time_absolute,exposure_auto,exposure_absolute"],
            capture_output=True, text=True, timeout=2,
        )
        if result.stdout:
            print("Exposure controls:\n" + result.stdout.strip())
    except (FileNotFoundError, subprocess.TimeoutExpired):
        pass

def continuously_capture_images():
    global camera_connection
    global image_number

    if camera_connection is None or not camera_connection.isOpened():
        print("Initializing camera...")

        # Configure manual exposure via v4l2-ctl BEFORE opening the device.
        # OpenCV's V4L2 backend frequently fails to set auto-exposure
        # (cap.set returns False and the control is silently ignored),
        # so drive the UVC controls directly. Some kernels expose the
        # controls as auto_exposure / exposure_time_absolute (newer UVC
        # driver) and some as exposure_auto / exposure_absolute (older).
        # Try both and accept whichever the driver accepts.
        _apply_manual_exposure(CAMERA_DEVICE, CAMERA_EXPOSURE)

        camera_connection = cv2.VideoCapture(CAMERA_DEVICE, cv2.CAP_V4L2)

        camera_connection.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        camera_connection.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        camera_connection.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        camera_connection.set(cv2.CAP_PROP_FPS, 30)
        camera_connection.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        # Re-apply after open: opening the device can reset UVC controls on some drivers.
        _apply_manual_exposure(CAMERA_DEVICE, CAMERA_EXPOSURE)

        if not camera_connection.isOpened():
            print("ERROR: Could not open camera")
            return

        # Optional: throw away a few startup frames
        for _ in range(5):
            camera_connection.read()

        # TODO: Remove this once we know the camera settings are solid.
        print("Camera opened with:")
        print("Width:", camera_connection.get(cv2.CAP_PROP_FRAME_WIDTH))
        print("Height:", camera_connection.get(cv2.CAP_PROP_FRAME_HEIGHT))
        print("FPS:", camera_connection.get(cv2.CAP_PROP_FPS))
        print("Auto-exposure mode:", camera_connection.get(cv2.CAP_PROP_AUTO_EXPOSURE))
        print("Exposure:", camera_connection.get(cv2.CAP_PROP_EXPOSURE))
        print("Gain:", camera_connection.get(cv2.CAP_PROP_GAIN))

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
        if camera_connection is not None:
            camera_connection.release()
            camera_connection = None

def take_picture(image_number, camera_connection, metadata):
    ret, frame = camera_connection.read()
    if not ret:
        print(f"WARN: Failed to capture frame {image_number}")
        return

    # Encode frame to JPEG bytes in memory — no disk write
    ret, buffer = cv2.imencode('.jpg', frame)
    if not ret:
        print(f"WARN: Failed to encode frame {image_number} as JPEG")
        return

    file_stem = f'{image_number:05d}'
    print(f"DEBUG: Image {file_stem} captured ({frame.shape[1]}x{frame.shape[0]}), uploading")

    try:
        response = requests.post(
            f"{GCS_URL}/submit",
            files={'file': (f'{file_stem}.jpg', BytesIO(buffer.tobytes()), 'image/jpeg')},
            timeout=10,
        )
        if not response.ok:
            print(f"WARN: Image upload failed for {file_stem}: {response.status_code}")
    except requests.RequestException as e:
        print(f"WARN: Image upload error for {file_stem}: {e}")

    try:
        response = requests.post(
            f"{GCS_URL}/submit",
            files={'file': (f'{file_stem}.json', BytesIO(json.dumps(metadata).encode()), 'application/json')},
            timeout=10,
        )
        if not response.ok:
            print(f"WARN: Metadata upload failed for {file_stem}: {response.status_code}")
    except requests.RequestException as e:
        print(f"WARN: Metadata upload error for {file_stem}: {e}")

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