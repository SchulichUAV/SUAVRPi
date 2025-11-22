vehicle_data = {
    "last_time": 0,
    "lat": 0,
    "lon": 0,
    "rel_alt": 0,
    "alt": 0,
    "roll": 14,
    "pitch": 0,
    "yaw": 12,
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

# Use wrapper dicts so they can be mutated from other modules
_kit = {}
_vehicle_conn = {}

picam2_state = {
    "picam2": None,
    "is_camera_on": False,
    "image_number": 0
}

def kit():
    return _kit["obj"]

def vehicle_connection():
    return _vehicle_conn["conn"]

def update_kit(obj):
    _kit["obj"] = obj

def update_vehicle_connection(conn):
    _vehicle_conn["conn"] = conn
