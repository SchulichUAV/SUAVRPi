from fastapi import APIRouter, HTTPException
from shared_state import vehicle_connection, vehicle_data, kit, picam2_state
import modules.AutopilotDevelopment.General.Operations.mode as autopilot_mode
import modules.AutopilotDevelopment.Plane.Operations.altitude as autopilot_altitude
import modules.AutopilotDevelopment.General.Operations.mission as mission
from camera import continuously_capture_images, stop_camera_thread
import modules.payload as payload
import threading

router = APIRouter()

# Track camera thread
camera_thread = None

@router.post("/set_flight_mode")
async def set_flight_mode(req: dict):
    try:
        mode_id = int(req["mode_id"])
        selected = list(autopilot_mode.plane_modes.keys())[mode_id]
        autopilot_mode.set_mode(vehicle_connection(), mode_id)
        return {"message": f"Mode set"}
    except:
        raise HTTPException(status_code=400, detail="Invalid operation")


@router.post("/set_altitude_goto")
async def set_altitude(req: dict):
    try:
        altitude = int(req["altitude"])
        if altitude < 0:
            raise ValueError()
        autopilot_altitude.set_current_altitude(vehicle_connection(), altitude)
        return {"message": "Altitude updated"}
    except:
        raise HTTPException(status_code=400, detail="Invalid altitude")


@router.post("/payload_drop_mission")
async def payload_drop_mission_route(req: dict):
    try:
        target_lat = req["latitude"]
        target_lon = req["longitude"]
        drop_alt = 20
        mission.upload_payload_drop_mission(vehicle_connection(), [target_lat, target_lon, drop_alt])
        return {"message": "Mission uploaded successfully"}
    except Exception as e:
        raise HTTPException(status_code=400, detail=str(e))
    
@router.post("/monitor_mission_and_drop")
async def monitor_mission_and_drop(req: dict):
    try:
        bay = req["bay"]
        def monitor_and_drop():
            try:
                while True:
                    msg = vehicle_connection().recv_match(type='MISSION_CURRENT', blocking=True, timeout=5)
                    if msg is not None and msg.seq == 2:
                        autopilot_mode.set_mode(vehicle_connection(), 10)
                        break

                mission.check_distance_and_drop(vehicle_connection(), bay - 1, kit(), vehicle_data)
            except Exception as e:
                print("[Thread] Mission drop error:", e)

        thread = threading.Thread(target=monitor_and_drop)
        thread.start()
        return {"message": "Payload drop monitoring started"}
    except Exception as e:
        raise HTTPException(status_code=400, detail=f"Failed: {e}")

@router.post("/payload_manual_control")
async def payload_manual_control(req: dict):
    pid = req.get("payload_id")
    state = req.get("payload_open")
    if not isinstance(pid, int) or not isinstance(state, bool):
        raise HTTPException(status_code=400, detail="Invalid payload parameters")

    if 1 <= pid <= 4:
        try:
            payload.set_servo_state(pid - 1, state)
            return {"servo_status": state}
        except:
            raise HTTPException(status_code=400, detail="Failed to set servo")
    else:
        raise HTTPException(status_code=400, detail="payload_id must be 1–4")

@router.post("/payload_release")
async def payload_release_route(req: dict):
    pid = req.get("bay")
    if not 1 <= pid <= 4:
        raise HTTPException(status_code=400, detail="Invalid payload ID")
    payload.payload_release(kit(), pid - 1, vehicle_data)
    return {"message": "Payload released"}
    
@router.post("/payload_release_all")
async def payload_release_all():
    try:
        payload.release_all(kit(), vehicle_data)
        return {"message": "All payloads released"}
    except Exception as e:
        raise HTTPException(status_code=400, detail=f"Failed: {e}")

@router.post("/payload_close_all")
async def payload_close_all():
    try:
        payload.close_all_servos(kit())
        return {"message": "All servos closed"}
    except Exception as e:
        raise HTTPException(status_code=400, detail=f"Failed: {e}")

@router.post("/payload_open_all")
async def payload_open_all():
    try:
        payload.open_all_servos(kit())
        return {"message": "All servos opened"}
    except Exception as e:
        raise HTTPException(status_code=400, detail=f"Failed: {e}")

@router.post("/payload_open")
async def payload_open(req: dict):
    pid = req.get("bay")
    if not isinstance(pid, int) or not (1 <= pid <= 4):
        raise HTTPException(status_code=400, detail="Invalid bay")

    try:
        payload.open_servo(kit(), pid - 1)
        return {"message": "Servo opened"}
    except Exception as e:
        raise HTTPException(status_code=400, detail=f"Failed: {e}")

@router.post("/payload_close")
async def payload_close(req: dict):
    pid = req.get("bay")
    if not isinstance(pid, int) or not (1 <= pid <= 4):
        raise HTTPException(status_code=400, detail="Invalid bay")

    try:
        payload.close_servo(kit(), pid - 1)
        return {"message": "Servo closed"}
    except Exception as e:
        raise HTTPException(status_code=400, detail=f"Failed: {e}")

@router.post("/toggle_camera")
async def toggle_camera(req: dict):
    global camera_thread

    try:
        is_on = req["is_camera_on"]
        image_num = req["image_count"]
    except:
        raise HTTPException(status_code=400, detail="Invalid camera parameters")

    picam2_state["is_camera_on"] = is_on
    picam2_state["image_number"] = image_num

    if is_on:
        if camera_thread is None or not camera_thread.is_alive():
            stop_camera_thread.clear()
            camera_thread = threading.Thread(target=continuously_capture_images)
            camera_thread.start()
    else:
        stop_camera_thread.set()

    return {"message": "Camera state updated"}


@router.get("/heartbeat-validate")
async def heartbeat():
    return vehicle_data