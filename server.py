# main.py
import uvicorn, asyncio
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware
from fastapi import FastAPI
from contextlib import asynccontextmanager

from routes_http import router as http_router
from shared_state import vehicle_data, update_kit, update_vehicle_connection

from modules.AutopilotDevelopment.General.Operations.initialize import connect_to_vehicle, verify_connection
from adafruit_servokit import ServoKit
import threading

@asynccontextmanager
async def lifespan(app: FastAPI):
    print("Initializing ServoKit...")
    kit_obj = ServoKit(channels=16)
    update_kit(kit_obj)

    print("Connecting to vehicle...")
    conn = connect_to_vehicle("udp:127.0.0.1:5006")
    if not verify_connection(conn):
        raise RuntimeError("Vehicle connection failed verification.")
    update_vehicle_connection(conn)

    print("Starting vehicle position thread...")
    def receive_vehicle_position():
        import socket
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind(("127.0.0.1", 5005))

        while True:
            data = sock.recvfrom(1024)
            items = data[0].decode()[1:-1].split(",")
            timestamp = float(items[0])

            if timestamp <= vehicle_data["last_time"]:
                continue

            if len(items) == len(vehicle_data):
                vehicle_data["last_time"] = timestamp
                for i, key in enumerate(list(vehicle_data.keys())[1:], start=1):
                    vehicle_data[key] = float(items[i])

    pos_thread = threading.Thread(target=receive_vehicle_position, daemon=True)
    pos_thread.start()
    yield


#   APP DEFINITION
app = FastAPI(lifespan=lifespan)

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

app.include_router(http_router)

#   WEBSOCKET MANAGER
class ConnectionManager:
    def __init__(self):
        self.active_connections: list[WebSocket] = []

    async def connect(self, websocket: WebSocket):
        await websocket.accept()
        self.active_connections.append(websocket)
        print(f"Client connected ({len(self.active_connections)} active)")

    def disconnect(self, websocket: WebSocket):
        if websocket in self.active_connections:
            self.active_connections.remove(websocket)

    async def broadcast(self, message: dict):
        for ws in list(self.active_connections):
            try:
                await ws.send_json(message)
            except:
                self.disconnect(ws)


manager = ConnectionManager()

#   WEBSOCKET ENDPOINT
@app.websocket("/ws/rpi")
async def websocket_rpi(websocket: WebSocket):
    await manager.connect(websocket)
    try:
        while True:
            await asyncio.sleep(1)
            await manager.broadcast(vehicle_data)
    except WebSocketDisconnect:
        manager.disconnect(websocket)


if __name__ == "__main__":
    uvicorn.run(app, host="0.0.0.0", port=8888)