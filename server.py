
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware
import uvicorn, asyncio, json

app = FastAPI()

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

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
            except Exception:
                self.disconnect(ws)

manager = ConnectionManager()

@app.websocket("/ws/rpi")
async def rpi_ws(websocket: WebSocket):
    await manager.connect(websocket)
    try:
        while True:
            await asyncio.sleep(1)
            vehicle_data = {
                "last_time": 0,
                "lat": 123.2323,
                "lon": -124.23442,
                "rel_alt": 0,
                "alt": 2323,
                "roll": 24.23,
                "pitch": 0,
                "yaw": 0,
                "dlat": 0,
                "dlon": 0,
                "dalt": 0,
                "heading": 0,
                "groundspeed": 54,
                "throttle": 0,
                "climb": 0,
                "flight_mode": 0,
                "battery_voltage": 43.64,
                "battery_current": 0,
                "battery_remaining": 44.5,
                "is_dropped": False
            }
            await manager.broadcast(vehicle_data)
    except WebSocketDisconnect:
        manager.disconnect(websocket)

if __name__ == "__main__":
    uvicorn.run(app, host="0.0.0.0", port=8888)

