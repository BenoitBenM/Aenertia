import os
import serial
import paho.mqtt.publish as publish
from fastapi import FastAPI, Form, HTTPException
from fastapi.responses import FileResponse, JSONResponse
from fastapi.staticfiles import StaticFiles

# Configuration
MQTT_HOST   = os.getenv("MQTT_BROKER", "localhost")
MQTT_PORT   = int(os.getenv("MQTT_PORT", 1883))
SERIAL_PORT = os.getenv("SERIAL_PORT", "/dev/ttyUSB0")
BAUDRATE    = int(os.getenv("BAUDRATE", 115200))

# Initialize FastAPI
app = FastAPI(title="AenerShark Control API")

# Mount static directory
app.mount("/static", StaticFiles(directory="static"), name="static")

# Serve the main HTML page
@app.get("/", response_class=FileResponse)
def root():
    return FileResponse("static/index.html")


# Initialize serial connection
serial_conn = None
@app.on_event("startup")
def startup_event():
    global serial_conn
    try:
        serial_conn = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=1)
    except Exception:
        serial_conn = None


# Utility: publish single MQTT message
def mqtt_pub(topic: str, msg: str):
    publish.single(topic, msg, hostname=MQTT_HOST, port=MQTT_PORT)


# --- MQTT-driven endpoints ---

@app.get("/led/{id}")
def control_led(id: int):
    mapping = {1: "UP", 2: "DOWN", 3: "LEFT", 4: "RIGHT"}
    cmd = mapping.get(id, "OFF")
    mqtt_pub("robot/serial", cmd)
    return {"status": cmd}

@app.get("/led/flash")
def flash_led():
    mqtt_pub("led/control", "FLASH")
    return {"status": "FLASH"}

@app.get("/cv/enable")
def enable_cv():
    mqtt_pub("cv/control", "ENABLE")
    return {"status": "CV ON"}

@app.get("/cv/disable")
def disable_cv():
    mqtt_pub("cv/control", "DISABLE")
    return {"status": "CV OFF"}

@app.get("/autonomous/follow")
def autonomous_follow():
    mqtt_pub("autonomous", "FOLLOW")
    return {"status": "FOLLOW"}

@app.get("/autonomous/return")
def autonomous_return():
    mqtt_pub("autonomous", "RETURN_HOME")
    return {"status": "RETURN"}


# --- Autonomous key locations ---

key_locations = []

@app.post("/autonomous/key_location", response_class=FileResponse)
def assign_location(loc: str = Form(...)):
    key_locations.append(loc)
    mqtt_pub("autonomous", f"KEY:{loc}")
    return FileResponse("static/index.html")

@app.get("/autonomous/set_key/{loc}")
def set_existing_location(loc: str):
    mqtt_pub("autonomous", f"KEY:{loc}")
    return FileResponse("static/index.html")


# --- PID tuning ---

inner_history = []
outer_history = []

@app.post("/submit_inner", response_class=FileResponse)
def submit_inner(pg: str = Form(...), dg: str = Form(...),
                 ig: str = Form(...), sp: str = Form(...)):
    inner_history.append([pg, dg, ig, sp])
    return FileResponse("static/index.html")

@app.post("/submit_outer", response_class=FileResponse)
def submit_outer(pg: str = Form(...), dg: str = Form(...),
                  ig: str = Form(...), sp: str = Form(...),
                  rot: str = Form(...)):
    outer_history.append([pg, dg, ig, sp, rot])
    return FileResponse("static/index.html")

@app.get("/pid-values")
def get_pid_values():
    return JSONResponse(content={
        "inner": inner_history,
        "outer": outer_history
    })
