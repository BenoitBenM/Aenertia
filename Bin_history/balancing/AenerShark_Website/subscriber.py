#!/usr/bin/env python3
import os, subprocess, logging
import serial
import paho.mqtt.client as mqtt

# --- Config ---
MQTT_BROKER = os.getenv("MQTT_BROKER", "localhost")
MQTT_PORT   = int(os.getenv("MQTT_PORT", 1883))
SERIAL_PORT = os.getenv("SERIAL_PORT", "/dev/ttyUSB0")
BAUDRATE    = int(os.getenv("BAUDRATE", 115200))
FLASH_SCRIPT= os.path.join(os.path.dirname(__file__), "flash_led.py")

logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s")
logger = logging.getLogger()

# --- Serial init ---
try:
    ser = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=1)
    logger.info(f"Serial on {SERIAL_PORT}@{BAUDRATE}")
except Exception as e:
    ser = None
    logger.warning(f"Serial unavailable: {e}")

# --- MQTT callbacks ---
def on_connect(client, userdata, flags, rc):
    logger.info(f"MQTT connected (rc={rc})")
    for t in ("robot/serial", "led/control", "cv/control", "autonomous", "robot/pid", "mode/set"):
        client.subscribe(t)
        logger.info(f"Subscribed to {t}")

def run_flash():
    subprocess.run(["python3", FLASH_SCRIPT], check=False)

def on_message(client, userdata, msg):
    topic = msg.topic
    payload = msg.payload.decode()
    logger.info(f"→ {topic} = {payload}")

    if topic == "led/control":
        if payload == "FLASH":
            run_flash()

    elif topic == "robot/serial" and ser:
        ser.write((payload + "\n").encode())

    elif topic == "cv/control":
        # placeholder: implement CV enable/disable logic
        logger.info(f"CV control: {payload}")

    elif topic == "autonomous":
        # handle follow, return_home, key:...
        logger.info(f"Autonomous cmd: {payload}")

    elif topic == "robot/pid":
        # just log or save to file
        logger.info(f"PID tuning: {payload}")

    elif topic == "mode/set":
        logger.info(f"Mode set to {payload}")

# --- Start MQTT client ---
client = mqtt.Client()
client.on_connect = on_connect
client.on_message = on_message
client.connect(MQTT_BROKER, MQTT_PORT, 60)
client.loop_forever()
