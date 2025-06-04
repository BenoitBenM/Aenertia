import paho.mqtt.client as mqtt
import serial
import threading # Threading is a library that allows us to run other tasks that the current one in the background
from time import sleep
#import app_video # runs the server to send back the video
from Advanced_CV.pose_detection import pose_detection, send_frame
#from Advanced_CV.yolo_detection import gen_frames
from flask import Flask, Response, send_from_directory
import global_var as gv
import json
import cv2
import os

#Serial config (i included many ports just n case we somehow connect to an unexpected port number. It is very unlikely it goes above 1) 
SERIAL_PORT = [ "/dev/ttyUSB0", "/dev/ttyUSB1", "/dev/ttyUSB2","/dev/ttyUSB3", "/dev/ttyUSB4", 
                "/dev/ttyUSB5", "/dev/ttyUSB6", "/dev/ttyUSB7", "/dev/ttyUSB8","/dev/ttyUSB9", 
                "/dev/ttyUSB10", "/dev/ttyUSB11", "/dev/ttyUSB12", "/dev/ttyUSB13","/dev/ttyUSB14",
                "/dev/ttyUSB15", "/dev/ttyUSB16", "/dev/ttyUSB17","/dev/ttyUSB18", "/dev/ttyUSB19" ]

baud_rate = 115200

#global variables
follow_mode = False
ser = None
mode = "manual"
# gv.HumanDetected = False
# gv.offset = 0

# FLASK APP SETUP
app = Flask(__name__, static_folder='Placeholder_UI/static', static_url_path='')

@app.route('/')
def index():
    return send_from_directory('static', 'index.html')

@app.route('/video_feed')
def video_feed():
    return Response(
        send_frame(),
        mimetype='multipart/x-mixed-replace; boundary=frame'
    )

def start_web():
    """Runs the Flask server for static files + MJPEG stream."""
    app.run(host='0.0.0.0', port=8001, threaded=True)

def send_2_esp(command):
    print(f"Sending to esp: {command}")
    if ser and ser.is_open:
        ser.write((command + "\n").encode())


def follow_me():
    gv.HumanDetected
    gv.offset
    print(gv.HumanDetected)
    while follow_mode:
        print(gv.HumanDetected)

        if gv.HumanDetected:
            # print("HUMAN DETECTED")
            if abs(gv.offset) < 200: 
                send_2_esp("forward")
            elif 200 <= gv.offset < 700:
                send_2_esp("forwardANDright")
            elif -200 >= gv.offset > -700:
                send_2_esp("forwardANDleft")
            elif gv.offset >= 700:
                send_2_esp("right")
            elif gv.offset <= -700:
                send_2_esp("left")
            else: 
                send_2_esp("stop")
        else:
            send_2_esp("stop")


def gotoKeyLocation():
    print("ROS2 works ; Python stays silent")

################################################################## TELEMETRY ##################################################################

def esp_read():

    while True:
        if ser.in_waiting > 0:
    #       print("code stuck1")
            try:
                incoming = ser.readline().decode().strip()
                print(incoming)
            except UnicodeDecodeError:
                print("ESP error")
    #         print("1")
    #         print(incoming[0:3])
    #         if incoming[0:3] == "PM:":
    #             print("2")
    #             json_pm = incoming.split()[1]
    #             data = json.loads(json_pm)
    #             print("Voltage: " + data["voltage"])
    #             print("Motor Current : " + data["current_motor"])
    #             print("Board Current : " + data["current_board"])
    pass
        #PM: json

def on_connect(client, userdata, flags, rc):
    gv.HumanDetected
    gv.offset
    
    print("MQTT connected with result code", rc)
    client.subscribe("robot/mode")
    client.subscribe("robot/auto")
    client.subscribe("robot/manual/command")

    # Run the CV pose detection in the background
    threading.Thread(target=pose_detection, daemon=True).start() #To fix this we use threading. Threading isolates the code we target and procceeds zith the rest of the code.
    threading.Thread(target=esp_read, daemon=True).start() #Continuously read value from ESP

def on_message(client, userdata, msg):
    #global cv_enabled
    global mode
    global follow_mode
    gv.HumanDetected
    gv.offset

    #MQTT input
    payload = msg.payload.decode()
    topic = msg.topic
    print(f"Topic:{msg.topic} ; Command: {payload}")

    # We start by checking if the mode was changed
    if topic == "robot/mode":
        mode = payload

    # Check if the robot should stop following
    if mode != "autonomous" or payload == "return":  # This should be GoToKeyLocation Instead of return
        follow_mode = False
        print("follow mode OFF")

    # Manual mode code
    if mode == "manual":
        if payload == "up":
            send_2_esp("forward")
        elif payload == "down":
            send_2_esp("backward")
        elif payload == "right":
            send_2_esp("right")
        elif payload == "left":
            send_2_esp("left")
        elif payload == "up-right":
            send_2_esp("forwardANDright")
        elif payload == "up-left":
            send_2_esp("forwardANDleft")
        elif payload == "down-right":
            send_2_esp("backwardANDright")
        elif payload == "down-left":
            send_2_esp("backwardANDleft")
        else:
            send_2_esp("stop")

    # Autnomous mode code
    elif mode == "autonomous":  
        if topic == "robot/auto":

            # In follow mode the robot follows the person around
            if payload == "follow":  
                follow_mode = True
                threading.Thread(target=follow_me, daemon=True).start() # Runs follow_me unless follow_mode is disabled

            elif payload == "return": # This should be GoToKeyLocation Instead of return 
                gotoKeyLocation()

            elif payload == "stop": # To be implemented later
                send_2_esp("stop")


def main():
    #Robot function
    #Telemetry loop
    global ser
    global SERIAL_PORT

    for port in SERIAL_PORT:
        try:
            ser = serial.Serial(port, baud_rate, timeout=1)
            print(f"Serial connection start using port: {port}")
            break

        except (FileNotFoundError, serial.SerialException):
            print(f"failed to connect to port: {port}")

    client = mqtt.Client() # Creat a client object from MQTT
    client.on_connect = on_connect
    client.on_message = on_message
    client.connect("localhost", 1883, 60)
    client.loop_forever()
    



if __name__ == "__main__":
    threading.Thread(target=start_web, daemon=True).start()
    main()
