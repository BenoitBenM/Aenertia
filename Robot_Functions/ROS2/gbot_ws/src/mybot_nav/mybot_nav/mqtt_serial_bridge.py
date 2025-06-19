import paho.mqtt.client as mqtt
import serial
import threading # Threading is a library that allows us to run other tasks that the current one in the background
from time import sleep
from Advanced_CV.pose_detection import pose_detection
#, send_frame
import cv2
from flask import Flask, Response, send_from_directory
import json
import csv
import global_var as gv
import os

from EnergyCalculation import (
    initialize_csv_file,
    append_to_csv,
    calculate_percentage
)

import math
import tf2_ros
import rclpy
from rclpy.node import Node
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist

import atexit

@atexit.register
def shutdown_ros2():
    global ros2_initialized
    if ros2_initialized:
        rclpy.shutdown()

#Serial config (i included many ports just n case we somehow connect to an unexpected port number. It is very unlikely it goes above 1) 
SERIAL_PORT = [ "/dev/esp32", "/dev/ttyUSB1", "/dev/ttyUSB2","/dev/ttyUSB3", "/dev/ttyUSB4", 
                "/dev/ttyUSB5", "/dev/ttyUSB6", "/dev/ttyUSB7", "/dev/ttyUSB8","/dev/ttyUSB9", 
                "/dev/ttyUSB10", "/dev/ttyUSB11", "/dev/ttyUSB12", "/dev/ttyUSB13","/dev/ttyUSB14",
                "/dev/ttyUSB15", "/dev/ttyUSB16", "/dev/ttyUSB17","/dev/ttyUSB18", "/dev/ttyUSB0" ]

baud_rate = 115200

#global variables
manual_mode = False
payload = "stop"
ser = None
mode = "manual"
#gv.follow_mode = False
#gv.HumanDetected = False
#gv.offset = 0

ros2_initialized = False


key_locations = {}

# # FLASK APP SETUP
# app = Flask(__name__, static_folder='Placeholder_UI/static', static_url_path='')

# @app.route('/')
# def index():
#     return send_from_directory('static', 'index.html')

# @app.route('/video_feed')
# def video_feed():
#     return Response(
#         send_frame(),
#         mimetype='multipart/x-mixed-replace; boundary=frame'
#     )

# def start_web():
#     """Runs the Flask server for static files + MJPEG stream."""
#     app.run(host='0.0.0.0', port=8001, threaded=True)



def send_2_esp(command):
    print(f"Sending to esp: {command}")
    if ser and ser.is_open:
        ser.write((command + "\n").encode())

def manual_control():
    while manual_mode:
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

        sleep(0.2)

def follow_me():
    gv.HumanDetected
    gv.offset
    threading.Thread(target=pose_detection, daemon=True).start() #To fix this we use threading. Threading isolates the code we target and procceeds zith the rest of the code.
    while gv.follow_mode:
        print(gv.offset)
        if gv.HumanDetected:
            # print("HUMAN DETECTED")
            if abs(gv.offset) < 320*0.62:
                send_2_esp("forward")
            elif 320*0.62 <= gv.offset:
                send_2_esp("forwardANDright")
                
            elif -320*0.62 >= gv.offset:
                send_2_esp("forwardANDleft")

            else: 
                send_2_esp("stop")
        else:
            send_2_esp("stop")
        
        sleep(0.05)


class PoseRecorder(Node):
    def __init__(self):
        super().__init__('pose_recorder')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def get_current_pose(self):
        try:
            now = rclpy.time.Time()
            trans: TransformStamped = self.tf_buffer.lookup_transform(
                'map',
                'base_link',
                now,
                timeout=rclpy.duration.Duration(seconds=1.0)
            )
            x = trans.transform.translation.x
            y = trans.transform.translation.y

            # Convert quaternion to yaw
            q = trans.transform.rotation
            siny_cosp = 2 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
            theta = math.atan2(siny_cosp, cosy_cosp)

            return {'x': x, 'y': y, 'theta': theta}
        except TransformException as e:
            self.get_logger().error(f'TF error: {str(e)}')
            return None


def gotoKeyLocation(pose_dict):
    global ros2_initialized
    if not ros2_initialized:
        try:
            rclpy.init()
            ros2_initialized = True
        except RuntimeError:
            pass
    node = rclpy.create_node('goto_key_location')
    pub = node.create_publisher(PoseStamped, '/goal_pose', 10)

    msg = PoseStamped()
    msg.header.frame_id = "map"
    msg.pose.position.x = pose_dict['x']
    msg.pose.position.y = pose_dict['y']
    msg.pose.position.z = 0.0

    theta = pose_dict['theta']
    msg.pose.orientation.z = math.sin(theta / 2)
    msg.pose.orientation.w = math.cos(theta / 2)

    pub.publish(msg)
    print(f"Pose sent to Nav2 : {pose_dict}")

    rclpy.spin_once(node, timeout_sec=0.5)
    print("GOTO finished")

    node.destroy_node()


def cmd_vel_listener_discrete():
    global ros2_initialized
    if not ros2_initialized:
        try:
            rclpy.init()
            ros2_initialized = True
        except RuntimeError:
            pass
        print("[CMD_VEL] rclpy already initialized in another context.")
    
    node = rclpy.create_node('cmd_vel_listener_discrete')

    def callback(msg):
        linear = msg.linear.x
        angular = msg.angular.z
        threshold = 0.05

        command = "stop"
        if abs(linear) < threshold and abs(angular) < threshold:
            command = "stop"
        elif linear >= threshold and abs(angular) < threshold:
            command = "forward"
        elif linear <= -threshold and abs(angular) < threshold:
            command = "backward"
        elif linear >= threshold and angular >= threshold:
            command = "forwardANDleft"
        elif linear >= threshold and angular <= -threshold:
            command = "forwardANDright"
        elif linear <= -threshold and angular >= threshold:
            command = "backwardANDleft"
        elif linear <= -threshold and angular <= -threshold:
            command = "backwardANDright"
        elif abs(linear) < threshold and angular >= threshold:
            command = "left"
        elif abs(linear) < threshold and angular <= -threshold:
            command = "right"

        send_2_esp(command)
        print(f"[CMD_VEL → ESP] {command}")

    node.create_subscription(Twist, '/cmd_vel', callback, 10)

    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()



# ################################################################## TELEMETRY ##################################################################

def esp_read(client):
    """
    Read serial lines; when a 'PM:' message arrives, parse VB/EU,
    log to CSV, compute %, and publish on 'robot/battery'.
    """
    global ser, mqtt_client

    # ensure CSV exists
    initialize_csv_file()

    while True:
        if not ser or not ser.is_open:
            sleep(0.1)
            continue

        raw = ser.readline().decode(errors='ignore').strip()
        if raw.startswith("PM:"):
            try:
                payload_json = raw.split(None, 1)[1]
                data = json.loads(payload_json)
                VB = data.get("VB")
                EU = data.get("EU")

                if VB is None or EU is None:
                    print("[ESP] PM: missing VB/EU")
                    continue

                # 1) log raw values
                append_to_csv(VB, EU)
                
                client.publish('robot/vb', str(VB))
                client.publish('robot/eu', str(EU))

                # 2) compute battery percentage
                pct, Et = calculate_percentage(VB, EU)
                print(f"[Battery] {pct}%")

                # 3) publish to MQTT
                client.publish('robot/battery', str(pct))

            except Exception as e:
                print("[ESP] Error parsing PM:", e)

        sleep(0.5)


def on_connect(client, userdata, flags, rc):
    gv.HumanDetected
    gv.offset
    
#     print("MQTT connected with result code", rc)
    client.subscribe("robot/mode")
    client.subscribe("robot/auto")
    client.subscribe("robot/manual/command")
    client.subscribe("robot/manual/command")
    client.subscribe("robot/goto_keyloc")
    client.subscribe("robot/auto/key/assign")
    client.publish("robot/auto/key/locations", json.dumps(key_locations))

#     # Removed the thread for continuous CV and put it in follow mode
    threading.Thread(target=esp_read, args=(client,), daemon=True).start() #Continuously read value from ESP

def on_message(client, userdata, msg):
    #global cv_enabled
    global mode
    global manual_mode
    global payload    
    gv.follow_mode
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
    if mode != "autonomous" or payload == "GoToKeyLocation":  # This should be GoToKeyLocation Instead of return
        gv.follow_mode = False
        print("follow mode OFF")

    # Manual mode code
    if mode == "manual":
        if manual_mode == False:
            manual_mode = True
            threading.Thread(target=manual_control, daemon=True).start() # Runs manual mode until disabled

    # Autnomous mode code
    elif mode == "autonomous":  
        manual_mode = False
        if topic == "robot/auto":

            # In follow mode the robot follows the person around
            if payload == "follow":  
                if gv.follow_mode == False:
                    gv.follow_mode = True
                    threading.Thread(target=follow_me, daemon=True).start() # Runs follow_me unless follow_mode is disabled

            elif payload == "return": # This should be GoToKeyLocation Instead of return 
                gotoKeyLocation()

            elif payload == "stop": # To be implemented later
                send_2_esp("stop")

    if topic == "robot/auto/key/assign":
        key_name = payload.strip()
        global ros2_initialized
        if not ros2_initialized:
            try:
                rclpy.init()
                ros2_initialized = True
            except RuntimeError:
                pass
        node = PoseRecorder()
        rclpy.spin_once(node, timeout_sec=1.0)
        pose = node.get_current_pose()
        if pose:
            key_locations[key_name] = pose
            print(f"[KeyLocation] Saved {key_name} → {pose}")
            client.publish("robot/auto/key/locations", json.dumps(key_locations))
        else:
            print(f"[KeyLocation] Could not get pose for '{key_name}'")
        node.destroy_node()

        
    elif topic == "robot/goto_keyloc":
        try:
            pose_dict = json.loads(payload)
            gotoKeyLocation(pose_dict)
        except Exception as e:
            print(f"Invalid pose payload: {e}")



def main():
    #Robot function
    #Telemetry loop
    global ser
    global SERIAL_PORT
    # threading.Thread(target=start_web, daemon=True).start()
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
    
    threading.Thread(target=cmd_vel_listener_discrete, daemon=True).start()

    client.loop_forever()


if __name__ == "__main__":
    main()