#!/usr/bin/env python3
import os
import json
import threading
import serial
from time import sleep

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient

import paho.mqtt.client as mqtt

# SERIAL CONFIG
SERIAL_PORTS = [
    "/dev/esp32", "/dev/ttyUSB0", "/dev/ttyUSB1", "/dev/ttyUSB2", # …
]
BAUD_RATE = 115200

# NAV2 GOAL TOLERANCES (not strictly needed when using NavigateToPose action)
GOAL_TOLERANCE = 0.15  # meters
ANGLE_TOLERANCE = 0.1  # radians

class RobotBridge(Node):
    def __init__(self):
        super().__init__('robot_bridge')

        # --- Serial port for ESP32 ---
        self.ser = self._init_serial()
        if not self.ser:
            self.get_logger().error("No serial port opened; manual/auto commands will fail.")

        # --- MQTT client ---
        self.mqtt_client = mqtt.Client()
        self.mqtt_client.on_connect = self._on_mqtt_connect
        self.mqtt_client.on_message = self._on_mqtt_message

        self.key_locations = {}          # name -> {'x':…, 'y':…, 'theta':…}
        self.manual_mode = False
        self.follow_mode = False

        # Map MQTT topics → handler methods
        self._mqtt_handlers = {
            'robot/mode':          self._handle_mode,
            'robot/manual/command':self._handle_manual,
            'robot/auto':          self._handle_auto,
            'robot/auto/key/assign': self._handle_assign,
            'robot/goto_keyloc':   self._handle_goto_keyloc,
        }

        # --- Nav2 action client ---
        self.nav2_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # --- AMCL pose subscriber (to track current pose) ---
        self.current_pose = {'x':0.0,'y':0.0,'theta':0.0}
        self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self._amcl_callback,
            10
        )

        # --- Start background threads ---
        threading.Thread(target=self._start_mqtt, daemon=True).start()
        threading.Thread(target=self._serial_reader, daemon=True).start()

    # ---- Serial init ----
    def _init_serial(self):
        for port in SERIAL_PORTS:
            try:
                ser = serial.Serial(port, BAUD_RATE, timeout=1)
                self.get_logger().info(f"Serial opened on {port}")
                return ser
            except serial.SerialException:
                continue
        return None

    def _serial_reader(self):
        """Continuously read battery telemetry from ESP and publish over MQTT."""
        while True:
            if not self.ser or not self.ser.is_open:
                sleep(0.5)
                continue
            try:
                line = self.ser.readline().decode('utf8', 'ignore').strip()
            except serial.SerialException:
                sleep(1.0)
                continue

            if line.startswith("PM:"):
                try:
                    data = json.loads(line.split(None,1)[1])
                    VB, EU = data['VB'], data['EU']
                    pct, _ = calculate_percentage(VB, EU)  # your existing function
                    # publish back to MQTT
                    self.mqtt_client.publish('robot/vb',  str(VB))
                    self.mqtt_client.publish('robot/eu',  str(EU))
                    self.mqtt_client.publish('robot/battery', f"{pct}")
                except Exception as e:
                    self.get_logger().warn(f"Serial parse error: {e}")
            sleep(0.1)

    # ---- MQTT thread ----
    def _start_mqtt(self):
        self.mqtt_client.connect("localhost", 1883, 60)
        self.mqtt_client.loop_forever()

    def _on_mqtt_connect(self, client, userdata, flags, rc):
        self.get_logger().info("MQTT connected")
        for topic in self._mqtt_handlers:
            client.subscribe(topic)

    def _on_mqtt_message(self, client, userdata, msg):
        handler = self._mqtt_handlers.get(msg.topic)
        if handler:
            payload = msg.payload.decode()
            handler(payload)
        else:
            self.get_logger().warn(f"No handler for topic {msg.topic}")

    # ---- Handlers for each MQTT topic ----
    def _handle_mode(self, payload):
        self.get_logger().info(f"Set mode → {payload}")
        self.manual_mode = (payload == 'manual')
        self.follow_mode = False

    def _handle_manual(self, payload):
        if not self.manual_mode: return
        cmd_map = {
            'up':'forward','down':'backward','left':'left','right':'right',
            'up-left':'forwardANDleft','up-right':'forwardANDright',
            'down-left':'backwardANDleft','down-right':'backwardANDright',
            'stop':'stop'
        }
        cmd = cmd_map.get(payload,'stop')
        if self.ser and self.ser.is_open:
            self.ser.write((cmd+"\n").encode())

    def _handle_auto(self, payload):
        if payload == 'follow':
            self.follow_mode = True
            threading.Thread(target=self._follow_loop, daemon=True).start()
        elif payload == 'stop':
            self.follow_mode = False

    def _follow_loop(self):
        import global_var as gv
        pose_detection_thread = threading.Thread(target=pose_detection, daemon=True)
        pose_detection_thread.start()
        while self.follow_mode:
            if gv.HumanDetected:
                off = gv.offset
                if abs(off) < 50:
                    cmd='forward'
                elif off>0:
                    cmd='forwardANDright'
                else:
                    cmd='forwardANDleft'
            else:
                cmd='stop'
            if self.ser and self.ser.is_open:
                self.ser.write((cmd+"\n").encode())
            sleep(0.1)

    def _handle_assign(self, name):
        # record current amcl pose
        p = self.current_pose.copy()
        self.key_locations[name] = p
        self.mqtt_client.publish('robot/auto/key/locations', json.dumps(self.key_locations))
        self.get_logger().info(f"Key '{name}' → {p}")

    def _handle_goto_keyloc(self, payload):
        try:
            pose = json.loads(payload)
        except Exception:
            self.get_logger().error("Invalid JSON for goto_keyloc")
            return

        # build and send Nav2 goal:
        if not self.nav2_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error("Nav2 action server not available")
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = pose['x']
        goal_msg.pose.pose.position.y = pose['y']
        goal_msg.pose.pose.orientation.z = math.sin(pose['theta']/2.0)
        goal_msg.pose.pose.orientation.w = math.cos(pose['theta']/2.0)

        self.get_logger().info(f"[NAV2] Sending goal → {pose}")
        send_goal = self.nav2_client.send_goal_async(
            goal_msg,
            feedback_callback=self._nav2_feedback
        )
        send_goal.add_done_callback(self._nav2_response)

    def _nav2_feedback(self, feedback):
        # optional: forward feedback via MQTT or just log
        self.get_logger().debug(f"Nav2 feedback: {feedback}")

    def _nav2_response(self, future):
        result = future.result().result
        status = future.result().status
        if status == 4:   # SUCCEEDED
            self.get_logger().info("[NAV2] Goal reached successfully")
        else:
            self.get_logger().warn(f"[NAV2] Goal failed with status {status}")

    # ---- AMCL subscriber ----
    def _amcl_callback(self, msg: PoseWithCovarianceStamped):
        p = msg.pose.pose
        self.current_pose['x'] = p.position.x
        self.current_pose['y'] = p.position.y
        q = p.orientation
        self.current_pose['theta'] = math.atan2(
            2*(q.w*q.z + q.x*q.y),
            1 - 2*(q.y*q.y + q.z*q.z)
        )

def main():
    rclpy.init()
    node = RobotBridge()
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__=="__main__":
    main()
