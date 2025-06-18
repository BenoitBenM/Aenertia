#!/usr/bin/env python3
import os
import json
import threading
import serial
from time import sleep

import math
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.action import ActionClient

from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
import paho.mqtt.client as mqtt

# SERIAL CONFIG
SERIAL_PORTS = [
    "/dev/esp32", "/dev/ttyUSB0", "/dev/ttyUSB1", "/dev/ttyUSB2",
]
BAUD_RATE = 115200

def calculate_percentage(VB, EU):
    # Dummy implementation; replace with your real function
    # VB = battery voltage, EU = energy used
    pct = max(0, min(100, int((EU / VB) * 100)))
    return pct, None

class RobotBridge(Node):
    def __init__(self):
        super().__init__('robot_bridge')

        # --- Serial port for ESP32 ---
        self.ser = self._init_serial()
        if not self.ser:
            self.get_logger().error("No serial port opened; ESP32 commands disabled")

        # --- MQTT client ---
        self.mqtt_client = mqtt.Client()
        self.mqtt_client.on_connect = self._on_mqtt_connect
        self.mqtt_client.on_message = self._on_mqtt_message

        # state
        self.manual_mode = False
        self.follow_mode = False
        self.key_locations = {}
        self.cmd_vel_listener_running = False

        # --- Nav2 action client ---
        self.nav2_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # --- Subscribe to AMCL for current pose ---
        self.current_pose = {'x':0.0,'y':0.0,'theta':0.0}
        self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self._amcl_callback,
            10
        )

        # --- Threaded MQTT & Serial readers ---
        threading.Thread(target=self._start_mqtt, daemon=True).start()
        threading.Thread(target=self._serial_reader, daemon=True).start()

    def _init_serial(self):
        for p in SERIAL_PORTS:
            try:
                s = serial.Serial(p, BAUD_RATE, timeout=1)
                self.get_logger().info(f"Serial opened on {p}")
                return s
            except serial.SerialException:
                continue
        return None

    def _serial_reader(self):
        """Read battery telemetry lines `PM: {...}` from ESP32 and publish via MQTT."""
        while True:
            if not self.ser or not self.ser.is_open:
                sleep(0.5)
                continue
            try:
                line = self.ser.readline().decode('utf8','ignore').strip()
            except serial.SerialException:
                sleep(1.0)
                continue

            if line.startswith("PM:"):
                try:
                    data = json.loads(line.split(None,1)[1])
                    VB, EU = data['VB'], data['EU']
                    pct, _ = calculate_percentage(VB, EU)
                    self.mqtt_client.publish('robot/vb', str(VB))
                    self.mqtt_client.publish('robot/eu', str(EU))
                    self.mqtt_client.publish('robot/battery', str(pct))
                except Exception as e:
                    self.get_logger().warn(f"Serial parse error: {e}")
            sleep(0.1)

    def _start_mqtt(self):
        self.mqtt_client.connect("localhost", 1883, 60)
        self.mqtt_client.loop_forever()

    def _on_mqtt_connect(self, client, userdata, flags, rc):
        self.get_logger().info("MQTT connected")
        for topic in [
            'robot/mode',
            'robot/manual/command',
            'robot/auto',
            'robot/auto/key/assign',
            'robot/goto_keyloc'
        ]:
            client.subscribe(topic)

    def _on_mqtt_message(self, client, userdata, msg):
        payload = msg.payload.decode()
        if msg.topic == 'robot/mode':
            self._handle_mode(payload)
        elif msg.topic == 'robot/manual/command':
            self._handle_manual(payload)
        elif msg.topic == 'robot/auto':
            self._handle_auto(payload)
        elif msg.topic == 'robot/auto/key/assign':
            self._handle_assign(payload)
        elif msg.topic == 'robot/goto_keyloc':
            self._handle_goto_keyloc(payload)

    # --- Handlers ---
    def _handle_mode(self, payload):
        self.get_logger().info(f"Mode set → {payload}")
        self.manual_mode = (payload == 'manual')
        self.follow_mode = False

    def _handle_manual(self, payload):
        if not self.manual_mode or not self.ser: return
        cmd_map = {
            'up':'forward','down':'backward','left':'left','right':'right',
            'up-left':'forwardANDleft','up-right':'forwardANDright',
            'down-left':'backwardANDleft','down-right':'backwardANDright',
            'stop':'stop'
        }
        cmd = cmd_map.get(payload, 'stop')
        self.ser.write((cmd+"\n").encode())

    def _handle_auto(self, payload):
        if payload == 'follow':
            self.follow_mode = True
            threading.Thread(target=self._follow_loop, daemon=True).start()
        elif payload == 'stop':
            self.follow_mode = False

    def _follow_loop(self):
        import global_var as gv
        threading.Thread(target=pose_detection, daemon=True).start()
        while self.follow_mode:
            if gv.HumanDetected:
                off = gv.offset
                if abs(off) < 50:
                    cmd = 'forward'
                elif off > 0:
                    cmd = 'forwardANDright'
                else:
                    cmd = 'forwardANDleft'
            else:
                cmd = 'stop'
            if self.ser:
                self.ser.write((cmd+"\n").encode())
            sleep(0.1)

    def _handle_assign(self, name):
        p = self.current_pose.copy()
        self.key_locations[name] = p
        self.mqtt_client.publish(
            'robot/auto/key/locations',
            json.dumps(self.key_locations)
        )
        self.get_logger().info(f"Key '{name}' → {p}")

    def _handle_goto_keyloc(self, payload):
        try:
            goal = json.loads(payload)
        except json.JSONDecodeError:
            self.get_logger().error("Invalid JSON for goto_keyloc")
            return

        if not self.nav2_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error("Nav2 action server not available")
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = goal['x']
        goal_msg.pose.pose.position.y = goal['y']
        goal_msg.pose.pose.orientation.z = math.sin(goal['theta']/2.0)
        goal_msg.pose.pose.orientation.w = math.cos(goal['theta']/2.0)

        self.get_logger().info(f"[NAV2] Sending goal → {goal}")
        future = self.nav2_client.send_goal_async(
            goal_msg,
            feedback_callback=self._nav2_feedback
        )
        future.add_done_callback(self._nav2_response)

        # spin up cmd_vel listener once:
        threading.Thread(target=self._nav2_cmd_vel_listener, daemon=True).start()

    def _nav2_feedback(self, feedback_msg):
        # optional: forward feedback via MQTT or log
        self.get_logger().debug(f"Nav2 fb: {feedback_msg.feedback.current_pose.pose}")

    def _nav2_response(self, future):
        res = future.result().result
        status = future.result().status
        if status == NavigateToPose.Result().SUCCESS:
            self.get_logger().info("[NAV2] Goal reached")
        else:
            self.get_logger().warn(f"[NAV2] Failed with status {status}")

    def _nav2_cmd_vel_listener(self):
        if self.cmd_vel_listener_running:
            return
        self.cmd_vel_listener_running = True

        node = rclpy.create_node('cmd_vel_listener')
        def cb(msg: Twist):
            lin, ang = msg.linear.x, msg.angular.z
            if lin > 0.1 and abs(ang) < 0.1:      c='forward'
            elif lin < -0.1 and abs(ang) < 0.1:   c='backward'
            elif ang > 0.1 and abs(lin) < 0.1:    c='right'
            elif ang < -0.1 and abs(lin) < 0.1:   c='left'
            elif lin>0.1 and ang>0.1:             c='forwardANDright'
            elif lin>0.1 and ang<-0.1:            c='forwardANDleft'
            elif lin<-0.1 and ang>0.1:            c='backwardANDright'
            elif lin<-0.1 and ang<-0.1:           c='backwardANDleft'
            else:                                 c='stop'
            if self.ser:
                self.ser.write((c+"\n").encode())

        sub = node.create_subscription(Twist, '/cmd_vel', cb, 10)
        try:
            self.get_logger().info("[NAV2] cmd_vel listener started")
            rclpy.spin(node)
        except:
            pass
        finally:
            node.destroy_node()
            self.cmd_vel_listener_running = False

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

if __name__ == "__main__":
    main()
