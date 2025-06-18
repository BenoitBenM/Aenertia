#!/usr/bin/env python3
import threading, serial, json, math
from time import sleep

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor

from tf2_ros import Buffer, TransformListener, TransformException
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient

import paho.mqtt.client as mqtt

# —— CONFIG —— 
SERIAL_PORTS = [ "/dev/esp32", "/dev/ttyUSB0", "/dev/ttyUSB1" ]
BAUD_RATE    = 115200

MQTT_BROKER  = "localhost"
MQTT_PORT    = 1883

class RobotBridge(Node):
    def __init__(self):
        super().__init__('robot_bridge')

        # --- Serial to ESP32 ---
        self.ser = self._try_serial()

        # --- TF2 listener on map→base_link ---
        self.tf_buffer   = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # --- Nav2 action client ---
        self.nav2_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # --- MQTT client ---
        self.mqtt = mqtt.Client()
        self.mqtt.on_connect = self._on_mqtt_connect
        self.mqtt.on_message = self._on_mqtt_message

        # key_name → {x,y,theta}
        self.key_locations = {}

        # mode flags
        self.manual_mode = False
        self.follow_mode = False

        # command maps
        self._cmd_map = {
            'up':'forward','down':'backward','left':'left','right':'right',
            'up-left':'forwardANDleft','up-right':'forwardANDright',
            'down-left':'backwardANDleft','down-right':'backwardANDright',
            'stop':'stop'
        }

        # topics → handler methods
        self._handlers = {
            'robot/mode':           self._handle_mode,
            'robot/manual/command': self._handle_manual,
            'robot/auto':           self._handle_auto,
            'robot/auto/key/assign':self._handle_assign,
            'robot/goto_keyloc':    self._handle_goto_keyloc,
        }

        # — start background threads —
        threading.Thread(target=self._serial_reader, daemon=True).start()
        threading.Thread(target=self._mqtt_loop,     daemon=True).start()
        # we also subscribe to /cmd_vel for Nav2 driving
        self.create_subscription(Twist, '/cmd_vel', self._on_cmd_vel, 10)

    # —— Serial port init ——
    def _try_serial(self):
        for p in SERIAL_PORTS:
            try:
                ser = serial.Serial(p, BAUD_RATE, timeout=1)
                self.get_logger().info(f"Serial opened on {p}")
                return ser
            except serial.SerialException:
                continue
        self.get_logger().warn("No serial port opened")
        return None

    # —— Read battery from ESP → MQTT —
    def _serial_reader(self):
        while True:
            if not self.ser or not self.ser.is_open:
                sleep(0.5)
                continue
            line = self.ser.readline().decode('utf8','ignore').strip()
            if line.startswith("PM:"):
                try:
                    data = json.loads(line.split(None,1)[1])
                    VB, EU = data['VB'], data['EU']
                    pct, _ = calculate_percentage(VB, EU)
                    self.mqtt.publish('robot/vb',  str(VB))
                    self.mqtt.publish('robot/eu',  str(EU))
                    self.mqtt.publish('robot/battery', f"{pct}")
                except Exception as e:
                    self.get_logger().warn(f"Bad telemetry: {e}")
            sleep(0.1)

    # —— MQTT loop ——
    def _mqtt_loop(self):
        self.mqtt.connect(MQTT_BROKER, MQTT_PORT, 60)
        self.mqtt.loop_forever()

    def _on_mqtt_connect(self, client, userdata, flags, rc):
        self.get_logger().info("MQTT connected")
        for t in self._handlers:
            client.subscribe(t)

    def _on_mqtt_message(self, client, userdata, msg):
        h = self._handlers.get(msg.topic)
        if h:
            payload = msg.payload.decode()
            h(payload)
        else:
            self.get_logger().warn(f"No handler for {msg.topic}")

    # —— MODE handlers ——
    def _handle_mode(self, m):
        self.get_logger().info(f"Mode → {m}")
        self.manual_mode = (m=='manual')
        self.follow_mode = False

    def _handle_manual(self, cmd):
        if not self.manual_mode: return
        to_send = self._cmd_map.get(cmd, 'stop')
        if self.ser and self.ser.is_open:
            self.ser.write((to_send+"\n").encode())

    def _handle_auto(self, cmd):
        if cmd=='follow':
            self.follow_mode = True
            threading.Thread(target=self._follow_loop, daemon=True).start()
        else:
            self.follow_mode = False

    def _follow_loop(self):
        import global_var as gv
        threading.Thread(target=pose_detection, daemon=True).start()
        while self.follow_mode:
            if gv.HumanDetected:
                off = gv.offset
                if abs(off)<50: 	next='forward'
                elif off>0:	next='forwardANDright'
                else:		next='forwardANDleft'
            else:
                next='stop'
            if self.ser and self.ser.is_open:
                self.ser.write((next+"\n").encode())
            sleep(0.1)

    # —— TF helper ——
    def _get_pose_from_tf(self):
        now = rclpy.time.Time()
        try:
            t = self.tf_buffer.lookup_transform(
                'map','base_link', now,
                timeout=Duration(seconds=1.0)
            )
            x = t.transform.translation.x
            y = t.transform.translation.y
            q = t.transform.rotation
            theta = math.atan2(2*(q.w*q.z+q.x*q.y),
                               1-2*(q.y*q.y+q.z*q.z))
            return {'x':x,'y':y,'theta':theta}
        except TransformException as e:
            self.get_logger().warn(f"TF failed: {e}")
            return None

    # —— Key assignment ——
    def _handle_assign(self, name):
        p = self._get_pose_from_tf()
        if not p:
            self.get_logger().error("Cannot assign key: TF unavailable")
            return
        self.key_locations[name] = p
        self.mqtt.publish('robot/auto/key/locations', json.dumps(self.key_locations))
        self.get_logger().info(f"Key '{name}' → {p}")

    # —— Go-to-KeyLocation → Nav2 ——
    def _handle_goto_keyloc(self, payload):
        try:
            pose = json.loads(payload)
        except:
            self.get_logger().error("Bad JSON for goto_keyloc")
            return

        if not self.nav2_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error("Nav2 action server not ready")
            return

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp    = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = pose['x']
        goal.pose.pose.position.y = pose['y']
        goal.pose.pose.orientation.z = math.sin(pose['theta']/2.0)
        goal.pose.pose.orientation.w = math.cos(pose['theta']/2.0)

        self.get_logger().info(f"→ Nav2 goal {pose}")
        fut = self.nav2_client.send_goal_async(goal, feedback_callback=self._nav2_fb)
        fut.add_done_callback(self._nav2_done)

    def _nav2_fb(self, feedback):
        # optional: forward to MQTT or just log
        self.get_logger().debug(f"Nav2 fb: {feedback}")

    def _nav2_done(self, future):
        res = future.result().result
        stat= future.result().status
        if stat==4:
            self.get_logger().info("[NAV2] Arrived")
        else:
            self.get_logger().warn(f"[NAV2] Failed ({stat})")

    # —— Relay /cmd_vel → ESP32  ——
    def _on_cmd_vel(self, msg: Twist):
        # only forward if in “return to key” mode
        # you can add a flag if you only want this under certain conditions
        lin  = msg.linear.x
        ang  = msg.angular.z
        cmd  = 'stop'
        if lin>0.1 and abs(ang)<0.1:	cmd='forward'
        elif lin< -0.1 and abs(ang)<0.1:	cmd='backward'
        elif ang>0.1 and abs(lin)<0.1:	cmd='right'
        elif ang< -0.1 and abs(lin)<0.1:	cmd='left'
        elif lin>0.1 and ang>0.1:		cmd='forwardANDright'
        elif lin>0.1 and ang< -0.1:		cmd='forwardANDleft'
        elif lin< -0.1 and ang>0.1:		cmd='backwardANDright'
        elif lin< -0.1 and ang< -0.1:		cmd='backwardANDleft'
        if self.ser and self.ser.is_open:
            self.ser.write((cmd+"\n").encode())

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

if __name__=='__main__':
    main()
