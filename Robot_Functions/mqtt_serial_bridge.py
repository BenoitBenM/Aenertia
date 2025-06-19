import paho.mqtt.client as mqtt
import json
import rclpy
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator
from nav2_simple_commander.task_result import TaskResult
from geometry_msgs.msg import PoseStamped
import math

# GLOBAL
key_locations = {}

def gotoKeyLocation(pose_dict):
    rclpy.init()
    navigator = BasicNavigator()
    navigator.waitUntilNav2Active()

    goal = PoseStamped()
    goal.header.frame_id = 'map'
    goal.header.stamp = navigator.get_clock().now().to_msg()
    goal.pose.position.x = pose_dict['x']
    goal.pose.position.y = pose_dict['y']
    theta = pose_dict['theta']
    goal.pose.orientation.z = math.sin(theta / 2)
    goal.pose.orientation.w = math.cos(theta / 2)

    navigator.goToPose(goal)
    print(f"[NAV2] Sent goal: {pose_dict}")

    while not navigator.isTaskComplete():
        feedback = navigator.getFeedback()
        if feedback:
            print(f"[NAV2] Distance remaining: {feedback.distance_remaining:.2f} m")

    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print("[NAV2] ✅ Goal reached!")
    else:
        print(f"[NAV2] ❌ Failed: {result}")

    navigator.lifecycleShutdown()
    rclpy.shutdown()

def on_connect(client, userdata, flags, rc):
    print("[MQTT] Connected")
    client.subscribe("robot/goto_keyloc")
    client.subscribe("robot/auto/key/assign")
    client.publish("robot/auto/key/locations", json.dumps(key_locations))

def on_message(client, userdata, msg):
    global key_locations
    topic = msg.topic
    payload = msg.payload.decode()
    print(f"[MQTT] ← {topic} : {payload}")

    if topic == "robot/goto_keyloc":
        try:
            coords = json.loads(payload)
            gotoKeyLocation(coords)
        except Exception as e:
            print(f"[ERROR] Invalid payload for goto_keyloc: {e}")

    elif topic == "robot/auto/key/assign":
        try:
            name = payload.strip()
            pose = get_current_pose()
            if pose:
                key_locations[name] = pose
                print(f"[KeyLocation] Saved {name} → {pose}")
                client.publish("robot/auto/key/locations", json.dumps(key_locations))
            else:
                print("[ERROR] Could not get pose (no TF?)")
        except Exception as e:
            print(f"[ERROR] in key assignment: {e}")

# --- Pose from TF ---
class PoseFetcher(Node):
    def __init__(self):
        super().__init__('pose_fetcher')
        self.tf_buffer = rclpy.time.Clock().create_ros_time()
        self.tf_listener = None

    def get_pose(self):
        from tf2_ros import Buffer, TransformListener, TransformException
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        try:
            trans = self.tf_buffer.lookup_transform(
                'map', 'base_link', rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=1.0)
            )
            x = trans.transform.translation.x
            y = trans.transform.translation.y
            q = trans.transform.rotation
            siny_cosp = 2 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1 - 2 * (q.y**2 + q.z**2)
            theta = math.atan2(siny_cosp, cosy_cosp)
            return {'x': x, 'y': y, 'theta': theta}
        except TransformException as e:
            self.get_logger().warn(f"TF exception: {e}")
            return None

def get_current_pose():
    rclpy.init()
    node = PoseFetcher()
    rclpy.spin_once(node, timeout_sec=1.0)
    pose = node.get_pose()
    node.destroy_node()
    rclpy.shutdown()
    return pose

def main():
    client = mqtt.Client()
    client.on_connect = on_connect
    client.on_message = on_message
    client.connect("localhost", 1883, 60)
    client.loop_forever()

if __name__ == "__main__":
    main()
