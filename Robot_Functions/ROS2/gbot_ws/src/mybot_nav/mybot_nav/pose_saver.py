import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import json

class PoseSaver(Node):
    def __init__(self):
        super().__init__('pose_saver')
        self.subscription = self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self.pose_callback,
            10
        )
        self.saved_poses = []

    def pose_callback(self, msg):
        pose = {
            'x': msg.pose.position.x,
            'y': msg.pose.position.y,
            'theta': 0.0  # TODO: extract yaw from quaternion
        }
        self.get_logger().info(f'📍 Saved pose: {pose}')
        self.saved_poses.append(pose)
        import os
        file_path = os.path.expanduser("~/.ros/saved_poses.json")
        print(f"Writing pose to: {file_path}")
        with open(file_path, 'w') as f:
            json.dump(self.saved_poses, f, indent=2)

def main(args=None):
    rclpy.init(args=args)
    node = PoseSaver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
