import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import json
import time

class NavToPose(Node):
    def __init__(self):
        super().__init__('nav_to_pose')
        self.publisher = self.create_publisher(PoseStamped, '/goal_pose', 10)
        time.sleep(1.0)  # wait for publisher to be ready

        try:
            with open('/tmp/saved_poses.json', 'r') as f:
                poses = json.load(f)
                if poses:
                    last_pose = poses[-1]
                    self.send_goal(last_pose)
                else:
                    self.get_logger().info(' No saved poses found.')
        except Exception as e:
            self.get_logger().error(f'Failed to load poses: {e}')

    def send_goal(self, pose_dict):
        msg = PoseStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = pose_dict['x']
        msg.pose.position.y = pose_dict['y']
        msg.pose.position.z = 0.0
        msg.pose.orientation.w = 1.0  # neutral orientation
        self.publisher.publish(msg)
        self.get_logger().info(f'🚀 Sent goal: {pose_dict}')

def main(args=None):
    rclpy.init(args=args)
    node = NavToPose()
    rclpy.spin_once(node, timeout_sec=2.0)  # give time to send the goal
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
