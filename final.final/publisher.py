import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import random
import time
from math import pi


class AnglePublisher(Node):
    def __init__(self):
        super().__init__('angle_publisher')
        self.publisher = self.create_publisher(String, 'joint_angles_json', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)  # Publish every second

    def timer_callback(self):
        # Generate random joint angles for a 3-DOF robotic arm
        joint_angles = {
            "joint1": random.uniform(-pi, pi),  # Random angle between -pi and pi
            "joint2": random.uniform(-pi, pi),
            "joint3": random.uniform(-pi, pi)
        }

        # Serialize the joint angles into a JSON string
        json_data = json.dumps(joint_angles)

        # Publish the JSON data
        msg = String()
        msg.data = json_data
        self.publisher.publish(msg)

        self.get_logger().info(f"Publishing joint angles: {json_data}")


def main():
    rclpy.init()
    angle_publisher = AnglePublisher()
    rclpy.spin(angle_publisher)
    angle_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
