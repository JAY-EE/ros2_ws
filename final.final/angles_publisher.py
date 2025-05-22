import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json

class JointAnglesPublisher(Node):
    def __init__(self):
        super().__init__('joint_angles_publisher')
        self.publisher_ = self.create_publisher(String, 'joint_angles_json', 10)
        
        # Initialize joint angles
        self.joint_angles = {
            "angle1": 90,  # Incremented
            "angle2": 100  # Incremented
            # 'angle3':0
        }
        
        # Timer to publish every second
        self.timer = self.create_timer(1.0, self.publish_joint_angles)

    def publish_joint_angles(self):
        # Increment joint2 and joint3 angles by 2 degrees, wrap around at 360
        # self.joint_angles["angle1"] = (self.joint_angles["angle1"] + 5) 
        self.joint_angles["angle2"] = (self.joint_angles["angle2"] - 0.5) 
        # self.joint_angles["angle3"] = (self.joint_angles["angle3"] + 3) % 360

        # Convert angles to JSON format
        angles_json = json.dumps(self.joint_angles)

        # Create and publish the message
        msg = String()
        msg.data = angles_json
        self.publisher_.publish(msg)

        # Log the message
        self.get_logger().info(f"Publishing: {angles_json}")

def main():
    rclpy.init()
    joint_angles_publisher = JointAnglesPublisher()
    rclpy.spin(joint_angles_publisher)

    # Shutdown
    joint_angles_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
