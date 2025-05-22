import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import random

class RandomTwistPublisher(Node):
    def __init__(self):
        super().__init__('random_twist_publisher')
        # Publisher to the cmd_vel topic
        self.publisher_ = self.create_publisher(Twist, '/rover_client', 10)
        # Timer to periodically publish messages
        self.timer = self.create_timer(0.5, self.publish_random_twist)  # Publish every 0.5 seconds
        self.get_logger().info('Random Twist Publisher Node has started.')

    def publish_random_twist(self):
        # Create a Twist message
        twist = Twist()
        # Generate random linear and angular velocities
        twist.linear.x = random.uniform(-1.0, 1.0)  # Linear velocity in x [-1, 1]
        twist.linear.y = random.uniform(-0.5, 0.5)  # Linear velocity in y [-0.5, 0.5]
        twist.linear.z = 0.0  # Linear velocity in z (set to 0)

        twist.angular.x = 0.0  # Angular velocity around x-axis (set to 0)
        twist.angular.y = 0.0  # Angular velocity around y-axis (set to 0)
        twist.angular.z = random.uniform(-1.0, 1.0)  # Angular velocity around z [-1, 1]

        # Log the values being published
        self.get_logger().info(f'Publishing: linear=({twist.linear.x}, {twist.linear.y}, {twist.linear.z}), angular=({twist.angular.x}, {twist.angular.y}, {twist.angular.z})')

        # Publish the message
        self.publisher_.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = RandomTwistPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Node stopped by user.')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()


# import serial
# import time

# try:
#     ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1)  # Adjust port if needed
#     ser.write(b'{"command":"identify"}\n')
#     time.sleep(2)  # Send a test command
#     while ser.in_waiting:
#         response = ser.readline().decode('utf-8').strip()
#         print(f"Received: {response}")
#     ser.close()
# except serial.SerialException as e:
#     print(f"Serial error: {e}")
