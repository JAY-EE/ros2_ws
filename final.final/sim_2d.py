import rclpy
from rclpy.node import Node
import json
import matplotlib.pyplot as plt
import numpy as np
from std_msgs.msg import String


class TwoLinkRoboticArmSimulator(Node):
    def __init__(self):
        super().__init__('two_link_robotic_arm_simulator')
        self.subscription = self.create_subscription(
            String,
            'joint_angles_json',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

        # Initialize plot
        plt.ion()  # Interactive mode on
        self.figure, self.ax = plt.subplots()
        self.ax.set_xlim(-30, 200)
        self.ax.set_ylim(-30, 200)
        self.ax.set_aspect('equal', 'box')
        self.line, = self.ax.plot([], [], 'o-', lw=2)  # Robotic arm plot
        self.base = np.array([0, 0])  # Base is stationary at origin

    def listener_callback(self, msg):
        global joint1
        try:


            # if np.abs(joint1[0]) > 50 or np.abs(joint1[1]) > 50:
    
            # Parse the JSON data
            data = json.loads(msg.data)
            angle1 = np.radians(data['angle1'])  # Convert to radians
            angle2 = -(np.radians(data['angle2']) ) # Convert to radians

            # Link lengths
            link1_length = 54.0
            link2_length = 54.0

            # Calculate joint positions
            # First link: tilted at angle1 from the vertical axis
            joint1 = self.base + link1_length * np.array([np.sin(angle1), np.cos(angle1)])

            # Second link: angle2 is measured from the horizontal axis
            end_effector = joint1 + link2_length * np.array([np.cos(angle2), np.sin(angle2)])

            if np.abs(joint1[0]) > 50 :
                self.get_logger().warn('Shoulder reaching max')
            
            if (np.abs(joint1[0]) > 50) and (np.abs(end_effector[0]) > 50) :
                self.get_logger().warn('Elbow reaching max')


            # Update the plot
            self.update_plot(joint1, end_effector)
        except Exception as e:
            self.get_logger().error(f"Failed to parse message: {e}")

    def update_plot(self, joint1, end_effector):
        # Update the line data
        x_data = [self.base[0], joint1[0], end_effector[0]]
        y_data = [self.base[1], joint1[1], end_effector[1]]
        self.line.set_data(x_data, y_data)

        # Redraw the plot
        self.figure.canvas.draw()
        self.figure.canvas.flush_events()

def main(args=None):
    rclpy.init(args=args)
    robotic_arm_simulator = TwoLinkRoboticArmSimulator()

    try:
        rclpy.spin(robotic_arm_simulator)
    except KeyboardInterrupt:
        robotic_arm_simulator.get_logger().info('Shutting down simulator.')
    finally:
        robotic_arm_simulator.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
