
# import rclpy
# from rclpy.node import Node
# from final_rover.msg import ArmClient as ArmClientMsg
# from std_msgs.msg import Int16

# class ArmPublisher(Node):
#     def __init__(self):
#         super().__init__('arm_publisher')
#         self.pub_arm = self.create_publisher(ArmClientMsg, '/arm_client', 10)
#         self.pub_science = self.create_publisher(Int16, '/science_client', 10)

#         self.y = 0
#         self.command = 'c'
#         self.position = 0
#         self.pitch = 0
#         self.yaw = 0
#         self.gripper = 0

#         self.timer = self.create_timer(0.5, self.timer_callback)  # Changed to 0.5 seconds (2 Hz)

#     def timer_callback(self):
#         msg_to_send = ArmClientMsg(
#             y=self.y,
#             command=ord(self.command),
#             position=self.position,
#             pitch=self.pitch,
#             yaw=self.yaw,
#             gripper=self.gripper
#         )
#         self.pub_arm.publish(msg_to_send)
        
#         # Correctly publish the Int16 message
#         self.pub_science.publish(Int16(data=self.y))  # Use 'data' as the argument

#         self.get_logger().info(f'Published: {msg_to_send}')  # Log published data

#         # Increment parameters for testing
#         self.y += 1
#         self.position += 1
#         self.pitch += 1
#         self.yaw += 1
#         self.gripper += 1

# def main():
#     rclpy.init()
#     arm_publisher = ArmPublisher()
#     rclpy.spin(arm_publisher)
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()


import pygame
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16
from final_rover.msg import ArmClient as ArmClientMsg # Updated for ROS2 message type
from pygame import joystick
import sys
import threading

# Global Variables
y = 0
command = 'c'
pos = 0
pitch = 0
yaw = 0
gripper = 0
step = 0
iskill = False

# ROS2 Node
class ArmClientNode(Node):
    def __init__(self):
        super().__init__('arm_client')
        self.get_logger().info('Press x to exit.')

        # Initialize Publishers
        self.pub_arm = self.create_publisher(ArmClientMsg, '/arm_client', 10)
        self.pub_science = self.create_publisher(Int16, '/science_client', 10)

        # Timer for publishing
        self.create_timer(0.01, self.publish_messages)  # 10Hz

        # Pygame Joystick Initialization
        pygame.init()
        pygame.joystick.init()
        joystick_count = pygame.joystick.get_count()

        if joystick_count == 0:
            self.get_logger().error("No joysticks found.")
            pygame.quit()
            sys.exit()

        self.joystick = pygame.joystick.Joystick(1)
        self.joystick.init()
        self.get_logger().info("Joystick detected: %s", self.joystick.get_name())

    def publish_messages(self):
        global y, command, pos, pitch, yaw, gripper, step, iskill

        if iskill:
            return

        # Create and publish messages
        msg = ArmClientMsg(float(y), ord(command), int(pos), float(pitch), float(yaw), int(gripper))
        self.pub_arm.publish(msg)
        self.pub_science.publish(Int16(step))

    def stop(self):
        global iskill
        iskill = True
        self.get_logger().info('Shutting down...')
        pygame.quit()


def main():
    rclpy.init()

    # Create ROS2 node
    arm_client_node = ArmClientNode()

    # Thread for handling joystick updates
    def joystick_input():
        global y, command, pos, pitch, yaw, gripper, step, iskill

        while True:
            pygame.event.pump()
            
            # Get joystick input
            y = 60 * arm_client_node.joystick.get_axis(0)
            x = +arm_client_node.joystick.get_axis(1)
            pitch = -50 * arm_client_node.joystick.get_axis(4)
            yaw = 100 * arm_client_node.joystick.get_axis(3)
            z = arm_client_node.joystick.get_hat(0)[1]
            gripper = arm_client_node.joystick.get_hat(0)[0]
            kill = arm_client_node.joystick.get_button(0)
            pos1 = arm_client_node.joystick.get_button(1)
            pos2 = arm_client_node.joystick.get_button(2)
            # step1 = arm_client_node.joystick.get_button(4)
            # step2 = arm_client_node.joystick.get_button(5)
            # step3 = arm_client_node.joystick.get_button(6)
            # step4 = arm_client_node.joystick.get_button(7)
            # step3_axis = arm_client_node.joystick.get_axis(2)
            # step4_axis = arm_client_node.joystick.get_axis(5)

            if kill == 1:
                arm_client_node.stop()
                time.sleep(1)
                sys.exit()

            # if step1 == 1:
            #     step = -40
            # elif step2 == 1:
            #     step = 40
            # elif step3 == 1 or step3_axis != -1:
            #     step = -10
            # elif step4 == 1 or step4_axis != -1:
            #     step = 10
            # else:
            #     step = 0

            if pos1 == 1:
                pos = 1
            elif pos2 == 1:
                pos = 2
            else:
                pos = 0

            if x > 0.5:
                command = 'w'
            elif x < -0.5:
                command = 's'
            elif z == 1:
                command = '8'
            elif z == -1:
                command = '2'
            else:
                command = 'c'

    # Start the joystick input thread
    joystick_thread = threading.Thread(target=joystick_input)
    joystick_thread.daemon = True
    joystick_thread.start()

    # Spin ROS2 node
    try:
        rclpy.spin(arm_client_node)
    except KeyboardInterrupt:
        arm_client_node.stop()
        pygame.quit()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
