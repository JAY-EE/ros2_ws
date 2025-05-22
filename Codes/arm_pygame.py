import rclpy
from rclpy.node import Node
from final_rover.msg import ArmClient
from geometry_msgs.msg import Twist
import threading
import numpy as np
from visual_kinematics import Frame
from visual_kinematics import *
from math import pi
import serial
import time
import pygame  # Import pygame for joystick handling

# Serial port setup
port = '/dev/ttyUSB0'
baudrate = 9600
ser = serial.Serial(port, baudrate, timeout=None)

# Global variables
y = 0
command = 'c'
position = 0
pitch = 0
yaw = 0
gripper = 0
isPreset = False
speed = 5
angles = {'first': 0, 'second': 0}
target = {'first': 0, 'second': 0}

np.set_printoptions(precision=3, suppress=True)

# RobotSerial class for kinematics calculations
class RobotSerial:
    def __init__(self, dh_params):
        self.dh_params = dh_params

    def forward(self, theta):
        # Calculate the transformation matrix t_4_4 based on theta
        t_4_4 = np.identity(4)  # Placeholder for actual forward kinematics
        return Frame(t_4_4)

    def inverse(self, end):
        # Placeholder for actual inverse kinematics implementation
        return np.array([0, 0, 0])  # Replace with actual angles


# ArmServer Node
class ArmServer(Node):  
    def __init__(self):
        super().__init__('arm_server')

        # Initialize pygame and joystick
        pygame.init()
        pygame.joystick.init()

        # Check if joystick is connected
        if pygame.joystick.get_count() == 0:
            self.get_logger().info("No joystick detected!")
            return
        
        self.joystick = pygame.joystick.Joystick(0)
        self.joystick.init()
        self.get_logger().info(f"Joystick initialized: {self.joystick.get_name()}")

        # ROS 2 subscribers and publishers
        self.subscription_arm = self.create_subscription(ArmClient, '/arm_controller_server', self.arm_callback, 10)

        # Timer for periodic publishing of arm angles
        self.create_timer(0.01, self.send_recieve)  # Publish at 100 Hz

        # Initialize robot with DH parameters
        self.dh_params = np.array([[4.5, 4.0, -0.5 * pi, 0.5 * pi],
                                   [0.0, 54.55, 0.0, -0.5 * pi],
                                   [0.0, 54.55, 0.0, 0.5 * pi]])
        self.robot = RobotSerial(self.dh_params)

        # Event to manage background thread
        self._thread_stop_event = threading.Event()

        # Start serial communication thread
        self.publisher_thread = threading.Thread(target=self.main_loop)
        self.publisher_thread.start()

    def arm_callback(self, data):
        global y, command, position, pitch, yaw, gripper
        y = data.y
        command = chr(data.command)
        position = data.position
        pitch = data.pitch
        yaw = data.yaw
        gripper = data.gripper

    def send_recieve(self):
        global target
        while not self._thread_stop_event.is_set():
            # Sending target angles to Arduino
            values = [target['first'], target['second']]
            data = ','.join(map(str, values))
            ser.write(f"{data}\n".encode())
            print(f"Sent: {data}")

            # Receiving feedback from Arduino
            if ser.in_waiting > 0:
                line = ser.readline().decode('utf-8').rstrip()
                print(f"Received: {line}")
            else:
                print("No data received")
            time.sleep(0.1)  # Control the rate of sending/receiving

    def goToPos1(self):
        global target, isPreset
        target = {'first': 0, 'second': 0}
        isPreset = True

    def goToPos2(self):
        global target, isPreset
        target = {'first': 10, 'second': 20}
        isPreset = True

    def handle_joystick_input(self):
        global command, target
        pygame.event.pump()  # Update pygame events

        # Joystick axes and buttons
        axis_0 = self.joystick.get_axis(0)  # X-axis (left-right)
        axis_1 = self.joystick.get_axis(1)  # Y-axis (up-down)
        button_0 = self.joystick.get_button(0)  # Button 0 (e.g., trigger)
        
        # Map joystick axis to commands or targets
        if axis_1 > 0.1:
            command = 'w'  # Move up (forward)
        elif axis_1 < -0.1:
            command = 's'  # Move down (backward)
        elif axis_0 > 0.1:
            command = '8'  # Move right
        elif axis_0 < -0.1:
            command = '2'  # Move left
        elif button_0:
            command = 'c'  # Example button press (e.g., stop or reset)

    def main_loop(self):
        global robot, angles, target, speed, isPreset, command, position
        while True:
            self.handle_joystick_input()  # Check joystick state and update command

            # Convert angles to radians
            angle_rad = {'first': angles['first'] * pi / 180, 'second': angles['second'] * pi / 180}
            theta = np.array([0, angle_rad['first'], angle_rad['second']])
            f = self.robot.forward(theta)
            x, y, z = f.t_3_1.reshape([3, ])

            # Handle positions based on command
            if position == 1:
                self.goToPos1()
            elif position == 2:
                self.goToPos2()

            # Move robot based on command
            if command == 'w':
                isPreset = False
                xyz = np.array([[x], [y + speed], [z]])
                abc = np.array([0, 0, 0])
                end = Frame.from_euler_3(abc, xyz)  # Adjust this method as needed
                ang = self.robot.inverse(end) * 180 / pi
                _, new_first, new_second = ang

                target = {'first': new_first, 'second': new_second}
            elif command == 's':
                isPreset = False
                xyz = np.array([[x], [y - speed], [z]])
                abc = np.array([0, 0, 0])
                end = Frame.from_euler_3(abc, xyz)  # Adjust this method as needed
                ang = self.robot.inverse(end) * 180 / pi
                _, new_first, new_second = ang

                target = {'first': new_first, 'second': new_second}
            elif command == '8':
                isPreset = False
                xyz = np.array([[x], [y], [z + speed]])
                abc = np.array([0, 0, 0])
                end = Frame.from_euler_3(abc, xyz)  # Adjust this method as needed
                ang = self.robot.inverse(end) * 180 / pi
                _, new_first, new_second = ang

                target = {'first': new_first, 'second': new_second}
            elif command == '2':
                isPreset = False
                xyz = np.array([[x], [y], [z - speed]])
                abc = np.array([0, 0, 0])
                end = Frame.from_euler_3(abc, xyz)  # Adjust this method as needed
                ang = self.robot.inverse(end) * 180 / pi
                _, new_first, new_second = ang

                target = {'first': new_first, 'second': new_second}
            elif not isPreset:
                target = angles

    def stop_thread(self):
        # Stop the background thread gracefully
        self._thread_stop_event.set()
        self.publisher_thread.join()  # Ensure the thread completes


def main(args=None):
    rclpy.init(args=args)
    arm_server = ArmServer()
    try:
        rclpy.spin(arm_server)
    except KeyboardInterrupt:
        pass
    finally:
        arm_server.stop_thread()  # Stop the background thread
