import rclpy
from rclpy.node import Node
from final_rover.msg import EncodersFeedback, AnglesMsg, ArmClient
import threading
import numpy as np
from math import pi
from visual_kinematics.RobotSerial import *

class ArmServer(Node):
    def __init__(self):
        super().__init__('arm_server')
        
        self.sub_arm_client = self.create_subscription(ArmClient, '/arm_controller_server', self.arm_callback, 10)
        self.sub_angle_feedback = self.create_subscription(EncodersFeedback, '/angle_feedback_server', self.angles_callback, 10)
        self.pub_angles = self.create_publisher(AnglesMsg, '/arm_angles_server', 10)

        self.y = 0
        self.command = 'c'
        self.position = 0
        self.pitch = 0
        self.yaw = 0
        self.gripper = 0
        self.isPreset = False
        self.speed = 5

        self.angles = {'first': 0, 'second': 0}
        self.target = {'first': 0, 'second': 0}
        dh_params = np.array([[4.5, 4.0, -0.5 * pi, 0.5 * pi],
                               [0.0, 54.55, 0.0, -0.5 * pi],
                               [0.0, 54.55, 0.0 * pi, 0.5 * pi]])
        self.robot = RobotSerial(dh_params)

        self.publisher_thread = threading.Thread(target=self.publisher)
        self.publisher_thread.start()

    def arm_callback(self, data):
        self.y = data.y
        self.command = chr(data.command)
        self.position = data.position
        self.pitch = data.pitch
        self.yaw = data.yaw
        self.gripper = data.gripper

    def angles_callback(self, data):
        self.angles['first'] = data.first
        self.angles['second'] = data.second

    def publisher(self):
        r = self.create_rate(100)
        while r.sleep():
            msg_to_send = AnglesMsg(self.y, self.target['first'], self.target['second'], self.pitch, self.yaw, self.gripper)
            self.pub_angles.publish(msg_to_send)

    def go_to_pos(self, pos):
        if pos == 1:
            self.target = {'first': 0, 'second': 0}
        elif pos == 2:
            self.target = {'first': 10, 'second': 20}
        self.isPreset = True

    def main_loop(self):
        while rclpy.ok():
            angle_rad = {'first': self.angles['first'] * pi / 180, 'second': self.angles['second'] * pi / 180}
            theta = np.array([0, angle_rad['first'], angle_rad['second']])
            f = self.robot.forward(theta)
            x, y, z = f.t_3_1.reshape([3, ])

            if self.position == 1:
                self.go_to_pos(1)
            elif self.position == 2:
                self.go_to_pos(2)

            # Command handling
            self.handle_commands(x, y, z)

    def handle_commands(self, x, y, z):
        if self.command == 'w':
            self.isPreset = False
            xyz = np.array([[x], [y + self.speed], [z]])
            abc = np.array([0, 0, 0])
            self.update_target(xyz, abc)
        elif self.command == 's':
            self.isPreset = False
            xyz = np.array([[x], [y - self.speed], [z]])
            abc = np.array([0, 0, 0])
            self.update_target(xyz, abc)
        elif self.command == '8':
            self.isPreset = False
            xyz = np.array([[x], [y], [z + self.speed]])
            abc = np.array([0, 0, 0])
            self.update_target(xyz, abc)
        elif self.command == '2':
            self.isPreset = False
            xyz = np.array([[x], [y], [z - self.speed]])
            abc = np.array([0, 0, 0])
            self.update_target(xyz, abc)
        elif not self.isPreset:
            self.target = self.angles

    def update_target(self, xyz, abc):
        end = Frame.from_euler_3(abc, xyz)
        ang = self.robot.inverse(end)
        ang = ang * 180 / pi
        _, new_first, new_second = ang
        self.target = {'first': new_first, 'second': new_second}

def main():
    rclpy.init()
    arm_server = ArmServer()
    arm_server.main_loop()
    rclpy.spin(arm_server)

if __name__ == '__main__':
    main()
