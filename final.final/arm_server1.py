
import rclpy
from rclpy.node import Node
from final_rover.msg import ArmClient
import time
import serial
import numpy as np
from math import pi
from visual_kinematics.RobotSerial import RobotSerial, Frame
from geometry_msgs.msg import Twist


# Serial port setup
port = '/dev/ttyUSB0'
baudrate = 9600
ser = serial.Serial(port, baudrate, timeout=None)

# Global variables
rec_y = 0
command = 0
position = 0
pitch = 0
yaw = 0
gripper = 0
base=0


isPreset = False
speed = 5

angles = {
    'first': 0,
    'second': 0
}

target = {
    'first': 0,
    'second': 0
}

np.set_printoptions(precision=3, suppress=True)

# DH parameters and RobotSerial initialization
dh_params = np.array([[4.5, 4.0, -0.5*pi, 0.5 * pi],
                      [0.0, 54.55, 0.0, -0.5 * pi],
                      [0.0, 54.55, 0.0, 0.5 * pi]])

robot = RobotSerial(dh_params)



def arm_callback(data):
    global rec_y, command, position, pitch, yaw, gripper ,base

    rec_y = data.y
    command = data.command
    position = data.position
    pitch = data.pitch
    yaw = data.yaw
    gripper = data.gripper
    base = data.base


# ROS2 node setup !
rclpy.init()
node = Node('arm_server')
node.create_subscription(ArmClient, '/arm_controller_server', arm_callback, 10)


def goToPos1():
        global target, isPreset
        target = {'first': 0, 'second': 0}
        isPreset = True

def goToPos2():
        global target, isPreset
        target = {'first': 10, 'second': 20}
        isPreset = True



def arm_server():

    while rclpy.ok():
        rclpy.spin_once(node)

        global target, angles, isPreset, position, command,rec_y,pitch,yaw,gripper , base

        print("commnad :",command)
        print(f"Current angles: {angles}")
        print(f"Target angles : {target} ")

        angle_rad = {
            'first': angles['first'] * pi / 180,
            'second': angles['second'] * pi / 180,
        }

        theta = np.array([0, angle_rad['first'], angle_rad['second']])
        f = robot.forward(theta)
        x, y, z = f.t_3_1.reshape([3, ])

        print(f"Forward kinematics result -> x: {x}, y: {y}, z: {z}")

        if position == 1:
            goToPos1()
        elif position == 2:
            goToPos2()

        # Update target based on received command
        if command == 119:  # 'w' -> Move up
            print("Processing 'w' command")
            isPreset = False
            xyz = np.array([[x], [y + speed], [z]])
            abc = np.array([0, 0, 0])

            end = Frame.from_euler_3(abc, xyz)
            ang = robot.inverse(end)
            if ang is not None:
                ang = ang * 180 / pi
                _, new_first, new_second = ang
                print(f"New angles for 'w' command: first = {new_first}, second = {new_second}")
                target = {'first': new_first, 'second': new_second}
                angles = target  # Update the angles for the next iteration
            else:
                print("Inverse kinematics failed for 'w' command")

        elif command == 115:  # 's' -> Move down
            print("Processing 's' command")
            isPreset = False
            xyz = np.array([[x], [y - speed], [z]])
            abc = np.array([0, 0, 0])

            end = Frame.from_euler_3(abc, xyz)
            ang = robot.inverse(end)
            if ang is not None:
                ang = ang * 180 / pi
                _, new_first, new_second = ang
                print(f"New angles for 's' command: first = {new_first}, second = {new_second}")
                target = {'first': new_first, 'second': new_second}
                angles = target  # Update the angles for the next iteration
            else:
                print("Inverse kinematics failed for 's' command")

        elif command == 56:  # '8' -> Move up in z
            print("Processing '8' command")
            isPreset = False
            xyz = np.array([[x], [y], [z + speed]])
            abc = np.array([0, 0, 0])

            end = Frame.from_euler_3(abc, xyz)
            ang = robot.inverse(end)
            if ang is not None:
                ang = ang * 180 / pi
                _, new_first, new_second = ang
                print(f"New angles for '8' command: first = {new_first}, second = {new_second}")
                target = {'first': new_first, 'second': new_second}
                angles = target  # Update the angles for the next iteration
            else:
                print("Inverse kinematics failed for '8' command")

        elif command == 50:  # '2' -> Move down in z
            print("Processing '2' command")
            isPreset = False
            xyz = np.array([[x], [y], [z - speed]])
            abc = np.array([0, 0, 0])

            end = Frame.from_euler_3(abc, xyz)
            ang = robot.inverse(end)
            if ang is not None:
                ang = ang * 180 / pi
                _, new_first, new_second = ang
                print(f"New angles for '2' command: first = {new_first}, second = {new_second}")
                target = {'first': new_first, 'second': new_second}
                angles = target  # Update the angles for the next iteration
            else:
                print("Inverse kinematics failed for '2' command")

        # If not using predefined positions, update target with calculated angles
        elif not isPreset:
            target = angles


        # Send target angles :
        values = [target['first'], target['second'],rec_y,pitch,yaw,gripper , base]
        if command == 99:
             values = [0,0,rec_y,pitch,yaw,gripper , base]
        data = ','.join(map(str, values))
        ser.write(f"{data}\n".encode())
        print(f"Sent: {data}")

        # Receiving feedback.
        if ser.in_waiting > 0:
            line = ser.readline().decode('utf-8').rstrip()
            data_parts = line.split(",")
            if len(data_parts) == 2:
                new_first = float(data_parts[0])
                new_second = float(data_parts[1])

                angles = {
                    'first':new_first,
                    'second':new_second
                }
                print(f"Recieved : {new_first}, {new_second}")      
        else:
            print("No data received")

        time.sleep(0.1)  # Control the rate of sending/receiving !

    # Shutdown ROS 2 node 
    node.destroy_node()
    ser.close()
    rclpy.shutdown()


if __name__ == '__main__':
    arm_server()
