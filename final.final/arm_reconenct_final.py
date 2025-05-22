
import rclpy
from rclpy.node import Node
from final_rover.msg import ArmClient
import time
import serial
import numpy as np
import json
from math import pi
from visual_kinematics.RobotSerial import RobotSerial, Frame
import sys
from std_msgs.msg import String


# port = str(sys.argv[1])
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
sent_bump = 0
recieved_bump = 0


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
pub_angle = node.create_publisher(String,'/encoder_angle',10)


def goToPos1():
        global target, isPreset
        target = {'first': 0, 'second': 0}
        isPreset = True

def goToPos2():
        global target, isPreset
        target = {'first': 10, 'second': 20}
        isPreset = True

def create_serial_connection():
    """Attempt to create a serial connection."""
    while True:
        try:
            ser = serial.Serial(port, baudrate, timeout=None)
            print(f"Successfully connected to {port}")
            return ser
        except serial.SerialException as e:
            print(f"Serial connection error: {e}.")
            return None

ser = create_serial_connection()


def arm_server():
    global ser
    ser = create_serial_connection()  

    while rclpy.ok():
        rclpy.spin_once(node)

        global target, angles, isPreset, position, command, rec_y, pitch, yaw, gripper, base, pub_angle
        global sent_bump, recieved_bump

        if ser is None or not ser.is_open:
            print("Serial connection lost. Attempting to reconnect...")
            ser = create_serial_connection()
            if ser is None:
                time.sleep(1)  # Wait before retrying
                continue

        print("Command:", command)
        print(f"Current angles: {angles}")
        print(f"Target angles: {target}")

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

       
        message = {
                "Target_1": target['first'],
                "Traget_2": target['second'],
                "Indiviual": rec_y,
                "Pitch": pitch,
                "Yaw": yaw,
                "Gripper": gripper,
                "Base": base,
                "Bump" : sent_bump
            }
        
        if (command ==  99 ):

             message = {
                "Target_1": 0,
                "Traget_2": 0,
                "Indiviual": rec_y,
                "Pitch": pitch,
                "Yaw": yaw,
                "Gripper": gripper,
                "Base": base,
                "Bump" : sent_bump
            }
             

        try:
            data = json.dumps(message)  
            ser.write(f"{data}\n".encode())
            print(f"Sent: {data}")


            if ser.in_waiting > 0:
                    incoming_data = ser.readline().decode(errors='ignore').strip()
                    try:
                        response = json.loads(incoming_data)
                        if 'Angle_1' in response and 'Angle_2' in response:
                            angles = {
                                'first': response['Angle_1'],
                                'second': response['Angle_2']
                            }
                            recieved_bump = response.get('Bump', 0)
                        else:
                            print(f"Missing angle data in response: {response}")
                            continue
                    except json.JSONDecodeError as e:
                        print(f"Failed to parse JSON: {e}")
                        print(f"Raw data received: {incoming_data}")
                        continue

            # if ser.in_waiting > 0:
            #     line = ser.readline()
            #     try:
            #         # Try to decode as UTF-8
            #         decoded_line = line.decode('utf-8').rstrip()
            #         response = json.loads(decoded_line)
            #         angles = {
            #         'first': response['Angle_1'],
            #         'second': response['Angle_2']
            #         } 
            #         recieved_bump = response['Bump']
            #     except UnicodeDecodeError:
            #         print(f"Received non-UTF-8 data: {line}")
            #     except json.JSONDecodeError:
            #         print("Failed to decode JSON response")


        except serial.SerialException as e:
            print(f"Serial communication error: {e}")
            ser = None  # Mark the connection as lost
            continue

        if recieved_bump == 1:
                sent_bump = 1
                
        if sent_bump == 1:
            if rec_y == 5:
                sent_bump = 0



         # Publish joint angles
        joint_angles = {
            "joint1": 0,
            "joint2": angles['first'],
            "joint3": angles['second']
        }
        json_data = json.dumps(joint_angles)
        msg = String()
        msg.data = json_data
        pub_angle.publish(msg)
        print(f"Publishing joint angles: {json_data}")

        time.sleep(0.1)  # Control the rate of sending/receiving

    # Shutdown ROS 2 node
    node.destroy_node()
    if ser:
        ser.close()
    rclpy.shutdown()


if __name__ == '__main__':
    arm_server()
