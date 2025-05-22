import serial
import serial.tools.list_ports
import time
import json
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


def arm_callback(data):
    global rec_y, command, position, pitch, yaw, gripper ,base

    rec_y = data.y
    command = data.command
    position = data.position
    pitch = data.pitch
    yaw = data.yaw
    gripper = data.gripper
    base = data.base

    msg = {
        'rec_y' :rec_y,
        'command' : command,
        'position' : position,
        'pitch' : pitch,
        'yaw' : yaw,
        'gripper' : gripper,
        'base' : base
    }

    print(msg)


np.set_printoptions(precision=3, suppress=True)

# DH parameters and RobotSerial initialization
dh_params = np.array([[4.5, 4.0, -0.5*pi, 0.5 * pi],
                      [0.0, 54.55, 0.0, -0.5 * pi],
                      [0.0, 54.55, 0.0, 0.5 * pi]])

robot = RobotSerial(dh_params)

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


def find_serial_port():
    """Scan available serial ports and return the one that responds with 'rover_controller'."""
    ports = serial.tools.list_ports.comports()
    for port in ports:
        if "ACM" in port.device or "USB" in port.device:
            print(port.device)
            try:
                ser = serial.Serial(port.device, 115200, timeout= None)
                time.sleep(2)  # Give some time for the serial connection to stabilize
                
                # Send identification command
                identification_command = json.dumps({"command": "identify"}) + '\n'
                ser.write(identification_command.encode())
                try :
                    response = ser.readline().decode().strip()
                except UnicodeDecodeError as e:
                        print(f"UTF-8 decode error: {e}")
                        continue
                
                # Parse response
                if response:
                    data = json.loads(response)
                    if data.get("device_type") == "arm":
                        print(f"Connected to {port.device}")
                        return ser
                ser.close()
            except (serial.SerialException, json.JSONDecodeError):
                pass  # Try the next port
    return None

def main():

    global sent_bump,recieved_bump,rec_y, angles,node,target,angles
    global rec_y,command,position,gripper,pitch,yaw,base,isPreset

    serial_connection = None

    while rclpy.ok():
        rclpy.spin_once(node)

        if serial_connection is None:
            print("Searching for the arm ..")
            serial_connection = find_serial_port()
            if serial_connection:
                print("Arm connected!")
            else:
                print("Arm not found. Retrying ...")
                time.sleep(2)
                continue

        print("commnad :",command)
        print(f"Current angles: {angles}")
        print(f"Target angles : {target} ")

        try:


            angle_rad = {
            'first': angles['first'] * pi / 180,
            'second': angles['second'] * pi / 180,
        }

            theta = np.array([0, angle_rad['first'], angle_rad['second']])
            f = robot.forward(theta)
            x, y, z = f.t_3_1.reshape([3, ])

            # print(f"Forward kinematics result -> x: {x}, y: {y}, z: {z}")

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

            # If not using predefined positions, update target with calculated angles
            elif not isPreset:
                target = angles


            joint_angles = {
            "joint1": 0,  
            "joint2": angles['first'],
            "joint3": angles['second']
            }
            json_data = json.dumps(joint_angles)
            msg = String()
            msg.data = json_data
            pub_angle.publish(msg)


            if (command == 99):
                target = {
                    'first' :0,
                    'second':0
                }

            response = {
                   "Target_1": target['first'],
                    "Traget_2": target['second'],
                    "Indiviual": rec_y,
                    "Pitch": pitch,
                    "Yaw": yaw,
                    "Gripper": gripper,
                    "Base": base,
                }
    

            serial_connection.write((json.dumps(response) + '\n').encode())
            

            try:
                if serial_connection.in_waiting > 0:
                    incoming_data = serial_connection.readline().decode(errors='ignore').strip()
                    try:
                        response = json.loads(incoming_data)
                        if 'Angle_1' in response and 'Angle_2' in response:
                            angles = {
                                'first': response['Angle_1'],
                                'second': response['Angle_2']
                            }
                            print(f"Recieved : {response}")
                        else:
                            print(f"Missing angle data in response: {response}")
                            continue
                    except json.JSONDecodeError as e:
                        print(f"Failed to parse JSON: {e}")
                        print(f"Raw data received: {incoming_data}")
                        continue
                    except UnicodeDecodeError as e:
                        print(f"UTF-8 decode error: {e}")
                        continue
            except serial.SerialException as e:
                print(f"Serial communication error: {e}")
                serial_connection.close()
                serial_connection = None

        except serial.SerialException:
            print("Connection lost. Reconnecting...")
            serial_connection.close()
            serial_connection = None

        time.sleep(0.005)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
