import rclpy
from rclpy.node import Node
from final_rover.msg import ArmClient
import time
import json
from math import pi
from visual_kinematics.RobotSerial import RobotSerial, Frame
import sys
from std_msgs.msg import String
import numpy as np

class SerialHandler:
    def __init__(self, baudrate: int, timeout: float = 1.0):
        self.baudrate = baudrate
        self.timeout = timeout
        self.serial_port = None
        self.last_successful_port = None
        self.device_identifier = None
        
    def get_device_info(self, port: str):
        try:
            import serial
            with serial.Serial(port, self.baudrate, timeout=1) as ser:
                time.sleep(2)
                ser.reset_input_buffer()
                ser.reset_output_buffer()
                
                ser.write(b'{"command": "identify"}\n')
                time.sleep(0.1)
                
                if ser.in_waiting > 0:
                    response = ser.readline().decode('utf-8').strip()
                    try:
                        return json.loads(response)
                    except json.JSONDecodeError:
                        return None
        except:
            return None
            
    def find_device(self):
        import serial.tools.list_ports
        # ports = list(serial.tools.list_ports.comports())

        ports = [
            port for port in serial.tools.list_ports.comports()
            if port.device.startswith('/dev/ttyACM') or port.device.startswith('/dev/ttyUSB')
        ]
        print(f"Filtered ports: {[port.device for port in ports]}")
        
        if self.last_successful_port:
            device_info = self.get_device_info(self.last_successful_port)
            if device_info and self._is_matching_device(device_info):
                return self.last_successful_port
                    
        for port in ports:
            if port.device != self.last_successful_port:
                device_info = self.get_device_info(port.device)
                if device_info and self._is_matching_device(device_info):
                    return port.device
        
        return None
        
    def _is_matching_device(self, device_info):
        if self.device_identifier is None:
            self.device_identifier = device_info
            return True
        
        return (device_info.get('board_type') == self.device_identifier.get('board_type') and
                device_info.get('firmware_version') == self.device_identifier.get('firmware_version'))
    
    def ensure_connection(self):
        import serial
        if self.serial_port is None or not self.serial_port.is_open:
            return self._establish_connection()
            
        try:
            # Send ping silently without expecting a response
            self.serial_port.write(b'{"command": "ping"}\n')
            # if self.serial_port.in_waiting > 0:
            #     self.serial_port.readline()  # Discard ping response
            return True
        except:
            self.serial_port = None
            return self._establish_connection()
    
    def _establish_connection(self):
        import serial
        try:
            port = self.find_device()
            if port:
                self.serial_port = serial.Serial(port, self.baudrate, timeout=self.timeout)
                self.last_successful_port = port
                time.sleep(2)
                return True
        except:
            pass
        return False
    
    def write(self, data: str):
        if not self.ensure_connection():
            return False
            
        try:
            self.serial_port.write(f"{data}\n".encode())
            return True
        except:
            self.serial_port = None
            return False
    
    def read_line(self):
        if not self.ensure_connection():
            return None
            
        try:
            if self.serial_port.in_waiting > 0:
                response = self.serial_port.readline().decode('utf-8').rstrip()
                try:
                    data = json.loads(response)
                    # Filter out status responses
                    if 'status' in data and len(data) == 1:
                        return None
                    return response
                except json.JSONDecodeError:
                    return None
        except:
            self.serial_port = None
        return None

# Global variables
rec_y = 0
command = 0
position = 0
pitch = 0
yaw = 0
gripper = 0
base = 0
sent_bump = False
recieved_bump = False

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

# DH parameters and RobotSerial initialization
dh_params = np.array([[4.5, 4.0, -0.5*pi, 0.5 * pi],
                      [0.0, 54.55, 0.0, -0.5 * pi],
                      [0.0, 54.55, 0.0, 0.5 * pi]])

robot = RobotSerial(dh_params)

def arm_callback(data):
    global rec_y, command, position, pitch, yaw, gripper, base
    rec_y = data.y
    command = data.command
    position = data.position
    pitch = data.pitch
    yaw = data.yaw
    gripper = data.gripper
    base = data.base

def goToPos1():
    global target, isPreset
    target = {'first': 0, 'second': 0}
    isPreset = True

def goToPos2():
    global target, isPreset
    target = {'first': 10, 'second': 20}
    isPreset = True

def arm_server():
    serial_handler = SerialHandler(baudrate=9600)
    
    while rclpy.ok():
        rclpy.spin_once(node)

        global target, angles, isPreset, position, command, rec_y
        global pitch, yaw, gripper, base, pub_angle, sent_bump, recieved_bump

        # print("command:", command)
        # print(f"Current angles: {angles}")
        # print(f"Target angles: {target}")

        angle_rad = {
            'first': angles['first'] * pi / 180,
            'second': angles['second'] * pi / 180,
        }

        theta = np.array([0, angle_rad['first'], angle_rad['second']])
        f = robot.forward(theta)
        x, y, z = f.t_3_1.reshape([3, ])

        # print(f"Forward kinematics result -> x: {x}, y: {y}, z: {z}")

        # Process position commands
        if position == 1:
            goToPos1()
        elif position == 2:
            goToPos2()

        # Your existing command processing code here
        if command == 119:  # 'w'
            # Process 'w' command code
            pass
        # ... rest of your command processing ...

        joint_angles = {
            "joint1": 0,  
            "joint2": angles['first'],
            "joint3": angles['second']
        }
        json_data = json.dumps(joint_angles)
        msg = String()
        msg.data = json_data
        pub_angle.publish(msg)

        message = {
            "Target_1": target['first'],
            "Target_2": target['second'],
            "Indiviual": rec_y,
            "Pitch": pitch,
            "Yaw": yaw,
            "Gripper": gripper,
            "Base": base,
            "Bool": sent_bump
        }
        
        if serial_handler.write(json.dumps(message)):
            print(f"Sent: {message}")

        response_data = serial_handler.read_line()
        if response_data:
            try:
                response = json.loads(response_data)
                if 'Angle_1' in response:  # Only process angle data responses
                    angles = {
                        'first': response['Angle_1'],
                        'second': response['Angle_2']
                    }
                    recieved_bump = response['Bump']
                    print(response)
            except json.JSONDecodeError:
                pass

        print(angles)

        if recieved_bump and rec_y == 5:
            sent_bump = False

        time.sleep(0.1)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    rclpy.init()
    node = Node('arm_server')
    node.create_subscription(ArmClient, '/arm_controller_server', arm_callback, 10)
    pub_angle = node.create_publisher(String, '/encoder_angle', 10)
    arm_server()