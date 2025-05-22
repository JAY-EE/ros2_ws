# # import rclpy
# # # from rclpy.node import Node
# # # from final_rover.msg import ArmClient
# # # from std_msgs.msg import Int32
# # # from geometry_msgs.msg import Twist
# # # import numpy as np
# # # from visual_kinematics.RobotSerial import *

# # # class ArmServer(Node):
# # #     def __init__(self):
# # #         super().__init__('arm_server')

# # #         self.sub_arm_client = self.create_subscription(ArmClient, '/arm_controller_server', self.arm_callback, 10)
# # #         self.pub_angles = self.create_publisher(Twist, '/arm_angles_server', 10)
# # #         self.angles_feedback = self.create_publisher()

# # #         self.y = 0
# # #         self.command = 'c'
# # #         self.position = 0
# # #         self.pitch = 0
# # #         self.yaw = 0
# # #         self.gripper = 0
# # #         self.speed = 5

# # #         self.angles = {'first': 0, 'second': 0}
# # #         self.target = {'first': 0, 'second': 0}

# # #         # Define DH parameters and create robot instance
# # #         dh_params = np.array([[4.5, 4.0, -0.5 * np.pi, 0.5 * np.pi],
# # #                                [0.0, 54.55, 0.0, -0.5 * np.pi],
# # #                                [0.0, 54.55, 0.0, 0.5 * np.pi]])
# # #         self.robot = RobotSerial(dh_params)

# # #         # Initialize a timer to publish angles at a regular interval
# # #         self.create_timer(0.1, self.publish_angles)  # Publish angles every 100 ms

# # #     def arm_callback(self, data):
# # #         self.y = data.y
# # #         self.command = chr(data.command)
# # #         self.position = data.position
# # #         self.pitch = data.pitch
# # #         self.yaw = data.yaw
# # #         self.gripper = data.gripper
# # #         self.handle_commands()  # Handle commands whenever data is received

# # #     def angles_callback(self,data):

# # #         self.angles['first'] = data.first
# # #         self.angles['second'] = data.second

# # #     def publish_angles(self):
# # #         msg_to_send = Twist()
# # #         msg_to_send.linear.x = float(self.y)
# # #         msg_to_send.linear.y = float(self.target['second'])
# # #         self.pub_angles.publish(msg_to_send)
# # #         self.get_logger().info(f'Published angles: {msg_to_send}')

# # #     def go_to_pos(self, pos):
# # #         if pos == 1:
# # #             self.target = {'first': 0, 'second': 0}
# # #         elif pos == 2:
# # #             self.target = {'first': 10, 'second': 20}

# # #     def handle_commands(self):
# # #         if self.command == 'w':
# # #             self.target['second'] += self.speed
# # #         elif self.command == 's':
# # #             self.target['second'] -= self.speed
# # #         elif self.command == '8':
# # #             self.target['first'] += self.speed
# # #         elif self.command == '2':
# # #             self.target['first'] -= self.speed

# # # def main():
# # #     rclpy.init()
# # #     arm_server = ArmServer()
# # #     rclpy.spin(arm_server)  # Spin the node to keep it alive

# # # if __name__ == '__main__':
# # #     main()




# # import rclpy
# # from rclpy.node import Node
# # from final_rover.msg import EncodersFeedback, AnglesMsg, ArmClient
# # import threading
# # from visual_kinematics import Frame
# # from visual_kinematics.RobotSerial import *
# # from geometry_msgs.msg import Twist
# # import numpy as np
# # from math import pi
# # import serial
# # import time

# # port = '/dev/ttyUSB0'
# # baudrate = 9600
# # ser = serial.Serial(port,baudrate,timeout=None)
# # time.sleep(1)

# # def send_recieve():
# #     while True:
# #         values = [target['first'],target['second']]
# #         data = ','.join(map(str,values))
# #         ser.write(f"{data}\n".encode())
# #         print(f"Sent : {data}")

# #         if ser.in_waiting > 0 :
# #             line = ser.readline().decode('utf-8').rstrip()
# #             print(f"Recieved : {line}")
# #         else:
# #             print("xyx")

# # y = 0
# # command = 'c'
# # position = 0
# # pitch = 0
# # yaw = 0
# # gripper = 0

# # isPreset = False
# # speed = 5

# # angles = {
# #     'first': 0,
# #     'second': 0
# # }

# # target = {
# #     'first': 0,
# #     'second': 0
# # }

# # np.set_printoptions(precision=3, suppress=True)

# # class RobotSerial:
# #     def __init__(self, dh_params):
# #         self.dh_params = dh_params

# #     def forward(self, theta):
# #         # Calculate the transformation matrix t_4_4 based on theta
# #         # This is a placeholder. Replace it with your actual kinematics logic.
# #         t_4_4 = np.identity(4)  # Example: Identity matrix
# #         return Frame(t_4_4)

# #     def inverse(self, end):
# #         # Placeholder for actual inverse kinematics implementation
# #         return np.array([0, 0, 0])  # Replace with actual angles


# # class ArmServer(Node):  
# #     def __init__(self):
# #         super().__init__('arm_server')

# #         self.subscription_arm = self.create_subscription(ArmClient, '/arm_controller_server', self.arm_callback, 10)
# #         self.subscription_angles = self.create_subscription(Twist, '/angle_feedback_server', self.angles_callback, 10)
# #         self.publisher = self.create_publisher(Twist, '/arm_angles_server', 10)

# #         self.timer = self.create_timer(0.01, self.publish_angles)  # Publish at 100 Hz

# #         self.dh_params = np.array([[4.5, 4.0, -0.5 * pi, 0.5 * pi],
# #                                    [0.0, 54.55, 0.0, -0.5 * pi],
# #                                    [0.0, 54.55, 0.0 * pi, 0.5 * pi]])
# #         self.robot = RobotSerial(self.dh_params)

# #         self.publisher_thread = threading.Thread(target=self.main_loop)
# #         self.publisher_thread.start()

# #     def arm_callback(self, data):
# #         global y, command, position, pitch, yaw, gripper

# #         y = data.y
# #         command = chr(data.command)
# #         position = data.position
# #         pitch = data.pitch
# #         yaw = data.yaw
# #         gripper = data.gripper

# #     def angles_callback(self, data):
# #         global angles
# #         angles = {
# #             'first': data.data[0],
# #             'second': data.data[1]
# #         }

# #         print(f"Angle 1: {data.data[0]}, Angle 2: {data.data[1]}")

# #     def publish_angles(self):
# #         global y, target, pitch, yaw, gripper
# #         msg_to_send = Twist()
# #         #msg_to_send.y = float(y)
# #         msg_to_send.linear.x = float(target['first'])
# #         msg_to_send.linear.y = float(target['second'])
# #         #msg_to_send.pitch = float(pitch)
# #         #msg_to_send.yaw = float(yaw)
# #         #msg_to_send.gripper = int(gripper)
# #         self.publisher.publish(msg_to_send)

# #     def goToPos1(self):
# #         global target, isPreset
# #         target = {'first': 0, 'second': 0}
# #         isPreset = True

# #     def goToPos2(self):
# #         global target, isPreset
# #         target = {'first': 10, 'second': 20}
# #         isPreset = True

# #     def main_loop(self):
# #         global robot, angles, target, speed, isPreset, command, position
        
# #         while True:

# #             send_recieve()


# #             angle_rad = {
# #                 'first': angles['first'] * pi / 180,
# #                 'second': angles['second'] * pi / 180,
# #             }

# #             theta = np.array([0, angle_rad['first'], angle_rad['second']])
# #             f = self.robot.forward(theta)

# #             x, y, z = f.t_3_1.reshape([3, ])

# #             if position == 1:
# #                 self.goToPos1()
# #             elif position == 2:
# #                 self.goToPos2()

# #             if command == 'w':
# #                 isPreset = False
# #                 xyz = np.array([[x], [y + speed], [z]])
# #                 abc = np.array([0, 0, 0])
# #                 end = Frame.from_euler_3(abc, xyz)  # Adjust this method as needed
# #                 ang = self.robot.inverse(end) * 180 / pi
# #                 _, new_first, new_second = ang

# #                 target = {
# #                     'first': new_first,
# #                     'second': new_second,
# #                 }
# #             elif command == 's':
# #                 isPreset = False
# #                 xyz = np.array([[x], [y - speed], [z]])
# #                 abc = np.array([0, 0, 0])
# #                 end = Frame.from_euler_3(abc, xyz)  # Adjust this method as needed
# #                 ang = self.robot.inverse(end) * 180 / pi
# #                 _, new_first, new_second = ang

# #                 target = {
# #                     'first': new_first,
# #                     'second': new_second,
# #                 }
# #             elif command == '8':
# #                 isPreset = False
# #                 xyz = np.array([[x], [y], [z + speed]])
# #                 abc = np.array([0, 0, 0])
# #                 end = Frame.from_euler_3(abc, xyz)  # Adjust this method as needed
# #                 ang = self.robot.inverse(end) * 180 / pi
# #                 _, new_first, new_second = ang

# #                 target = {
# #                     'first': new_first,
# #                     'second': new_second,
# #                 }
# #             elif command == '2':
# #                 isPreset = False
# #                 xyz = np.array([[x], [y], [z - speed]])
# #                 abc = np.array([0, 0, 0])
# #                 end = Frame.from_euler_3(abc, xyz)  # Adjust this method as needed
# #                 ang = self.robot.inverse(end) * 180 / pi
# #                 _, new_first, new_second = ang

# #                 target = {
# #                     'first': new_first,
# #                     'second': new_second,
# #                 }
# #             elif not isPreset:
# #                 target = angles

          


# # def main(args=None):
# #     rclpy.init(args=args)
# #     arm_server = ArmServer()
# #     try:
# #         rclpy.spin(arm_server)
# #     except KeyboardInterrupt:
# #         pass
# #     finally:
# #         arm_server.destroy_node()
# #         rclpy.shutdown()


# # if __name__ == '__main__':
# #     main()

# import rclpy
# from rclpy.node import Node
# from final_rover.msg import ArmClient
# from geometry_msgs.msg import Twist
# import threading
# import numpy as np
# from visual_kinematics import Frame
# from visual_kinematics import *
# from math import pi
# import serial
# import time

# # Serial port setup
# port = '/dev/ttyACM0'
# baudrate = 9600
# ser = serial.Serial(port, baudrate, timeout=None)

# # Global variables
# y = 0
# command = 'c'
# position = 0
# pitch = 0
# yaw = 0
# gripper = 0
# isPreset = False
# speed = 5
# angles = {'first': 0, 'second': 0}
# target = {'first': 0, 'second': 0}

# np.set_printoptions(precision=3, suppress=True)

# # RobotSerial class for kinematics calculations
# class RobotSerial:
#     def __init__(self, dh_params):
#         self.dh_params = dh_params

#     def forward(self, theta):
#         # Calculate the transformation matrix t_4_4 based on theta
#         t_4_4 = np.identity(4)  # Placeholder for actual forward kinematics
#         return Frame(t_4_4)

#     def inverse(self, end):
#         # Placeholder for actual inverse kinematics implementation
#         return np.array([0, 0, 0])  # Replace with actual angles


# # ArmServer Node
# class ArmServer(Node):  
#     def __init__(self):
#         super().__init__('arm_server')

#         # ROS 2 subscribers and publishers
#         self.subscription_arm = self.create_subscription(ArmClient, '/arm_controller_server', self.arm_callback, 10)

#         # Timer for periodic publishing of arm angles
#         #self.create_timer(0.01, self.send_recieve)  # Publish at 100 Hz

#         # Initialize robot with DH parameters
#         self.dh_params = np.array([[4.5, 4.0, -0.5 * pi, 0.5 * pi],
#                                    [0.0, 54.55, 0.0, -0.5 * pi],
#                                    [0.0, 54.55, 0.0, 0.5 * pi]])
#         self.robot = RobotSerial(self.dh_params)

#         # Event to manage background thread
#         self._thread_stop_event = threading.Event()

#         # Start serial communication thread
#         self.publisher_thread = threading.Thread(target=self.main_loop)
#         self.publisher_thread.start()

#         self.pysrial_thread = threading.Thread(target=self.send_recieve)
#         self.pysrial_thread.start()

#     def arm_callback(self, data):
#         global y, command, position, pitch, yaw, gripper
#         y = data.y
#         command = chr(data.command)
#         position = data.position
#         pitch = data.pitch
#         yaw = data.yaw
#         gripper = data.gripper

#         self.get_logger().info(f"Received command: {command}")

#     def send_recieve(self):
#         global target
#         # while not self._thread_stop_event.is_set():
#         while rclpy.ok():
#             # Sending target angles to Arduino
#             values = [target['first'], target['second']]
#             data = ','.join(map(str, values))
#             ser.write(f"{data}\n".encode())
#             print(f"Sent: {data}")

#             # Receiving feedback from Arduino
#             if ser.in_waiting > 0:
#                 line = ser.readline().decode('utf-8').rstrip()
#                 print(f"Received: {line}")
#             else:
#                 print("No data received")
#             time.sleep(0.1)  # Control the rate of sending/receiving

#     def goToPos1(self):
#         global target, isPreset
#         target = {'first': 0, 'second': 0}
#         isPreset = True

#     def goToPos2(self):
#         global target, isPreset
#         target = {'first': 10, 'second': 20}
#         isPreset = True

#     def main_loop(self):
#         print('arm_server')
#         global robot, angles, target, speed, isPreset, command, position

#         while rclpy.ok():
#             # Continuously send and receive data in the background

#             # Convert angles to radians
#             angle_rad = {'first': angles['first'] * pi / 180, 'second': angles['second'] * pi / 180}
#             theta = np.array([0, angle_rad['first'], angle_rad['second']])
#             f = self.robot.forward(theta)
#             x, y, z = f.t_3_1.reshape([3, ])

#             # Handle positions based on command
#             if position == 1:
#                 self.goToPos1()
#             elif position == 2:
#                 self.goToPos2()

#             # Move robot based on command
#             if command == 'w':
#                 isPreset = False
#                 xyz = np.array([[x], [y + speed], [z]])
#                 abc = np.array([0, 0, 0])
#                 end = Frame.from_euler_3(abc, xyz)  # Adjust this method as needed
#                 ang = self.robot.inverse(end) * 180 / pi
#                 _, new_first, new_second = ang

#                 target = {'first': new_first, 'second': new_second}
    
#             elif command == 's':
#                 isPreset = False
#                 xyz = np.array([[x], [y - speed], [z]])
#                 abc = np.array([0, 0, 0])
#                 end = Frame.from_euler_3(abc, xyz)  # Adjust this method as needed
#                 ang = self.robot.inverse(end) * 180 / pi
#                 _, new_first, new_second = ang

#                 target = {'first': new_first, 'second': new_second}
#             elif command == '8':
#                 isPreset = False
#                 xyz = np.array([[x], [y], [z + speed]])
#                 abc = np.array([0, 0, 0])
#                 end = Frame.from_euler_3(abc, xyz)  # Adjust this method as needed
#                 ang = self.robot.inverse(end) * 180 / pi
#                 _, new_first, new_second = ang

#                 target = {'first': new_first, 'second': new_second}
#             elif command == '2':
#                 isPreset = False
#                 xyz = np.array([[x], [y], [z - speed]])
#                 abc = np.array([0, 0, 0])
#                 end = Frame.from_euler_3(abc, xyz)  # Adjust this method as needed
#                 ang = self.robot.inverse(end) * 180 / pi
#                 _, new_first, new_second = ang

#                 target = {'first': new_first, 'second': new_second}
#             elif not isPreset:
#                 target = angles


#     def stop_thread(self):
#         # Stop the background thread gracefully
#         self._thread_stop_event.set()
#         self.publisher_thread.join()  # Ensure the thread completes
#         self.pysrial_thread.join()


# def main(args=None):
#     rclpy.init(args=args)
#     arm_server = ArmServer()
#     try:
#         rclpy.spin(arm_server)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         arm_server.stop_thread()  # Stop the background thread
#         ser.close()  # Close the serial connection
#         arm_server.destroy_node()
#         rclpy.shutdown()

# if __name__ == '__main__':
#     main()


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

# Serial port setup
port = '/dev/ttyACM0'
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


def arm_callback(data):
    global y, command, position, pitch, yaw, gripper
    y = data.y
    command = chr(data.command)
    position = data.position
    pitch = data.pitch
    yaw = data.yaw
    gripper = data.gripper

    #print(f"Received command: {command}")


def send_recieve():
    global target
    while rclpy.ok():
        # Sending target angles to Arduino
        values = [target['first'], target['second']]
        data = ','.join(map(str, values))
        ser.write(f"{data}\n".encode())
        #print(f"Sent: {data}")

        # Receiving feedback from Arduino
        if ser.in_waiting > 0:
            line = ser.readline().decode('utf-8').rstrip()
            #print(f"Received: {line}")
        else:
            #print("No data received")
            pass
        
        time.sleep(0.1)  # Control the rate of sending/receiving


def goToPos1():
    global target, isPreset
    target = {'first': 0, 'second': 0}
    isPreset = True


def goToPos2():
    global target, isPreset
    target = {'first': 10, 'second': 20}
    isPreset = True


def main_loop(robot, angles, target, speed, isPreset, command, position):

    print('arm_server')
    while rclpy.ok():
        # Continuously send and receive data in the background

        # Convert angles to radians
        angle_rad = {'first': angles['first'] * pi / 180, 'second': angles['second'] * pi / 180}
        theta = np.array([0, angle_rad['first'], angle_rad['second']])
        f = robot.forward(theta)
        x, y, z = f.t_3_1.reshape([3, ])

        # Handle positions based on command
        if position == 1:
            goToPos1()
        elif position == 2:
            goToPos2()

        # Move robot based on command
        if command == 'w':
            isPreset = False
            xyz = np.array([[x], [y + speed], [z]])
            abc = np.array([0, 0, 0])
            end = Frame.from_euler_3(abc, xyz)  # Adjust this method as needed
            ang = robot.inverse(end) * 180 / pi
            _, new_first, new_second = ang

            target = {'first': new_first, 'second': new_second}

            print(target)
    
        elif command == 's':
            isPreset = False
            xyz = np.array([[x], [y - speed], [z]])
            abc = np.array([0, 0, 0])
            end = Frame.from_euler_3(abc, xyz)  # Adjust this method as needed
            ang = robot.inverse(end) * 180 / pi
            _, new_first, new_second = ang

            target = {'first': new_first, 'second': new_second}
            print(target)
        elif command == '8':
            isPreset = False
            xyz = np.array([[x], [y], [z + speed]])
            abc = np.array([0, 0, 0])
            end = Frame.from_euler_3(abc, xyz)  # Adjust this method as needed
            ang = robot.inverse(end) * 180 / pi
            _, new_first, new_second = ang

            target = {'first': new_first, 'second': new_second}
        elif command == '2':
            isPreset = False
            xyz = np.array([[x], [y], [z - speed]])
            abc = np.array([0, 0, 0])
            end = Frame.from_euler_3(abc, xyz)  # Adjust this method as needed
            ang = robot.inverse(end) * 180 / pi
            _, new_first, new_second = ang

            target = {'first': new_first, 'second': new_second}
        elif not isPreset:
            target = angles

       

def main(args=None):
    # Initialize ROS 2 node
    rclpy.init(args=args)
    node = Node('arm_server')

    # Create the RobotSerial instance
    dh_params = np.array([[4.5, 4.0, -0.5 * pi, 0.5 * pi],
                          [0.0, 54.55, 0.0, -0.5 * pi],
                          [0.0, 54.55, 0.0, 0.5 * pi]])
    robot = RobotSerial(dh_params)

    # Set up the ROS 2 subscriptions
    node.create_subscription(ArmClient, '/arm_controller_server', arm_callback, 10)

    # Start the serial communication and main loop in separate threads
    publisher_thread = threading.Thread(target=main_loop, args=(robot, angles, target, speed, isPreset, command, position))
    publisher_thread.start()

    pysrial_thread = threading.Thread(target=send_recieve)
    pysrial_thread.start()

    try:
        rclpy.spin(node)  # Keep the node alive
    except KeyboardInterrupt:
        pass
    finally:
        # Close the serial connection and shut down the ROS node
        publisher_thread.join()
        pysrial_thread.join()
        ser.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

