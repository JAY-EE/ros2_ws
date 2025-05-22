import rclpy
from rclpy.node import Node
from final_rover.msg import ArmClient as ArmClientMsg
from std_msgs.msg import Int32
import pygame
import time
import sys

class ArmClient(Node):
    def __init__(self):
        super().__init__('arm_client')
        self.pub_arm = self.create_publisher(ArmClientMsg, '/arm_client', 10)
        self.pub_science = self.create_publisher(Int32, '/science_client', 10)

        pygame.init()
        pygame.joystick.init()
        self.iskill = False
        self.step = 0

        # Initialize joystick-related attributes
        self.y = 0
        self.x = 0
        self.command = 'c'
        self.pitch = 0
        self.yaw = 0
        self.gripper = 0  
        self.base = 0

        self.z = 0
        self.position = 0
        self.kill = 0
        self.pos1 = 0
        self.pos2 = 0
        self.step1 = 0
        self.step2 = 0
        self.step3 = 0
        self.step4 = 0
        self.step3_axis = 0
        self.step4_axis = 0


        self.ac1_fwd = 0
        self.ac1_rev = 0
        self.ac2_fwd = 0
        self.ac2_rev = 0

        self.dpad = 0
        self.up_down = 0
        self.roll = 0

        self.grip_open = 0
        self.grp_close = 0
    
        self.setup_joystick()

    def setup_joystick(self):
        joystick_count = pygame.joystick.get_count()
        if joystick_count == 0:
            print("No joysticks found.")
            pygame.quit()
            sys.exit()

        self.joystick = pygame.joystick.Joystick(0)
        self.joystick.init()
        print("Joystick detected:", self.joystick.get_name())

    def main_loop(self):
        while rclpy.ok():
            pygame.event.pump()  # Get joystick events

            # Update joystick inputs
            self.update_joystick()

            # Handle joystick input logic
            self.handle_input()

            # Publish the joystick data
            self.publish_data()

    def update_joystick(self):
        # Update joystick values on every iteration

        
        self.x = self.joystick.get_axis(1)


        self.ac1_fwd = self.joystick.get_button(4)
        self.ac1_rev = self.joystick.get_button(2)
        self.ac2_fwd = self.joystick.get_button(5)
        self.ac2_rev = self.joystick.get_button(3)


        self.dpad = self.joystick.get_hat(0)
        self.dpad_x, self.dpad_y = self.dpad
        self.up_down = self.dpad_y
        self.roll = self.dpad_x

        self.grip_open = self.joystick.get_button(0)
        self.grip_close = self.joystick.get_button(1)


        #self.y = 60 * self.joystick.get_axis(0)
        # self.pitch = -50 * self.joystick.get_axis(4)
        # self.yaw = 100 * self.joystick.get_axis(3)
        # self.gripper = self.joystick.get_hat(0)[0]
        # self.kill = self.joystick.get_button(0)
        # self.pos1 = self.joystick.get_button(1)
        #self.pos2 = self.joystick.get_button(5)
        # self.step1 = self.joystick.get_button(4)
        # self.step2 = self.joystick.get_button(5)
        # self.step3 = self.joystick.get_button(6)
        # self.step4 = self.joystick.get_button(7)
        self.base = self.joystick.get_axis(2)
        # self.step4_axis = self.joystick.get_axis(5)

    def handle_input(self):

        # if self.kill == 1:
        #     self.handle_kill()

        # if self.step1 == 1:
        #     self.step = -40
        # elif self.step2 == 1:
        #     self.step = 40
        # elif self.step3 == 1 or self.step3_axis != -1:
        #     self.step = -10
        # elif self.step4 == 1 or self.step4_axis != -1:
        #     self.step = 10
        # else:
        #     self.step = 0

        # if self.pos1 == 1:
        #     self.position = 1
        # elif self.pos2 == 1:
        #     self.position = 2
        # else:
        #     self.position = 0



        if self.ac1_fwd > 0:
            self.y = 1
        elif self.ac1_rev == 1:
            self.y = 2
        elif self.ac2_fwd > 0:
            self.y = 3
        elif self.ac2_rev == 1:
            self.y = 4
        else:
            self.y = 0
        
        if self.up_down == 1:
            self.pitch = 1
        elif self.up_down == (-1):
            self.pitch = -1
        else : 
            self.pitch = 0

        if self.roll == 1:
            self.yaw = -1
        elif self.roll == (-1):
            self.yaw = 1
        else :
            self.yaw = 0

        if self.grip_open == 1:
            self.gripper = 1
        elif self.grip_close == 1:
            self.gripper = -1
        else :
            self.gripper = 0


        if self.base > 0.5:
            self.base = 1
        elif self.base < -0.5:
            self.base = -1
        else :
            self.base = 0
            
        print("base" , self.base)
        print("grip" , self.grip_open)
        print("gclose" ,self.grip_close)
        if self.x > 0.5:
            self.command = 's'
        elif self.x < -0.5:
            self.command = 'w'
        elif self.z == 1:
            self.command = '8'
        elif self.z == -1:
            self.command = '2'
        else:
            self.command = 'c'

    def publish_data(self):
        # Cast values to int before publishing
        msg_to_send = ArmClientMsg(
            y=int(self.y),  # Ensure y is an integer
            command=ord(self.command),  # Convert command back to integer
            position=self.position,
            pitch=int(self.pitch),  # Ensure pitch is an integer
            yaw=int(self.yaw),  # Ensure yaw is an integer
            gripper=int(self.gripper),
            base=int(self.base), 
        )
        self.pub_arm.publish(msg_to_send)

        # self.pub_science.publish(Int32(self.step))

    def handle_kill(self):

        self.get_logger().info('Bye!!!')
        self.command = 'c'
        self.position = 0
        self.y = 0
        self.pitch = 0
        self.yaw = 0
        self.gripper = 0
        self.base=0
        self.step = 0
        time.sleep(1)
        self.iskill = True
        pygame.quit()
        sys.exit()

def main():
    rclpy.init()
    arm_client = ArmClient()
    arm_client.main_loop()
    rclpy.spin(arm_client)

if __name__ == '__main__':
    main()
