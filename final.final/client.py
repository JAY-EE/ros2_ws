import pygame
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from final_rover.msg import ArmClient as ArmClientMsg
import sys

# Initialize Pygame and ROS 2
pygame.init()
pygame.joystick.init()

rclpy.init(args=None)
node = rclpy.create_node('Client_Server')

joysticks = []

kfront = 100
kleftright = 175
frontback = 0
rightleft = 0
msg_rover = 0

y = 0
x = 0
position = 0
command = 'c'
pitch = 0
yaw = 0
gripper = 0  
base = 0
base_speed = 0

ac1_fwd = 0
ac1_rev = 0
ac2_fwd = 0
ac2_rev = 0
bump_button = 0

dpad = 0
up_down = 0
roll = 0

grip_open = 0
grip_close = 0

joystick_arm = None
joystick_rover = None

# Initialize publishers
publisher_rover = node.create_publisher(Twist, '/rover_client', 10)
publisher_arm = node.create_publisher(ArmClientMsg, '/arm_client', 10)


def setup_joystick():
    global joystick_arm, joystick_rover

    joystick_count = pygame.joystick.get_count()
    print(f"Joystick count: {joystick_count}")

    if joystick_count == 1:
        joystick = pygame.joystick.Joystick(0)
        joystick.init()
        print(f"Joystick detected: {joystick.get_name()}")

        if joystick.get_name() == "Sony Interactive Entertainment Wireless Controller" or "Wireless Control":
            joystick_rover = None
            joystick_arm = joystick
        elif joystick.get_name() == "Logitech Extreme 3D pro":
            joystick_arm = None
            joystick_rover = joystick
        else:
            print("Unexpected joystick name !!")
            sys.exit()

    elif joystick_count > 1:
        joysticks = [pygame.joystick.Joystick(i) for i in range(2)]
        joysticks[0].init()
        joysticks[1].init()

        print(f"Joystick 1 detected: {joysticks[0].get_name()}")
        print(f"Joystick 2 detected: {joysticks[1].get_name()}")

        if joysticks[0].get_name() == "Sony Interactive Entertainment Wireless Controller" or "Wireless Controller":
            joystick_rover = joysticks[1]
            joystick_arm = joysticks[0]
        else:
            joystick_arm = joysticks[0]
            joystick_rover = joysticks[1]

    else:
        print("No Joysticks found.")
        pygame.quit()
        sys.exit()


def process_events():

    for event in pygame.event.get():
        if event.type == pygame.JOYDEVICEADDED:
            joystick = pygame.joystick.Joystick(event.device_index)
            joystick.init()
            print(f"Joystick {joystick.get_name()} added.")
            setup_joystick()


def update_joystick():
    
    global kfront, kleftright, msg_rover, x , ac1_fwd, ac1_rev, ac2_fwd, ac2_rev
    global up_down, roll, grip_close, grip_open, base, base_speed, bump_button

    msg_rover = Twist()

    if joystick_arm:
        x = joystick_arm.get_axis(1)
        ac1_fwd = joystick_arm.get_button(4)
        ac1_rev = joystick_arm.get_button(2)
        ac2_fwd = joystick_arm.get_button(5)
        ac2_rev = joystick_arm.get_button(3)
        dpad = joystick_arm.get_hat(0)
        dpad_x, dpad_y = dpad
        up_down = dpad_x
        roll = dpad_y
        grip_open = joystick_arm.get_button(0)
        grip_close = joystick_arm.get_button(1)
        base = joystick_arm.get_axis(2)
        base_speed = -(joystick_arm.get_axis(3))
        base_speed = (base_speed + 1)*50
        # bump_button = joystick_arm.get_button(1)

    if joystick_rover:
        frontback = joystick_rover.get_axis(1)
        rightleft = joystick_rover.get_axis(0)

        if joystick_rover.get_button(4) == 1:
            kfront += 2
            kleftright += 1

        if joystick_rover.get_button(5) == 1 and kfront > 1:
            kfront -= 2
            kleftright -= 1

        if abs(frontback) > 0.2:
            msg_rover.linear.x = float(-frontback * kfront)
        else:
            msg_rover.linear.x = 0.0

        if abs(rightleft) > 0.2:
            msg_rover.angular.z = float(-rightleft * kleftright)
        else:
            msg_rover.angular.z = 0.0

        rover_msg = {
            "linear": msg_rover.linear.x,
            "angular": msg_rover.angular.z
        }

        print(rover_msg)


def handle_input():
    global y, pitch, yaw, gripper, base, command, bump_button

    if ac1_fwd > 0:
        y = 1
    elif ac1_rev == 1:
        y = 2
    elif ac2_fwd > 0:
        y = 3
    elif ac2_rev == 1:
        y = 4
    # elif bump_button == 1:
    #     y = 5
    else:
        y = 0

    if up_down == 1:
        pitch = 1
    elif up_down == (-1):
        pitch = -1
    else:
        pitch = 0

    if roll == 1:
        yaw = -1
    elif roll == (-1):
        yaw = 1
    else:
        yaw = 0

    if grip_open == 1:
        gripper = 1
    elif grip_close == 1:
        gripper = -1
    else:
        gripper = 0

    if base > 0.5:
        base = 1*base_speed
    elif base < -0.5:
        base = -1*base_speed
    else:
        base = 0

    if x > 0.5:
        command = 's'
    elif x < -0.5:
        command = 'w'
    else:
        command = 'c'


def publish_data():
    global msg_rover, msg_arm

    publisher_rover.publish(msg_rover)

    msg_arm = ArmClientMsg(
        y=int(y),
        command=ord(command),
        position=position,
        pitch=int(pitch),
        yaw=int(yaw),
        gripper=int(gripper),
        base=int(base)
    )
    publisher_arm.publish(msg_arm)

    if joystick_arm:
        print(msg_arm)


def main_loop():

    while rclpy.ok():
        pygame.event.pump()

        update_joystick()

        handle_input()

        publish_data()

        process_events()

        rclpy.spin_once(node, timeout_sec= 0.01)

        # Exit condition
        if joysticks and joysticks[0].get_button(10):
            break

    # Shutdown once loop exits
    node.destroy_node()
    rclpy.shutdown()
    pygame.quit()


def main():
    global joystick_arm, joystick_rover
    setup_joystick()
    main_loop()

    node.destroy_node()
    rclpy.shutdown()
    pygame.quit()


if __name__ == '__main__':
    main()

