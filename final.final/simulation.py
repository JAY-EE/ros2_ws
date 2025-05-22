import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from math import pi, cos, sin

# DH parameters and forward kinematics function
def dh_transform(a, alpha, d, theta):
    """Returns the transformation matrix for a single link based on DH parameters."""
    return np.array([
        [cos(theta), -sin(theta)*cos(alpha), sin(theta)*sin(alpha), a*cos(theta)],
        [sin(theta), cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta)],
        [0, sin(alpha), cos(alpha), d],
        [0, 0, 0, 1]
    ])

def forward_kinematics(dh_params, joint_angles):
    """Calculate the forward kinematics for a multi-joint robot arm."""
    transformation = np.eye(4)  # Start with the identity matrix
    joint_positions = []

    # Loop over each joint and apply the DH transformation
    for i, (a, alpha, d, theta) in enumerate(dh_params):
        theta_i = joint_angles[i] if i < len(joint_angles) else 0  # Use joint angle for the ith joint
        T = dh_transform(a, alpha, d, theta_i)
        transformation = transformation @ T  # Update the cumulative transformation matrix
        joint_positions.append(transformation[:3, 3])  # Extract the position of the joint (x, y, z)

    return np.array(joint_positions)

# Define DH parameters for the arm
dh_params = np.array([
    [4.0,   4.5, -0.5 * pi, 0.5 * pi],  # Link 1: a, alpha, d, theta
    [54.55, 0.0, 0.0, -0.5 * pi],     # Link 2: a, alpha, d, theta
    [54.55, 0.0, 0.0, 0.5 * pi]       # Link 3: a, alpha, d, theta
])

class ArmSimulation(Node):
    def __init__(self):
        super().__init__('arm_simulation')
        self.subscription = self.create_subscription(
            String,
            'joint_angles_json',
            self.listener_callback,
            10
        )
        self.subscription  # prevent unused variable warning

        # Set up the figure for the 3D plot
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(111, projection='3d')
        self.ax.set_xlim([-100, 100])
        self.ax.set_ylim([-100, 100])
        self.ax.set_zlim([0, 100])
        self.ax.set_xlabel('X')
        self.ax.set_ylabel('Y')
        self.ax.set_zlabel('Z')
        self.ax.set_title("3D Arm Simulation")

        # Turn on interactive mode for live updates
        plt.ion()

    def listener_callback(self, msg):
        joint_angles = json.loads(msg.data)  # Parse the JSON data
        joint1 = joint_angles.get("joint1", 0)
        joint2 = joint_angles.get("joint2", 0)
        joint3 = joint_angles.get("joint3", 0)

        # Convert angles to radians
        joint_angles_rad = np.array([joint1, joint2, joint3])

        # Update the plot with the new angles
        self.update_plot(joint_angles_rad)

    def update_plot(self, joint_angles_rad):
        joint_positions = forward_kinematics(dh_params, joint_angles_rad)

        # Clear the previous plot
        self.ax.cla()

        # Replot with the updated angles
        x_vals = joint_positions[:, 0]
        y_vals = joint_positions[:, 1]
        z_vals = joint_positions[:, 2]

        # Plot the joints as red points
        self.ax.scatter(x_vals, y_vals, z_vals, color='r', label="Joints")

        # Plot lines connecting the joints to represent the arm
        for i in range(1, len(joint_positions)):
            self.ax.plot([x_vals[i-1], x_vals[i]], [y_vals[i-1], y_vals[i]], [z_vals[i-1], z_vals[i]], color='b')

        # Set labels and limits
        self.ax.set_xlabel('X')
        self.ax.set_ylabel('Y')
        self.ax.set_zlabel('Z')
        self.ax.set_xlim([-100, 100])
        self.ax.set_ylim([-100, 100])
        self.ax.set_zlim([0, 100])

        self.ax.set_title("3D Arm Simulation")
        self.ax.legend()

        # Redraw the figure and pause to simulate real-time updates
        plt.draw()
        plt.pause(0.1)  # Pause to allow the plot to update

def main():
    rclpy.init()
    arm_simulation = ArmSimulation()
    rclpy.spin(arm_simulation)

    # Close the ROS 2 node
    arm_simulation.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
