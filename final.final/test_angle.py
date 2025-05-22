import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from math import pi, cos, sin
import random
import time

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

# Initial angles in degrees
angles_deg = {
    'first': 0,  # First joint angle
    'second': 0,  # Second joint angle
    'third': 0    # Third joint angle
}

# Convert angles from degrees to radians
angles_rad = np.array([angles_deg['first'] * pi / 180, 
                       angles_deg['second'] * pi / 180, 
                       angles_deg['third'] * pi / 180])

# Set up the figure for the 3D plot
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Plot limits for better visualization
ax.set_xlim([-100, 100])
ax.set_ylim([-100, 100])
ax.set_zlim([0, 100])

ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.set_title("3D Arm Simulation")

# Turn on interactive mode for live updates
plt.ion()

# Function to update the plot based on new angles
def update_plot(angles_rad):
    joint_positions = forward_kinematics(dh_params, angles_rad)

    # Clear the previous plot
    ax.cla()

    # Replot with the updated angles
    x_vals = joint_positions[:, 0]
    y_vals = joint_positions[:, 1]
    z_vals = joint_positions[:, 2]

    # Plot the joints as red points with a label for the legend
    ax.scatter(x_vals, y_vals, z_vals, color='r', label="Joints")

    # Plot lines connecting the joints to represent the arm with labels for the legend
    for i in range(1, len(joint_positions)):
        ax.plot([x_vals[i-1], x_vals[i]], [y_vals[i-1], y_vals[i]], [z_vals[i-1], z_vals[i]], color='b', label=f"Link {i}")

    # Set labels and limits
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_xlim([-100, 100])
    ax.set_ylim([-100, 100])
    ax.set_zlim([0, 100])

    ax.set_title("3D Arm Simulation")

    # Show the legend for the plot
    ax.legend()

    # Redraw the figure and pause to simulate real-time updates
    plt.draw()
    plt.pause(0.1)  # Pause to allow the plot to update

# Simulate receiving new angles and updating the plot
for i in range(100):  # Simulate 100 iterations of receiving new angles
    # Randomize the joint angles slightly for demonstration (replace with actual ROS data in practice)
    angles_deg['first'] += random.uniform(-5, 5)
    angles_deg['second'] += random.uniform(-5, 5)
    angles_deg['third'] += random.uniform(-5, 5)

    # Convert angles to radians
    angles_rad = np.array([angles_deg['first'] * pi / 180, 
                           angles_deg['second'] * pi / 180, 
                           angles_deg['third'] * pi / 180])

    # Update the plot with the new angles
    update_plot(angles_rad)

    # Pause to simulate real-time updates
    time.sleep(0.5)

# Keep the plot window open after the loop ends
plt.ioff()  # Turn off interactive mode
plt.show()   # Show the final plot
