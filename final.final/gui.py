import rclpy
import json
import tkinter as tk
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from final_rover.msg import ArmClient
from tkinter import ttk
from functools import partial

class ROS2GUI(Node):
    def __init__(self):
        super().__init__('GUI_node')
        
        # Initialize variables
        self.joint_shoulder = 0.0
        self.joint_elbow = 0.0
        self.base_speed = 0.0
        self.longi = 0.0
        self.lati = 0.0
        self.alti = 0.0
        self.linear_x = 0.0
        self.angular_z = 0.0
        self.encoder = 0
        
        # Create subscriptions
        self.create_subscription(String, 'joint_angles_json', self.joint_angles_callback, 10)
        self.create_subscription(String, 'gps_publisher', self.gps_callback, 10)
        self.create_subscription(Twist, 'rover_client', self.speed_callback, 10)
        self.create_subscription(ArmClient, 'arm_client', self.arm_callback, 10)
        
        # Initialize GUI
        self.root = tk.Tk()
        self.root.title("ROS 2 Data Monitor")
        self.root.geometry("600x400")
        
        # Create frames for organization
        self.joint_frame = ttk.LabelFrame(self.root, text="Arm Details", padding="10")
        self.joint_frame.grid(row=0, column=0, padx=10, pady=5, sticky="nsew")
        
        self.gps_frame = ttk.LabelFrame(self.root, text="GPS Data", padding="10")
        self.gps_frame.grid(row=1, column=0, padx=10, pady=5, sticky="nsew")
        
        self.speed_frame = ttk.LabelFrame(self.root, text="Speed Data", padding="10")
        self.speed_frame.grid(row=2, column=0, padx=10, pady=5, sticky="nsew")
        
        # Joint angle labels
        self.angle_shoulder_label = ttk.Label(self.joint_frame, text="Angle Shoulder: 0.0°")
        self.angle_shoulder_label.grid(row=0, column=0, padx=5, pady=2)
        
        self.angle_elbow_label = ttk.Label(self.joint_frame, text="Angle Elbow: 0.0°")
        self.angle_elbow_label.grid(row=0, column=1, padx=5, pady=2)
        
        self.base_speed_label = ttk.Label(self.joint_frame, text="Base Speed: 0.0")
        self.base_speed_label.grid(row=0, column=2, padx=5, pady=2)
        
        # GPS labels
        self.longi_label = ttk.Label(self.gps_frame, text="Longitude: 0.0°")
        self.longi_label.grid(row=0, column=0, padx=5, pady=2)
        
        self.lati_label = ttk.Label(self.gps_frame, text="Latitude: 0.0°")
        self.lati_label.grid(row=0, column=1, padx=5, pady=2)
        
        self.alti_label = ttk.Label(self.gps_frame, text="Altitude: 0.0 m")
        self.alti_label.grid(row=0, column=2, padx=5, pady=2)
        
        # Speed labels
        self.linear_label = ttk.Label(self.speed_frame, text="Linear: 0.0 ")
        self.linear_label.grid(row=0, column=0, padx=5, pady=2)
        
        self.angular_label = ttk.Label(self.speed_frame, text="Angular: 0.0")
        self.angular_label.grid(row=0, column=1, padx=5, pady=2)

        self.angular_label = ttk.Label(self.speed_frame, text="Angular: 0.0")
        self.angular_label.grid(row=0, column=1, padx=5, pady=2)

        self.encoder_label = ttk.Label(self.speed_frame, text="Encoder: True")
        self.encoder_label.grid(row=0, column=2, padx=5, pady=2)
        
        # Configure grid weights
        self.root.grid_rowconfigure(0, weight=1)
        self.root.grid_rowconfigure(1, weight=1)
        self.root.grid_rowconfigure(2, weight=1)
        self.root.grid_columnconfigure(0, weight=1)
        
    def joint_angles_callback(self, msg):
        try:
            joint_data = json.loads(msg.data)
            self.joint_shoulder = float(joint_data.get("angle1", 0))
            self.joint_elbow = float(joint_data.get("angle2", 0))
            self.encoder = joint_data.get("encoder",True)
            print(self.joint_shoulder)
            
            # Update labels
            self.root.after(0, self.update_joint_labels)
        except Exception as e:
            self.get_logger().error(f'Error in joint_angles_callback: {str(e)}')

    def update_joint_labels(self):
        self.angle_shoulder_label.config(text=f"Angle Shoulder: {self.joint_shoulder:.2f}°")
        self.angle_elbow_label.config(text=f"Angle Elbow: {self.joint_elbow:.2f}°")
        self.base_speed_label.config(text=f"Base Speed: {self.base_speed:.2f}°")
        encoder_status = "True" if self.encoder else "False"
        self.encoder_label.config(text=f"Encoder: {encoder_status}")
        
    def gps_callback(self, msg):
        try:
            gps_data = json.loads(msg.data)
            self.longi = float(gps_data.get("longitude", 0))
            self.lati = float(gps_data.get("latitude", 0))
            self.alti = float(gps_data.get("altitude", 0))
            
            # Update labels
            self.root.after(0, self.update_gps_labels)
        except Exception as e:
            self.get_logger().error(f'Error in gps_callback: {str(e)}')

    def update_gps_labels(self):
        self.longi_label.config(text=f"Longitude: {self.longi:.6f}°")
        self.lati_label.config(text=f"Latitude: {self.lati:.6f}°")
        self.alti_label.config(text=f"Altitude: {self.alti:.2f} m")
        
    def speed_callback(self, msg):
        try:
            self.linear_x = float(msg.linear.x)
            self.angular_z = float(msg.angular.z)
            
            # Update labels
            self.root.after(0, self.update_speed_labels)
        except Exception as e:
            self.get_logger().error(f'Error in speed_callback: {str(e)}')

    def update_speed_labels(self):
        self.linear_label.config(text=f"Linear X: {self.linear_x:.2f} m/s")
        self.angular_label.config(text=f"Angular Z: {self.angular_z:.2f} rad/s")

    def arm_callback(self, msg):
        try:
            self.base_speed = float(msg.base)
            # Update label
            self.root.after(0, self.update_joint_labels)
        except Exception as e:
            self.get_logger().error(f'Error in arm_callback: {str(e)}')
        
    def update_gui(self):
        try:
            self.root.update()
        except tk.TclError as e:
            self.get_logger().error(f'Error updating GUI: {str(e)}')
            return False
        return True

def main(args=None):
    rclpy.init(args=args)
    gui_node = ROS2GUI()
    
    try:
        while rclpy.ok():
            rclpy.spin_once(gui_node, timeout_sec=0.1)
            if not gui_node.update_gui():
                break
    except KeyboardInterrupt:
        pass
    finally:
        gui_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()