import rclpy
from rclpy.node import Node
from final_rover.msg import GpsMsg as gps
import time
import serial
import json
from geometry_msgs.msg import Twist

# Serial port setup
port = '/dev/ttyUSB0'
baudrate = 9600
ser = serial.Serial(port, baudrate, timeout=None)

linear = 0
angular = 0 
longi = 0
lati = 30
alti = 0  

def rover_callback(data):
    global linear, angular

    linear = float(data.linear.x)
    angular = float(data.angular.z)


rclpy.init()
node = Node('Rover_Server')
node.create_subscription(Twist, '/cmd_vel', rover_callback, 10)
gps_pub = node.create_publisher(gps, '/gps_data', 10)


def rover_server():

    global linear, angular, longi, lati, alti, gps_pub
    
    while rclpy.ok():
        rclpy.spin_once(node)

        # Send movement command to the Arduino
        message = {
            "linear": linear,
            "angular": angular,
        }
        data = json.dumps(message)  
        ser.write(f"{data}\n".encode())
        print(f"Sent: {data}")

        # Receiving feedback from Arduino
        if ser.in_waiting > 0:
            line = ser.readline().decode(errors='ignore').rstrip()
            try:
                response = json.loads(line) 
                
                longi = response['longitude']
                lati = response['latitude']
                alti = response['altitude']
                print(f"Longitude: {longi}, Latitude: {lati}, Altitude: {alti}")
            except json.JSONDecodeError:
                print("Failed to decode JSON response")

        # print(gps.__slots__)  # To inspect the field names in the message

        # Publish GPS data
        msg_gps = gps(
            longitude=float(longi),
            latitude=float(lati),
            altitude=float(alti),
        )
        gps_pub.publish(msg_gps)

        print(msg_gps)

        time.sleep(0.01) 

    # Shutdown ROS 2 node 
    node.destroy_node()
    ser.close()
    rclpy.shutdown()


if __name__ == '__main__':
    rover_server()
