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

def create_serial_connection():
    """Creates and returns a serial connection."""
    try:
        ser = serial.Serial(port, baudrate, timeout=None)
        print(f"Connected to serial port {port}")
        return ser
    except serial.SerialException as e:
        print(f"Error opening serial port: {e}")
        return None

ser = create_serial_connection()

linear = 0
angular = 0 
longi = 0
lati = 0
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
    global linear, angular, longi, lati, alti, gps_pub, ser

    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec = 0)

        # Check if the serial connection is active
        if ser is None or not ser.is_open:
            print("Serial connection lost. Attempting to reconnect...")
            ser = create_serial_connection()
            if ser is None:
                time.sleep(1)  # Wait before retrying
                continue

        # Send movement command to the Arduino
        try:

            message = {
            "linear": linear,
            "angular": angular,
            }
            
            data = json.dumps(message)  
            ser.write(f"{data}\n".encode())
            print(f"Sent: {data}")

            # Receiving feedback from Arduino
            if ser.in_waiting > 0:
                line = ser.readline()
                try:
                    decoded_line = line.decode(errors='ignore').strip()
                    response = json.loads(decoded_line)
                    longi = response['longitude']
                    lati = response['latitude']
                    alti = response['altitude']
                    print(f"Longitude: {longi}, Latitude: {lati}, Altitude: {alti}")
                except UnicodeDecodeError:
                    print(f"Received non-UTF-8 data: {line}")
                except json.JSONDecodeError:
                    print("Failed to decode JSON response")
        except serial.SerialException as e:
            print(f"Serial communication error: {e}")
            ser = None  
            continue

        msg_gps = gps(
            longitude=float(longi),
            latitude=float(lati),
            altitude=float(alti),
        )
        gps_pub.publish(msg_gps)

        print(msg_gps)

        time.sleep(0.01)

    node.destroy_node()
    if ser:
        ser.close()
    rclpy.shutdown()

if __name__ == '__main__':
    rover_server()