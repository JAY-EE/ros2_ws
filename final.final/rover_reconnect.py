import serial
import serial.tools.list_ports
import time
import json
import rclpy
from rclpy.node import Node
from final_rover.msg import GpsMsg as gps
import time
import serial
import json
from geometry_msgs.msg import Twist


def find_serial_port():
    """Scan available serial ports and return the one that responds with 'rover_controller'."""
    ports = serial.tools.list_ports.comports()
    for port in ports:
        if "ACM" in port.device or "USB" in port.device:
            print(port.device)
            try:
                ser = serial.Serial(port.device, 115200, timeout=1)
                time.sleep(2)  # Give some time for the serial connection to stabilize
                
                # Send identification command
                identification_command = json.dumps({"command": "identify"}) + '\n'
                ser.write(identification_command.encode())
                try:
                    response = ser.readline().decode().strip()
                except UnicodeDecodeError as e:
                        print(f"UTF-8 decode error: {e}")
                        continue
                
                # Parse response
                if response:
                    data = json.loads(response)
                    if data.get("device_type") == "rover":
                        print(f"Connected to {port.device}")
                        return ser
                ser.close()
            except (serial.SerialException, json.JSONDecodeError):
                pass  # Try the next port
    return None


linear = 0
angular = 0 
longi = 0
lati = 10
alti = 40 

def rover_callback(data):
    global linear, angular

    linear = float(data.linear.x)
    angular = float(data.angular.z)



rclpy.init()
node = Node('Rover_Server')
node.create_subscription(Twist, '/cmd_vel', rover_callback, 10)
gps_pub = node.create_publisher(gps, '/gps_data', 10)



def main():
    global node, linear, angular, longi, lati, alti

    serial_connection = None

    while rclpy.ok:
        rclpy.spin_once(node)

        if serial_connection is None:
            print("Searching for the rover...")
            serial_connection = find_serial_port()
            if serial_connection:
                print("Rover connected!")
            else:
                print("Rover not found. Retrying...")
                time.sleep(2)
                continue

        try:

            response = {
                    "linear": linear,
                    "angular": angular
                }
            serial_connection.write((json.dumps(response) + '\n').encode())
            
            # Read data from the rover
            if serial_connection.in_waiting > 0:
                line = serial_connection.readline().decode(errors='ignore').rstrip()
                if line:  # Check if line is not empty
                    try:
                        response = json.loads(line)
                        # Safely get values with defaults if keys don't exist
                        new_longi = response.get('lo', longi)  # Use existing value as default
                        new_lati = response.get('la', lati)
                        new_alti = response.get('al', alti)
                        
                        # Only update if we got valid values
                        longi = new_longi
                        lati = new_lati
                        alti = new_alti
                        
                        print(f"Longitude: {longi}, Latitude: {lati}, Altitude: {alti}")
                    except json.JSONDecodeError as e:
                        print(f"JSON parsing error: {e}")
                        print(f"Raw data received: {line}")

                    except UnicodeDecodeError as e:
                        print(f"UTF-8 decode error: {e}")
                        continue
                
            msg_gps = gps(
            longitude=float(longi),
            latitude=float(lati),
            altitude=float(alti),
            )
            gps_pub.publish(msg_gps)

            print(msg_gps)

        except serial.SerialException:
            print("Connection lost. Reconnecting...")
            serial_connection.close()
            serial_connection = None

        time.sleep(0.01)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
