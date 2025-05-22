import rclpy
from rclpy.node import Node
from final_rover.msg import GpsMsg as gps
import time
import json
from geometry_msgs.msg import Twist

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
            if self.serial_port.in_waiting > 0:
                self.serial_port.readline()  # Discard ping response
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

linear = 0
angular = 0 
longi = 0
lati = 0
alti = 0  

def rover_callback(data):
    global linear, angular
    linear = float(data.linear.x)
    angular = float(data.angular.z)

def rover_server():
    serial_handler = SerialHandler(baudrate=9600)
    
    global linear, angular, longi, lati, alti, gps_pub
    
    while rclpy.ok():
        rclpy.spin_once(node)

        message = {
            "linear": linear,
            "angular": angular,
        }
        
        if serial_handler.write(json.dumps(message)):
            print(f"Sent: {message}")

        response_data = serial_handler.read_line()
        if response_data:
            try:
                response = json.loads(response_data)
                if 'longitude' in response:  # Only process GPS data responses
                    rec_linear = response['linear']
                    rec_angular = response['angular']
                    longi = response['longitude']
                    lati = response['latitude']
                    alti = response['altitude']
                    print(f"Recieved : linear: {rec_linear}, angular: {rec_angular} Longitude: {longi}, Latitude: {lati}, Altitude: {alti}")
            except json.JSONDecodeError:
                print('error')

        msg_gps = gps(
            longitude=float(longi),
            latitude=float(lati),
            altitude=float(alti),
        )
        gps_pub.publish(msg_gps)

        print(msg_gps)
        time.sleep(0.01)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    rclpy.init()
    node = Node('Rover_Server')
    node.create_subscription(Twist, '/cmd_vel', rover_callback, 10)
    gps_pub = node.create_publisher(gps, '/gps_data', 10)
    rover_server()