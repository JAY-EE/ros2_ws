import serial
import serial.tools.list_ports
import json
import time

class RobustSerialConnection:
    def __init__(self, device_id, baudrate=9600, timeout=None):
        """
        Initialize serial connection manager
        device_id: Expected identifier string from ESP32 ("arm" or "chassis")
        """
        self.device_id = device_id
        self.baudrate = baudrate
        self.timeout = timeout
        self.serial = None
        self.port = None
        
    def find_device(self):
        """Scan all ports and find the correct ESP32 by querying device ID"""
        available_ports = [
            port for port in serial.tools.list_ports.comports()
            if port.device.startswith('/dev/ttyACM') or port.device.startswith('/dev/ttyUSB')
        ]
        print(f"Filtered ports: {[port.device for port in available_ports]}")
        
        for port in available_ports:
            try:
                # Try to open the port
                test_serial = serial.Serial(
                    port=port.device,
                    baudrate=self.baudrate,
                    timeout=None  # Short timeout for testing
                )
                
                # Clear any pending data
                test_serial.reset_input_buffer()
                test_serial.reset_output_buffer()
                
                # Query device identity
                test_serial.write(json.dumps({"command": "identify"}).encode() + b'\n')
                time.sleep(0.1)  # Give device time to respond
                
                if test_serial.in_waiting:
                    response = test_serial.readline().decode('utf-8').strip()
                    try:
                        device_info = json.loads(response)
                        if device_info.get("device_type") == self.device_id:
                            self.port = port.device
                            test_serial.close()
                            return True
                    except json.JSONDecodeError:
                        pass
                
                test_serial.close()
                
            except (serial.SerialException, OSError):
                continue
                
        return False
    
    def ensure_connection(self):
        """Ensure serial connection is active, reconnect if necessary"""
        if self.serial is None or not self.serial.is_open:
            if self.find_device():
                try:
                    self.serial = serial.Serial(
                        port=self.port,
                        baudrate=self.baudrate,
                        timeout=self.timeout
                    )
                    print(f"Connected to {self.device_id} on port {self.port}")
                    return True
                except serial.SerialException as e:
                    print(f"Failed to connect to {self.device_id}: {e}")
                    return False
            return False
        return True
    
    def write(self, data):
        """Send data with automatic reconnection"""
        try:
            if self.ensure_connection():
                self.serial.write(data)
                return True
        except serial.SerialException:
            self.serial = None
        return False
    
    def readline(self):
        """Read line with automatic reconnection"""
        try:
            if self.ensure_connection():
                return self.serial.readline()
        except serial.SerialException:
            self.serial = None
        return None
    
    def close(self):
        """Close the serial connection"""
        if self.serial and self.serial.is_open:
            self.serial.close()