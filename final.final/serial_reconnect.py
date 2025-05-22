import serial

try:
    ser = serial.Serial('/dev/ttyUSB0', baudrate=115200, timeout=1)  # Replace 'COM3' with your port
    # print("Connected to ESP32")
    ser.write(b'ping\n')  # Test sending data
    while True:
        data = ser.readline().decode('utf-8').strip()
        if data:
            print(f"Received: {data}")
except serial.SerialException as e:
    print(f"Error: {e}")
except Exception as e:
    print(f"Unexpected Error: {e}")
finally:
    if 'ser' in locals() and ser.is_open:
        ser.close()
