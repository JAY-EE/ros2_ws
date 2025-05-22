import serial
import time

# Initialize serial connection to Arduino
arduino = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)
time.sleep(2)  # Wait for Arduino to initialize

# Initial target angles (example)
target_angle_first = 0
target_angle_second = 0

while True:
    # Read the current angles sent by Arduino
    if arduino.in_waiting > 0:
        try:
            # Attempt to read and decode the data from the Arduino
            data = arduino.readline().decode('utf-8').strip()
            if data:
                angles = data.split(',')
                angle_first = int(angles[0])
                angle_second = int(angles[1])
                print(f"Current Angles from Arduino: {angle_first}, {angle_second}")
        except UnicodeDecodeError:
            # If a UnicodeDecodeError occurs, skip the current data
            print("Invalid data received, skipping...")

    # Send the target angles to Arduino
    target_data = f"{target_angle_first},{target_angle_second}\n"
    arduino.write(target_data.encode())
    print(f"Sent Target Angles: {target_angle_first}, {target_angle_second}")

    # Example control of target angles (increment them)
    target_angle_first += 1
    target_angle_second += 1
    if target_angle_first > 360:
        target_angle_first = 0
    if target_angle_second > 360:
        target_angle_second = 0

    time.sleep(1)  # Send data every second
