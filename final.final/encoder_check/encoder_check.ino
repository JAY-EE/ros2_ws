#include <Wire.h>

// I2C addresses and registers
#define AS5600_ADDR 0x36
#define ANGLE_REG_H 0x0E
#define ANGLE_REG_L 0x0F

// Define the I2C buses
TwoWire I2C_0 = TwoWire(0); // Primary I2C bus
TwoWire I2C_1 = TwoWire(1); // Secondary I2C bus

int16_t raw_angle_1, raw_angle_2;
float angle_1, angle_2;
float offset_1 = 0;
float offset_2 = 0;

void setup() {
  // Initialize I2C communication on both buses
  I2C_0.begin(18, 19);  // SDA, SCL pins for I2C_0
  I2C_1.begin(21, 22);   // SDA, SCL pins for I2C_1
  Serial.begin(9600); // Match the baud rate with Python
}

void readAngle(TwoWire &i2c_bus, int16_t &raw_angle, float &angle, float &offset) {
  // Request data from the AS5600
  i2c_bus.beginTransmission(AS5600_ADDR);
  i2c_bus.write(ANGLE_REG_H);
  i2c_bus.endTransmission(false);
  i2c_bus.requestFrom((uint8_t)AS5600_ADDR, (size_t)2, true);

  // Read the angle data (2 bytes)
  uint8_t angle_high = i2c_bus.read();
  uint8_t angle_low = i2c_bus.read();
  
  // Combine high and low bytes
  raw_angle = (angle_high << 8) | angle_low;
  
  // Convert raw angle to a range of -180 to 180 degrees
  // The AS5600 gives a 12-bit value (0 to 4095)
  // By subtracting 2048, we get a range of -2048 to 2047
  // Then, we scale it to -180 to 180 degrees
  //angle = (float)(raw_angle - 2048) * 180.0 / 2048.0;

  angle = (float)(raw_angle)*360/4096.0;
  angle = angle - offset;
}

void loop() {
  readAngle(I2C_0, raw_angle_1, angle_1, offset_1);
  readAngle(I2C_1, raw_angle_2, angle_2, offset_2);

  Serial.print("Angle 1 : ");
  Serial.println(angle_1);
  Serial.print("Angle 2 : ");
  Serial.println(angle_2);

  delay(50); // Short delay to avoid overwhelming the serial output
}
