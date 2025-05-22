#include <Wire.h>

#define motor1_pwm 33
#define motor2_pwm 26
#define motor1_dir 32
#define motor2_dir 25

#define grip_pwm 17
#define grip_dir 5

#define diff_m1_pwm 27 //
#define diff_m1_dir 14 //

#define diff_m2_pwm 12 //
#define diff_m2_dir 13 //

#define base_pwm 16
#define base_dir 4


// I2C addresses and registers
#define AS5600_ADDR 0x36
#define ANGLE_REG_H 0x0E
#define ANGLE_REG_L 0x0F

// Define the I2C buses
TwoWire I2C_0 = TwoWire(0); // Primary I2C bus
TwoWire I2C_1 = TwoWire(1); // Secondary I2C bus

int16_t raw_angle_1, raw_angle_2;

float angle_1 = 0;
float angle_2 = 0;

float offset_1 = 163;
float offset_2 = 86;

float target_1 = 0;
float target_2 = 0;

int manual = 0;
int pitch = 0;
int roll = 0;
int gripper = 0;
int base = 0;


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
  
  // Convert raw angle to degrees (0-360 range)
  angle = (float)raw_angle * 360.0 / 4096.0;

  angle = angle - offset;
}

void setup() {
  // Initialize I2C communication on both buses
  I2C_0.begin(18, 19);  // SDA, SCL pins for I2C_0
  I2C_1.begin(21, 22);   // SDA, SCL pins for I2C_1
  Serial.begin(9600); // Match the baud rate with Python

  pinMode(motor1_pwm, OUTPUT);
  pinMode(motor2_pwm, OUTPUT);
  pinMode(motor1_dir, OUTPUT);
  pinMode(motor2_dir, OUTPUT);

  pinMode(grip_pwm, OUTPUT);
  pinMode(grip_dir, OUTPUT);

  pinMode(diff_m1_dir, OUTPUT);
  pinMode(diff_m2_dir, OUTPUT);
  
  pinMode(diff_m1_pwm, OUTPUT);
  pinMode(diff_m2_pwm, OUTPUT);

  pinMode(base_pwm, OUTPUT);
  pinMode(base_dir, OUTPUT);

  readAngle(I2C_0, raw_angle_1, angle_1,offset_1);
  readAngle(I2C_1, raw_angle_2, angle_2,offset_2);

}

void loop() {
  // Read the current angles
  readAngle(I2C_0, raw_angle_1, angle_1,offset_1);
  readAngle(I2C_1, raw_angle_2, angle_2,offset_2);

  // Check if there is data available to read from serial
  if (Serial.available() > 0) {
    // Read the incoming data until newline character ('\n')
    String incomingData = Serial.readStringUntil('\n');

    // Parse the incoming data (you can still use these values if needed)
    int commaIndex1 = incomingData.indexOf(',');
    int commaIndex2 = incomingData.indexOf(',', commaIndex1 + 1);
    int commaIndex3 = incomingData.indexOf(',', commaIndex2 + 1);
    int commaIndex4 = incomingData.indexOf(',', commaIndex3 + 1);
    int commaIndex5 = incomingData.indexOf(',', commaIndex4 + 1);
    int commaIndex6 = incomingData.indexOf(',', commaIndex5 + 1);

    target_1 = incomingData.substring(0, commaIndex1).toInt();
    target_2 = incomingData.substring(commaIndex1 + 1,commaIndex2).toInt();
    manual = incomingData.substring(commaIndex2 + 1,commaIndex3).toInt();
    pitch = incomingData.substring(commaIndex3 + 1,commaIndex4).toInt();
    roll = incomingData.substring(commaIndex4 + 1,commaIndex5).toInt();
    gripper = incomingData.substring(commaIndex5 + 1 , commaIndex6).toInt();
    base = incomingData.substring(commaIndex6 + 1).toInt();

    if (gripper == 1){
          analogWrite(grip_pwm,255);
          digitalWrite(grip_dir,HIGH);
        }
    else if (gripper == -1) {
          analogWrite(grip_pwm,255);
          digitalWrite(grip_dir,LOW);
        }
    else{
      analogWrite(grip_pwm,0);
    }
    
    if (base == -1){
          analogWrite(base_pwm,50);
          digitalWrite(base_dir,LOW);
        }
    else if (base == 1) {
          analogWrite(base_pwm,50);
          digitalWrite(base_dir,HIGH);
        }
    else{
      analogWrite(base_pwm,0);
    }


    if (pitch == 1){
          analogWrite(diff_m1_pwm,120);
          digitalWrite(diff_m1_dir,HIGH);
          analogWrite(diff_m2_pwm,120);
          digitalWrite(diff_m2_dir,LOW);
        }
    else if (pitch == (-1)){
          analogWrite(diff_m1_pwm,120);
          digitalWrite(diff_m1_dir,LOW);
          analogWrite(diff_m2_pwm,120);
          digitalWrite(diff_m2_dir,HIGH);
        }
    else if (roll == 1){
          analogWrite(diff_m1_pwm,120);
          digitalWrite(diff_m1_dir,HIGH);
          analogWrite(diff_m2_pwm,120);
          digitalWrite(diff_m2_dir,HIGH);
        }
    else if (roll == (-1)){
          analogWrite(diff_m1_pwm,120);
          digitalWrite(diff_m1_dir,LOW);
          analogWrite(diff_m2_pwm,120);
          digitalWrite(diff_m2_dir,LOW);
        }
    else{
      analogWrite(diff_m1_pwm, 0);  // Stop motor 1
      analogWrite(diff_m2_pwm, 0);  // Stop motor 2
      digitalWrite(diff_m1_dir, LOW);  // Reset motor 1 direction
      digitalWrite(diff_m2_dir, LOW);  // Reset motor 2 direction
    }


    if (manual == 1){
      analogWrite(motor1_pwm, 255); 
      digitalWrite(motor1_dir,LOW);
    }
    else if (manual == 2){
      analogWrite(motor1_pwm, 255); 
      digitalWrite(motor1_dir,HIGH);
    }
    else if (manual == 3){
      analogWrite(motor2_pwm, 255); 
      digitalWrite(motor2_dir,HIGH);
    }
    else if (manual == 4){
      analogWrite(motor2_pwm, 255); 
      digitalWrite(motor2_dir,LOW);
    }
    
    else{

      // digitalWrite(diff_m1_pwm,LOW);
      // digitalWrite(diff_m2_pwm,LOW);
      // digitalWrite(grip_pwm,LOW);

      // If target_1 and target_2 are both zero, stop the motors
      if ((target_1 == 0) && (target_2 == 0)) {
        analogWrite(motor1_pwm, 0);  // Stop motor 1
        analogWrite(motor2_pwm, 0);  // Stop motor 2
      }

      else {

        // Control motor 1 to reach target_1
        if ((int)target_1 != (int)angle_1) {
          if (target_1 > angle_1)
            digitalWrite(motor1_dir, HIGH);  // Rotate motor 1 clockwise
          else
            digitalWrite(motor1_dir, LOW);   // Rotate motor 1 counterclockwise
        analogWrite(motor1_pwm, 255);  // Full speed for motor 1
        } 
        else {
          analogWrite(motor1_pwm, 0);  // Stop motor 1 when target angle is reached
        }

        // Control motor 2 to reach target_2
        if ((int)target_2 != (int)angle_2) {
          if (target_2 > angle_2)
            digitalWrite(motor2_dir, LOW);  // Rotate motor 2 clockwise
          else
            digitalWrite(motor2_dir, HIGH);   // Rotate motor 2 counterclockwise
        analogWrite(motor2_pwm, 255);  // Full speed for motor 2
        } 
        else {
          analogWrite(motor2_pwm, 0);  // Stop motor 2 when target angle is reached
        }

    }
}


    // Create a comma-separated string of the encoder angles
    String outputData = String(angle_1, 2) + "," + String(angle_2, 2);
    // Send the encoder angles back to the serial port
    Serial.println(outputData);
}

  // Ensure a short delay to allow serial buffer to be cleared
  delay(10);
}





