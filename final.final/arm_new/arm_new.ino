#include <Wire.h>
#include <ArduinoJson.h>

#define motor1_pwm 33
#define motor2_pwm 26
#define motor1_dir 32
#define motor2_dir 25

#define grip_pwm 17
#define grip_dir 5

#define diff_m1_pwm 27 
#define diff_m1_dir 14 

#define diff_m2_pwm 12 
#define diff_m2_dir 13 

#define base_pwm 16
#define base_dir 4

#define AS5600_ADDR 0x36
#define ANGLE_REG_H 0x0E
#define ANGLE_REG_L 0x0F

// Define the I2C buses
TwoWire I2C_0 = TwoWire(0); 
TwoWire I2C_1 = TwoWire(1); 

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

int wrist_pwm = 120;


void readAngle(TwoWire &i2c_bus, int16_t &raw_angle, float &angle, float &offset) {
 
  i2c_bus.beginTransmission(AS5600_ADDR);
  i2c_bus.write(ANGLE_REG_H);
  i2c_bus.endTransmission(false);
  i2c_bus.requestFrom((uint8_t)AS5600_ADDR, (size_t)2, true);

  uint8_t angle_high = i2c_bus.read();
  uint8_t angle_low = i2c_bus.read();
  raw_angle = (angle_high << 8) | angle_low;
  angle = (float)raw_angle * 360.0 / 4096.0;

  angle = angle - offset;
}

void setup() {

  // Initialize I2C communication on both buses
  I2C_0.begin(18, 19);  // SDA, SCL pins for I2C_0
  I2C_1.begin(21, 22);   // SDA, SCL pins for I2C_1
  Serial.begin(9600); 

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
  
  readAngle(I2C_0, raw_angle_1, angle_1,offset_1);
  readAngle(I2C_1, raw_angle_2, angle_2,offset_2);

  if (Serial.available() > 0) {

    String incomingData = Serial.readStringUntil('\n');

    StaticJsonDocument<200> doc;
    DeserializationError error = deserializeJson(doc, incomingData);

    if (error) {
            return;
    }

    target_1 = doc["Target_1"];
    target_2 = doc["Target_2"];
    manual = doc["Indiviual"];
    pitch = doc["Pitch"];
    roll = doc["Yaw"];
    gripper = doc["Gripper"];
    base = doc["Base"];

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
    
    if (base < 0){
          analogWrite(base_pwm,abs(base));
          digitalWrite(base_dir,LOW);
        }
    else if (base > 0) {
          analogWrite(base_pwm,abs(base));
          digitalWrite(base_dir,HIGH);
        }
    else{
      analogWrite(base_pwm,0);
    }


    if (pitch == 1){
          analogWrite(diff_m1_pwm,wrist_pwm);
          digitalWrite(diff_m1_dir,HIGH);
          analogWrite(diff_m2_pwm,wrist_pwm);
          digitalWrite(diff_m2_dir,LOW);
        }
    else if (pitch == (-1)){
          analogWrite(diff_m1_pwm,wrist_pwm);
          digitalWrite(diff_m1_dir,LOW);
          analogWrite(diff_m2_pwm,wrist_pwm);
          digitalWrite(diff_m2_dir,HIGH);
        }
    else if (roll == 1){
          analogWrite(diff_m1_pwm,wrist_pwm);
          digitalWrite(diff_m1_dir,HIGH);
          analogWrite(diff_m2_pwm,wrist_pwm);
          digitalWrite(diff_m2_dir,HIGH);
        }
    else if (roll == (-1)){
          analogWrite(diff_m1_pwm,wrist_pwm);
          digitalWrite(diff_m1_dir,LOW);
          analogWrite(diff_m2_pwm,wrist_pwm);
          digitalWrite(diff_m2_dir,LOW);
        }
    else{
      analogWrite(diff_m1_pwm, 0); 
      analogWrite(diff_m2_pwm, 0);  
      digitalWrite(diff_m1_dir, LOW);  
      digitalWrite(diff_m2_dir, LOW);  
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

      // If target_1 and target_2 are both zero, stop the motors
      if ((target_1 == 0) && (target_2 == 0)) {
        analogWrite(motor1_pwm, 0); 
        analogWrite(motor2_pwm, 0); 
      }

      else {

        // Control motor 1 to reach target_1
        if ((int)target_1 != (int)angle_1) {
          if (target_1 > angle_1)
            digitalWrite(motor1_dir, HIGH);  
          else
            digitalWrite(motor1_dir, LOW);   
        analogWrite(motor1_pwm, 255); 
        } 
        else {
          analogWrite(motor1_pwm, 0); 
        }

        // Control motor 2 to reach target_2
        if ((int)target_2 != (int)angle_2) {
          if (target_2 > angle_2)
            digitalWrite(motor2_dir, LOW);  
          else
            digitalWrite(motor2_dir, HIGH);   
        analogWrite(motor2_pwm, 255);  
        } 
        else {
          analogWrite(motor2_pwm, 0); 
        }

    }
}

    StaticJsonDocument<200> response;
    
    response["Angle_1"] = angle_1;
    response["Angle_2"] = angle_2;

    String output;
    serializeJson(response, output);
    Serial.println(output);

}
  delay(10);
}





