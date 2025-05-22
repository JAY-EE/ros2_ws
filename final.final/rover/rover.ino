
#include <Wire.h>
#include <ArduinoJson.h>  // Include ArduinoJson library

#define F_M_R 27
#define PWM_F_M_R 14
#define F_M_L 15
#define PWM_F_M_L 2
#define R_M_R 32
#define PWM_R_M_R 33
#define R_M_L 5
#define PWM_R_M_L 18

float linear;
float angular;
float manual;

unsigned long last_msg_time = 0;
const unsigned long msg_timeout = 100; // 1 second timeout

// ON LOW Front motors Rotate such that it moves forward
void movement(int speed, int angular_speed) {
    int right_speed = speed + angular_speed;
    int left_speed = speed - angular_speed;

    // Set motor directions
    if (right_speed > 0) {
        digitalWrite(F_M_R, LOW);
        digitalWrite(R_M_R, HIGH);
    } else {
        digitalWrite(F_M_R, HIGH);
        digitalWrite(R_M_R, LOW);
    }

    if (left_speed > 0) {
        digitalWrite(F_M_L, LOW);
        digitalWrite(R_M_L, HIGH);
    } else {
        digitalWrite(F_M_L, HIGH);
        digitalWrite(R_M_L, LOW);
    }

    // Set PWM values for both motors
    analogWrite(PWM_F_M_R, abs(right_speed) < 255 ? abs(right_speed) : 255);
    analogWrite(PWM_F_M_L, abs(left_speed) < 255 ? abs(left_speed) : 255);
    analogWrite(PWM_R_M_R, abs(right_speed) < 255 ? abs(right_speed) : 255);
    analogWrite(PWM_R_M_L, abs(left_speed) < 255 ? abs(left_speed) : 255);
}

void setup() {
    Serial.begin(9600);  // Initialize serial communication
    pinMode(F_M_R, OUTPUT);
    pinMode(PWM_F_M_R, OUTPUT);
    pinMode(F_M_L, OUTPUT);
    pinMode(PWM_F_M_L, OUTPUT);
    pinMode(R_M_R, OUTPUT);
    pinMode(PWM_R_M_R, OUTPUT);
    pinMode(R_M_L, OUTPUT);
    pinMode(PWM_R_M_L, OUTPUT);

    delay(1000);

    last_msg_time = millis();  // Initialize last message time
}

void loop() {
    if (Serial.available() > 0) {
        String incomingData = Serial.readStringUntil('\n');

        StaticJsonDocument<200> doc;
        DeserializationError error = deserializeJson(doc, incomingData);

        if (error) {      
            return;
        }

        linear = doc["linear"];
        angular = doc["angular"];
        manual = doc["manual"];

        movement(linear, angular);

        StaticJsonDocument<200> response;
        response["linear"] = linear;
        response["angular"] = angular;

        String output;
        serializeJson(response, output);
        Serial.println(output);

        last_msg_time = millis();
    }

    if (millis() - last_msg_time > msg_timeout) {
        movement(0, 0);  // Stop the motors
    }

    delay(10);
}

