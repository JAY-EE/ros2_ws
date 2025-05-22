#define RXD2 16  // RX pin for GPS (connected to GPS module TX)
#define TXD2 17  // TX pin for GPS (connected to GPS module RX)
#include <HardwareSerial.h>
#include <TinyGPS++.h>   

#define GPS_BAUD 9600

HardwareSerial gpsSerial(2);  // Create a HardwareSerial object for Serial2

float longitude;
float latitude;
float altitude;

void setup(){
  Serial.begin(115200);  // Start the Serial Monitor at 9600 baud rate
  
  // Initialize the GPS serial connection on Serial2 with the specified baud rate
  gpsSerial.begin(GPS_BAUD, SERIAL_8N1, RXD2, TXD2);
  Serial.println("GPS Serial (Serial2) started at 9600 baud rate");
}

void loop(){
  while (gpsSerial.available() > 0){
    // Read and output GPS data byte by byte
    char gpsData = gpsSerial.read();
    Serial.print(gpsData);  // Print the GPS data to the Serial Monitor
  }
  delay(100);  // Delay for readability
  Serial.println("-------------------------------");
}
