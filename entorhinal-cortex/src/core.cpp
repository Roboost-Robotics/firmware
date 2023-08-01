/**
 * @file core.cpp
 * @author Friedl Jakob (friedl.jak@gmail.com)
 * @brief //todo
 * @version 0.1
 * @date 2023-07-06
 *
 * @copyright Copyright (c) 2023
 *
 */

#include <Arduino.h>
#include <RPLidar.h>

#define RPLIDAR_MOTOR 18
#define RPLIDAR_RX 16
#define RPLIDAR_TX 17

HardwareSerial RPLidarSerial(2);  // Using HardwareSerial 2
RPLidar lidar;

void setup() {
  // Initialize serial and lidar
  Serial.begin(115200);
  RPLidarSerial.begin(115200, SERIAL_8N1, RPLIDAR_RX, RPLIDAR_TX);

  // Start the lidar's motor
  pinMode(RPLIDAR_MOTOR, OUTPUT);
  digitalWrite(RPLIDAR_MOTOR, HIGH);

  lidar.begin(RPLidarSerial);
  lidar.startScan();
}

void loop() {
  // Check if lidar is open
  if (lidar.isOpen()) {
    RPLidarMeasurement measurement;
    while (lidar.waitPoint()) {
      measurement = lidar.getCurrentPoint();

      // measurement.angle is already in degrees
      float angle = measurement.angle;

      // Print the distance at the middle point
      if (angle >= 180.0 && angle < 181.0) {
        Serial.print("Distance at 180 degrees: ");
        Serial.print(measurement.distance);
        Serial.println("mm");
      }
    }
  } else {
    Serial.println("Lidar is not open!");
  }

  // Wait a bit before the next scan
  delay(1000);
}
