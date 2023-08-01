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

RPLidar lidar;

void setup() {
  // Initialize serial and lidar
  Serial.begin(115200);
  lidar.begin(RPLIDAR_RX, RPLIDAR_TX);

  // Start the lidar's motor
  pinMode(RPLIDAR_MOTOR, OUTPUT);
  digitalWrite(RPLIDAR_MOTOR, HIGH);
}

void loop() {
  // Make sure lidar is connected
  if (lidar.isRunning()) {
    rplidar_response_measurement_node_hq_t nodes[8192];
    size_t count = _countof(nodes);

    // Perform a 360 degree scan and grab the measurements
    lidar.scan(nodes, count);

    // Find the node closest to 180 degrees
    for (int i = 0; i < count; i++) {
      float angle = nodes[i].angle_z_q14 * 90.f / (1 << 14);

      // Print the distance at the middle point
      if (angle >= 180.0 && angle <= 180.1) {
        Serial.print("Distance at 180 degrees: ");
        Serial.print(nodes[i].dist_mm_q2/4.0);
        Serial.println("mm");
      }
    }
  } else {
    Serial.println("Lidar is not running!");
  }

  // Wait a bit before the next scan
  delay(1000);
}