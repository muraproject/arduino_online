#ifndef SENSOR_ACCESS_H
#define SENSOR_ACCESS_H

#include "variables_settings.h"

// Forward declaration
bool checkI2CDevice(uint8_t address);

// Function declarations
void requestRPMData();
void requestSensorData();

// Function to request RPM data from Slave 1
void requestRPMData() {
  // Set default values to 0 first
  currentRPM = 0;
  currentRPM2 = 0;
  currentRPM3 = 0;
  rpmDataReceived = false;

  // Check if device is responding
  if (!checkI2CDevice(SLAVE1_ADDR)) {
    // Device not responding, values already set to 0
    return;
  }

  // Request 32 bytes which should be enough for RPM data
  uint8_t bytesToRequest = 32;
  uint8_t bytesReceived = Wire.requestFrom(SLAVE1_ADDR, bytesToRequest);

  // Clear buffer
  memset(buffer, 0, sizeof(buffer));

  // Read all available bytes (up to our buffer size)
  int idx = 0;
  while (Wire.available() && idx < sizeof(buffer) - 1) {
    buffer[idx++] = Wire.read();
  }
  buffer[idx] = '\0';  // Ensure null termination

  if (bytesReceived > 0) {
    // Parse RPM data
    // Expected format: "RPM1:xxxx RPM2:xxxx RPM3:xxxx"
    char* rpm1Ptr = strstr(buffer, "RPM1:");
    char* rpm2Ptr = strstr(buffer, "RPM2:");
    char* rpm3Ptr = strstr(buffer, "RPM3:");

    if (rpm1Ptr && rpm2Ptr && rpm3Ptr) {
      int rpm1 = atoi(rpm1Ptr + 5);
      int rpm2 = atoi(rpm2Ptr + 5);
      int rpm3 = atoi(rpm3Ptr + 5);

      // Make sure values are reasonable
      if (rpm1 >= 0 && rpm1 < 50000) {
        currentRPM2 = rpm1;  // Note the swap in numbering to match your original code
        rpmDataReceived = true;
      }
      if (rpm2 >= 0 && rpm2 < 50000) {
        currentRPM = rpm2;  // Note the swap in numbering to match your original code
        rpmDataReceived = true;
      }
      if (rpm3 >= 0 && rpm3 < 50000) {
        currentRPM3 = rpm3;
        rpmDataReceived = true;
      }
    }
  }
  // If no valid data received, values remain 0 and rpmDataReceived stays false
}

// Function to request sensor data from Slave 2
void requestSensorData() {
  // Set default values to 0 first
  groundSpeed_kmh = 0.0;
  heading_deg = 0.0;
  voltage1 = 0.0;
  voltage2 = 0.0;
  current1 = 0.0;
  current2 = 0.0;
  sensorDataReceived = false;

  // Check if device is responding
  if (!checkI2CDevice(SLAVE2_ADDR)) {
    // Device not responding, values already set to 0
    return;
  }

  // Request 64 bytes which should be enough for sensor data
  uint8_t bytesToRequest = 64;
  uint8_t bytesReceived = Wire.requestFrom(SLAVE2_ADDR, bytesToRequest);

  // Clear buffer
  memset(buffer, 0, sizeof(buffer));

  // Read all available bytes (up to our buffer size)
  int idx = 0;
  while (Wire.available() && idx < sizeof(buffer) - 1) {
    buffer[idx++] = Wire.read();
  }
  buffer[idx] = '\0';  // Ensure null termination

  if (bytesReceived > 0) {
    // Parse sensor data
    // Expected format: "SPD:xx.x,HDG:xxx,V1:xx.x,V2:xx.x,I1:xx.x,I2:xx.x"
    char* spdPtr = strstr(buffer, "SPD:");
    char* hdgPtr = strstr(buffer, "HDG:");
    char* v1Ptr = strstr(buffer, "V1:");
    char* v2Ptr = strstr(buffer, "V2:");
    char* i1Ptr = strstr(buffer, "I1:");
    char* i2Ptr = strstr(buffer, "I2:");

    if (spdPtr) {
      groundSpeed_kmh = atof(spdPtr + 4);
      sensorDataReceived = true;
    }
    if (hdgPtr) {
      heading_deg = atof(hdgPtr + 4);
      sensorDataReceived = true;
    }
    if (v1Ptr) {
      voltage1 = atof(v1Ptr + 3);
      sensorDataReceived = true;
    }
    if (v2Ptr) {
      voltage2 = atof(v2Ptr + 3);
      sensorDataReceived = true;
    }
    if (i1Ptr) {
      current1 = atof(i1Ptr + 3);
      sensorDataReceived = true;
    }
    if (i2Ptr) {
      current2 = atof(i2Ptr + 3);
      sensorDataReceived = true;
    }
  }
  // Serial.println(groundSpeed_kmh);
  // If no valid data received, values remain 0 and sensorDataReceived stays false
}

#endif // SENSOR_ACCESS_H