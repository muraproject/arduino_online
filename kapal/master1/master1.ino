/*********
  I2C Master program for STM32 Nucleo-64 G0B1RE
  Final version with simplified reliable data reading
*********/

#include <Wire.h>

// Slave addresses
#define SLAVE1_ADDR 0x55
#define SLAVE2_ADDR 0x56

// Set a slower I2C clock speed for better reliability
#define I2C_CLOCK_SPEED 50000  // 50 kHz

uint32_t i = 0;
char buffer[64]; // Buffer for receiving data

// Function to reset I2C bus if needed
void resetI2C() {
  Wire.end();
  delay(100);
  Wire.begin();
  Wire.setClock(I2C_CLOCK_SPEED);
  Serial.println("I2C bus reset performed");
}

bool checkI2CDevice(uint8_t address) {
  Wire.beginTransmission(address);
  byte error = Wire.endTransmission();
  
  if (error == 0) {
    return true; // Device found
  }
  return false; // Device not found
}

void readFromSlave(uint8_t slaveAddr) {
  // First check if device is responding
  if (!checkI2CDevice(slaveAddr)) {
    Serial.printf("Slave 0x%X not responding, trying to reset I2C\n", slaveAddr);
    resetI2C();
    delay(100);
    if (!checkI2CDevice(slaveAddr)) {
      Serial.printf("Slave 0x%X still not responding after reset\n", slaveAddr);
      return;
    }
  }
  
  // Request 32 bytes which should be enough for all data
  uint8_t bytesToRequest = 32;
  uint8_t bytesReceived = Wire.requestFrom(slaveAddr, bytesToRequest);
  
  // Clear buffer
  memset(buffer, 0, sizeof(buffer));
  
  // Read all available bytes (up to our buffer size)
  int idx = 0;
  while (Wire.available() && idx < sizeof(buffer)-1) {
    buffer[idx++] = Wire.read();
  }
  buffer[idx] = '\0'; // Ensure null termination
  
  // Now simply find where the valid ASCII string ends
  int validLength = 0;
  for (int j = 0; j < idx; j++) {
    if (buffer[j] < 32 || buffer[j] > 126) {
      // Found a non-printable character, this is the end of valid data
      validLength = j;
      break;
    }
    validLength = j + 1; // If we don't find any non-printables, use the whole string
  }
  
  // Create a copy of just the valid portion
  char validBuffer[64];
  strncpy(validBuffer, buffer, validLength);
  validBuffer[validLength] = '\0';
  
  Serial.printf("Received %u bytes from slave 0x%X: %s\n", bytesReceived, slaveAddr, validBuffer);
}

void setup() {
  // Initialize serial communication
  Serial.begin(115200);
  delay(1000); // Give time for serial to initialize
  
  // Initialize I2C as master with slower clock for better reliability
  Wire.begin();
  Wire.setClock(I2C_CLOCK_SPEED);
  
  Serial.println("\n\nSTM32 Nucleo G0B1RE I2C Master initialized");
  Serial.printf("I2C Clock: %d Hz\n", I2C_CLOCK_SPEED);
  Serial.printf("Slave 1 address: 0x%X\n", SLAVE1_ADDR);
  Serial.printf("Slave 2 address: 0x%X\n", SLAVE2_ADDR);
  
  // Check if slaves are initially connected
  Serial.print("Checking initial connection to Slave 1: ");
  Serial.println(checkI2CDevice(SLAVE1_ADDR) ? "Connected" : "Not found");
  
  Serial.print("Checking initial connection to Slave 2: ");
  Serial.println(checkI2CDevice(SLAVE2_ADDR) ? "Connected" : "Not found");
}

void loop() {
  // Wait between requests
  delay(5000);
  Serial.printf("\n--- Request cycle %lu ---\n", i++);
  
  // Request data from first slave
  Serial.println("Requesting data from Slave 1...");
  Wire.beginTransmission(SLAVE1_ADDR);
  Wire.write("REQUEST_DATA");
  uint8_t error = Wire.endTransmission(true);
  
  if (error == 0) {
    // Add a small delay to give slave time to prepare response
    delay(10);
    // Read response if transmission was successful
    readFromSlave(SLAVE1_ADDR);
  } else {
    Serial.printf("Error communicating with Slave 1: %u\n", error);
  }
  
  delay(1000); // Short delay between slaves
  
  // Request data from second slave
  Serial.println("Requesting data from Slave 2...");
  Wire.beginTransmission(SLAVE2_ADDR);
  Wire.write("REQUEST_DATA");
  error = Wire.endTransmission(true);
  
  if (error == 0) {
    // Add a small delay to give slave time to prepare response
    delay(10);
    // Read response if transmission was successful
    readFromSlave(SLAVE2_ADDR);
  } else {
    Serial.printf("Error communicating with Slave 2: %u\n", error);
  }
}