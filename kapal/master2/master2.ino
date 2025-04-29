zz/*********
  I2C Master program for STM32 Nucleo-64 G0B1RE with LoRa communication
  Integrated with LoRa for remote monitoring and control
*********/

#include <Wire.h>

// Slave addresses
#define SLAVE1_ADDR 0x55
#define SLAVE2_ADDR 0x56

// Set a slower I2C clock speed for better reliability
#define I2C_CLOCK_SPEED 50000  // 50 kHz

// Hardware Serial for LoRa module
HardwareSerial LoraSerial(PA10, PA9); // RX, TX

uint32_t i = 0;
char buffer[64]; // Buffer for receiving data

// Buffer for LoRa commands
char loraBuffer[64];
int loraBufferIndex = 0;

// Variables for heartbeat
unsigned long lastHeartbeatTime = 0;
const unsigned long heartbeatInterval = 5000; // Heartbeat every 5 seconds

// Function to reset I2C bus if needed
void resetI2C() {
  Wire.end();
  delay(100);
  Wire.begin();
  Wire.setClock(I2C_CLOCK_SPEED);
  Serial.println("I2C bus reset performed");
  LoraSerial.println("I2C bus reset performed");
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
    char message[50];
    sprintf(message, "Slave 0x%X not responding, trying to reset I2C", slaveAddr);
    Serial.println(message);
    LoraSerial.println(message);
    
    resetI2C();
    delay(100);
    if (!checkI2CDevice(slaveAddr)) {
      sprintf(message, "Slave 0x%X still not responding after reset", slaveAddr);
      Serial.println(message);
      LoraSerial.println(message);
      return;
    }
  }
  
  // Request 64 bytes which should be enough for all data
  uint8_t bytesToRequest = 64;
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
  
  char message[128];
  sprintf(message, "Received %u bytes from slave 0x%X: %s", bytesReceived, slaveAddr, validBuffer);
  Serial.println(message);
  LoraSerial.println(message);
}

void processLoraCommand(const char* command) {
  // Check for commands
  if (strcmp(command, "REQUEST_SLAVE1") == 0) {
    LoraSerial.println("Requesting data from Slave 1 (via LoRa command)...");
    requestFromSlave(SLAVE1_ADDR);
  }
  else if (strcmp(command, "REQUEST_SLAVE2") == 0) {
    LoraSerial.println("Requesting data from Slave 2 (via LoRa command)...");
    requestFromSlave(SLAVE2_ADDR);
  }
  else if (strcmp(command, "CHECK_STATUS") == 0) {
    sendSystemStatus();
  }
  else if (strcmp(command, "RESET_I2C") == 0) {
    LoraSerial.println("Resetting I2C bus (via LoRa command)...");
    resetI2C();
  }
  else if (strcmp(command, "PING") == 0) {
    LoraSerial.println("PONG");
  }
  else {
    LoraSerial.print("Unknown command: ");
    LoraSerial.println(command);
  }
}

void requestFromSlave(uint8_t slaveAddr) {
  // Request data from slave
  char message[50];
  sprintf(message, "Requesting data from slave 0x%X...", slaveAddr);
  Serial.println(message);
  LoraSerial.println(message);
  
  Wire.beginTransmission(slaveAddr);
  Wire.write("REQUEST_DATA");
  uint8_t error = Wire.endTransmission(true);
  
  if (error == 0) {
    // Add a small delay to give slave time to prepare response
    delay(10);
    // Read response if transmission was successful
    readFromSlave(slaveAddr);
  } else {
    sprintf(message, "Error communicating with slave 0x%X: %u", slaveAddr, error);
    Serial.println(message);
    LoraSerial.println(message);
  }
}

void sendSystemStatus() {
  LoraSerial.println("=== System Status ===");
  
  // Check I2C devices
  LoraSerial.print("Slave 1 (0x");
  LoraSerial.print(SLAVE1_ADDR, HEX);
  LoraSerial.print("): ");
  LoraSerial.println(checkI2CDevice(SLAVE1_ADDR) ? "Connected" : "Not found");
  
  LoraSerial.print("Slave 2 (0x");
  LoraSerial.print(SLAVE2_ADDR, HEX);
  LoraSerial.print("): ");
  LoraSerial.println(checkI2CDevice(SLAVE2_ADDR) ? "Connected" : "Not found");
  
  LoraSerial.print("Request cycle: ");
  LoraSerial.println(i);
  
  LoraSerial.println("====================");
}

void sendHeartbeat() {
  // Send a heartbeat message to indicate the system is alive
  LoraSerial.println("STM32_ALIVE");
}

void setup() {
  // Initialize serial communication
  Serial.begin(115200);
  LoraSerial.begin(9600); // For LoRa module
  
  delay(1000); // Give time for serial to initialize
  
  // Initialize I2C as master with slower clock for better reliability
  Wire.begin();
  Wire.setClock(I2C_CLOCK_SPEED);
  
  String initMessage = "\n\nSTM32 Nucleo G0B1RE I2C Master initialized with LoRa";
  Serial.println(initMessage);
  LoraSerial.println(initMessage);
  
  char message[50];
  
  sprintf(message, "I2C Clock: %d Hz", I2C_CLOCK_SPEED);
  Serial.println(message);
  LoraSerial.println(message);
  
  sprintf(message, "Slave 1 address: 0x%X", SLAVE1_ADDR);
  Serial.println(message);
  LoraSerial.println(message);
  
  sprintf(message, "Slave 2 address: 0x%X", SLAVE2_ADDR);
  Serial.println(message);
  LoraSerial.println(message);
  
  // Check if slaves are initially connected
  Serial.print("Checking initial connection to Slave 1: ");
  LoraSerial.print("Checking initial connection to Slave 1: ");
  
  bool slave1Found = checkI2CDevice(SLAVE1_ADDR);
  Serial.println(slave1Found ? "Connected" : "Not found");
  LoraSerial.println(slave1Found ? "Connected" : "Not found");
  
  Serial.print("Checking initial connection to Slave 2: ");
  LoraSerial.print("Checking initial connection to Slave 2: ");
  
  bool slave2Found = checkI2CDevice(SLAVE2_ADDR);
  Serial.println(slave2Found ? "Connected" : "Not found");
  LoraSerial.println(slave2Found ? "Connected" : "Not found");
  
  // Send system ready message
  LoraSerial.println("SYSTEM_READY");
}

void loop() {
  // Handle LoRa communication
  if (LoraSerial.available()) {
    char c = LoraSerial.read();
    
    // Process character
    if (c == '\n' || c == '\r') {
      if (loraBufferIndex > 0) {
        loraBuffer[loraBufferIndex] = '\0';
        processLoraCommand(loraBuffer);
        loraBufferIndex = 0;
      }
    } else if (loraBufferIndex < sizeof(loraBuffer) - 1) {
      loraBuffer[loraBufferIndex++] = c;
    }
  }
  
  // Original I2C master code
  static unsigned long lastRequestTime = 0;
  unsigned long currentTime = millis();
  
  // Execute regular I2C requests every 5 seconds
  if (currentTime - lastRequestTime >= 5000) {
    lastRequestTime = currentTime;
    
    Serial.printf("\n--- Request cycle %lu ---\n", i++);
    LoraSerial.print("\n--- Request cycle ");
    LoraSerial.print(i-1);
    LoraSerial.println(" ---");
    
    // Request data from first slave
    requestFromSlave(SLAVE1_ADDR);
    
    delay(1000); // Short delay between slaves
    
    // Request data from second slave
    requestFromSlave(SLAVE2_ADDR);
  }
  
  // Send heartbeat periodically
  if (currentTime - lastHeartbeatTime >= heartbeatInterval) {
    sendHeartbeat();
    lastHeartbeatTime = currentTime;
  }
}