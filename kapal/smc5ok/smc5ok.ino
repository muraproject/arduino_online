#include <Servo.h>
#include <Wire.h>

// I2C Slave addresses (matching the slave programs)
#define SLAVE1_ADDR 0x55  // RPM slave
#define SLAVE2_ADDR 0x56  // Current, voltage, speed slave

// Set a slower I2C clock speed for better reliability
#define I2C_CLOCK_SPEED 50000  // 50 kHz

// Servo/ESC
Servo ESC1;
Servo ESC2;
Servo ESC3;    // Servo ketiga
Servo RUDDER;  // Rudder servo
const int esc1Pin = PA0;
const int esc2Pin = PA1;
const int esc3Pin = PA4;     // Pin untuk servo ketiga
const int rudderPin = PA7;  // Pin untuk rudder

// Konstanta ESC untuk motor 1 & 2 (dalam microseconds)
const int MIN_PULSE = 1500;    // Minimum pulse width
const int START_PULSE = 1530;  // Starting pulse width
const int MAX_PULSE = 1950;    // Maximum pulse width
const int STEP_SIZE = 5;       // Step size (lebih halus)

// Konstanta ESC untuk motor 3 (microseconds)
const int MIN_PULSE3_US = 1000;    // Minimum pulse width dalam microseconds
const int START_PULSE3_US = 1100;  // Starting pulse width dalam microseconds
const int MAX_PULSE3_US = 2000;    // Maximum pulse width dalam microseconds
const int STEP_SIZE3_US = 10;      // Step size dalam microseconds (lebih halus)

// Konstanta Rudder (microseconds)
const int RUDDER_MIN = 1000;     // Minimum rudder angle
const int RUDDER_CENTER = 1500;  // Center rudder position
const int RUDDER_MAX = 2000;     // Maximum rudder angle

// Variabel throttle motor 3 dalam microseconds
float currentThrottle3_us = MIN_PULSE3_US;

// Remote control variables
int remoteThrottle = 100;   // Throttle from remote (90-110)
int remoteRudderAngle = 0;  // Rudder angle from remote
int lastRemoteThrottle = -1;
int lastRemoteRudderAngle = -999;

// ESP32 komunikasi (tetap dipertahankan untuk kompatibilitas dengan LoRa)
HardwareSerial ESP32Serial(PA10, PA9);

// Throttle variables
int currentThrottle1 = MIN_PULSE;
int currentThrottle2 = MIN_PULSE;
int currentThrottle3 = MIN_PULSE;  // Disimpan untuk kompatibilitas

// RPM variables
int targetRPM = 0;
int targetRPM3 = 0;   // Target RPM untuk motor ketiga
int currentRPM = 0;   // RPM dari sensor 1 (pin 35)
int currentRPM2 = 0;  // RPM dari sensor 2 (pin 34)
int currentRPM3 = 0;  // RPM dari sensor ketiga (pin 32)
unsigned long lastRPMCheck = 0;
const unsigned long RPM_CHECK_INTERVAL = 100;

// Additional variables for I2C data from slave2
float groundSpeed_kmh = 0.0;
float heading_deg = 0.0;
float voltage1 = 0.0;
float voltage2 = 0.0;
float current1 = 0.0;
float current2 = 0.0;

// I2C communication timing
unsigned long lastI2CRequestTime = 0;
const unsigned long I2C_REQUEST_INTERVAL = 100;  // 100ms = 10Hz

// Control Mode
bool controlActive = false;
bool control3Active = false;       // Flag untuk kontrol motor ketiga
bool remoteControlActive = false;  // Flag for remote control mode
enum ControlMode { NONE,
                   SMC,
                   FUZZY,
                   REMOTE };
ControlMode currentMode = NONE;

// SMC variables
float prevError1_SMC = 0;
float prevError2_SMC = 0;

// SMC params (adjusted for microsecond range)
const int LAMBDA_INT = 3;  // 0.03 * 100
const int ETA_INT = 50;    // 0.5 * 100
const int K_INT = 5;       // 0.05 * 100
const int BOUNDARY_INT = 800;

// Fuzzy Variables
int prevError1_Fuzzy = 0;
int prevError2_Fuzzy = 0;
int prevError3_Fuzzy = 0;  // Error sebelumnya untuk motor ketiga

// Parameter Fuzzy
const int ERROR_NEGL = -5000;
const int ERROR_NEGS = -2000;
const int ERROR_ZERO = 0;
const int ERROR_POSS = 2000;
const int ERROR_POSL = 5000;

// LoRa communication
unsigned long lastHeartbeatTime = 0;
const unsigned long HEARTBEAT_INTERVAL = 5000;  // 5 seconds
char loraBuffer[64];
int loraBufferIndex = 0;
unsigned long messageCount = 0;

// Buffer for I2C communication
char buffer[64];  // Buffer for receiving data

// Flag to track if we're received any data from slaves
bool rpmDataReceived = false;
bool sensorDataReceived = false;

// Prototypes
int calculateSMC(int error, int prevError, unsigned long dt_millis, bool isMotor2);
int calculateFuzzy(int error, bool isMotor2);
float calculateFuzzy3(int error);
int membershipNL(int error);
int membershipNS(int error);
int membershipZE(int error);
int membershipPS(int error);
int membershipPL(int error);
void requestRPMData();
void requestSensorData();
void processLoraCommand(const char* command);
void reportStatusToLora();
void sendHeartbeat();
void handleRemoteControl();
void setRemoteThrottle(int throttle);
void setRemoteRudder(int angle);

// Function to check if an I2C device is responding
bool checkI2CDevice(uint8_t address) {
  Wire.beginTransmission(address);
  byte error = Wire.endTransmission();

  if (error == 0) {
    return true;  // Device found
  }
  return false;  // Device not found
}

// Function to reset I2C bus if needed
void resetI2C() {
  Wire.end();
  delay(100);
  Wire.begin();
  Wire.setClock(I2C_CLOCK_SPEED);
  Serial.println("I2C bus reset performed");
  ESP32Serial.println("I2C bus reset performed");
}

// Function to request RPM data from Slave 1
void requestRPMData() {
  // First check if device is responding
  if (!checkI2CDevice(SLAVE1_ADDR)) {
    Serial.println("RPM slave not responding, trying to reset I2C");
    ESP32Serial.println("RPM slave not responding, trying to reset I2C");

    resetI2C();
    delay(100);
    if (!checkI2CDevice(SLAVE1_ADDR)) {
      Serial.println("RPM slave still not responding after reset");
      ESP32Serial.println("RPM slave still not responding after reset");
      return;
    }
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
    rpmDataReceived = true;

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
      if (rpm1 >= 0 && rpm1 < 50000) currentRPM2 = rpm1;  // Note the swap in numbering to match your original code
      if (rpm2 >= 0 && rpm2 < 50000) currentRPM = rpm2;   // Note the swap in numbering to match your original code
      if (rpm3 >= 0 && rpm3 < 50000) currentRPM3 = rpm3;

      // Debug output
      // Serial.print("I2C RPM Data: RPM1=");
      // Serial.print(currentRPM2);
      // Serial.print(", RPM2=");
      // Serial.print(currentRPM);
      // Serial.print(", RPM3=");
      // Serial.println(currentRPM3);
    }
  }
}

// Function to request sensor data from Slave 2
void requestSensorData() {
  // First check if device is responding
  if (!checkI2CDevice(SLAVE2_ADDR)) {
    Serial.println("Sensor slave not responding, trying to reset I2C");
    ESP32Serial.println("Sensor slave not responding, trying to reset I2C");

    resetI2C();
    delay(100);
    if (!checkI2CDevice(SLAVE2_ADDR)) {
      Serial.println("Sensor slave still not responding after reset");
      ESP32Serial.println("Sensor slave still not responding after reset");
      return;
    }
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
    sensorDataReceived = true;

    // Parse sensor data
    // Expected format: "SPD:xx.x,HDG:xxx,V1:xx.x,V2:xx.x,I1:xx.x,I2:xx.x"
    char* spdPtr = strstr(buffer, "SPD:");
    char* hdgPtr = strstr(buffer, "HDG:");
    char* v1Ptr = strstr(buffer, "V1:");
    char* v2Ptr = strstr(buffer, "V2:");
    char* i1Ptr = strstr(buffer, "I1:");
    char* i2Ptr = strstr(buffer, "I2:");

    if (spdPtr) groundSpeed_kmh = atof(spdPtr + 4);
    if (hdgPtr) heading_deg = atof(hdgPtr + 4);
    if (v1Ptr) voltage1 = atof(v1Ptr + 3);
    if (v2Ptr) voltage2 = atof(v2Ptr + 3);
    if (i1Ptr) current1 = atof(i1Ptr + 3);
    if (i2Ptr) current2 = atof(i2Ptr + 3);

    // Debug output
    // Serial.print("I2C Sensor Data: Speed=");
    // Serial.print(groundSpeed_kmh);
    // Serial.print(" km/h, Heading=");
    // Serial.print(heading_deg);
    // Serial.print("°, V1=");
    // Serial.print(voltage1);
    // Serial.print("V, V2=");
    // Serial.print(voltage2);
    // Serial.print("V, I1=");
    // Serial.print(current1);
    // Serial.print("A, I2=");
    // Serial.println(current2);
  }
}

// Handle remote control from ESP32
void handleRemoteControl() {
  // Map throttle from 90-110 to ESC3 microseconds range
  int mappedThrottle = map(remoteThrottle, 90, 110, MIN_PULSE3_US, MAX_PULSE3_US);
  mappedThrottle = constrain(mappedThrottle, MIN_PULSE3_US, MAX_PULSE3_US);

  // Apply throttle to ESC3
  currentThrottle3_us = mappedThrottle;
  ESC3.writeMicroseconds(mappedThrottle);

  // Map rudder angle to servo microseconds (-110 to +110 degrees to servo range)
  int mappedRudder = map(remoteRudderAngle, -110, 110, RUDDER_MIN, RUDDER_MAX);
  mappedRudder = constrain(mappedRudder, RUDDER_MIN, RUDDER_MAX);

  // Apply rudder angle
  RUDDER.writeMicroseconds(mappedRudder);
}

// Set remote throttle value
void setRemoteThrottle(int throttle) {
  remoteThrottle = constrain(throttle, 90, 110);
}

// Set remote rudder angle
void setRemoteRudder(int angle) {
  remoteRudderAngle = constrain(angle, -110, 110);
}

// Process LoRa commands
// Process LoRa commands
void processLoraCommand(const char* command) {
  // Check for remote control data format: T:xxx,A:xxx
  if (strstr(command, "T:") && strstr(command, "A:")) {
    char* throttlePtr = strstr(command, "T:");
    char* anglePtr = strstr(command, "A:");
    
    if (throttlePtr && anglePtr) {
      int throttle = atoi(throttlePtr + 2);
      int angle = atoi(anglePtr + 2);
      
      // Update remote control values
      setRemoteThrottle(throttle);
      setRemoteRudder(angle);
      
      // Always apply remote control (don't change other modes)
      handleRemoteControl();
      
      // Print feedback for remote control
      Serial.print("Remote - Throttle: ");
      Serial.print(throttle);
      Serial.print(" -> ESC3: ");
      Serial.print((int)currentThrottle3_us);
      Serial.print("us, Rudder: ");
      Serial.print(angle);
      Serial.print("° -> ");
      int mappedRudder = map(angle, -110, 110, RUDDER_MIN, RUDDER_MAX);
      Serial.print(mappedRudder);
      Serial.println("us");
      
      ESP32Serial.print("Remote applied - T:");
      ESP32Serial.print(throttle);
      ESP32Serial.print(",A:");
      ESP32Serial.println(angle);
      
      return; // Exit early since we processed remote control data
    }
  }
  
  // Check for single character commands
  if (strlen(command) == 1) {
    char cmd = command[0];
    
    // Process single character commands identical to serial interface
    if (cmd == 's') {
      // Start motors
      currentThrottle1 = START_PULSE;
      currentThrottle2 = START_PULSE;
      currentThrottle3_us = START_PULSE3_US;
      
      ESC1.writeMicroseconds(currentThrottle1);
      ESC2.writeMicroseconds(currentThrottle2);
      // ESC3 will be controlled by remote, don't override here
      
      ESP32Serial.println("Motors 1&2 started at minimum speed");
      
      // Disable automatic control for motors 1&2 only
      controlActive = false;
      control3Active = false;
      if (currentMode != REMOTE) currentMode = NONE;
    }
    else if (cmd == 'x') {
      // Stop motors
      currentThrottle1 = MIN_PULSE;
      currentThrottle2 = MIN_PULSE;
      // Don't stop ESC3 and RUDDER - let remote control them
      
      ESC1.writeMicroseconds(currentThrottle1);
      ESC2.writeMicroseconds(currentThrottle2);
      
      // Disable control for motors 1&2 only
      controlActive = false;
      control3Active = false;
      if (currentMode != REMOTE) currentMode = NONE;
      
      ESP32Serial.println("Motors 1&2 stopped - Remote control still active");
    }
    else if (cmd == '+') {
      // Increase throttle for motors 1&2 only
      bool throttleIncreased = false;
      
      // Motor 1 & 2
      if (currentThrottle1 < MAX_PULSE) {
        currentThrottle1 += STEP_SIZE;
        currentThrottle2 += STEP_SIZE;
        ESC1.writeMicroseconds(currentThrottle1);
        ESC2.writeMicroseconds(currentThrottle2);
        throttleIncreased = true;
      }
      
      // Don't change ESC3 - controlled by remote
      
      if (throttleIncreased) {
        ESP32Serial.println("Motors 1&2 throttle increased");
      } else {
        ESP32Serial.println("Maximum throttle reached for motors 1&2");
      }
      
      // Disable automatic control for motors 1&2
      controlActive = false;
      control3Active = false;
      if (currentMode != REMOTE) currentMode = NONE;
    }
    else if (cmd == '-') {
      // Decrease throttle for motors 1&2 only
      bool throttleDecreased = false;
      
      // Motor 1 & 2
      if (currentThrottle1 > START_PULSE) {
        currentThrottle1 -= STEP_SIZE;
        currentThrottle2 -= STEP_SIZE;
        ESC1.writeMicroseconds(currentThrottle1);
        ESC2.writeMicroseconds(currentThrottle2);
        throttleDecreased = true;
      }
      
      // Don't change ESC3 - controlled by remote
      
      if (throttleDecreased) {
        ESP32Serial.println("Motors 1&2 throttle decreased");
      } else {
        ESP32Serial.println("Minimum throttle reached for motors 1&2");
      }
      
      // Disable automatic control for motors 1&2
      controlActive = false;
      control3Active = false;
      if (currentMode != REMOTE) currentMode = NONE;
    }
    else if (cmd == 'p') {
      // Print status
      reportStatusToLora();
    }
    else if (cmd == 'i') {
      // Reset I2C bus
      ESP32Serial.println("Resetting I2C bus...");
      resetI2C();
    }
    else {
      ESP32Serial.print("Unknown command: ");
      ESP32Serial.println(cmd);
    }
  }
  // Process multi-character commands (m, f, t followed by numbers)
  else if (strlen(command) > 1) {
    char cmd = command[0];
    
    if (cmd == 'm' || cmd == 'f' || cmd == 't') {
      // Extract the number part
      int targetVal = atoi(command + 1);
      
      // Don't disable remote control - let it coexist
      
      if (cmd == 'm') {
        // SMC untuk motor 1 & 2
        targetRPM = targetVal;
        
        if (targetRPM > 0) {
          controlActive = true;
          currentMode = SMC;
          prevError1_SMC = 0;
          prevError2_SMC = 0;
          
          // Start motors jika belum berjalan
          if (currentThrottle1 <= MIN_PULSE) {
            currentThrottle1 = START_PULSE;
            currentThrottle2 = START_PULSE;
            ESC1.writeMicroseconds(currentThrottle1);
            ESC2.writeMicroseconds(currentThrottle2);
          }
          
          ESP32Serial.print("SMC mode activated for motors 1&2. Target RPM: ");
          ESP32Serial.println(targetRPM);
          ESP32Serial.println("Remote control still active for ESC3 & Rudder");
        } else {
          controlActive = false;
          currentMode = NONE;
          ESP32Serial.println("Main motors control deactivated");
        }
      }
      else if (cmd == 'f') {
        // Fuzzy untuk motor 1 & 2
        targetRPM = targetVal;
        
        if (targetRPM > 0) {
          controlActive = true;
          currentMode = FUZZY;
          prevError1_Fuzzy = 0;
          prevError2_Fuzzy = 0;
          
          // Start motors jika belum berjalan
          if (currentThrottle1 <= MIN_PULSE) {
            currentThrottle1 = START_PULSE;
            currentThrottle2 = START_PULSE;
            ESC1.writeMicroseconds(currentThrottle1);
            ESC2.writeMicroseconds(currentThrottle2);
          }
          
          ESP32Serial.print("Fuzzy mode activated for motors 1&2. Target RPM: ");
          ESP32Serial.println(targetRPM);
          ESP32Serial.println("Remote control still active for ESC3 & Rudder");
        } else {
          controlActive = false;
          currentMode = NONE;
          ESP32Serial.println("Main motors control deactivated");
        }
      }
      else if (cmd == 't') {
        // Fuzzy untuk motor 3 - this will conflict with remote, inform user
        targetRPM3 = targetVal;
        
        if (targetRPM3 > 0) {
          control3Active = true;
          prevError3_Fuzzy = 0;
          
          ESP32Serial.print("Third motor Fuzzy control activated. Target RPM: ");
          ESP32Serial.println(targetRPM3);
          ESP32Serial.println("WARNING: This will override remote throttle control for ESC3");
        } else {
          control3Active = false;
          ESP32Serial.println("Third motor control deactivated - Remote throttle control resumed");
        }
      }
    }
    else {
      ESP32Serial.print("Unknown command: ");
      ESP32Serial.println(command);
    }
  }
}

// Report system status to LoRa
void reportStatusToLora() {
  ESP32Serial.println("=== Status Report ===");

  ESP32Serial.print("Mode: ");
  if (remoteControlActive) {
    ESP32Serial.println("Remote Control");
    ESP32Serial.print("Remote Throttle: ");
    ESP32Serial.print(remoteThrottle);
    ESP32Serial.print(", Remote Rudder: ");
    ESP32Serial.println(remoteRudderAngle);
  } else if (controlActive) {
    ESP32Serial.println(currentMode == SMC ? "SMC" : "Fuzzy");
    ESP32Serial.print("Target RPM 1&2: ");
    ESP32Serial.println(targetRPM);
  } else {
    ESP32Serial.println("Manual");
  }

  ESP32Serial.print("Mode 3: ");
  if (control3Active) {
    ESP32Serial.println("Fuzzy");
    ESP32Serial.print("Target RPM 3: ");
    ESP32Serial.println(targetRPM3);
  } else {
    ESP32Serial.println("Manual");
  }

  ESP32Serial.print("Motor 1 - Throttle: ");
  ESP32Serial.print(currentThrottle1);
  ESP32Serial.print("us, RPM: ");
  ESP32Serial.println(currentRPM);

  ESP32Serial.print("Motor 2 - Throttle: ");
  ESP32Serial.print(currentThrottle2);
  ESP32Serial.print("us, RPM: ");
  ESP32Serial.println(currentRPM2);

  ESP32Serial.print("Motor 3 - Throttle: ");
  ESP32Serial.print(currentThrottle3_us);
  ESP32Serial.print("us, RPM: ");
  ESP32Serial.println(currentRPM3);

  ESP32Serial.print("Speed: ");
  ESP32Serial.print(groundSpeed_kmh, 1);
  ESP32Serial.print(" km/h, Heading: ");
  ESP32Serial.print(heading_deg, 0);
  ESP32Serial.println(" degrees");

  ESP32Serial.print("Voltage 1: ");
  ESP32Serial.print(voltage1, 1);
  ESP32Serial.print("V, Voltage 2: ");
  ESP32Serial.print(voltage2, 1);
  ESP32Serial.println("V");

  ESP32Serial.print("Current 1: ");
  ESP32Serial.print(current1, 1);
  ESP32Serial.print("A, Current 2: ");
  ESP32Serial.print(current2, 1);
  ESP32Serial.println("A");

  ESP32Serial.print("I2C Data Received - RPM: ");
  ESP32Serial.print(rpmDataReceived ? "Yes" : "No");
  ESP32Serial.print(", Sensors: ");
  ESP32Serial.println(sensorDataReceived ? "Yes" : "No");

  ESP32Serial.println("===================");
}

// Send a heartbeat message
void sendHeartbeat() {
  ESP32Serial.println("STM32_HEARTBEAT");
}

// Fungsi untuk memberikan status throttle
void reportThrottle() {
  Serial.print("Throttle positions: Main1=");
  Serial.print(currentThrottle1);
  Serial.print("us, Main2=");
  Serial.print(currentThrottle2);
  Serial.print("us, Third=");
  Serial.print(currentThrottle3_us);
  Serial.println("us");
}

void setup() {
  // Initialize serial communication for debugging
  Serial.begin(115200);

  // Add delay to ensure Serial is ready
  delay(1000);

  // Initialize I2C as master with slower clock for better reliability
  Wire.begin();
  Wire.setClock(I2C_CLOCK_SPEED);

  // Initialize LoRa communication
  ESP32Serial.begin(9600);

  // ESC setup
  ESC1.attach(esc1Pin, 1000, 2000);
  ESC2.attach(esc2Pin, 1000, 2000);
  ESC3.attach(esc3Pin, 1000, 2000);      // Inisialisasi servo ketiga
  RUDDER.attach(rudderPin, 1000, 2000);  // Initialize rudder servo

  // Set ESCs to minimum
  ESC1.writeMicroseconds(MIN_PULSE);
  ESC2.writeMicroseconds(MIN_PULSE);
  ESC3.writeMicroseconds(MIN_PULSE3_US);    // Gunakan writeMicroseconds untuk motor 3
  RUDDER.writeMicroseconds(RUDDER_CENTER);  // Center rudder position

  currentThrottle3_us = MIN_PULSE3_US;

  // Laporan bahwa sistem siap
  Serial.println("=========================================");
  Serial.println("Triple Control System with Remote Control Ready");
  Serial.println("=========================================");
  Serial.println("Commands via Serial or LoRa:");
  Serial.println("'s' - Start motors at minimum");
  Serial.println("'x' - Stop motors");
  Serial.println("'+' - Increase throttle");
  Serial.println("'-' - Decrease throttle");
  Serial.println("'p' - Print status");
  Serial.println("'mXXXX' - SMC control with target RPM XXXX (main motors)");
  Serial.println("'fXXXX' - Fuzzy control with target RPM XXXX (main motors)");
  Serial.println("'tXXXX' - Fuzzy control with target RPM XXXX (third motor)");
  Serial.println("'m0', 'f0', or 't0' - Disable respective control");
  Serial.println("'i' - Reset I2C bus");
  Serial.println("Remote Control Format: 'T:xxx,A:xxx' (Throttle & Rudder Angle)");
  Serial.println("=========================================");
  Serial.println("I2C Master setup to communicate with:");
  Serial.print("RPM Slave at address: 0x");
  Serial.println(SLAVE1_ADDR, HEX);
  Serial.print("Sensor Slave at address: 0x");
  Serial.println(SLAVE2_ADDR, HEX);
  Serial.println("=========================================");

  // Send system ready message to LoRa
  ESP32Serial.println("SYSTEM_READY");

  // Check if slaves are initially connected
  Serial.print("Checking initial connection to RPM Slave: ");
  bool slave1Found = checkI2CDevice(SLAVE1_ADDR);
  Serial.println(slave1Found ? "Connected" : "Not found");

  Serial.print("Checking initial connection to Sensor Slave: ");
  bool slave2Found = checkI2CDevice(SLAVE2_ADDR);
  Serial.println(slave2Found ? "Connected" : "Not found");
}

void loop() {
  unsigned long currentTime = millis();

  // I2C data acquisition
  if (currentTime - lastI2CRequestTime >= I2C_REQUEST_INTERVAL) {
    lastI2CRequestTime = currentTime;

    // Request RPM data from Slave 1
    // requestRPMData();

    // // Small delay between requests
    // delay(10);

    // // Request sensor data from Slave 2
    // requestSensorData();
  }

  // Apply control for third motor (independent)
  if (control3Active && targetRPM3 > 0) {
    if (currentTime - lastRPMCheck > RPM_CHECK_INTERVAL) {
      int error3 = targetRPM3 - currentRPM3;
      float throttleChange3 = calculateFuzzy3(error3);

      // Update throttle with float precision
      currentThrottle3_us += throttleChange3;

      // Constrain to valid range
      currentThrottle3_us = constrain(currentThrottle3_us, START_PULSE3_US, MAX_PULSE3_US);

      // Apply new throttle using writeMicroseconds
      ESC3.writeMicroseconds((int)currentThrottle3_us);

      // Update compatibility variable
      currentThrottle3 = map((int)currentThrottle3_us, 1000, 2000, 0, 180);

      prevError3_Fuzzy = error3;

      // Print debug info every few cycles
      static int debug3Counter = 0;
      if (++debug3Counter >= 5) {
        debug3Counter = 0;
        Serial.print("Motor3 control: Target=");
        Serial.print(targetRPM3);
        Serial.print(", Current=");
        Serial.print(currentRPM3);
        Serial.print(", Error=");
        Serial.print(error3);
        Serial.print(", Throttle=");
        Serial.print(currentThrottle3_us);
        Serial.println("us");
      }
    }
  }

  // Apply control if active for motors 1 & 2
  if (controlActive && targetRPM > 0) {
    if (currentTime - lastRPMCheck > RPM_CHECK_INTERVAL) {
      unsigned long dt_millis = currentTime - lastRPMCheck;
      lastRPMCheck = currentTime;

      int error1, error2;
      int throttleChange1 = 0, throttleChange2 = 0;

      if (currentMode == SMC) {
        // Untuk SMC, error dibagi 200 (seperti pada kode yang berfungsi)
        error1 = (targetRPM - currentRPM2) / 200;
        error2 = (targetRPM - currentRPM) / 200;

        throttleChange1 = calculateSMC(error1, prevError1_SMC, dt_millis, false);
        throttleChange2 = calculateSMC(error2, prevError2_SMC, dt_millis, true);

        prevError1_SMC = error1;
        // prevError2_SMC = error2;
      } else if (currentMode == FUZZY) {
        currentRPM2 = currentRPM2 - 2500;  ///kalibrasi disini
        if (currentRPM2 < 0) {
          currentRPM2 = 0;
        }
        error1 = targetRPM - currentRPM2;
        error2 = targetRPM - currentRPM;

        throttleChange1 = calculateFuzzy(error1, true);
        throttleChange2 = calculateFuzzy(error2, true);

        prevError1_Fuzzy = error1;
        prevError2_Fuzzy = error2;
      }

      currentThrottle1 += throttleChange1;
      currentThrottle2 += throttleChange2;

      currentThrottle1 = constrain(currentThrottle1, START_PULSE, MAX_PULSE);
      currentThrottle2 = constrain(currentThrottle2, START_PULSE, MAX_PULSE);

      ESC1.writeMicroseconds(currentThrottle1);
      ESC2.writeMicroseconds(currentThrottle2);

      // Print status periodically
      static int statusCounter = 0;
      if (++statusCounter >= 1) {  // Print every 10 cycles (≈1 second)
        statusCounter = 0;

        Serial.print("Mode: ");
        Serial.print(currentMode == SMC ? "SMC" : "Fuzzy");
        Serial.print(", Target: ");
        Serial.print(targetRPM);
        Serial.print(" | RPM1:");
        Serial.print(currentRPM2);
        Serial.print(", RPM2:");
        Serial.println(currentRPM);
      }
    }
  }

  // Check for user commands via Serial interface
  if (Serial.available() > 0) {
    // Read one character
    char cmd = Serial.read();

    // Process single character commands
    if (cmd == 's') {
      // Start motors
      currentThrottle1 = START_PULSE;
      currentThrottle2 = START_PULSE;
      currentThrottle3_us = START_PULSE3_US;

      ESC1.writeMicroseconds(currentThrottle1);
      ESC2.writeMicroseconds(currentThrottle2);
      ESC3.writeMicroseconds(START_PULSE3_US);

      Serial.println("Motors started at minimum speed");
      reportThrottle();

      // Disable automatic control and remote control
      controlActive = false;
      control3Active = false;
      remoteControlActive = false;
      currentMode = NONE;
    } else if (cmd == 'x') {
      // Stop motors
      currentThrottle1 = MIN_PULSE;
      currentThrottle2 = MIN_PULSE;
      currentThrottle3_us = MIN_PULSE3_US;

      ESC1.writeMicroseconds(currentThrottle1);
      ESC2.writeMicroseconds(currentThrottle2);
      ESC3.writeMicroseconds(MIN_PULSE3_US);
      RUDDER.writeMicroseconds(RUDDER_CENTER);  // Center rudder on stop

      // Disable all control modes
      controlActive = false;
      control3Active = false;
      remoteControlActive = false;
      currentMode = NONE;

      Serial.println("Motors stopped");
    } else if (cmd == '+') {
      // Increase throttle (disable remote control)
      remoteControlActive = false;
      bool throttleIncreased = false;

      // Motor 1 & 2
      if (currentThrottle1 < MAX_PULSE) {
        currentThrottle1 += STEP_SIZE;
        currentThrottle2 += STEP_SIZE;
        ESC1.writeMicroseconds(currentThrottle1);
        ESC2.writeMicroseconds(currentThrottle2);
        throttleIncreased = true;
      }

      // Motor 3 (more precision)
      if (currentThrottle3_us < MAX_PULSE3_US) {
        currentThrottle3_us += STEP_SIZE3_US;
        ESC3.writeMicroseconds((int)currentThrottle3_us);
        throttleIncreased = true;
      }

      if (throttleIncreased) {
        Serial.println("Throttle increased:");
        reportThrottle();
      } else {
        Serial.println("Maximum throttle reached");
      }

      // Disable automatic control
      controlActive = false;
      control3Active = false;
      currentMode = NONE;
    } else if (cmd == '-') {
      // Decrease throttle (disable remote control)
      remoteControlActive = false;
      bool throttleDecreased = false;

      // Motor 1 & 2
      if (currentThrottle1 > START_PULSE) {
        currentThrottle1 -= STEP_SIZE;
        currentThrottle2 -= STEP_SIZE;
        ESC1.writeMicroseconds(currentThrottle1);
        ESC2.writeMicroseconds(currentThrottle2);
        throttleDecreased = true;
      }

      // Motor 3 (more precision)
      if (currentThrottle3_us > START_PULSE3_US) {
        currentThrottle3_us -= STEP_SIZE3_US;
        ESC3.writeMicroseconds((int)currentThrottle3_us);
        throttleDecreased = true;
      }

      if (throttleDecreased) {
        Serial.println("Throttle decreased:");
        reportThrottle();
      } else {
        Serial.println("Minimum throttle reached");
      }

      // Disable automatic control
      controlActive = false;
      control3Active = false;
      currentMode = NONE;
    } else if (cmd == 'p') {
      // Print status
      Serial.println("=== Status Report ===");

      Serial.print("Mode: ");
      if (remoteControlActive) {
        Serial.println("Remote Control");
        Serial.print("Remote Throttle: ");
        Serial.print(remoteThrottle);
        Serial.print(", Remote Rudder: ");
        Serial.println(remoteRudderAngle);
      } else if (controlActive) {
        Serial.println(currentMode == SMC ? "SMC" : "Fuzzy");
        Serial.print("Target RPM 1&2: ");
        Serial.println(targetRPM);
      } else {
        Serial.println("Manual");
      }

      Serial.print("Mode 3: ");
      if (control3Active) {
        Serial.println("Fuzzy");
        Serial.print("Target RPM 3: ");
        Serial.println(targetRPM3);
      } else {
        Serial.println("Manual");
      }

      Serial.print("Motor 1 - Throttle: ");
      Serial.print(currentThrottle1);
      Serial.print("us, RPM: ");
      Serial.println(currentRPM);

      Serial.print("Motor 2 - Throttle: ");
      Serial.print(currentThrottle2);
      Serial.print("us, RPM: ");
      Serial.println(currentRPM2);

      Serial.print("Motor 3 - Throttle: ");
      Serial.print(currentThrottle3_us);
      Serial.print("us, RPM: ");
      Serial.println(currentRPM3);

      Serial.print("Speed: ");
      Serial.print(groundSpeed_kmh, 1);
      Serial.print(" km/h, Heading: ");
      Serial.print(heading_deg, 0);
      Serial.println(" degrees");

      Serial.print("Voltage 1: ");
      Serial.print(voltage1, 1);
      Serial.print("V, Voltage 2: ");
      Serial.print(voltage2, 1);
      Serial.println("V");

      Serial.print("Current 1: ");
      Serial.print(current1, 1);
      Serial.print("A, Current 2: ");
      Serial.print(current2, 1);
      Serial.println("A");

      Serial.print("I2C Data Received - RPM: ");
      Serial.print(rpmDataReceived ? "Yes" : "No");
      Serial.print(", Sensors: ");
      Serial.println(sensorDataReceived ? "Yes" : "No");

      Serial.println("===================");
    } else if (cmd == 'i') {
      // Reset I2C bus
      Serial.println("Resetting I2C bus...");
      resetI2C();
    }
    // Multi-character commands (m, f, t followed by numbers)
    else if (cmd == 'm' || cmd == 'f' || cmd == 't') {
      // Read the following numbers
      String numStr = "";
      unsigned long startTime = millis();

      // Buffer time to get all data
      delay(5);

      // Read until timeout or no more data
      while (millis() - startTime < 500) {  // 500ms timeout
        if (Serial.available() > 0) {
          char c = Serial.read();
          if (c == '\n' || c == '\r')
            break;
          if (c >= '0' && c <= '9')
            numStr += c;
        } else {
          // No more data, exit loop
          break;
        }
        delay(1);
      }

      int targetVal = numStr.toInt();

      // Disable remote control when using manual commands
      remoteControlActive = false;

      if (cmd == 'm') {
        // SMC for motors 1 & 2
        targetRPM = targetVal;

        if (targetRPM > 0) {
          controlActive = true;
          currentMode = SMC;
          prevError1_SMC = 0;
          prevError2_SMC = 0;

          // Start motors if not running
          if (currentThrottle1 <= MIN_PULSE) {
            currentThrottle1 = START_PULSE;
            currentThrottle2 = START_PULSE;
            ESC1.writeMicroseconds(currentThrottle1);
            ESC2.writeMicroseconds(currentThrottle2);
          }

          Serial.print("SMC mode activated. Target RPM: ");
          Serial.println(targetRPM);
        } else {
          controlActive = false;
          currentMode = NONE;
          Serial.println("Main motors control deactivated");
        }
      } else if (cmd == 'f') {
        // Fuzzy for motors 1 & 2
        targetRPM = targetVal;

        if (targetRPM > 0) {
          controlActive = true;
          currentMode = FUZZY;
          prevError1_Fuzzy = 0;
          prevError2_Fuzzy = 0;

          // Start motors if not running
          if (currentThrottle1 <= MIN_PULSE) {
            currentThrottle1 = START_PULSE;
            currentThrottle2 = START_PULSE;
            ESC1.writeMicroseconds(currentThrottle1);
            ESC2.writeMicroseconds(currentThrottle2);
          }

          Serial.print("Fuzzy mode activated. Target RPM: ");
          Serial.println(targetRPM);
        } else {
          controlActive = false;
          currentMode = NONE;
          Serial.println("Main motors control deactivated");
        }
      } else if (cmd == 't') {
        // Fuzzy for motor 3
        targetRPM3 = targetVal;

        if (targetRPM3 > 0) {
          control3Active = true;
          prevError3_Fuzzy = 0;

          // Start motor 3 if not running
          if (currentThrottle3_us <= MIN_PULSE3_US) {
            currentThrottle3_us = START_PULSE3_US;
            ESC3.writeMicroseconds(START_PULSE3_US);
          }

          Serial.print("Third motor Fuzzy control activated. Target RPM: ");
          Serial.println(targetRPM3);
        } else {
          control3Active = false;
          Serial.println("Third motor control deactivated");
        }
      }
    } else {
      // Unknown command
      Serial.print("Unknown command: ");
      Serial.println(cmd);
    }

    // Clear remaining serial buffer
    while (Serial.available() > 0) {
      Serial.read();
    }
  }

  // Handle LoRa communication
  if (ESP32Serial.available()) {
    char c = ESP32Serial.read();

    // Process character
    if (c == '\n' || c == '\r') {
      if (loraBufferIndex > 0) {
        loraBuffer[loraBufferIndex] = '\0';
        processLoraCommand(loraBuffer);
        loraBufferIndex = 0;
        messageCount++;
      }
    } else if (loraBufferIndex < sizeof(loraBuffer) - 1) {
      loraBuffer[loraBufferIndex++] = c;
    }
  }

  // Send heartbeat periodically
  if (currentTime - lastHeartbeatTime >= HEARTBEAT_INTERVAL) {
    sendHeartbeat();
    lastHeartbeatTime = currentTime;
  }

  // Small delay to prevent watchdog issues
  delay(5);
}

// SMC Implementation using integer math for microsecond range
int calculateSMC(int error, int prevError, unsigned long dt_millis, bool isMotor2) {
  // Safeguard for dt (minimum 1ms)
  if (dt_millis < 1) dt_millis = 1;

  // Calculate error derivative
  int error_dot = ((error - prevError) * 1000) / dt_millis;

  // Limit error_dot to reasonable values
  error_dot = constrain(error_dot, -1000, 1000);

  // Calculate sliding surface (s = error_dot + lambda * error)
  int s = error_dot + (LAMBDA_INT * error) / 100;

  // Calculate control output (smaller for finer microsecond control)
  int control;
  if (abs(s) <= BOUNDARY_INT) {
    // Inside boundary layer
    control = ((ETA_INT * s) / BOUNDARY_INT + K_INT) * error / 100;
  } else {
    // Outside boundary layer
    control = (ETA_INT + K_INT * (s > 0 ? 120 : -120) / 100) * error / 100;
  }

  // Limit control output for microsecond precision (smaller step size)
  return constrain(control, -2, 2);
}

// Fuzzy Implementation - membership functions
int membershipNL(int error) {
  if (error <= ERROR_NEGL) return 100;
  if (error >= ERROR_NEGS) return 0;
  return (100 * (ERROR_NEGS - error)) / (ERROR_NEGS - ERROR_NEGL);
}

int membershipNS(int error) {
  if (error <= ERROR_NEGL || error >= ERROR_ZERO) return 0;
  if (error <= ERROR_NEGS)
    return (100 * (error - ERROR_NEGL)) / (ERROR_NEGS - ERROR_NEGL);
  return (100 * (ERROR_ZERO - error)) / (ERROR_ZERO - ERROR_NEGS);
}

int membershipZE(int error) {
  if (error <= ERROR_NEGS || error >= ERROR_POSS) return 0;
  if (error <= ERROR_ZERO)
    return (100 * (error - ERROR_NEGS)) / (ERROR_ZERO - ERROR_NEGS);
  return (100 * (ERROR_POSS - error)) / (ERROR_POSS - ERROR_ZERO);
}

int membershipPS(int error) {
  if (error <= ERROR_ZERO || error >= ERROR_POSL) return 0;
  if (error <= ERROR_POSS)
    return (100 * (error - ERROR_ZERO)) / (ERROR_POSS - ERROR_ZERO);
  return (100 * (ERROR_POSL - error)) / (ERROR_POSL - ERROR_POSS);
}

int membershipPL(int error) {
  if (error <= ERROR_POSS) return 0;
  if (error >= ERROR_POSL) return 100;
  return (100 * (error - ERROR_POSS)) / (ERROR_POSL - ERROR_POSS);
}

// Sugeno Fuzzy control for motors 1 & 2 (adjusted for finer microsecond control)
int calculateFuzzy(int error, bool isMotor2) {
  // Calculate membership degrees
  int mu_NL = membershipNL(error);
  int mu_NS = membershipNS(error);
  int mu_ZE = membershipZE(error);
  int mu_PS = membershipPS(error);
  int mu_PL = membershipPL(error);

  // Define rule consequents (smaller values for finer microsecond control)
  int z1 = -10;  // -1.0 * 10
  int z2 = -5;   // -0.5 * 10
  int z3 = 0;    //  0.0 * 10

  // Different values for motor 2
  int z4, z5;
  if (isMotor2) {
    z4 = 7;   //  0.7 * 10
    z5 = 15;  //  1.5 * 10
  } else {
    z4 = 5;   //  0.5 * 10
    z5 = 10;  //  1.0 * 10
  }

  // Calculate weighted average (Sugeno defuzzification)
  long numerator = (long)mu_NL * z1 + (long)mu_NS * z2 + (long)mu_ZE * z3 + (long)mu_PS * z4 + (long)mu_PL * z5;
  int denominator = mu_NL + mu_NS + mu_ZE + mu_PS + mu_PL;

  // Avoid division by zero
  if (denominator <= 0) return 0;

  // Calculate result and convert back to original units
  int result = (int)(numerator / denominator / 10);

  // Limit output (smaller step size for microsecond precision)
  return constrain(result, -2, 2);
}

// Fuzzy control for third motor (with float output for smoother control)
float calculateFuzzy3(int error) {
  // Calculate membership degrees
  int mu_NL = membershipNL(error);
  int mu_NS = membershipNS(error);
  int mu_ZE = membershipZE(error);
  int mu_PS = membershipPS(error);
  int mu_PL = membershipPL(error);

  // Define rule consequents specifically for third motor (in microseconds)
  float z1 = -7.5;  // Larger negative throttle change
  float z2 = -3.0;  // Small negative throttle change
  float z3 = 0.0;   // No change
  float z4 = 3.0;   // Small positive throttle change
  float z5 = 7.5;   // Larger positive throttle change

  // Calculate weighted average (Sugeno defuzzification)
  float numerator = mu_NL * z1 + mu_NS * z2 + mu_ZE * z3 + mu_PS * z4 + mu_PL * z5;
  int denominator = mu_NL + mu_NS + mu_ZE + mu_PS + mu_PL;

  // Avoid division by zero
  if (denominator <= 0) return 0.0;

  // Calculate result with float precision
  float result = numerator / denominator;

  // Limit output (maximum throttle change of 5 microseconds)
  return constrain(result, -5.0, 5.0);
}