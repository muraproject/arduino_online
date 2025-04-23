/*********
  I2C Slave 1 program for ESP32 with Triple RPM Sensors
  Responds with 3 RPM sensor values upon master request
*********/

#include "Wire.h"

// Slave address
#define I2C_DEV_ADDR 0x55

// RPM variables
int rpm = 0;
int rpm2 = 0;
int rpm3 = 0;
unsigned long last_millis = 0;
int interval = 50;
TaskHandle_t SensorTask;
volatile int currentRPM = 0;
volatile int currentRPM2 = 0;
volatile int currentRPM3 = 0;
int rpm_state1 = 0, rpm_state2 = 0, rpm_state3 = 0;
int dummy3 = 0, rod = 0;

// Function called when master requests data from this slave
void onRequest() {
  // Create response string with all three RPM values
  char response[32];
  sprintf(response, "RPM1:%d RPM2:%d RPM3:%d", currentRPM, currentRPM2, currentRPM3);
  
  // Send the response
  Wire.print(response);
  
  Serial.print("I2C Request - Sent: ");
  Serial.println(response);
}

// Function called when master sends data to this slave
void onReceive(int len) {
  String command = "";
  
  Serial.printf("I2C Receive[%d]: ", len);
  
  // Read the command
  while (Wire.available()) {
    char c = Wire.read();
    command += c;
    Serial.write(c);
  }
  Serial.println();
}

// Sensor reading task that runs on Core 0
void SensorHandler(void* parameter) {
  Serial.println("Sensor task started on Core 0");
  
  while (true) {
    // Sensor 1 (pin 35) reading
    if (analogRead(35) < 500 && rpm_state1 == 1) {
      // Wait until signal changes back
      while (analogRead(35) < 500 && rpm_state1 == 1) {
        delayMicroseconds(1);
      }
      rpm++;
    }
    
    // Sensor 2 (pin 34) reading  
    if (analogRead(34) < 500 && rpm_state2 == 1) {
      // Wait until signal changes back
      while (analogRead(34) < 500 && rpm_state2 == 1) {
        delayMicroseconds(1);
      }
      rpm2++;
    }
    
    // Sensor 3 (pin 32) reading
    if (analogRead(32) < 500 && rpm_state3 == 1) {
      // Wait until signal changes back
      while (analogRead(32) < 500 && rpm_state3 == 1) {
        delayMicroseconds(1);
      }
      rpm3++;
    }
    
    // Small delay to prevent watchdog issues
    vTaskDelay(1);
  }
}

void setup() {
  // Initialize serial for debugging
  Serial.begin(115200);
  Serial.setDebugOutput(true);
  
  // Setup sensor pins as inputs
  pinMode(35, INPUT);
  pinMode(34, INPUT);
  pinMode(32, INPUT);
  
  // Setup I2C slave
  Wire.onReceive(onReceive);
  Wire.onRequest(onRequest);
  Wire.begin((uint8_t)I2C_DEV_ADDR);
  
  Serial.printf("ESP32 I2C Slave 1 initialized at address: 0x%X\n", I2C_DEV_ADDR);
  Serial.println("Triple RPM Sensor measurement active");
  
  // Create task for sensor handling on core 0
  xTaskCreatePinnedToCore(
    SensorHandler,
    "SensorTask",
    10000,
    NULL,
    1,
    &SensorTask,
    0
  );
  
  Serial.println("Sensor handler task created on Core 0");
  Serial.println("Main loop running on Core 1");
}

void loop() {
  // Main loop runs on Core 1
  
  // Measurement cycle for RPM sensors
  if (millis() - last_millis > 160) {  // Interval for sensing
    last_millis = millis();
    
    // First measurement window
    rpm_state1 = 1;
    rpm_state2 = 0;
    rpm_state3 = 0;
    delay(50);
    
    // Second measurement window
    rpm_state2 = 1;
    rpm_state1 = 0;
    rpm_state3 = 0;
    delay(50);
    
    // Third measurement window
    rpm_state3 = 1;
    rpm_state1 = 0;
    rpm_state2 = 0;
    delay(50);
    
    // Reset states after measurements
    rpm_state1 = 0;
    rpm_state2 = 0;
    rpm_state3 = 0;
    
    // Calculate RPM values
    currentRPM = rpm * 60000 / interval;
    currentRPM2 = rpm2 * 60000 / interval;
    currentRPM3 = rpm3 * 60000 / interval;
    
    // Debug output
    Serial.print("RPM values - ");
    Serial.print(currentRPM);
    Serial.print(", ");
    Serial.print(currentRPM2);
    Serial.print(", ");
    Serial.println(currentRPM3);
    
    // Reset counters for next cycle
    rpm = 0;
    rpm2 = 0;
    rpm3 = 0;
    
    // Optional: Simulate changing values for testing
    if (rod == 1) {
      dummy3 = dummy3 + 1000;
    } else {
      dummy3 = dummy3 - 1000;
    }
    
    if (dummy3 > 15000) {
      rod = 0;
    }
    
    if (dummy3 < 5) {
      rod = 1;
    }
  }
  
  // Small delay to allow other processes
  delay(5);
}