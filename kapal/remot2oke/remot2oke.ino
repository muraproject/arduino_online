#include <EEPROM.h>

// Pin definitions for joystick analog
#define JOYSTICK_X 34
#define JOYSTICK_Y 35

// Button pins with pullup
#define BUTTON_1 13  // Rudder +20
#define BUTTON_2 2   // Rudder -20
#define BUTTON_3 4   // Not used
#define BUTTON_4 5   // Base angle +1
#define BUTTON_5 12  // Base angle -1

// LoRa pins
#define RXD2 16  // LoRa TX to ESP32 RX2
#define TXD2 17  // LoRa RX to ESP32 TX2

// LED pin for visual feedback
#define LED_BUILTIN 2

// EEPROM address for base angle
#define EEPROM_BASE_ANGLE_ADDR 0

// Variables
int xValue, yValue;
int throttle;
int baseAngle = 0;      // Base rudder angle (stored in EEPROM)
int currentAngle = 0;   // Current rudder angle
bool button1, button2, button3, button4, button5;
bool prevButton1 = false, prevButton2 = false;
bool prevButton4 = false, prevButton5 = false;

// Variables to track changes for LoRa transmission
int lastThrottle = -1;
int lastCurrentAngle = -999;

// Task handle for Core 2
TaskHandle_t LoRaBridgeTask;

void setup() {
  Serial.begin(115200);
  
  // Initialize LoRa Serial
  Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2);
  
  // Initialize EEPROM
  EEPROM.begin(512);
  
  // Set pin mode for LED
  pinMode(LED_BUILTIN, OUTPUT);
  
  // Read base angle from EEPROM
  baseAngle = EEPROM.read(EEPROM_BASE_ANGLE_ADDR);
  if (baseAngle > 127) baseAngle = baseAngle - 256; // Convert unsigned to signed
  currentAngle = baseAngle;
  
  // Joystick analog setup
  pinMode(JOYSTICK_X, INPUT);
  pinMode(JOYSTICK_Y, INPUT);
  
  // Button setup with pullup
  pinMode(BUTTON_1, INPUT_PULLUP);
  pinMode(BUTTON_2, INPUT_PULLUP);
  pinMode(BUTTON_3, INPUT_PULLUP);
  pinMode(BUTTON_4, INPUT_PULLUP);
  pinMode(BUTTON_5, INPUT_PULLUP);
  
  // Create LoRa Bridge task on Core 2
  xTaskCreatePinnedToCore(
    loRaBridgeTask,    // Task function
    "LoRaBridge",      // Task name
    10000,             // Stack size
    NULL,              // Task parameters
    1,                 // Priority
    &LoRaBridgeTask,   // Task handle
    0                  // Core number (0 or 1, using 0 as Core 2)
  );
  
  Serial.println("\n=========================================");
  Serial.println("Throttle and Rudder Control Ready");
  Serial.println("LoRa Bridge running on Core 2");
  Serial.print("Base Angle from EEPROM: ");
  Serial.println(baseAngle);
  Serial.println("=========================================");
}

void loop() {
  // This runs on Core 1 - Main control logic
  
  // Read analog values
  xValue = analogRead(JOYSTICK_X);
  yValue = analogRead(JOYSTICK_Y);
  
  // Map X value from 1840-4095 to 90-110 for throttle
  throttle = map(xValue, 1840, 4095, 90, 110);
  throttle = constrain(throttle, 90, 110);
  
  // Read button values
  button1 = !digitalRead(BUTTON_1);
  button2 = !digitalRead(BUTTON_2);
  button3 = !digitalRead(BUTTON_3);
  button4 = !digitalRead(BUTTON_4);
  button5 = !digitalRead(BUTTON_5);
  
  // Handle rudder control (buttons 1 & 2)
  if (button1) {
    currentAngle = baseAngle + 20;  // +20 degrees from base
  } else if (button2) {
    currentAngle = baseAngle - 20;  // -20 degrees from base
  } else {
    currentAngle = baseAngle;       // Return to base angle
  }
  
  // Handle base angle adjustment (buttons 4 & 5) with edge detection
  if (button4 && !prevButton4) {  // Button 4 pressed (rising edge)
    baseAngle++;
    baseAngle = constrain(baseAngle, -90, 90);  // Limit to reasonable range
    saveBaseAngle();
    currentAngle = baseAngle;  // Update current angle if no rudder input
  }
  
  if (button5 && !prevButton5) {  // Button 5 pressed (rising edge)
    baseAngle--;
    baseAngle = constrain(baseAngle, -90, 90);  // Limit to reasonable range
    saveBaseAngle();
    currentAngle = baseAngle;  // Update current angle if no rudder input
  }
  
  // Store previous button states for edge detection
  prevButton4 = button4;
  prevButton5 = button5;
  
  // Constrain current angle to safe limits
  currentAngle = constrain(currentAngle, -110, 110);
  
  // Check if throttle or current angle changed, then send to LoRa
  if (throttle != lastThrottle || currentAngle != lastCurrentAngle) {
    // Send data to LoRa
    Serial2.print("T:");
    Serial2.print(throttle);
    Serial2.print(",A:");
    Serial2.println(currentAngle);
    
    // Update last values
    lastThrottle = throttle;
    lastCurrentAngle = currentAngle;
  }
  
  // Print status to Serial Monitor
  Serial.print("Throttle: ");
  Serial.print(throttle);
  Serial.print(" | Base Angle: ");
  Serial.print(baseAngle);
  Serial.print(" | Current Angle: ");
  Serial.print(currentAngle);
  Serial.print(" | Buttons: ");
  Serial.print(button1 ? "1" : "0");
  Serial.print(button2 ? "1" : "0");
  Serial.print(button3 ? "1" : "0");
  Serial.print(button4 ? "1" : "0");
  Serial.print(button5 ? "1" : "0");
  
  Serial.println();
  
  delay(100);
}

// LoRa Bridge Task running on Core 2
void loRaBridgeTask(void *pvParameters) {
  Serial.println("LoRa Bridge Task started on Core 2");
  
  for(;;) {  // Infinite loop for the task
    
    // Forward data from LoRa to Serial (USB)
    while (Serial2.available()) {
      char c = Serial2.read();
      Serial.write(c);
      
      // Brief LED flash for received data
      digitalWrite(LED_BUILTIN, HIGH);
      delayMicroseconds(500);
      digitalWrite(LED_BUILTIN, LOW);
    }
    
    // Forward data from Serial (USB) to LoRa
    while (Serial.available()) {
      char c = Serial.read();
      Serial2.write(c);
      
      // Brief LED flash for transmitted data
      digitalWrite(LED_BUILTIN, HIGH);
      delayMicroseconds(1000);
      digitalWrite(LED_BUILTIN, LOW);
    }
    
    // Small delay to prevent task from hogging the CPU
    vTaskDelay(1 / portTICK_PERIOD_MS);
  }
}

void saveBaseAngle() {
  // Convert signed angle to unsigned for EEPROM storage
  uint8_t angleToSave = (baseAngle < 0) ? (256 + baseAngle) : baseAngle;
  EEPROM.write(EEPROM_BASE_ANGLE_ADDR, angleToSave);
  EEPROM.commit();
  
  Serial.print("Base angle saved to EEPROM: ");
  Serial.println(baseAngle);
}