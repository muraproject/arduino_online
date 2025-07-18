// ESP32 LoRa Receiver Test Program
// Simple receiver to test remote control commands
// Listens and prints all incoming data from LoRa module
// Using RXD2(17) and TXD2(16) for LoRa module

#include "BluetoothSerial.h"

// LoRa pins (same as remote)
#define RXD2 17  // LoRa TX to ESP32 RX2
#define TXD2 16  // LoRa RX to ESP32 TX2

// LED pin for visual feedback
#define LED_BUILTIN 2

// Bluetooth Serial object (optional for testing)
BluetoothSerial SerialBT;

// Device name for Bluetooth
String device_name = "ESP32-Receiver-Test";

// Variables for statistics
unsigned long totalCommands = 0;
unsigned long lastCommandTime = 0;
unsigned long startTime = 0;

// Variables for LED indication
unsigned long lastLedTime = 0;

void setup() {
  // Initialize serial communications
  Serial.begin(115200);
  Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2); // LoRa module
  
  // Initialize Bluetooth Serial (optional)
  if(!SerialBT.begin(device_name)) {
    Serial.println("An error occurred initializing Bluetooth");
  } else {
    Serial.println("Bluetooth initialized successfully (optional)");
    Serial.print("Device Name: ");
    Serial.println(device_name);
  }
  
  // Set pin mode for LED
  pinMode(LED_BUILTIN, OUTPUT);
  
  // Store start time
  startTime = millis();
  
  // Display startup message
  Serial.println("\n=============================================");
  Serial.println("ESP32 LoRa Receiver Test Program Started");
  Serial.println("=============================================");
  Serial.println("Listening for commands from remote control...");
  Serial.println("Commands will be displayed with timestamp");
  Serial.println("=============================================\n");
  
  // LED startup sequence
  for(int i = 0; i < 5; i++) {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(100);
    digitalWrite(LED_BUILTIN, LOW);
    delay(100);
  }
  
  Serial.println("Ready to receive data!\n");
}

void flashLED(int duration_ms) {
  digitalWrite(LED_BUILTIN, HIGH);
  delay(duration_ms);
  digitalWrite(LED_BUILTIN, LOW);
}

void printTimestamp() {
  unsigned long currentTime = millis();
  unsigned long elapsed = currentTime - startTime;
  
  // Format: [HH:MM:SS.mmm]
  unsigned long hours = elapsed / 3600000;
  unsigned long minutes = (elapsed % 3600000) / 60000;
  unsigned long seconds = (elapsed % 60000) / 1000;
  unsigned long millisecs = elapsed % 1000;
  
  Serial.print("[");
  if(hours < 10) Serial.print("0");
  Serial.print(hours);
  Serial.print(":");
  if(minutes < 10) Serial.print("0");
  Serial.print(minutes);
  Serial.print(":");
  if(seconds < 10) Serial.print("0");
  Serial.print(seconds);
  Serial.print(".");
  if(millisecs < 100) Serial.print("0");
  if(millisecs < 10) Serial.print("0");
  Serial.print(millisecs);
  Serial.print("] ");
}

void printStatistics() {
  if(millis() - lastCommandTime > 5000 && totalCommands > 0) {
    Serial.println("\n--- STATISTICS ---");
    Serial.print("Total commands received: ");
    Serial.println(totalCommands);
    Serial.print("Uptime: ");
    Serial.print((millis() - startTime) / 1000);
    Serial.println(" seconds");
    Serial.print("Average rate: ");
    if(totalCommands > 0) {
      float rate = (float)totalCommands / ((millis() - startTime) / 1000.0);
      Serial.print(rate, 2);
      Serial.println(" commands/sec");
    }
    Serial.println("------------------\n");
  }
}

void loop() {
  // 1. Listen for LoRa data
  if (Serial2.available()) {
    String receivedData = "";
    
    // Read complete line
    while(Serial2.available()) {
      char c = Serial2.read();
      if(c == '\n' || c == '\r') {
        if(receivedData.length() > 0) {
          break;
        }
      } else {
        receivedData += c;
      }
      delay(1); // Small delay to ensure complete data reception
    }
    
    if(receivedData.length() > 0) {
      // Print with timestamp
      printTimestamp();
      Serial.print("LoRa RX: ");
      Serial.println(receivedData);
      
      // Update statistics
      totalCommands++;
      lastCommandTime = millis();
      
      // Flash LED for received data
      flashLED(50);
    }
  }
  
  // 2. Listen for Bluetooth data (optional)
  if (SerialBT.available()) {
    String receivedData = "";
    
    // Read complete line
    while(SerialBT.available()) {
      char c = SerialBT.read();
      if(c == '\n' || c == '\r') {
        if(receivedData.length() > 0) {
          break;
        }
      } else {
        receivedData += c;
      }
      delay(1);
    }
    
    if(receivedData.length() > 0) {
      // Print with timestamp
      printTimestamp();
      Serial.print("BT RX:   ");
      Serial.println(receivedData);
      
      // Update statistics
      totalCommands++;
      lastCommandTime = millis();
      
      // Flash LED for received data
      flashLED(50);
    }
  }
  
  // 3. Listen for Serial data (commands from PC)
  if (Serial.available()) {
    String receivedData = "";
    
    // Read complete line
    while(Serial.available()) {
      char c = Serial.read();
      if(c == '\n' || c == '\r') {
        if(receivedData.length() > 0) {
          break;
        }
      } else {
        receivedData += c;
      }
      delay(1);
    }
    
    if(receivedData.length() > 0) {
      // Echo command with timestamp
      printTimestamp();
      Serial.print("PC CMD:  ");
      Serial.println(receivedData);
      
      // Optional: Forward to LoRa for testing
      if(receivedData == "test") {
        Serial2.println("TEST_RESPONSE");
        Serial.println("Test command sent to LoRa");
      }
    }
  }
  
  // Print statistics periodically
  printStatistics();
  
  // Heartbeat LED (slow blink when idle)
  if(millis() - lastLedTime > 2000) {
    if(millis() - lastCommandTime > 1000) { // Only blink when idle
      digitalWrite(LED_BUILTIN, HIGH);
      delay(20);
      digitalWrite(LED_BUILTIN, LOW);
    }
    lastLedTime = millis();
  }
}