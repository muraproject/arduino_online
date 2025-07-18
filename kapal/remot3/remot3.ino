// ESP32 LoRa Remote Control Bridge
// Forwards data between Serial, LoRa, and Bluetooth with remote control features
// Using RXD2(16) and TXD2(17) for LoRa module
// Using built-in Bluetooth Classic (Serial Profile)
// Remote control with 8 modes and analog steering (spam mode)

#include "BluetoothSerial.h"
#include <LiquidCrystal_I2C.h>
#include <Wire.h>

// LoRa pins
#define RXD2 17  // LoRa TX to ESP32 RX2
#define TXD2 16  // LoRa RX to ESP32 TX2

// LED pin for visual feedback
#define LED_BUILTIN 2

// Button pins configuration
const int buttonPins[] = {19, 23, 18, 5, 4, 32, 33};
const int numButtons = sizeof(buttonPins) / sizeof(buttonPins[0]);

// Analog steering pin
const int analogPin = 35;

// Bluetooth Serial object
BluetoothSerial SerialBT;

// LCD Setup
LiquidCrystal_I2C lcd(0x27, 16, 2);

// Device name for Bluetooth
String device_name = "ESP32-LoRa-Remote";

// Variables for LED indication
unsigned long lastLedTime = 0;
bool ledState = false;

// Remote control variables
int currentMode = 1; // Start with mode 1 (Manual)
bool buttonStates[numButtons];
bool lastButtonStates[numButtons];
unsigned long lastButtonTime[numButtons];
const unsigned long debounceDelay = 50;

// Analog control variables (spam mode)
String currentDirection = "";
unsigned long lastAnalogTime = 0;
const unsigned long analogDelay = 200; // Spam every 200ms

// Mode names for LCD
const String modeNames[] = {
  "", "Manual", "F-BLDC", "F-SMC", "F-Engine", 
  "F-Hybrid", "PID", "F-PID", "NN"
};

// Command arrays for each mode
const String modeCommands[][5] = {
  {"", "", "", "", ""}, // Mode 0 (unused)
  {"s", "+", "-", "p", "l"}, // Mode 1: Manual
  {"f6000", "f7000", "f8000", "f9000", "f10000"}, // Mode 2: Fuzzy BLDC
  {"m6000", "m7000", "m8000", "m9000", "m10000"}, // Mode 3: Fuzzy SMC
  {"t6000", "t7000", "t8000", "t9000", "t10000"}, // Mode 4: Fuzzy Engine
  {"g4", "g5", "g7", "g9", "g11"}, // Mode 5: Fuzzy Hybrid
  {"p4", "p5", "p7", "p9", "p12"}, // Mode 6: PID
  {"d4", "d5", "d7", "d9", "d11"}, // Mode 7: Fuzzy PID
  {"n4", "n5", "n7", "n9", "n11"}  // Mode 8: NN
};

// Buffer for LCD
char lcdBuffer[17];

void setup() {
  // Initialize serial communications
  Serial.begin(115200);
  Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2);
  
  // Initialize LCD
  lcd.init();
  lcd.backlight();
  
  // Initialize Bluetooth Serial
  if(!SerialBT.begin(device_name)) {
    Serial.println("An error occurred initializing Bluetooth");
  } else {
    Serial.println("Bluetooth initialized successfully");
    Serial.println("The device started, now you can pair it with bluetooth!");
    Serial.print("Device Name: ");
    Serial.println(device_name);
  }
  
  // Setup button pins
  for(int i = 0; i < numButtons; i++) {
    pinMode(buttonPins[i], INPUT_PULLUP);
    buttonStates[i] = HIGH;
    lastButtonStates[i] = HIGH;
    lastButtonTime[i] = 0;
  }
  
  // Set pin mode for LED
  pinMode(LED_BUILTIN, OUTPUT);
  
  // Display startup message
  Serial.println("\n=========================================");
  Serial.println("ESP32 LoRa Remote Control Bridge Started");
  Serial.println("Remote Control with 8 modes + Spam Mode");
  Serial.println("Analog steering: 200ms spam mode");
  Serial.println("=========================================");
  
  // LCD startup
  sprintf(lcdBuffer, "LoRa Remote Ctrl");
  lcd.setCursor(0, 0);
  lcd.print(lcdBuffer);
  sprintf(lcdBuffer, "Starting...     ");
  lcd.setCursor(0, 1);
  lcd.print(lcdBuffer);
  
  // LED startup sequence
  for(int i = 0; i < 3; i++) {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(200);
    digitalWrite(LED_BUILTIN, LOW);
    delay(200);
  }
  
  delay(1000);
  updateLCD();
}

void flashLED(int duration_us) {
  digitalWrite(LED_BUILTIN, HIGH);
  delayMicroseconds(duration_us);
  digitalWrite(LED_BUILTIN, LOW);
}

void sendCommand(String command) {
  // Send to Serial
  Serial.println(command);
  
  // Send to LoRa
  Serial2.println(command);
  
  // Send to Bluetooth if connected
  if(SerialBT.hasClient()) {
    SerialBT.println(command);
  }
  
  // Flash LED for command sent
  flashLED(500);
  
  Serial.print("CMD: ");
  Serial.println(command);
}

void updateLCD() {
  // Line 1: Mode info
  sprintf(lcdBuffer, "Mode%d:%s     ", currentMode, modeNames[currentMode].c_str());
  lcd.setCursor(0, 0);
  lcd.print(lcdBuffer);
  
  // Line 2: Current direction and analog value
  int analogValue = analogRead(analogPin);
  String dirText = "";
  if(analogValue > 1500) dirText = "KIRI";
  else if(analogValue < 500) dirText = "KANAN";
  else dirText = "LURUS";
  
  sprintf(lcdBuffer, "%s A:%4d   ", dirText.c_str(), analogValue);
  lcd.setCursor(0, 1);
  lcd.print(lcdBuffer);
}

void handleButtons() {
  for(int i = 0; i < numButtons; i++) {
    int reading = digitalRead(buttonPins[i]);
    
    if(reading != lastButtonStates[i]) {
      lastButtonTime[i] = millis();
    }
    
    if((millis() - lastButtonTime[i]) > debounceDelay) {
      if(reading != buttonStates[i]) {
        buttonStates[i] = reading;
        
        if(buttonStates[i] == LOW) { // Button pressed
          if(buttonPins[i] == 33) { // Mode change button
            currentMode++;
            if(currentMode > 8) currentMode = 1;
            updateLCD();
            Serial.print("Mode changed to: ");
            Serial.print(currentMode);
            Serial.print(" (");
            Serial.print(modeNames[currentMode]);
            Serial.println(")");
          }
          else if(buttonPins[i] == 19) { // Always send 'x'
            sendCommand("x");
          }
          else { // Other buttons - send mode-specific commands
            int buttonIndex = -1;
            // Find button index in the command array
            if(buttonPins[i] == 32) buttonIndex = 0;      // Pin 32 -> Command 0
            else if(buttonPins[i] == 4) buttonIndex = 1;  // Pin 4  -> Command 1
            else if(buttonPins[i] == 5) buttonIndex = 2;  // Pin 5  -> Command 2
            else if(buttonPins[i] == 18) buttonIndex = 3; // Pin 18 -> Command 3
            else if(buttonPins[i] == 23) buttonIndex = 4; // Pin 23 -> Command 4
            
            if(buttonIndex >= 0) {
              sendCommand(modeCommands[currentMode][buttonIndex]);
            }
          }
        }
      }
    }
    
    lastButtonStates[i] = reading;
  }
}

void handleAnalogSteering() {
  // Spam mode: send command every 200ms regardless of change
  if(millis() - lastAnalogTime >= analogDelay) {
    int analogValue = analogRead(analogPin);
    String direction = "";
    
    if(analogValue > 1500) {
      direction = "T:90,A:130"; // Kiri
    }
    else if(analogValue < 500) {
      direction = "T:90,A:50";  // Kanan
    }
    else {
      direction = "T:90,A:90";  // Lurus
    }
    
    // Send command every 200ms (spam mode)
    sendCommand(direction);
    currentDirection = direction;
    
    lastAnalogTime = millis();
  }
}

void loop() {
  // Handle remote control
  handleButtons();
  handleAnalogSteering();
  
  // Update LCD periodically (not too often to avoid flicker)
  static unsigned long lastLCDUpdate = 0;
  if(millis() - lastLCDUpdate > 300) {
    updateLCD();
    lastLCDUpdate = millis();
  }
  
  // 1. Forward data from LoRa to Serial and Bluetooth
  while (Serial2.available()) {
    char c = Serial2.read();
    
    // Forward to USB Serial
    Serial.write(c);
    
    // Forward to Bluetooth if connected
    if(SerialBT.hasClient()) {
      SerialBT.write(c);
    }
    
    // LED flash for LoRa received data (short flash)
    flashLED(300);
  }
  
  // 2. Forward data from USB Serial to LoRa and Bluetooth
  while (Serial.available()) {
    char c = Serial.read();
    
    // Forward to LoRa
    Serial2.write(c);
    
    // Forward to Bluetooth if connected
    if(SerialBT.hasClient()) {
      SerialBT.write(c);
    }
    
    // LED flash for Serial transmitted data (medium flash)
    flashLED(600);
  }
  
  // 3. Forward data from Bluetooth to Serial and LoRa
  while (SerialBT.available()) {
    char c = SerialBT.read();
    
    // Forward to USB Serial
    Serial.write(c);
    
    // Forward to LoRa
    Serial2.write(c);
    
    // LED flash for Bluetooth received data (long flash)
    flashLED(900);
  }
  
  // Connection status indicator (slow blink when Bluetooth connected)
  if(millis() - lastLedTime > 2000) {
    if(SerialBT.hasClient()) {
      digitalWrite(LED_BUILTIN, HIGH);
      delay(50);
      digitalWrite(LED_BUILTIN, LOW);
    }
    lastLedTime = millis();
  }
}

// Optional: Bluetooth event callbacks for debugging
void bluetoothEventCallback(esp_spp_cb_event_t event, esp_spp_cb_param_t *param) {
  if(event == ESP_SPP_SRV_OPEN_EVT) {
    Serial.println("Bluetooth Client Connected");
  }
  if(event == ESP_SPP_CLOSE_EVT) {
    Serial.println("Bluetooth Client Disconnected");
  }
}