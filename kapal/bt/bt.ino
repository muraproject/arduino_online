// ESP32 LoRa Serial Bluetooth Bridge
// Forwards data between Serial, LoRa, and Bluetooth in all directions
// Using RXD2(16) and TXD2(17) for LoRa module
// Using built-in Bluetooth Classic (Serial Profile)

#include "BluetoothSerial.h"

// LoRa pins
#define RXD2 17  // LoRa TX to ESP32 RX2
#define TXD2 16  // LoRa RX to ESP32 TX2

// LED pin for visual feedback
#define LED_BUILTIN 2  // Most ESP32 boards have an LED on GPIO2

// Bluetooth Serial object
BluetoothSerial SerialBT;

// Device name for Bluetooth
String device_name = "ESP32-LoRa-Bridge";

// Variables for LED indication
unsigned long lastLedTime = 0;
bool ledState = false;

void setup() {
  // Initialize serial communications
  Serial.begin(115200);  // USB Serial
  Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2); // LoRa module
  
  // Initialize Bluetooth Serial
  if(!SerialBT.begin(device_name)) {
    Serial.println("An error occurred initializing Bluetooth");
  } else {
    Serial.println("Bluetooth initialized successfully");
    Serial.println("The device started, now you can pair it with bluetooth!");
    Serial.print("Device Name: ");
    Serial.println(device_name);
  }
  
  // Set pin mode for LED
  pinMode(LED_BUILTIN, OUTPUT);
  
  Serial.println("\n=========================================");
  Serial.println("ESP32 LoRa Serial Bluetooth Bridge Started");
  Serial.println("Data forwarding:");
  Serial.println("- Serial <-> LoRa <-> Bluetooth");
  Serial.println("- All interfaces are interconnected");
  Serial.println("=========================================");
  
  // LED startup sequence
  for(int i = 0; i < 3; i++) {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(200);
    digitalWrite(LED_BUILTIN, LOW);
    delay(200);
  }
}

void flashLED(int duration_us) {
  digitalWrite(LED_BUILTIN, HIGH);
  delayMicroseconds(duration_us);
  digitalWrite(LED_BUILTIN, LOW);
}

void loop() {
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
  if(millis() - lastLedTime > 2000) {  // Every 2 seconds
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