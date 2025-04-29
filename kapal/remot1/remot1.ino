// ESP32 Simple LoRa Serial Bridge
// Using RXD2(16) and TXD2(17) for LoRa module
// This code simply forwards data between Serial and LoRa in both directions

// LoRa pins
#define RXD2 16  // LoRa TX to ESP32 RX2
#define TXD2 17  // LoRa RX to ESP32 TX2

// LED pin for visual feedback
#define LED_BUILTIN 2  // Most ESP32 boards have an LED on GPIO2

void setup() {
  // Initialize serial communications
  Serial.begin(115200);  // USB Serial
  Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2); // LoRa module
  
  // Set pin mode for LED
  pinMode(LED_BUILTIN, OUTPUT);
  
  Serial.println("\n=========================================");
  Serial.println("ESP32 LoRa Serial Bridge Started");
  Serial.println("All serial data will be forwarded to LoRa");
  Serial.println("All LoRa data will be shown in Serial Monitor");
  Serial.println("=========================================");
}

void loop() {
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
}