
// RECEIVER CODE (STM32)
// File: LoRa_LED_Control_RX.ino

#include <SoftwareSerial.h>

#define M0_PIN PB5
#define M1_PIN PB4
#define AUX_PIN PB3
#define LORA_RX 11
#define LORA_TX 10
// #define LED_BUILTIN PC13  // LED builtin STM32 Nucleo

// Inisialisasi Software Serial
SoftwareSerial LoRaSerial(LORA_RX, LORA_TX); // RX, TX

// Buffer untuk data
char receivedData[32];
int dataIndex = 0;

void setup() {
  // Inisialisasi Serial untuk debugging
  Serial.begin(9600);
  
  // Inisialisasi Software Serial untuk E220
  LoRaSerial.begin(9600);
  
  // Setup pin modes
  pinMode(M0_PIN, OUTPUT);
  pinMode(M1_PIN, OUTPUT);
  pinMode(AUX_PIN, INPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  
  // Set ke normal mode
  digitalWrite(M0_PIN, LOW);
  digitalWrite(M1_PIN, LOW);
  
  // Set LED ke OFF saat startup
  digitalWrite(LED_BUILTIN, HIGH);  // HIGH adalah OFF untuk LED builtin STM32
  
  delay(1000);
  Serial.println("LoRa E220 LED Control Receiver Ready");
}

void loop() {
  if (LoRaSerial.available()) {
    char c = LoRaSerial.read();
    
    if (c != '\n' && dataIndex < 31) {
      receivedData[dataIndex] = c;
      dataIndex++;
    } else {
      receivedData[dataIndex] = '\0';
      String command = String(receivedData);
      command.trim();
      command.toLowerCase();
      
      // Proses perintah
      if (command == "on") {
        digitalWrite(LED_BUILTIN, LOW);  // LOW adalah ON untuk LED builtin STM32
        Serial.println("LED ON");
      } 
      else if (command == "off") {
        digitalWrite(LED_BUILTIN, HIGH); // HIGH adalah OFF untuk LED builtin STM32
        Serial.println("LED OFF");
      }
      
      // Reset buffer
      dataIndex = 0;
    }
  }
}