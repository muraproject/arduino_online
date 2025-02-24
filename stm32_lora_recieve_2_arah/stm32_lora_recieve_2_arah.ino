#include <SoftwareSerial.h>

// LoRa on Hardware Serial
HardwareSerial LoRaSerial(PA10, PA9);  // RX, TX for STM32

// LoRa control pins
#define M0_PIN PB5
#define M1_PIN PB4
#define AUX_PIN PB3

String currentCommand = "STP";
unsigned long messageCount = 0;
unsigned long lastMessageTime = 0;

void setup() {
  // Debug Serial
  Serial.begin(9600);
  while(!Serial) { }
  delay(1000);
  
  Serial.println("\n--- LoRa Debug Test Started ---");
  
  // Debug pin status
  Serial.println("\nChecking Pins:");
  Serial.print("M0_PIN (PB5): "); 
  Serial.println(digitalRead(M0_PIN));
  Serial.print("M1_PIN (PB4): ");
  Serial.println(digitalRead(M1_PIN));
  Serial.print("AUX_PIN (PB3): ");
  Serial.println(digitalRead(AUX_PIN));
  
  // LoRa Setup
  Serial.println("\nInitializing LoRa...");
  
  // LoRa Control Pins
  pinMode(M0_PIN, OUTPUT);
  pinMode(M1_PIN, OUTPUT);
  pinMode(AUX_PIN, INPUT);
  
  // Set Normal Mode
  digitalWrite(M0_PIN, LOW);
  digitalWrite(M1_PIN, LOW);
  
  Serial.println("Waiting for AUX to stabilize...");
  delay(100);  // Wait for AUX to stabilize
  
  Serial.print("AUX Status: ");
  Serial.println(digitalRead(AUX_PIN));
  
  // Initialize LoRa Serial
  LoRaSerial.begin(9600);
  Serial.println("LoRa Serial Initialized");
  
  // Send test message
  delay(1000);
  Serial.println("\nSending test message...");
  LoRaSerial.println("TEST:INIT");
  
  printStatus();
}

void loop() {
  static unsigned long lastStatus = 0;
  
  // Print status every 5 seconds
  if (millis() - lastStatus >= 5000) {
    printStatus();
    lastStatus = millis();
    
    // Send periodic test message
    String sent = "SPD:";
    sent+= lastStatus/1000;
    LoRaSerial.println(sent);
    Serial.println("Sent: SPD:67");
  }

  // Check for incoming data
  if (LoRaSerial.available()) {
    String data = LoRaSerial.readStringUntil('\n');
    data.trim();
    messageCount++;
    lastMessageTime = millis();
    
    Serial.println("\n--- Received Message ---");
    Serial.print("Data: ");
    Serial.println(data);
    Serial.print("Message Count: ");
    Serial.println(messageCount);
    Serial.print("RSSI: N/A");  // E220 doesn't provide RSSI
    Serial.print("AUX Pin: ");
    Serial.println(digitalRead(AUX_PIN));
    
    // Echo back
    LoRaSerial.print("ACK:");
    LoRaSerial.println(data);
    Serial.print("Sent ACK: ");
    Serial.println(data);
  }

  // Check for any Serial input for testing
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    Serial.print("Sending manual command: ");
    Serial.println(command);
    LoRaSerial.println(command);
  }
}

void printStatus() {
  Serial.println("\n=== LoRa Status ===");
  Serial.print("Runtime (s): ");
  Serial.println(millis()/1000);
  Serial.print("Messages Received: ");
  Serial.println(messageCount);
  Serial.print("Last Message (ms ago): ");
  if (lastMessageTime > 0) {
    Serial.println(millis() - lastMessageTime);
  } else {
    Serial.println("No messages yet");
  }
  Serial.print("AUX Pin: ");
  Serial.println(digitalRead(AUX_PIN));
  Serial.print("M0 Pin: ");
  Serial.println(digitalRead(M0_PIN));
  Serial.print("M1 Pin: ");
  Serial.println(digitalRead(M1_PIN));
  Serial.println("===================\n");
}