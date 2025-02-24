#include <HardwareSerial.h>
#include "BluetoothSerial.h"

#define RX_PIN 17
#define TX_PIN 16

HardwareSerial TimbanganSerial(2);
BluetoothSerial SerialBT;

String lastData = "";
unsigned long lastSendTime = 0;
const unsigned long sendInterval = 1000; // Interval pengiriman 1 detik

void setup() {
  Serial.begin(115200);
  TimbanganSerial.begin(9600, SERIAL_8N1, RX_PIN, TX_PIN);
  SerialBT.begin("ESP32_Timbangan"); // Nama Bluetooth device

  Serial.println("Sistem Timbangan Sonic A12E dengan ESP32");
  Serial.println("Menunggu data...");
}

void loop() {
  if (TimbanganSerial.available()) {
    String data = TimbanganSerial.readStringUntil('\n');
    data.trim();
    
    if (data.length() > 0) {
      lastData = data;
      Serial.println("Data diterima: " + lastData);
    }
  }

  unsigned long currentTime = millis();
  if (currentTime - lastSendTime >= sendInterval) {
    if (lastData != "") {
      SerialBT.println(lastData);
      Serial.println("Data terkirim via Bluetooth: " + lastData);
    }
    lastSendTime = currentTime;
  }
}