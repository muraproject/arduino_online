#include "BluetoothSerial.h"

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` enable it
#endif

BluetoothSerial SerialBT;

// Define pin untuk setiap lampu
const int LAMP_KAMAR_MANDI = 18;  // GPIO2
const int LAMP_KAMAR_TIDUR = 19;  // GPIO4
const int LAMP_RUANG_TAMU = 21;   // GPIO5

void setup() {
  Serial.begin(115200);
  SerialBT.begin("ESP32_Smart_Home"); 
  
  pinMode(LAMP_KAMAR_MANDI, OUTPUT);
  pinMode(LAMP_KAMAR_TIDUR, OUTPUT);
  pinMode(LAMP_RUANG_TAMU, OUTPUT);
  
  // digitalWrite(LAMP_KAMAR_MANDI, HIGH);
  // digitalWrite(LAMP_KAMAR_TIDUR, HIGH);
  // digitalWrite(LAMP_RUANG_TAMU, HIGH);
}

void loop() {
  if (SerialBT.available()) {
    String command = SerialBT.readStringUntil('\n');
    command.toLowerCase();
    command.trim();

    if (command == "kamar mandi") {
      digitalWrite(LAMP_KAMAR_MANDI, HIGH);
      digitalWrite(LAMP_KAMAR_TIDUR, LOW);
      digitalWrite(LAMP_RUANG_TAMU, LOW);
      Serial.println("Kamar Mandi");
    }
    else if (command == "kamar tidur") {
      digitalWrite(LAMP_KAMAR_MANDI, LOW);
      digitalWrite(LAMP_KAMAR_TIDUR, HIGH);
      digitalWrite(LAMP_RUANG_TAMU, LOW);
      Serial.println("Kamar Tidur");
    }
    else if (command == "ruang tamu") {
      digitalWrite(LAMP_KAMAR_MANDI, LOW);
      digitalWrite(LAMP_KAMAR_TIDUR, LOW);
      digitalWrite(LAMP_RUANG_TAMU, HIGH);
      Serial.println("Ruang Tamu");
    }
  }
  delay(20);
}