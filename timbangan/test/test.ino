#include <HardwareSerial.h>

// Tentukan pin RX dan TX yang akan digunakan
#define RX_PIN 17 // Ganti dengan pin RX yang Anda gunakan
#define TX_PIN 16 // Ganti dengan pin TX yang Anda gunakan

HardwareSerial TimbanganSerial(2); // Gunakan UART1

void setup() {
  Serial.begin(115200); // Inisialisasi Serial Monitor
  TimbanganSerial.begin(9600, SERIAL_8N1, RX_PIN, TX_PIN); // Sesuaikan baud rate jika diperlukan
  
  Serial.println("Sistem Timbangan Sonic A12E dengan ESP32");
  Serial.println("Menunggu data...");
}

void loop() {
  if (TimbanganSerial.available()) {
    String data = TimbanganSerial.readStringUntil('\n');
    data.trim(); // Hapus whitespace
    
    if (data.length() > 0) {
      // Serial.print("Berat: ");
      Serial.println(data);
    }
  }
  // Serial.println(TimbanganSerial);
}