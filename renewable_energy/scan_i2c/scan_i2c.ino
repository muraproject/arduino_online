/*
 * Program I2C Scanner untuk Arduino Nano
 * Memindai semua alamat I2C setiap 3 detik dan menampilkan hasilnya di Serial Monitor
 */

#include <Wire.h>

void setup() {
  Wire.begin();        // Inisialisasi I2C sebagai master
  Serial.begin(9600);  // Inisialisasi komunikasi serial dengan kecepatan 9600 baud
  
  Serial.println("\nProgram Scan I2C Arduino Nano");
  Serial.println("Memindai alamat perangkat I2C setiap 3 detik...");
}

void loop() {
  byte error, address;
  int deviceCount = 0;
  
  Serial.println("\n----- Memulai pemindaian I2C -----");
  
  // Pindai alamat dari 1 sampai 127 (alamat 7-bit)
  for (address = 1; address < 128; address++) {
    // Fungsi endTransmission mengembalikan 0 jika perangkat ditemukan
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
    
    if (error == 0) {
      // Perangkat ditemukan
      Serial.print("Perangkat I2C ditemukan pada alamat 0x");
      if (address < 16) {
        Serial.print("0"); // Menambahkan nol di depan untuk alamat < 16 (0x10)
      }
      Serial.print(address, HEX);
      Serial.println();
      deviceCount++;
    }
    else if (error == 4) {
      // Error saat komunikasi
      Serial.print("Error pada alamat 0x");
      if (address < 16) {
        Serial.print("0");
      }
      Serial.println(address, HEX);
    }
  }
  
  if (deviceCount == 0) {
    Serial.println("Tidak ada perangkat I2C yang ditemukan!");
  } else {
    Serial.print("Jumlah perangkat yang ditemukan: ");
    Serial.println(deviceCount);
  }
  
  Serial.println("----- Pemindaian selesai -----");
  
  // Tunggu 3 detik sebelum pemindaian berikutnya
  delay(3000);
}