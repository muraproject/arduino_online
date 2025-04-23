/*
 * Program Monitoring Sensor Tegangan-Arus INA226
 * Untuk Arduino Nano
 * Membaca data dari 4 sensor INA226 pada alamat 0x40, 0x41, 0x44, dan 0x45
 */

#include <Wire.h>
#include "INA226.h"

// Inisialisasi 4 sensor INA226 di alamat yang sudah diketahui
INA226 INA_40(0x40);  // Sensor 1 pada alamat 0x40 solar voltage
INA226 INA_41(0x41);  // Sensor 2 pada alamat 0x41 solar charging voltage / batt
INA226 INA_44(0x44);  // Sensor 3 pada alamat 0x44  wind voltage
INA226 INA_45(0x45);  // Sensor 4 pada alamat 0x45  wind charging voltage / batt

// Flag untuk menandai sensor yang berhasil diinisialisasi
bool INA_40_connected = false;
bool INA_41_connected = false;
bool INA_44_connected = false;
bool INA_45_connected = false;

// Interval pembacaan sensor
const unsigned long READ_INTERVAL = 1000;  // Membaca sensor setiap 1 detik
unsigned long lastReadTime = 0;

void setup() {
  // Inisialisasi komunikasi serial dan I2C
  Serial.begin(115200);
  Wire.begin();
  
  // Tampilkan header informasi program
  Serial.println(F("\nProgram Monitoring Sensor Tegangan-Arus INA226"));
  Serial.print(F("INA226_LIB_VERSION: "));
  Serial.println(INA226_LIB_VERSION);
  
  // Inisialisasi sensor 1 (alamat 0x40)
  INA_40_connected = INA_40.begin();
  if (INA_40_connected) {
    INA_40.setMaxCurrentShunt(1, 0.002);  // 1A max current, 2 mOhm shunt resistor
    Serial.println(F("Sensor 1 (0x40): Terhubung dan diinisialisasi"));
  } else {
    Serial.println(F("Sensor 1 (0x40): GAGAL terhubung"));
  }
  
  // Inisialisasi sensor 2 (alamat 0x41)
  INA_41_connected = INA_41.begin();
  if (INA_41_connected) {
    INA_41.setMaxCurrentShunt(1, 0.002);
    Serial.println(F("Sensor 2 (0x41): Terhubung dan diinisialisasi"));
  } else {
    Serial.println(F("Sensor 2 (0x41): GAGAL terhubung"));
  }
  
  // Inisialisasi sensor 3 (alamat 0x44)
  INA_44_connected = INA_44.begin();
  if (INA_44_connected) {
    INA_44.setMaxCurrentShunt(1, 0.002);
    Serial.println(F("Sensor 3 (0x44): Terhubung dan diinisialisasi"));
  } else {
    Serial.println(F("Sensor 3 (0x44): GAGAL terhubung"));
  }
  
  // Inisialisasi sensor 4 (alamat 0x45)
  INA_45_connected = INA_45.begin();
  if (INA_45_connected) {
    INA_45.setMaxCurrentShunt(1, 0.002);
    Serial.println(F("Sensor 4 (0x45): Terhubung dan diinisialisasi"));
  } else {
    Serial.println(F("Sensor 4 (0x45): GAGAL terhubung"));
  }
  
  // Periksa apakah ada sensor yang terhubung
  if (!INA_40_connected && !INA_41_connected && !INA_44_connected && !INA_45_connected) {
    Serial.println(F("ERROR: Tidak ada sensor yang terhubung!"));
    Serial.println(F("Periksa koneksi dan restart Arduino."));
    while (1); // Berhenti jika tidak ada sensor yang terhubung
  }
  
  // Cetak header tabel data
  printTableHeader();
}

void loop() {
  unsigned long currentMillis = millis();
  
  // Baca dan tampilkan data sensor setiap READ_INTERVAL (1 detik)
  if (currentMillis - lastReadTime >= READ_INTERVAL) {
    lastReadTime = currentMillis;
    
    // Baca dan tampilkan data dari semua sensor
    readAndDisplaySensorData();
  }
}

// Fungsi untuk mencetak header tabel data
void printTableHeader() {
  Serial.println(F("\n-----------------------------------------------------------------------------------------------------------------"));
  Serial.println(F("| Sensor | Bus Voltage (V) | Shunt Voltage (mV) | Current (mA) | Power (mW) | Status |"));
  Serial.println(F("-----------------------------------------------------------------------------------------------------------------"));
}

// Fungsi untuk membaca dan menampilkan data dari semua sensor
void readAndDisplaySensorData() {
  // Cek apakah perlu mencetak header tabel baru (setiap 20 baris)
  static int rowCount = 0;
  if (rowCount >= 20) {
    printTableHeader();
    rowCount = 0;
  }
  rowCount++;
  
  // Baca dan tampilkan data dari sensor 1 (0x40)
  if (INA_40_connected) {
    Serial.print(F("| 0x40   | "));
    Serial.print(INA_40.getBusVoltage(), 3);
    Serial.print(F(" V        | "));
    Serial.print(INA_40.getShuntVoltage_mV(), 3);
    Serial.print(F(" mV         | "));
    Serial.print(INA_40.getCurrent_mA(), 3);
    Serial.print(F(" mA    | "));
    Serial.print(INA_40.getPower_mW(), 3);
    Serial.println(F(" mW    | OK     |"));
  } else {
    Serial.println(F("| 0x40   | -             | -                 | -           | -          | OFFLINE |"));
  }
  
  // Baca dan tampilkan data dari sensor 2 (0x41)
  if (INA_41_connected) {
    Serial.print(F("| 0x41   | "));
    Serial.print(INA_41.getBusVoltage(), 3);
    Serial.print(F(" V        | "));
    Serial.print(INA_41.getShuntVoltage_mV(), 3);
    Serial.print(F(" mV         | "));
    Serial.print(INA_41.getCurrent_mA(), 3);
    Serial.print(F(" mA    | "));
    Serial.print(INA_41.getPower_mW(), 3);
    Serial.println(F(" mW    | OK     |"));
  } else {
    Serial.println(F("| 0x41   | -             | -                 | -           | -          | OFFLINE |"));
  }
  
  // Baca dan tampilkan data dari sensor 3 (0x44)
  if (INA_44_connected) {
    Serial.print(F("| 0x44   | "));
    Serial.print(INA_44.getBusVoltage(), 3);
    Serial.print(F(" V        | "));
    Serial.print(INA_44.getShuntVoltage_mV(), 3);
    Serial.print(F(" mV         | "));
    Serial.print(INA_44.getCurrent_mA(), 3);
    Serial.print(F(" mA    | "));
    Serial.print(INA_44.getPower_mW(), 3);
    Serial.println(F(" mW    | OK     |"));
  } else {
    Serial.println(F("| 0x44   | -             | -                 | -           | -          | OFFLINE |"));
  }
  
  // Baca dan tampilkan data dari sensor 4 (0x45)
  if (INA_45_connected) {
    Serial.print(F("| 0x45   | "));
    Serial.print(INA_45.getBusVoltage(), 3);
    Serial.print(F(" V        | "));
    Serial.print(INA_45.getShuntVoltage_mV(), 3);
    Serial.print(F(" mV         | "));
    Serial.print(INA_45.getCurrent_mA(), 3);
    Serial.print(F(" mA    | "));
    Serial.print(INA_45.getPower_mW(), 3);
    Serial.println(F(" mW    | OK     |"));
  } else {
    Serial.println(F("| 0x45   | -             | -                 | -           | -          | OFFLINE |"));
  }
  
  Serial.println(F("-----------------------------------------------------------------------------------------------------------------"));
}
