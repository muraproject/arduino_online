/*
 * Program Monitoring Sensor Tegangan-Arus INA226
 * Untuk Arduino Nano
 * Membaca data dari 4 sensor INA226 pada alamat 0x40, 0x41, 0x44, dan 0x45
 * Melakukan 20 pembacaan dan menghitung nilai RMS
 * Output berupa API URL untuk pengiriman data
 * Nilai minimum sensor dibatasi ke 0
 */

#include <Wire.h>
#include "INA226.h"
#include <SoftwareSerial.h>
#include <math.h>  // Untuk fungsi sqrt() dalam perhitungan RMS

// Definisi pin untuk SoftwareSerial
#define RX_PIN 11  // Sesuaikan dengan pin yang digunakan
#define TX_PIN 12  // Sesuaikan dengan pin yang digunakan

// Inisialisasi SoftwareSerial
SoftwareSerial mySerial(RX_PIN, TX_PIN);

// Inisialisasi 4 sensor INA226 di alamat yang sudah diketahui
INA226 INA_40(0x40);  // Sensor 1 pada alamat 0x40 solar voltage
INA226 INA_41(0x41);  // Sensor 2 pada alamat 0x41 solar charging voltage / batt
INA226 INA_44(0x44);  // Sensor 3 pada alamat 0x44 wind voltage
INA226 INA_45(0x45);  // Sensor 4 pada alamat 0x45 wind charging voltage / batt

// Flag untuk menandai sensor yang berhasil diinisialisasi
bool INA_40_connected = false;
bool INA_41_connected = false;
bool INA_44_connected = false;
bool INA_45_connected = false;

// Interval pembacaan sensor
const unsigned long READ_INTERVAL = 5000;  // Membaca sensor setiap 5 detik
unsigned long lastReadTime = 0;

// Variabel untuk menyimpan data sensor
float solar_voltage = 0.0;        // Tegangan panel surya (INA_40)
float solar_current = 0.0;        // Arus panel surya (INA_40)
float solar_charging_voltage = 0.0;  // Tegangan charging solar (INA_41)
float solar_charging_current = 0.0;  // Arus charging solar (INA_41)
float solar_battery_voltage = 12.46; // Tegangan baterai solar (tetap)
float wind_voltage = 0.0;         // Tegangan turbin angin (INA_44)
float wind_current = 0.0;         // Arus turbin angin (INA_44)
float wind_charging_voltage = 0.0;   // Tegangan charging angin (INA_45)
float wind_charging_current = 0.0;   // Arus charging angin (INA_45)
float wind_battery_voltage = 11.77;  // Tegangan baterai angin (tetap)
String load_status = "Disconnected"; // Status beban

// Jumlah sampel yang akan diambil untuk perhitungan RMS
const int NUM_SAMPLES = 20;

// Base URL untuk API
const String API_BASE_URL = "http://103.130.16.22:3311/pc/renewable2/api.php?action=insert";

// Fungsi untuk memastikan nilai tidak negatif
float ensurePositive(float value) {
  return (value < 0) ? 0.0 : value;
}

// Fungsi untuk menghitung nilai RMS dari array nilai
float calculateRMS(float values[], int numValues) {
  float sumOfSquares = 0.0;
  for (int i = 0; i < numValues; i++) {
    sumOfSquares += values[i] * values[i];
  }
  return sqrt(sumOfSquares / numValues);
}

void setup() {
  // Inisialisasi komunikasi serial dan I2C
  Serial.begin(115200);
  mySerial.begin(9600);  // Inisialisasi SoftwareSerial
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
  
  // Cetak informasi mode API
  Serial.println(F("\nOutput mode: API URL"));
  mySerial.println(F("Output mode: API URL"));
}

void loop() {
  unsigned long currentMillis = millis();
  
  // Baca dan tampilkan data sensor setiap READ_INTERVAL
  if (currentMillis - lastReadTime >= READ_INTERVAL) {
    lastReadTime = currentMillis;
    
    // Baca data dari semua sensor
    readSensorData();
    
    // Format dan kirim data sebagai API URL
    sendApiUrl();
  }
}

// Fungsi untuk membaca data dari semua sensor dengan RMS
void readSensorData() {
  // Array untuk menyimpan sampel untuk perhitungan RMS
  float solar_voltage_samples[NUM_SAMPLES] = {0};
  float solar_current_samples[NUM_SAMPLES] = {0};
  float solar_charging_voltage_samples[NUM_SAMPLES] = {0};
  float solar_charging_current_samples[NUM_SAMPLES] = {0};
  float wind_voltage_samples[NUM_SAMPLES] = {0};
  float wind_current_samples[NUM_SAMPLES] = {0};
  float wind_charging_voltage_samples[NUM_SAMPLES] = {0};
  float wind_charging_current_samples[NUM_SAMPLES] = {0};
  
  // Ambil NUM_SAMPLES sampel dari setiap sensor
  for (int i = 0; i < NUM_SAMPLES; i++) {
    // Baca data dari sensor 1 (0x40) - Panel surya
    if (INA_40_connected) {
      solar_voltage_samples[i] = ensurePositive(INA_40.getBusVoltage());
      solar_current_samples[i] = ensurePositive(INA_40.getCurrent_mA() / 1000.0); // Convert mA to A
    }
    
    // Baca data dari sensor 2 (0x41) - Charging solar
    if (INA_41_connected) {
      solar_charging_voltage_samples[i] = ensurePositive(INA_41.getBusVoltage());
      solar_charging_current_samples[i] = ensurePositive(INA_41.getCurrent_mA() / 1000.0); // Convert mA to A
    }
    
    // Baca data dari sensor 3 (0x44) - Turbin angin
    if (INA_44_connected) {
      wind_voltage_samples[i] = ensurePositive(INA_44.getBusVoltage());
      wind_current_samples[i] = ensurePositive(INA_44.getCurrent_mA() / 1000.0); // Convert mA to A
    }
    
    // Baca data dari sensor 4 (0x45) - Charging angin
    if (INA_45_connected) {
      wind_charging_voltage_samples[i] = ensurePositive(INA_45.getBusVoltage());
      wind_charging_current_samples[i] = ensurePositive(INA_45.getCurrent_mA() / 1000.0); // Convert mA to A
    }
    
    // Delay kecil antar pembacaan sampel
    delay(10);
  }
  
  // Hitung nilai RMS untuk setiap parameter
  if (INA_40_connected) {
    solar_voltage = calculateRMS(solar_voltage_samples, NUM_SAMPLES);
    solar_current = calculateRMS(solar_current_samples, NUM_SAMPLES);
  }
  
  if (INA_41_connected) {
    solar_charging_voltage = calculateRMS(solar_charging_voltage_samples, NUM_SAMPLES);
    solar_charging_current = calculateRMS(solar_charging_current_samples, NUM_SAMPLES);
  }
  
  if (INA_44_connected) {
    wind_voltage = calculateRMS(wind_voltage_samples, NUM_SAMPLES);
    wind_current = calculateRMS(wind_current_samples, NUM_SAMPLES);
  }
  
  if (INA_45_connected) {
    wind_charging_voltage = calculateRMS(wind_charging_voltage_samples, NUM_SAMPLES);
    wind_charging_current = calculateRMS(wind_charging_current_samples, NUM_SAMPLES);
  }
  
  // Tampilkan informasi tentang sampel yang diambil
  Serial.print(F("Mengambil "));
  Serial.print(NUM_SAMPLES);
  Serial.println(F(" sampel dan menghitung nilai RMS"));
}

// Fungsi untuk memformat dan mengirim data sebagai API URL
void sendApiUrl() {
  // Format data menjadi API URL
  String apiUrl = API_BASE_URL;
  
  // Tambahkan parameter solar
  apiUrl += "&solar_voltage=" + String(solar_voltage, 2);
  apiUrl += "&solar_current=" + String(solar_current, 2);
  apiUrl += "&solar_charging_voltage=" + String(solar_charging_voltage, 2);
  apiUrl += "&solar_charging_current=" + String(solar_charging_current, 2);
  apiUrl += "&solar_battery_voltage=" + String(solar_battery_voltage, 2);
  
  // Tambahkan parameter wind
  apiUrl += "&wind_voltage=" + String(wind_voltage, 2);
  apiUrl += "&wind_current=" + String(wind_current, 2);
  apiUrl += "&wind_charging_voltage=" + String(wind_charging_voltage, 2);
  apiUrl += "&wind_charging_current=" + String(wind_charging_current, 2);
  apiUrl += "&wind_battery_voltage=" + String(wind_battery_voltage, 2);
  
  // Tambahkan parameter load
  apiUrl += "&load_status=" + load_status;
  
  // Kirim API URL ke hardware Serial
  Serial.println(apiUrl);
  
  // Kirim API URL ke SoftwareSerial
  mySerial.println(apiUrl);
}