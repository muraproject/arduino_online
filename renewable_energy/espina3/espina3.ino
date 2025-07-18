/*
 * Program Monitoring Sensor Tegangan-Arus INA226
 * Untuk ESP32
 * Membaca data dari 4 sensor INA226 pada alamat 0x40, 0x41, 0x44, dan 0x45
 * Melakukan 20 pembacaan dan menghitung nilai RMS
 * Mengirim data langsung ke API via WiFi
 * Nilai minimum sensor dibatasi ke 0
 * Power Management: Solar vs Wind switching logic
 * Auto Restart: 1 menit dengan memory kondisi relay terakhir
 */

#include <Wire.h>
#include <INA226_WE.h>  // Library baru
#include <WiFi.h>
#include <HTTPClient.h>
#include <math.h>  // Untuk fungsi sqrt() dalam perhitungan RMS
#include <EEPROM.h>  // Untuk menyimpan kondisi relay terakhir

// Konfigurasi WiFi - Ganti dengan kredensial WiFi Anda
const char* ssid = "admin";
const char* password = "admin123";

// Definisi pin I2C untuk ESP32
#define SDA_PIN 21
#define SCL_PIN 22

// EEPROM address untuk menyimpan relay state
#define EEPROM_SIZE 1
#define RELAY_STATE_ADDR 0

// Auto restart timer (1 menit = 60000 ms)
const unsigned long AUTO_RESTART_INTERVAL = 60000;
unsigned long programStartTime = 0;

// Inisialisasi 4 sensor INA226 di alamat yang sudah diketahui
INA226_WE INA_40(0x40);  // Sensor 1 pada alamat 0x40 solar voltage
INA226_WE INA_41(0x44);  // Sensor 2 pada alamat 0x44 solar charging voltage / batt
INA226_WE INA_44(0x41);  // Sensor 3 pada alamat 0x41 wind voltage
INA226_WE INA_45(0x45);  // Sensor 4 pada alamat 0x45 wind charging voltage / batt

// Flag untuk menandai sensor yang berhasil diinisialisasi
bool INA_40_connected = false;
bool INA_41_connected = false;
bool INA_44_connected = false;
bool INA_45_connected = false;

// Interval pembacaan sensor
const unsigned long READ_INTERVAL = 5000;  // Membaca sensor setiap 20 detik
unsigned long lastReadTime = 0;

// Power management variables
const unsigned long POWER_COMPARISON_DURATION = 10000;  // 10 detik
unsigned long powerComparisonStartTime = 0;
bool solarGreaterOrEqual = false;
bool windGreater = false;
bool powerComparisonActive = false;
String currentPowerSource = "None";

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

// WiFi status variables
bool wifiConnected = false;
unsigned long lastWiFiCheck = 0;
const unsigned long WIFI_CHECK_INTERVAL = 30000; // Check WiFi every 30 seconds

// Fungsi untuk menyimpan kondisi relay ke EEPROM
void saveRelayState(uint8_t state) {
  EEPROM.write(RELAY_STATE_ADDR, state);
  EEPROM.commit();
}

// Fungsi untuk membaca kondisi relay dari EEPROM
uint8_t loadRelayState() {
  return EEPROM.read(RELAY_STATE_ADDR);
}

// Fungsi untuk mengembalikan kondisi relay terakhir saat startup
void restoreLastRelayState() {
  uint8_t lastState = loadRelayState();
  
  Serial.print("Restoring last relay state: ");
  
  if (lastState == 1) {
    // Solar aktif terakhir kali
    digitalWrite(14, LOW);   // Pin 14 OFF (Wind)
    digitalWrite(27, HIGH);  // Pin 27 ON (Solar)
    currentPowerSource = "Solar";
    load_status = "Solar & Battery";
    Serial.println("Solar");
  } else if (lastState == 2) {
    // Wind aktif terakhir kali
    digitalWrite(27, LOW);   // Pin 27 OFF (Solar)
    digitalWrite(14, HIGH);  // Pin 14 ON (Wind)
    currentPowerSource = "Wind";
    load_status = "Wind & Battery";
    Serial.println("Wind");
  } else {
    // Default atau kondisi awal
    digitalWrite(14, LOW);   // Pin 14 OFF (Wind)
    digitalWrite(27, LOW);   // Pin 27 OFF (Solar)
    currentPowerSource = "None";
    load_status = "Disconnected";
    Serial.println("None (default)");
  }
}

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

// Fungsi untuk power management
void managePowerSource() {
  // Hitung power solar dan wind
  float solarPower = solar_voltage * solar_current;
  float windPower = wind_voltage * wind_current;
  
  Serial.print("Solar Power: "); Serial.print(solarPower, 3); Serial.println(" W");
  Serial.print("Wind Power: "); Serial.print(windPower, 3); Serial.println(" W");
  
  unsigned long currentTime = millis();
  
  // Tentukan kondisi power saat ini
  bool currentSolarGreaterOrEqual = (solarPower >= windPower);
  bool currentWindGreater = (windPower > solarPower);
  
  // Jika kondisi berubah atau baru pertama kali, reset timer
  if ((currentSolarGreaterOrEqual && !solarGreaterOrEqual && !powerComparisonActive) ||
      (currentWindGreater && !windGreater && !powerComparisonActive) ||
      (currentSolarGreaterOrEqual && windGreater) ||
      (currentWindGreater && solarGreaterOrEqual)) {
    
    powerComparisonStartTime = currentTime;
    powerComparisonActive = true;
    solarGreaterOrEqual = currentSolarGreaterOrEqual;
    windGreater = currentWindGreater;
    
    Serial.println("Power comparison timer started/reset");
  }
  
  // Cek apakah sudah mencapai 10 detik
  if (powerComparisonActive && (currentTime - powerComparisonStartTime >= POWER_COMPARISON_DURATION)) {
    if (solarGreaterOrEqual) {
      // Solar >= Wind (termasuk sama) selama 10 detik
      Serial.println("Solar power >= Wind power for 10 seconds - Switching to Solar");
      digitalWrite(14, LOW);   // Pin 14 OFF
      delay(100);              // Small delay
      digitalWrite(27, HIGH);  // Pin 27 ON
      currentPowerSource = "Solar";
      load_status = "Solar & Battery";
      saveRelayState(1);       // Simpan kondisi Solar aktif
    } else if (windGreater) {
      // Wind > Solar selama 10 detik
      Serial.println("Wind power > Solar power for 10 seconds - Switching to Wind");
      digitalWrite(27, LOW);   // Pin 27 OFF
      delay(1000);             // Delay 1000ms
      digitalWrite(14, HIGH);  // Pin 14 ON
      currentPowerSource = "Wind";
      load_status = "Wind & Battery";
      saveRelayState(2);       // Simpan kondisi Wind aktif
    }
    
    // Reset power comparison
    powerComparisonActive = false;
    solarGreaterOrEqual = false;
    windGreater = false;
  }
  
  Serial.print("Current Power Source: "); Serial.println(currentPowerSource);
  Serial.print("Load Status: "); Serial.println(load_status);
}

// Fungsi untuk koneksi WiFi
void connectToWiFi() {
  Serial.println("Menghubungkan ke WiFi...");
  WiFi.begin(ssid, password);
  
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 20) {
    delay(500);
    Serial.print(".");
    attempts++;
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    wifiConnected = true;
    Serial.println();
    Serial.print("WiFi terhubung! IP Address: ");
    Serial.println(WiFi.localIP());
  } else {
    wifiConnected = false;
    Serial.println();
    Serial.println("Gagal terhubung ke WiFi!");
  }
}

// Fungsi untuk memeriksa status WiFi
void checkWiFiConnection() {
  if (WiFi.status() != WL_CONNECTED) {
    wifiConnected = false;
    Serial.println("WiFi terputus! Mencoba menghubungkan kembali...");
    connectToWiFi();
  } else {
    wifiConnected = true;
  }
}

void setup() {
  // Inisialisasi komunikasi serial dan I2C
  Serial.begin(115200);
  Wire.begin(SDA_PIN, SCL_PIN);
  
  // Inisialisasi EEPROM
  EEPROM.begin(EEPROM_SIZE);
  
  // Inisialisasi pin output
  pinMode(14, OUTPUT);
  pinMode(27, OUTPUT);
  
  // Catat waktu mulai program untuk auto restart
  programStartTime = millis();
 
  // Tampilkan header informasi program
  Serial.println(F("\nProgram Monitoring Sensor Tegangan-Arus INA226 - ESP32"));
  Serial.println(F("Library: INA226_WE"));
  Serial.println(F("Auto Restart: 1 menit"));
  delay(1000);
  
  // Kembalikan kondisi relay terakhir
  restoreLastRelayState();
  
  // Inisialisasi WiFi
  connectToWiFi();
  
  // Inisialisasi sensor 1 (alamat 0x40)
  INA_40_connected = INA_40.init();
  if (INA_40_connected) {
    Serial.println(F("Sensor 1 (0x40): Terhubung dan diinisialisasi"));
  } else {
    Serial.println(F("Sensor 1 (0x40): GAGAL terhubung"));
  }
  
  // Inisialisasi sensor 2 (alamat 0x41)
  INA_41_connected = INA_41.init();
  if (INA_41_connected) {
    Serial.println(F("Sensor 2 (0x41): Terhubung dan diinisialisasi"));
  } else {
    Serial.println(F("Sensor 2 (0x41): GAGAL terhubung"));
  }
  
  // Inisialisasi sensor 3 (alamat 0x44)
  INA_44_connected = INA_44.init();
  if (INA_44_connected) {
    Serial.println(F("Sensor 3 (0x44): Terhubung dan diinisialisasi"));
  } else {
    Serial.println(F("Sensor 3 (0x44): GAGAL terhubung"));
  }
  
  // Inisialisasi sensor 4 (alamat 0x45)
  INA_45_connected = INA_45.init();
  if (INA_45_connected) {
    Serial.println(F("Sensor 4 (0x45): Terhubung dan diinisialisasi"));
  } else {
    Serial.println(F("Sensor 4 (0x45): GAGAL terhubung"));
  }
  
  // Periksa apakah ada sensor yang terhubung
  if (!INA_40_connected && !INA_41_connected && !INA_44_connected && !INA_45_connected) {
    Serial.println(F("ERROR: Tidak ada sensor yang terhubung!"));
    Serial.println(F("Periksa koneksi dan restart ESP32."));
    while (1); // Berhenti jika tidak ada sensor yang terhubung
  }
  
  // Cetak informasi mode
  Serial.println(F("\nMode: Kirim data langsung ke API via WiFi dengan Power Management"));
  Serial.println(F("====================================="));
}

void loop() {
  unsigned long currentMillis = millis();
  
  // Auto restart setelah 1 menit
  if (currentMillis - programStartTime >= AUTO_RESTART_INTERVAL) {
    Serial.println("Auto restart setelah 1 menit...");
    delay(1000);
    ESP.restart();
  }
  
  // Periksa koneksi WiFi secara berkala
  if (currentMillis - lastWiFiCheck >= WIFI_CHECK_INTERVAL) {
    lastWiFiCheck = currentMillis;
    checkWiFiConnection();
  }
  
  // Baca dan kirim data sensor setiap READ_INTERVAL
  if (currentMillis - lastReadTime >= READ_INTERVAL) {
    lastReadTime = currentMillis;
    
    // Baca data dari semua sensor
    readSensorData();
    
    // Kelola sumber daya berdasarkan power comparison
    managePowerSource();
    
    // Kirim data ke API jika WiFi terhubung
    if (wifiConnected) {
      sendDataToAPI();
    } else {
      Serial.println("WiFi tidak terhubung. Data tidak dikirim.");
      // Tampilkan data lokal sebagai backup
      displayLocalData();
    }
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
  
  Serial.print(F("Mengambil "));
  Serial.print(NUM_SAMPLES);
  Serial.print(F(" sampel"));
  
  // Ambil NUM_SAMPLES sampel dari setiap sensor
  for (int i = 0; i < NUM_SAMPLES; i++) {
    // Baca data dari sensor 1 (0x40) - Panel surya
    if (INA_40_connected) {
      INA_40.readAndClearFlags();
      solar_voltage_samples[i] = ensurePositive(INA_40.getBusVoltage_V());
      solar_current_samples[i] = ensurePositive(INA_40.getCurrent_mA() / 1000.0); // Convert mA to A
    }
    
    // Baca data dari sensor 2 (0x41) - Charging solar
    if (INA_41_connected) {
      INA_41.readAndClearFlags();
      solar_charging_voltage_samples[i] = ensurePositive(INA_41.getBusVoltage_V());
      solar_charging_current_samples[i] = ensurePositive(INA_41.getCurrent_mA() / 1000.0); // Convert mA to A
    }
    
    // Baca data dari sensor 3 (0x44) - Turbin angin
    if (INA_44_connected) {
      INA_44.readAndClearFlags();
      wind_voltage_samples[i] = ensurePositive(INA_44.getBusVoltage_V());
      wind_current_samples[i] = ensurePositive(INA_44.getCurrent_mA() / 1000.0); // Convert mA to A
    }
    
    // Baca data dari sensor 4 (0x45) - Charging angin
    if (INA_45_connected) {
      INA_45.readAndClearFlags();
      wind_charging_voltage_samples[i] = ensurePositive(INA_45.getBusVoltage_V());
      wind_charging_current_samples[i] = ensurePositive(INA_45.getCurrent_mA() / 1000.0); // Convert mA to A
    }
    
    // Progress indicator
    if (i % 5 == 0) Serial.print(".");
    
    // Delay kecil antar pembacaan sampel
    delay(10);
  }
  
  Serial.println(" selesai");
  
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
}

// Fungsi untuk URL encoding
String urlEncode(String str) {
  String encodedString = "";
  char c;
  char code0;
  char code1;
  for (int i = 0; i < str.length(); i++) {
    c = str.charAt(i);
    if (c == ' ') {
      encodedString += "%20";
    } else if (c == '&') {
      encodedString += "%26";
    } else if (isalnum(c)) {
      encodedString += c;
    } else {
      code1 = (c & 0xf) + '0';
      if ((c & 0xf) > 9) {
        code1 = (c & 0xf) - 10 + 'A';
      }
      c = (c >> 4) & 0xf;
      code0 = c + '0';
      if (c > 9) {
        code0 = c - 10 + 'A';
      }
      encodedString += '%';
      encodedString += code0;
      encodedString += code1;
    }
  }
  return encodedString;
}

// Fungsi untuk mengirim data ke API via HTTP
void sendDataToAPI() {
  HTTPClient http;
  
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
  
  // Tambahkan parameter load dengan URL encoding untuk mengatasi karakter & dan spasi
  apiUrl += "&load_status=" + urlEncode(load_status);
  
  // Tampilkan URL yang akan dikirim
  Serial.println("Mengirim data ke API:");
  Serial.println(apiUrl);
  
  // Kirim HTTP GET request
  http.begin(apiUrl);
  int httpResponseCode = http.GET();
  
  if (httpResponseCode > 0) {
    String response = http.getString();
    Serial.print("Response code: ");
    Serial.println(httpResponseCode);
    Serial.print("Response: ");
    Serial.println(response);
  } else {
    Serial.print("Error sending data. HTTP response code: ");
    Serial.println(httpResponseCode);
  }
  
  http.end();
  Serial.println("=====================================");
}

// Fungsi untuk menampilkan data lokal ketika WiFi tidak tersedia
void displayLocalData() {
  float solarPower = solar_voltage * solar_current;
  float windPower = wind_voltage * wind_current;
  
  Serial.println("=== Data Sensor (Mode Offline) ===");
  Serial.println("Solar Panel:");
  Serial.print("  Voltage: "); Serial.print(solar_voltage, 2); Serial.println(" V");
  Serial.print("  Current: "); Serial.print(solar_current, 2); Serial.println(" A");
  Serial.print("  Power: "); Serial.print(solarPower, 3); Serial.println(" W");
  Serial.print("  Charging Voltage: "); Serial.print(solar_charging_voltage, 2); Serial.println(" V");
  Serial.print("  Charging Current: "); Serial.print(solar_charging_current, 2); Serial.println(" A");
  Serial.print("  Battery Voltage: "); Serial.print(solar_battery_voltage, 2); Serial.println(" V");
  
  Serial.println("Wind Turbine:");
  Serial.print("  Voltage: "); Serial.print(wind_voltage, 2); Serial.println(" V");
  Serial.print("  Current: "); Serial.print(wind_current, 2); Serial.println(" A");
  Serial.print("  Power: "); Serial.print(windPower, 3); Serial.println(" W");
  Serial.print("  Charging Voltage: "); Serial.print(wind_charging_voltage, 2); Serial.println(" V");
  Serial.print("  Charging Current: "); Serial.print(wind_charging_current, 2); Serial.println(" A");
  Serial.print("  Battery Voltage: "); Serial.print(wind_battery_voltage, 2); Serial.println(" V");
  
  Serial.print("Load Status: "); Serial.println(load_status);
  Serial.print("Active Power Source: "); Serial.println(currentPowerSource);
  Serial.println("=====================================");
}