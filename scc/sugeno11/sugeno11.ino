#include <Arduino.h>
#include <ModbusMaster.h>
#include <Wire.h>               // Library untuk I2C
#include <LiquidCrystal_I2C.h>  // Library untuk LCD I2C
#include <WiFi.h>               // Library untuk WiFi
#include <ESPAsyncWebServer.h>  // Library untuk web server
#include <EEPROM.h>             // Library untuk EEPROM

// Definisi pin untuk komunikasi RS485
#define RX_PIN 17
#define TX_PIN 16
#define SERIAL_COMMUNICATION Serial2

// Definisi pin untuk analog input
#define ANALOG_PIN 18
// Pin DO_PIN tidak digunakan lagi karena sensor DO di ESP32 lain

// Pin untuk relay kontrol
#define RELAY_PIN 4  // Pin untuk relay

// Pin untuk LCD I2C
#define SDA_PIN 8  // Pin SDA pada ESP32 S3
#define SCL_PIN 9  // Pin SCL pada ESP32 S3

// Alamat I2C untuk ESP32 slave yang menangani sensor DO
#define DO_SENSOR_I2C_ADDRESS 0x08

// Inisialisasi LCD (alamat I2C 0x27, 20 kolom, 4 baris)
// Jika LCD tidak terdeteksi, coba alamat 0x3F atau alamat lainnya
LiquidCrystal_I2C lcd(0x27, 20, 4);

// Threshold untuk deteksi sumber
#define SOURCE_THRESHOLD 1000  // Nilai ini bisa disesuaikan berdasarkan pembacaan sebenarnya

// Variabel untuk status sumber
bool sourceDetected = false;

// Alamat default untuk PZEM-017
#define PZEM_ADDR 0x01

// Instance untuk ModbusMaster
ModbusMaster node;

// Definisi pin dan parameter PWM
const int pwmPin = 5;        // Pin output PWM (GPIO5)
const int pwmChannel = 0;    // Channel PWM (0-15)
const int pwmFreq = 8500;    // Frekuensi PWM 8.5 kHz
const int pwmResolution = 12; // Resolusi PWM 12-bit (0-4095)
const int pwmMaxValue = 4000; // Nilai maksimum PWM dibatasi 3000

// Variabel untuk menyimpan nilai PWM saat ini
int pwmValue = 0;
float trigger_do= 3.5;// ini yang di ubah

// Pin untuk memicu masuk mode AP (tombol fisik)
#define AP_TRIGGER_PIN 14  // Ganti dengan pin yang tersedia

// SCC parameters (akan diupdate dari EEPROM)
float MAX_CHARGING_CURRENT = 4.0;   // Target arus charging maksimal (A)
float VOLTAGE_LIMIT = 13.6;         // Batas tegangan charging (V)
float CURRENT_THRESHOLD = 0.1;      // Threshold arus minimal untuk mendeteksi charging
float DO_THRESHOLD = 1.0;           // Threshold DO dalam mg/L
float BATTERY_LOW_THRESHOLD = 11.9; // Batas bawah tegangan baterai
float BATTERY_RECOVER_THRESHOLD = 12.3; // Batas atas recovery tegangan baterai

// EEPROM size dan alamat
#define EEPROM_SIZE 512
#define EEPROM_INITIALIZED_MARKER 0xAA  // Marker untuk menandai EEPROM sudah diinisialisasi

// Struktur untuk parameter yang disimpan di EEPROM
struct SystemParams {
  byte initialized;              // Marker (0xAA jika sudah diinisialisasi)
  float maxChargingCurrent;      // Target arus charging maksimal
  float voltageLimit;            // Batas tegangan charging
  float currentThreshold;        // Threshold arus minimal
  float doThreshold;             // Threshold DO
  float batteryLowThreshold;     // Batas bawah tegangan baterai
  float batteryRecoverThreshold; // Batas atas recovery tegangan baterai
};

// Variabel untuk sistem charging
float targetCurrent = MAX_CHARGING_CURRENT;  // Target arus charging awal
bool isCharging = false;                     // Status charging aktif
enum ChargingMode {
  BULK,     // Mode arus konstan
  ABSORPTION // Mode tegangan konstan
};
ChargingMode chargingMode = BULK;

// Struktur untuk menyimpan data dari PZEM-017
struct PZEM_DATA {
  float voltage;
  float current;
  float power;
};

// Variabel untuk sensor DO yang dibaca via I2C
float DO_mgL = 0.0;
bool DO_sensor_available = false;

// Mutex untuk akses LCD
SemaphoreHandle_t lcdMutex;

// Struktur untuk menyimpan data yang akan ditampilkan di LCD
struct DisplayData {
  PZEM_DATA pzem;
  bool sourceDetected;
  ChargingMode mode;
  float doMgL;
  int pwmValue;
  bool relayStatus; // Tambah status relay untuk tampilan LCD
};

// Instance global dari DisplayData untuk diakses oleh kedua core
DisplayData displayData;

// Task handle untuk sensor DO dan web server
TaskHandle_t DoSensorTask;
TaskHandle_t WebServerTask;

// Variabel untuk kontrol relay
bool relayStatus = false;      // Status relay (false = OFF, true = ON)
bool batteryLowState = false;  // Status baterai low

// WiFi credentials untuk mode AP
const char* apSSID = "SolarCharger";
const char* apPassword = "12345678";

// Web Server
AsyncWebServer server(80);

// Mode AP flag
bool isAPMode = false;

// Prototype fungsi
void saveParamsToEEPROM();
void loadParamsFromEEPROM();
void setDefaultParams();
void startAPMode();
void setupWebServer();
void controlRelay(float doValue, float batteryVoltage);
float readDOFromI2C();

// HTML untuk halaman konfigurasi
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML>
<html>
<head>
  <title>Solar Charge Controller Config</title>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <style>
    html {font-family: Arial; display: inline-block; text-align: center;}
    h2 {font-size: 2.0rem; margin: 20px 0;}
    .container {width: 300px; margin: 0 auto; padding: 20px; border: 1px solid #ccc; border-radius: 10px;}
    label {display: block; margin: 10px 0 5px; text-align: left;}
    input {width: 100%; padding: 5px; box-sizing: border-box;}
    .btn {background-color: #4CAF50; border: none; color: white; padding: 10px 20px;
           text-decoration: none; font-size: 16px; margin: 20px 0; cursor: pointer; border-radius: 5px;}
    .restart-btn {background-color: #f44336;}
    .footer {margin-top: 20px; font-size: 0.8rem; color: #888;}
  </style>
</head>
<body>
  <h2>Solar Charge Controller</h2>
  <div class="container">
    <form action="/save" method="POST">
      <label for="max_current">Max Charging Current (A):</label>
      <input type="number" id="max_current" name="max_current" min="0.1" max="10" step="0.1" value="%MAX_CURRENT%">
      
      <label for="voltage_limit">Voltage Limit (V):</label>
      <input type="number" id="voltage_limit" name="voltage_limit" min="10" max="15" step="0.1" value="%VOLTAGE_LIMIT%">
      
      <label for="current_threshold">Current Threshold (A):</label>
      <input type="number" id="current_threshold" name="current_threshold" min="0.01" max="1" step="0.01" value="%CURRENT_THRESHOLD%">
      
      <label for="do_threshold">DO Threshold (mg/L):</label>
      <input type="number" id="do_threshold" name="do_threshold" min="0.1" max="10" step="0.1" value="%DO_THRESHOLD%">
      
      <label for="batt_low">Battery Low Threshold (V):</label>
      <input type="number" id="batt_low" name="batt_low" min="10" max="12.5" step="0.1" value="%BATT_LOW%">
      
      <label for="batt_recover">Battery Recover Threshold (V):</label>
      <input type="number" id="batt_recover" name="batt_recover" min="10" max="13" step="0.1" value="%BATT_RECOVER%">
      
      <input type="submit" value="Save" class="btn">
    </form>
    <a href="/restart" class="btn restart-btn">Restart System</a>
  </div>
  <div class="footer">
    Solar Charge Controller with DO Sensor v1.0
  </div>
</body>
</html>
)rawliteral";

// Fungsi untuk mendapatkan parameter sebagai string
String getParam(AsyncWebServerRequest *request, String paramName) {
  if (request->hasParam(paramName, true)) {
    return request->getParam(paramName, true)->value();
  }
  return "";
}

// Fungsi untuk membaca data DO dari ESP32 slave via I2C
float readDOFromI2C() {
  float doValue = 0.0;
  
  // Request 4 bytes (float) dari slave
  Wire.requestFrom(DO_SENSOR_I2C_ADDRESS, 4);
  
  if (Wire.available() >= 4) {
    // Baca 4 bytes dan konversi ke float
    byte bytes[4];
    for (int i = 0; i < 4; i++) {
      bytes[i] = Wire.read();
    }
    
    // Konversi bytes ke float
    memcpy(&doValue, bytes, 4);
    
    DO_sensor_available = true;
    
    // Validasi data (pastikan dalam range yang masuk akal)
    if (doValue < 0 || doValue > 20) {
      doValue = 0.0;
      DO_sensor_available = false;
    }
  } else {
    DO_sensor_available = false;
    doValue = 0.0;
  }
  
  return doValue;
}

// Fungsi untuk kontrol relay berdasarkan DO dan tegangan baterai
void controlRelay(float doValue, float batteryVoltage) {
  // Cek jika baterai dalam keadaan low
  if (batteryVoltage < BATTERY_LOW_THRESHOLD) {
    batteryLowState = true;
    relayStatus = false; // Matikan relay jika baterai low
    Serial.println("Relay OFF: Baterai Low (<" + String(BATTERY_LOW_THRESHOLD) + "V)");
  } 
  // Cek jika baterai sudah recovery dari low state
  else if (batteryLowState && batteryVoltage >= BATTERY_RECOVER_THRESHOLD) {
    batteryLowState = false;
    Serial.println("Baterai pulih (>" + String(BATTERY_RECOVER_THRESHOLD) + "V)");
  }
  
  // Jika baterai tidak dalam status low, cek nilai DO untuk kontrol relay
  if (!batteryLowState) {
    if (doValue < trigger_do) {
      // DO rendah, nyalakan relay jika belum menyala
      if (!relayStatus) {
        relayStatus = true;
        Serial.println("Relay ON: DO rendah (<" + String(trigger_do) + " mg/L)");
      }
    } else {
      // DO normal, matikan relay jika belum mati
      if (relayStatus) {
        relayStatus = false;
        Serial.println("Relay OFF: DO normal (>" + String(trigger_do) + " mg/L)");
      }
    }
  }
  
  // Terapkan status relay ke pin
  digitalWrite(RELAY_PIN, relayStatus ? HIGH : LOW);
  
  // Update status relay di displayData
  displayData.relayStatus = relayStatus;
}

// Function untuk meng-update LCD (akan dipanggil dari Core 2)
void updateLCD(DisplayData data) {
  // Ambil semaphore untuk akses ke LCD
  if (xSemaphoreTake(lcdMutex, (TickType_t)10) == pdTRUE) {
    // Baris 1: Status sumber dan mode charging/AP
    lcd.setCursor(0, 0);
    if (isAPMode) {
      lcd.print("AP MODE: ");
      lcd.print(apSSID);
      lcd.print("     ");
    } else if (!data.sourceDetected) {
      lcd.print("Sumber: TIDAK ADA    ");
    } else {
      lcd.print("Sumber: ADA ");
      
      // Tampilkan mode charging
      if (data.mode == BULK) {
        lcd.print("BULK     ");
      } else {
        lcd.print("ABSRP    ");
      }
    }
    
    // Baris 2: Tegangan
    lcd.setCursor(0, 1);
    if (isAPMode) {
      lcd.print("IP: ");
      lcd.print(WiFi.softAPIP().toString());
      lcd.print("       ");
    } else {
      lcd.print("V:");
      char voltageStr[10];
      sprintf(voltageStr, "%.2f", data.pzem.voltage);
      lcd.print(voltageStr);
      lcd.print("V            ");
    }
    
    // Baris 3: Arus dan DO
    lcd.setCursor(0, 2);
    if (isAPMode) {
      lcd.print("Connect to config    ");
    } else {
      lcd.print("I:");
      char currentStr[10];
      sprintf(currentStr, "%.2f", data.pzem.current);
      lcd.print(currentStr);
      lcd.print("A DO:");
      if (DO_sensor_available) {
        char doStr[10];
        sprintf(doStr, "%.1f", data.doMgL);
        lcd.print(doStr);
        lcd.print("mg/L ");
      } else {
        lcd.print("N/A   ");
      }
    }
    
    // Baris 4: Daya, PWM, dan status relay
    lcd.setCursor(0, 3);
    if (isAPMode) {
      lcd.print("Password: ");
      lcd.print(apPassword);
      lcd.print("      ");
    } else {
      lcd.print("P:");
      char powerStr[10];
      sprintf(powerStr, "%.1f", data.pzem.power);
      lcd.print(powerStr);
      lcd.print("W PWM:");
      int pwmPercent = (int)((float)data.pwmValue / pwmMaxValue * 100.0);
      char pwmStr[5];
      sprintf(pwmStr, "%d", pwmPercent);
      lcd.print(pwmStr);
      lcd.print("% R:");
      lcd.print(data.relayStatus ? "ON " : "OFF");
    }
    
    // Lepaskan semaphore
    xSemaphoreGive(lcdMutex);
  }
}

// Task untuk sensor DO dan update LCD yang berjalan di Core 2
void DOSensorTaskCode(void * parameter) {
  Serial.println("DO Sensor I2C Task initialized on Core 2");
  
  // Timestamp untuk memantau interval update sensor DO
  unsigned long lastDoUpdate = 0;
  const unsigned long DO_UPDATE_INTERVAL = 1000; // Update DO setiap 1 detik
  
  // Timestamp untuk memantau interval update LCD
  unsigned long lastLcdUpdate = 0;
  const unsigned long LCD_UPDATE_INTERVAL = 500; // Update LCD setiap 500ms
  
  for(;;) {
    unsigned long currentMillis = millis();
    
    // Cek jika sedang dalam mode AP
    if (!isAPMode) {
      // Update sensor DO setiap DO_UPDATE_INTERVAL
      if (currentMillis - lastDoUpdate >= DO_UPDATE_INTERVAL) {
        lastDoUpdate = currentMillis;
        
        // Baca data DO dari ESP32 slave via I2C
        DO_mgL = readDOFromI2C();
        
        // Update nilai DO dalam struct displayData yang diakses global
        displayData.doMgL = DO_mgL;
        
        // Kontrol relay berdasarkan data terbaru
        controlRelay(DO_mgL, displayData.pzem.voltage);
        
        // Log hasil ke serial (untuk debugging)
        if (DO_sensor_available) {
          Serial.print("DO Sensor (I2C): ");
          Serial.print(DO_mgL, 2);
          Serial.println(" mg/L");
        } else {
          Serial.println("DO Sensor (I2C): Tidak tersedia");
        }
      }
    }
    
    // Update LCD setiap LCD_UPDATE_INTERVAL (selalu update LCD meskipun dalam mode AP)
    if (currentMillis - lastLcdUpdate >= LCD_UPDATE_INTERVAL) {
      lastLcdUpdate = currentMillis;
      
      // Update LCD dengan data terbaru
      updateLCD(displayData);
    }
    
    // Delay kecil untuk memperlancar multitasking
    delay(10);
  }
}

// Simpan parameter ke EEPROM
void saveParamsToEEPROM() {
  SystemParams params;
  
  // Isi struktur dengan nilai parameter saat ini
  params.initialized = EEPROM_INITIALIZED_MARKER;
  params.maxChargingCurrent = MAX_CHARGING_CURRENT;
  params.voltageLimit = VOLTAGE_LIMIT;
  params.currentThreshold = CURRENT_THRESHOLD;
  params.doThreshold = DO_THRESHOLD;
  params.batteryLowThreshold = BATTERY_LOW_THRESHOLD;
  params.batteryRecoverThreshold = BATTERY_RECOVER_THRESHOLD;
  
  // Tulis ke EEPROM
  EEPROM.put(0, params);
  EEPROM.commit();
  
  Serial.println("Parameter saved to EEPROM");
}

// Muat parameter dari EEPROM
void loadParamsFromEEPROM() {
  SystemParams params;
  
  // Baca struktur dari EEPROM
  EEPROM.get(0, params);
  
  // Periksa apakah EEPROM sudah diinisialisasi
  if (params.initialized == EEPROM_INITIALIZED_MARKER) {
    MAX_CHARGING_CURRENT = params.maxChargingCurrent;
    VOLTAGE_LIMIT = params.voltageLimit;
    CURRENT_THRESHOLD = params.currentThreshold;
    DO_THRESHOLD = params.doThreshold;
    BATTERY_LOW_THRESHOLD = params.batteryLowThreshold;
    BATTERY_RECOVER_THRESHOLD = params.batteryRecoverThreshold;
    
    Serial.println("Parameter loaded from EEPROM:");
    Serial.println("MAX_CHARGING_CURRENT: " + String(MAX_CHARGING_CURRENT));
    Serial.println("VOLTAGE_LIMIT: " + String(VOLTAGE_LIMIT));
    Serial.println("CURRENT_THRESHOLD: " + String(CURRENT_THRESHOLD));
    Serial.println("DO_THRESHOLD: " + String(DO_THRESHOLD));
    Serial.println("BATTERY_LOW_THRESHOLD: " + String(BATTERY_LOW_THRESHOLD));
    Serial.println("BATTERY_RECOVER_THRESHOLD: " + String(BATTERY_RECOVER_THRESHOLD));
  } else {
    Serial.println("EEPROM not initialized, using default values");
    setDefaultParams();
    saveParamsToEEPROM();
  }
}

// Set parameter default
void setDefaultParams() {
  MAX_CHARGING_CURRENT = 4.0;
  VOLTAGE_LIMIT = 13.6;
  CURRENT_THRESHOLD = 0.1;
  DO_THRESHOLD = 1.0;
  BATTERY_LOW_THRESHOLD = 11.9;
  BATTERY_RECOVER_THRESHOLD = 12.3;
  
  Serial.println("Default parameters set");
}

// Mulai mode AP
void startAPMode() {
  isAPMode = true;
  
  // Matikan relay
  digitalWrite(RELAY_PIN, LOW);
  relayStatus = false;
  
  // Set PWM ke 0
  pwmValue = 0;
  ledcWrite(pwmChannel, pwmValue);
  
  // Mulai WiFi AP
  WiFi.softAP(apSSID, apPassword);
  
  Serial.println("AP Mode started");
  Serial.print("SSID: ");
  Serial.println(apSSID);
  Serial.print("Password: ");
  Serial.println(apPassword);
  Serial.print("IP address: ");
  Serial.println(WiFi.softAPIP());
  
  // Setup web server
  setupWebServer();
}

// Process HTML template dengan nilai parameter saat ini
String processHtmlTemplate() {
  String html = String(index_html);
  html.replace("%MAX_CURRENT%", String(MAX_CHARGING_CURRENT));
  html.replace("%VOLTAGE_LIMIT%", String(VOLTAGE_LIMIT));
  html.replace("%CURRENT_THRESHOLD%", String(CURRENT_THRESHOLD));
  html.replace("%DO_THRESHOLD%", String(DO_THRESHOLD));
  html.replace("%BATT_LOW%", String(BATTERY_LOW_THRESHOLD));
  html.replace("%BATT_RECOVER%", String(BATTERY_RECOVER_THRESHOLD));
  return html;
}

// Setup web server
void setupWebServer() {
  // Route untuk halaman utama
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(200, "text/html", processHtmlTemplate());
  });
  
  // Route untuk menyimpan parameter
  server.on("/save", HTTP_POST, [](AsyncWebServerRequest *request) {
    // Ambil nilai parameter dari form
    if (request->hasParam("max_current", true)) {
      MAX_CHARGING_CURRENT = getParam(request, "max_current").toFloat();
    }
    
    if (request->hasParam("voltage_limit", true)) {
      VOLTAGE_LIMIT = getParam(request, "voltage_limit").toFloat();
    }
    
    if (request->hasParam("current_threshold", true)) {
      CURRENT_THRESHOLD = getParam(request, "current_threshold").toFloat();
    }
    
    if (request->hasParam("do_threshold", true)) {
      DO_THRESHOLD = getParam(request, "do_threshold").toFloat();
    }
    
    if (request->hasParam("batt_low", true)) {
      BATTERY_LOW_THRESHOLD = getParam(request, "batt_low").toFloat();
    }
    
    if (request->hasParam("batt_recover", true)) {
      BATTERY_RECOVER_THRESHOLD = getParam(request, "batt_recover").toFloat();
    }
    
    // Simpan parameter ke EEPROM
    saveParamsToEEPROM();
    
    // Redirect ke halaman utama
    request->redirect("/");
  });
  
  // Route untuk restart sistem
  server.on("/restart", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(200, "text/plain", "Restarting system...");
    delay(1000);
    ESP.restart();
  });
  
  // Mulai server
  server.begin();
}

// Fungsi untuk membaca data dari PZEM-017
PZEM_DATA readPZEM() {
  PZEM_DATA data;
  
  // Inisialisasi data dengan nilai default
  data.voltage = 0.0;
  data.current = 0.0;
  data.power = 0.0;
  
  // Membaca semua register data dalam satu permintaan (alamat 0x0000, baca 3 register)
  if (node.readInputRegisters(0x0000, 3) == node.ku8MBSuccess) {
    // Register 0x0000: Tegangan
    data.voltage = node.getResponseBuffer(0) * 0.01; // Konversi ke volt
    
    // Register 0x0001: Arus
    data.current = node.getResponseBuffer(1) * 0.01; // Konversi ke ampere
    
    // // Kompensasi arus: tambah 0.7A jika pembacaan melebihi 1A
    // if (data.current > 1.0) {
    //   data.current += 0.2;
    // }
    
    // Register 0x0002: Daya
    data.power = node.getResponseBuffer(2) * 0.1; // Konversi ke watt
  } else {
    // Silent fail - tidak menampilkan pesan error setiap kali
  }
  
  return data;
}

// Fungsi untuk menghitung daya secara manual
float calculatePower(float voltage, float current) {
  return voltage * current;
}

// Fungsi untuk membaca nilai analog dan menentukan status sumber
bool checkSourceDetection() {
  int analogValue = analogRead(ANALOG_PIN);
  Serial.println(analogValue);
  
  // Periksa apakah nilai analog di atas threshold
  bool isSourceDetected = (analogValue > SOURCE_THRESHOLD);
  
  // Jika status berubah, kirim notifikasi
  if (isSourceDetected != sourceDetected) {
    sourceDetected = isSourceDetected;
    
    String statusMessage = "///////////////////Sumber: " + String(sourceDetected ? "Terdeteksi" : "Tidak Terdeteksi");
    statusMessage += " (Nilai Analog: " + String(analogValue) + ")";
    
    Serial.println(statusMessage);
    
    // Jika sumber baru terdeteksi, kirim informasi tambahan
    if (sourceDetected) {
      String infoMsg = "Memulai charging dengan target arus " + String(targetCurrent, 2) + " A dan batas tegangan " + String(VOLTAGE_LIMIT, 2) + " V";
      Serial.println(infoMsg);
    } else {
      // Jika sumber hilang, reset PWM
      pwmValue = 0;
      ledcWrite(pwmChannel, pwmValue);
      
      String infoMsg = "Charging dihentikan karena sumber tidak terdeteksi";
      Serial.println(infoMsg);
    }
    
    // Update displayData dengan status sumber yang baru
    displayData.sourceDetected = sourceDetected;
  }
  
  return sourceDetected;
}

// Fungsi untuk logika fuzzy Sugeno berdasarkan error arus atau tegangan
int fuzzyPWMControl(float currentError, float voltageError) {
  // Jika sumber tidak terdeteksi, langsung return 0
  if (!sourceDetected) {
    return 0;
  }
  
  // Jika tegangan diatas batas, gunakan error tegangan
  if (voltageError < 0) {
    // Kondisi tegangan melebihi batas
    if (voltageError < -0.1) {
      // Lebih agresif menurunkan PWM jika tegangan terlalu tinggi
      return max(0, pwmValue - 5);
    } else {
      // Pelan-pelan turunkan PWM untuk menjaga tegangan
      return max(0, pwmValue - 1);
    }
  }
  
  // Jika tegangan masih dibawah batas, gunakan error arus
  // Fungsi keanggotaan fuzzy sederhana untuk error
  // NB: Negative Big, NS: Negative Small, ZE: Zero, PS_val: Positive Small, PB_val: Positive Big
  float NB = 0.0, NS = 0.0, ZE = 0.0, PS_val = 0.0, PB_val = 0.0;
  
  // Fuzzifikasi error dengan rentang yang lebih sensitif
  if (currentError <= -0.5) {
    NB = 1.0;  // Error negatif besar - kurangi PWM
  } else if (currentError > -0.5 && currentError < -0.1) {
    NB = (-0.1 - currentError) / 0.4;
    NS = (currentError + 0.5) / 0.4;
  } else if (currentError >= -0.1 && currentError < 0) {
    NS = (-currentError) / 0.1;
    ZE = (currentError + 0.1) / 0.1;
  } else if (currentError >= 0 && currentError < 0.1) {
    ZE = (0.1 - currentError) / 0.1;
    PS_val = currentError / 0.1;
  } else if (currentError >= 0.1 && currentError < 0.5) {
    PS_val = (0.5 - currentError) / 0.4;
    PB_val = (currentError - 0.1) / 0.4;
  } else {  // currentError >= 0.5
    PB_val = 1.0;  // Error positif besar - naikkan PWM agresif
  }
  
  // Output singleton untuk Sugeno - lebih agresif naik daripada turun
  int NBOutput = -10;   // Decrease PWM by 10
  int NSOutput = -1;    // Decrease PWM by 1
  int ZEOutput = 0;     // No change
  int PSOutput = 1;     // Increase PWM by 1
  int PBOutput = 10;    // Increase PWM by 10
  
  // Defuzzifikasi (weighted average - Sugeno)
  float totalWeight = NB + NS + ZE + PS_val + PB_val;
  
  if (totalWeight > 0) {
    int deltaOutput = (int)((NB * NBOutput + NS * NSOutput + ZE * ZEOutput + PS_val * PSOutput + PB_val * PBOutput) / totalWeight);
    
    // Jika masih di bawah target dan error positif, minimal naik 1
    if (currentError > 0.05 && deltaOutput == 0) {
      deltaOutput = 1;  // Pastikan selalu naik jika target belum tercapai
    }
    
    // Hitung nilai PWM baru
    int newPWM = pwmValue + deltaOutput;
    
    // Batasi dalam rentang yang valid
    return constrain(newPWM, 0, pwmMaxValue);
  }
  
  return pwmValue; // Jika tidak ada aturan yang terpicu, pertahankan nilai lama
}

// Fungsi untuk memeriksa masuk mode AP
void checkAPMode() {
  // Cek jika pin trigger AP ditekan
  if (digitalRead(AP_TRIGGER_PIN) == LOW) {
    // Tunggu debounce
    delay(50);
    
    // Jika masih ditekan, masuk mode AP
    if (digitalRead(AP_TRIGGER_PIN) == LOW) {
      // Tunggu sampai tombol dilepas
      while (digitalRead(AP_TRIGGER_PIN) == LOW) {
        delay(10);
      }
      
      // Masuk mode AP
      startAPMode();
    }
  }
}

void setup() {
  // Inisialisasi Serial untuk debugging
  Serial.begin(115200);
  Serial.println("ESP32-S3 - Solar Charge Controller (SCC) with PZEM-017, LCD, DO Sensor (I2C), and Relay Control");
  
  // Inisialisasi EEPROM
  EEPROM.begin(EEPROM_SIZE);
  
  // Load parameter dari EEPROM
  loadParamsFromEEPROM();
  
  // Inisialisasi pin untuk trigger mode AP
  pinMode(AP_TRIGGER_PIN, INPUT_PULLUP);
  
  // Inisialisasi pin relay
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, LOW);  // Relay start in OFF state
  
  // Inisialisasi I2C untuk LCD dan DO sensor
  Wire.begin(SDA_PIN, SCL_PIN);
  
  // Inisialisasi LCD
  lcd.init();
  lcd.backlight();
  lcd.clear();
  
  // Buat mutex untuk LCD
  lcdMutex = xSemaphoreCreateMutex();
  
  // Tampilkan pesan startup pada LCD
  lcd.setCursor(0, 0);
  lcd.print("Solar Charge Control");
  lcd.setCursor(0, 1);
  lcd.print("Starting...");
  lcd.setCursor(0, 2);
  lcd.print("DO via I2C: 0x");
  lcd.print(DO_SENSOR_I2C_ADDRESS, HEX);
  lcd.setCursor(0, 3);
  lcd.print("Batt Low: ");
  lcd.print(BATTERY_LOW_THRESHOLD);
  lcd.print("V");
  
  // Konfigurasi pin analog
  pinMode(ANALOG_PIN, INPUT);
  
  // Inisialisasi Serial2 untuk komunikasi RS485
  SERIAL_COMMUNICATION.begin(9600, SERIAL_8N1, RX_PIN, TX_PIN);
  
  // Inisialisasi ModbusMaster
  node.begin(PZEM_ADDR, SERIAL_COMMUNICATION);
  
  // Konfigurasi PWM
  ledcSetup(pwmChannel, pwmFreq, pwmResolution);
  ledcAttachPin(pwmPin, pwmChannel);
  
  // Set target arus awal ke MAX_CHARGING_CURRENT
  targetCurrent = MAX_CHARGING_CURRENT;
  
  // Reset status battery low state saat restart
  batteryLowState = false;
  
  // Inisialisasi displayData dengan nilai default
  displayData.pzem.voltage = 0.0;
  displayData.pzem.current = 0.0;
  displayData.pzem.power = 0.0;
  displayData.sourceDetected = false;
  displayData.mode = BULK;
  displayData.doMgL = 0.0;
  displayData.pwmValue = 0;
  displayData.relayStatus = false;
  
  // Jalankan task DO sensor di Core 2
  xTaskCreatePinnedToCore(
    DOSensorTaskCode,   // Function yang akan dijalankan
    "DOSensorTask",     // Nama task
    10000,              // Stack size
    NULL,               // Task parameter
    1,                  // Priority
    &DoSensorTask,      // Task handle
    1                   // Jalankan di Core 1 (core kedua)
  );
  
  // Cetak panduan penggunaan ke Serial
  String helpMessage = "Solar Charge Controller initialized\n";
  helpMessage += "Max Charging Current: " + String(MAX_CHARGING_CURRENT, 1) + " A\n";
  helpMessage += "Voltage Limit: " + String(VOLTAGE_LIMIT, 1) + " V\n";
  helpMessage += "DO Sensor via I2C (Address: 0x" + String(DO_SENSOR_I2C_ADDRESS, HEX) + ")\n";
  helpMessage += "DO threshold: " + String(DO_THRESHOLD, 1) + " mg/L\n";
  helpMessage += "Relay (Pin " + String(RELAY_PIN) + ") control enabled\n";
  helpMessage += "Battery Low Threshold: " + String(BATTERY_LOW_THRESHOLD, 1) + " V\n";
  helpMessage += "Battery Recovery Threshold: " + String(BATTERY_RECOVER_THRESHOLD, 1) + " V\n";
  helpMessage += "AP Mode Trigger: Press button on Pin " + String(AP_TRIGGER_PIN) + " to enter configuration mode\n";
  Serial.println(helpMessage);
  
  // Tambahkan delay untuk memastikan LCD telah selesai dengan pesan startup
  delay(2000);
  
  // Hapus layar LCD untuk memulai tampilan yang bersih
  lcd.clear();
  
  // Cek apakah masuk mode AP sesuai kondisi
  checkAPMode();
}

void loop() {
  // Cek apakah harus masuk mode AP
  if (!isAPMode) {
    checkAPMode();
  
    // Timestamp untuk timing loop
    unsigned long currentMillis = millis();
    
    // Periksa deteksi sumber
    bool isSourceDetected = checkSourceDetection();
    
    // Baca data dari PZEM-017
    PZEM_DATA data = readPZEM();
    
    // Hitung daya secara manual sebagai backup jika pembacaan dari sensor adalah 0
    float calculatedPower = calculatePower(data.voltage, data.current);
    
    // Jika daya dari sensor adalah 0 tapi tegangan dan arus tidak 0, gunakan daya yang dihitung
    if (data.power == 0 && data.voltage > 0 && data.current > 0) {
      data.power = calculatedPower;
    }
    
    // Tentukan mode charging berdasarkan tegangan dan arus
    if (data.voltage >= VOLTAGE_LIMIT) {
      chargingMode = ABSORPTION; // Masuk ke mode tegangan konstan
    } else if (data.voltage < (VOLTAGE_LIMIT - 0.3)) {
      chargingMode = BULK; // Kembali ke mode arus konstan
    }
    
    // Set nilai PWM berdasarkan deteksi sumber dan mode charging
    if (!isSourceDetected) {
      // Jika sumber tidak terdeteksi, matikan PWM
      pwmValue = 0;
      isCharging = false;
    } else {
      // Sumber terdeteksi, tentukan PWM berdasarkan mode
      isCharging = true;
      
      // Hitung error berdasarkan mode
      float currentError = targetCurrent - data.current;
      float voltageError = VOLTAGE_LIMIT - data.voltage;
      
      // Update PWM berdasarkan fuzzy control dan mode charging
      int newPWM = fuzzyPWMControl(currentError, voltageError);
      
      // Terapkan PWM baru jika ada perubahan
      if (newPWM != pwmValue) {
        pwmValue = newPWM;
      }
      
      // Jika dalam mode BULK dan arus terlalu rendah, paksa naikkan PWM
      if (chargingMode == BULK && targetCurrent > CURRENT_THRESHOLD && 
          data.current < (targetCurrent * 0.8) && pwmValue < pwmMaxValue) {
        pwmValue += 1;  // Paksa naik 1 agar mencapai target
      }
    }
    
    // Terapkan nilai PWM
    ledcWrite(pwmChannel, pwmValue);
    
    // Update displayData struct untuk LCD di Core 2
    displayData.pzem = data;
    displayData.sourceDetected = sourceDetected;
    displayData.mode = chargingMode;
    displayData.pwmValue = pwmValue;
    
    // Tentukan status charging untuk log di serial
    String chargingStatus;
    if (!isSourceDetected) {
      chargingStatus = "Mati (Sumber Tidak Terdeteksi)";
    } else if (chargingMode == BULK) {
      chargingStatus = "Bulk Charging (Arus Konstan)";
    } else {
      chargingStatus = "Absorption Charging (Tegangan Konstan)";
    }
    
    // Log data ke Serial untuk debugging - tidak terlalu sering
    static unsigned long lastSerialLogTime = 0;
    if (currentMillis - lastSerialLogTime >= 1000) { // Log setiap 1 detik
      lastSerialLogTime = currentMillis;
      
      String dataOutput = "------------------------\n";
      dataOutput += "Status Sumber: " + String(isSourceDetected ? "Terdeteksi" : "Tidak Terdeteksi") + "\n";
      dataOutput += "Mode Charging: " + chargingStatus + "\n";
      dataOutput += "Tegangan: " + String(data.voltage, 2) + " V (Limit: " + String(VOLTAGE_LIMIT, 1) + " V)\n";
      dataOutput += "Arus: " + String(data.current, 2) + " A (Target: " + String(targetCurrent, 2) + " A)\n";
      
      // Tampilkan error sesuai mode
      if (chargingMode == BULK) {
        dataOutput += "Error Arus: " + String(targetCurrent - data.current, 3) + " A\n";
      } else {
        dataOutput += "Error Tegangan: " + String(VOLTAGE_LIMIT - data.voltage, 3) + " V\n";
      }
      
      dataOutput += "Daya: " + String(data.power, 2) + " W\n";
      dataOutput += "PWM: " + String(pwmValue) + "/" + String(pwmMaxValue) + " (" + String((float)pwmValue / pwmMaxValue * 100.0, 2) + "%)\n";
      
      if (DO_sensor_available) {
        dataOutput += "DO (I2C): " + String(displayData.doMgL, 2) + " mg/L (Threshold: " + String(DO_THRESHOLD, 1) + " mg/L)\n";
      } else {
        dataOutput += "DO (I2C): Sensor tidak tersedia\n";
      }
      
      dataOutput += "Status Relay: " + String(relayStatus ? "ON" : "OFF") + "\n";
      dataOutput += "Status Baterai: " + String(batteryLowState ? "LOW" : "NORMAL") + "\n";
      
      Serial.print(dataOutput);
    }
  }
  
  // Delay kecil untuk stabilitas
  delay(10);  // Delay minimal untuk stabilitas loop
}