#include <Arduino.h>
#include <ModbusMaster.h>
#include <Wire.h>               // Library untuk I2C
#include <LiquidCrystal_I2C.h>  // Library untuk LCD I2C

// Definisi pin untuk komunikasi RS485
#define RX_PIN 17
#define TX_PIN 16
#define SERIAL_COMMUNICATION Serial2

// Definisi pin untuk analog input
#define ANALOG_PIN 18
#define DO_PIN 15  // Pin untuk sensor DO

// Pin untuk relay kontrol
#define RELAY_PIN 4  // Pin untuk relay

// Pin untuk LCD I2C
#define SDA_PIN 8  // Pin SDA pada ESP32 S3
#define SCL_PIN 9  // Pin SCL pada ESP32 S3

// Inisialisasi LCD (alamat I2C 0x27, 20 kolom, 4 baris)
// Jika LCD tidak terdeteksi, coba alamat 0x3F atau alamat lainnya
LiquidCrystal_I2C lcd(0x27, 20, 4);

// Threshold untuk deteksi sumber
#define SOURCE_THRESHOLD 500  // Nilai ini bisa disesuaikan berdasarkan pembacaan sebenarnya

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
const int pwmMaxValue = 3000; // Nilai maksimum PWM dibatasi 3000

// Variabel untuk menyimpan nilai PWM saat ini
int pwmValue = 0;

// SCC parameters
const float MAX_CHARGING_CURRENT = 4.0;  // Target arus charging maksimal (A)
const float VOLTAGE_LIMIT = 14.0;        // Batas tegangan charging (V)
const float CURRENT_THRESHOLD = 0.1;     // Threshold arus minimal untuk mendeteksi charging

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

// DO sensor configuration
#define VREF 3300         // VREF untuk ESP32 (3.3V = 3300 mV)
#define ADC_RES 4095      // Resolusi ADC ESP32 (12-bit: 0-4095)
#define READ_TEMP 25      // Suhu air dalam °C, ganti dengan sensor suhu jika ada

// Single point calibration
#define CAL1_V 1600       // mV
#define CAL1_T 25         // °C

// Parameter untuk stabilisasi pembacaan
#define SAMPLE_INTERVAL 10      // Interval antara sampel (ms)
#define NUM_SAMPLES 10          // Kurangi jumlah sampel untuk mengurangi beban
#define WINDOW_SIZE 5           // Kurangi ukuran window untuk mengurangi beban
#define STABILITY_THRESHOLD 5   // Ambang batas stabilitas (mV)
#define STABILITY_COUNT 3       // Kurangi jumlah hitungan stabilitas

// DO solubility table (saturation DO dalam air pada tekanan 101.325 kPa, unit: μg/L)
const uint16_t DO_Table[41] = {
    14460, 14220, 13820, 13440, 13090, 12740, 12420, 12110, 11810, 11530,
    11260, 11010, 10770, 10530, 10300, 10080, 9860, 9660, 9460, 9270,
    9080, 8900, 8730, 8570, 8410, 8250, 8110, 7960, 7820, 7690,
    7560, 7430, 7300, 7180, 7070, 6950, 6840, 6730, 6630, 6530, 6410
};

// DO sensor variables
uint8_t doTemperature;
uint16_t DO_ADC_Raw;
uint16_t DO_ADC_Voltage;
uint16_t DO_Value;
float DO_mgL;

// Arrays for DO sensor samples
uint16_t doSamples[NUM_SAMPLES];
uint16_t doMedianWindow[WINDOW_SIZE];
uint16_t doLastStableVoltage = 0;
uint8_t doStabilityCounter = 0;

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

// Task handle for DO sensor readings on Core 2
TaskHandle_t DoSensorTask;

// Konstanta untuk kontrol relay
#define DO_THRESHOLD 1.0           // Threshold DO dalam mg/L
#define BATTERY_LOW_THRESHOLD 11.9  // Batas bawah tegangan baterai
#define BATTERY_RECOVER_THRESHOLD 12.4 // Batas atas recovery tegangan baterai

// Variabel untuk kontrol relay
bool relayStatus = false;      // Status relay (false = OFF, true = ON)
bool batteryLowState = false;  // Status baterai low

// Fungsi untuk membaca sampel ADC dari sensor DO
void getDOSamples() {
  for (int i = 0; i < NUM_SAMPLES; i++) {
    doSamples[i] = analogRead(DO_PIN);
    delay(SAMPLE_INTERVAL);
  }
}

// Fungsi untuk mengurutkan array (untuk mendapatkan median)
void sortArray(uint16_t arr[], int size) {
  for (int i = 0; i < size - 1; i++) {
    for (int j = 0; j < size - i - 1; j++) {
      if (arr[j] > arr[j + 1]) {
        uint16_t temp = arr[j];
        arr[j] = arr[j + 1];
        arr[j + 1] = temp;
      }
    }
  }
}

// Fungsi untuk mendapatkan nilai median dari array
uint16_t getMedian(uint16_t arr[], int size) {
  // Buat salinan array agar aslinya tidak terubah
  uint16_t temp[size];
  for (int i = 0; i < size; i++) {
    temp[i] = arr[i];
  }
  
  // Sortir array
  sortArray(temp, size);
  
  // Kembalikan nilai median
  if (size % 2 == 0) {
    return (temp[size/2] + temp[size/2 - 1]) / 2;
  } else {
    return temp[size/2];
  }
}

// Fungsi untuk memperbarui median window
void updateDOMedianWindow(uint16_t newValue) {
  // Geser semua nilai satu posisi
  for (int i = 0; i < WINDOW_SIZE - 1; i++) {
    doMedianWindow[i] = doMedianWindow[i + 1];
  }
  
  // Tambahkan nilai baru di akhir
  doMedianWindow[WINDOW_SIZE - 1] = newValue;
}

// Fungsi untuk menghitung DO
int16_t readDO(uint32_t voltage_mv, uint8_t temperature_c) {
  // Single point calibration
  uint16_t V_saturation = (uint32_t)CAL1_V + (uint32_t)35 * temperature_c - (uint32_t)CAL1_T * 35;
  return (voltage_mv * DO_Table[temperature_c] / V_saturation);
}

// Fungsi untuk memeriksa stabilitas pembacaan DO
bool isDOStable(uint16_t currentVoltage) {
  if (abs((int)currentVoltage - (int)doLastStableVoltage) <= STABILITY_THRESHOLD) {
    doStabilityCounter++;
    if (doStabilityCounter >= STABILITY_COUNT) {
      return true;
    }
  } else {
    doLastStableVoltage = currentVoltage;
    doStabilityCounter = 0;
  }
  return false;
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
    if (doValue < DO_THRESHOLD) {
      // DO rendah, nyalakan relay jika belum menyala
      if (!relayStatus) {
        relayStatus = true;
        Serial.println("Relay ON: DO rendah (<" + String(DO_THRESHOLD) + " mg/L)");
      }
    } else {
      // DO normal, matikan relay jika belum mati
      if (relayStatus) {
        relayStatus = false;
        Serial.println("Relay OFF: DO normal (>" + String(DO_THRESHOLD) + " mg/L)");
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
    // Baris 1: Status sumber dan mode charging
    lcd.setCursor(0, 0);
    if (!data.sourceDetected) {
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
    lcd.print("V:");
    char voltageStr[10];
    sprintf(voltageStr, "%.2f", data.pzem.voltage);
    lcd.print(voltageStr);
    lcd.print("V            ");
    
    // Baris 3: Arus dan DO
    lcd.setCursor(0, 2);
    lcd.print("I:");
    char currentStr[10];
    sprintf(currentStr, "%.2f", data.pzem.current);
    lcd.print(currentStr);
    lcd.print("A DO:");
    char doStr[10];
    sprintf(doStr, "%.1f", data.doMgL);
    lcd.print(doStr);
    lcd.print("mg/L ");
    
    // Baris 4: Daya, PWM, dan status relay
    lcd.setCursor(0, 3);
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
    
    // Lepaskan semaphore
    xSemaphoreGive(lcdMutex);
  }
}

// Task untuk sensor DO dan update LCD yang berjalan di Core 2
void DOSensorTaskCode(void * parameter) {
  // Inisialisasi window median untuk DO sensor
  for (int i = 0; i < WINDOW_SIZE; i++) {
    getDOSamples();
    doMedianWindow[i] = getMedian(doSamples, NUM_SAMPLES);
    delay(50);
  }
  
  Serial.println("DO Sensor initialized on Core 2");
  
  // Timestamp untuk memantau interval update sensor DO
  unsigned long lastDoUpdate = 0;
  const unsigned long DO_UPDATE_INTERVAL = 1000; // Update DO setiap 1 detik
  
  // Timestamp untuk memantau interval update LCD
  unsigned long lastLcdUpdate = 0;
  const unsigned long LCD_UPDATE_INTERVAL = 500; // Update LCD setiap 500ms
  
  for(;;) {
    unsigned long currentMillis = millis();
    
    // Update sensor DO setiap DO_UPDATE_INTERVAL
    if (currentMillis - lastDoUpdate >= DO_UPDATE_INTERVAL) {
      lastDoUpdate = currentMillis;
      
      // Set suhu
      doTemperature = (uint8_t)READ_TEMP;
      
      // Ambil sampel
      getDOSamples();
      
      // Hitung median dari sampel
      uint16_t medianADC = getMedian(doSamples, NUM_SAMPLES);
      
      // Update median window
      updateDOMedianWindow(medianADC);
      
      // Hitung median dari window
      DO_ADC_Raw = getMedian(doMedianWindow, WINDOW_SIZE);
      
      // Konversi ke tegangan (mV)
      DO_ADC_Voltage = uint32_t(VREF) * DO_ADC_Raw / ADC_RES;
      
      // Periksa stabilitas pembacaan
      bool stable = isDOStable(DO_ADC_Voltage);
      
      // Hitung nilai DO jika stabil
      DO_Value = readDO(DO_ADC_Voltage, doTemperature);
      DO_mgL = DO_Value / 1000.0;
      
      // Update nilai DO dalam struct displayData yang diakses global
      displayData.doMgL = DO_mgL;
      
      // Kontrol relay berdasarkan data terbaru
      controlRelay(DO_mgL, displayData.pzem.voltage);
      
      // Log hasil ke serial (untuk debugging)
      Serial.print("DO Sensor: ");
      Serial.print(DO_mgL, 2);
      Serial.print(" mg/L (");
      Serial.print(stable ? "STABIL" : "Stabilizing...");
      Serial.println(")");
    }
    
    // Update LCD setiap LCD_UPDATE_INTERVAL
    if (currentMillis - lastLcdUpdate >= LCD_UPDATE_INTERVAL) {
      lastLcdUpdate = currentMillis;
      
      // Update LCD dengan data terbaru
      updateLCD(displayData);
    }
    
    // Delay kecil untuk memperlancar multitasking
    delay(10);
  }
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
  
  // Periksa apakah nilai analog di atas threshold
  bool isSourceDetected = (analogValue > SOURCE_THRESHOLD);
  
  // Jika status berubah, kirim notifikasi
  if (isSourceDetected != sourceDetected) {
    sourceDetected = isSourceDetected;
    
    String statusMessage = "Sumber: " + String(sourceDetected ? "Terdeteksi" : "Tidak Terdeteksi");
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

void setup() {
  // Inisialisasi Serial untuk debugging
  Serial.begin(115200);
  Serial.println("ESP32-S3 - Solar Charge Controller (SCC) with PZEM-017, LCD, DO Sensor, and Relay Control");
  
  // Inisialisasi pin relay
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, LOW);  // Relay start in OFF state
  
  // Inisialisasi I2C untuk LCD
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
  lcd.print("DO Threshold: ");
  lcd.print(DO_THRESHOLD);
  lcd.print("mg/L");
  lcd.setCursor(0, 3);
  lcd.print("Batt Low: ");
  lcd.print(BATTERY_LOW_THRESHOLD);
  lcd.print("V");
  
  // Konfigurasi pin analog
  pinMode(ANALOG_PIN, INPUT);
  pinMode(DO_PIN, INPUT);  // Set DO_PIN sebagai input
  
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
  helpMessage += "DO Sensor threshold: " + String(DO_THRESHOLD, 1) + " mg/L\n";
  helpMessage += "Relay (Pin " + String(RELAY_PIN) + ") control enabled\n";
  helpMessage += "Battery Low Threshold: " + String(BATTERY_LOW_THRESHOLD, 1) + " V\n";
  helpMessage += "Battery Recovery Threshold: " + String(BATTERY_RECOVER_THRESHOLD, 1) + " V\n";
  Serial.println(helpMessage);
  
  // Tambahkan delay untuk memastikan LCD telah selesai dengan pesan startup
  // digitalWrite(4,1);
  // delay(2000);
  digitalWrite(4,0);
}

void loop() {
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
    dataOutput += "DO: " + String(displayData.doMgL, 2) + " mg/L (Threshold: " + String(DO_THRESHOLD, 1) + " mg/L)\n";
    dataOutput += "Status Relay: " + String(relayStatus ? "ON" : "OFF") + "\n";
    dataOutput += "Status Baterai: " + String(batteryLowState ? "LOW" : "NORMAL") + "\n";
    
    Serial.print(dataOutput);
  }
  
  // Delay kecil untuk stabilitas
  delay(10);  // Delay minimal untuk stabilitas loop
}