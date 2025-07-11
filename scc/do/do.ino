#include <Arduino.h>
#include <Wire.h>

// ================== KONFIGURASI DO SENSOR ====================
#define DO_PIN 32               // Pin analog untuk sensor DO
#define VREF 3300               // VREF untuk ESP32 (3.3V = 3300 mV)
#define ADC_RES 4095            // Resolusi ADC ESP32 (12-bit: 0-4095)
#define TWO_POINT_CALIBRATION 0 // 0 = single point, 1 = two point
#define READ_TEMP 25            // Suhu air dalam °C, ganti dengan sensor suhu jika ada

// Single point calibration
#define CAL1_V 1600             // mV
#define CAL1_T 25               // °C

// Two point calibration (jika digunakan)
#define CAL2_V 1300             // mV
#define CAL2_T 15               // °C

// Parameter untuk stabilisasi pembacaan
#define SAMPLE_INTERVAL 10      // Interval antara sampel (ms)
#define NUM_SAMPLES 10          // Jumlah sampel untuk median
#define WINDOW_SIZE 5           // Ukuran window untuk median filtering
#define STABILITY_THRESHOLD 5   // Ambang batas stabilitas (mV)
#define STABILITY_COUNT 3       // Jumlah hitungan stabilitas

// Alamat I2C untuk slave ini
#define I2C_SLAVE_ADDRESS 0x08

// Tabel DO jenuh (0–40°C) - saturation DO dalam air pada tekanan 101.325 kPa, unit: μg/L
const uint16_t DO_Table[41] = {
  14460, 14220, 13820, 13440, 13090, 12740, 12420, 12110, 11810, 11530,
  11260, 11010, 10770, 10530, 10300, 10080, 9860, 9660, 9460, 9270,
  9080, 8900, 8730, 8570, 8410, 8250, 8110, 7960, 7820, 7690,
  7560, 7430, 7300, 7180, 7070, 6950, 6840, 6730, 6630, 6530, 6410
};

// Variabel global DO sensor
uint8_t doTemperature;
uint16_t DO_ADC_Raw;
uint16_t DO_ADC_Voltage;
uint16_t DO_Value;
float DO_mgL = 0.0;

// Arrays untuk DO sensor samples
uint16_t doSamples[NUM_SAMPLES];
uint16_t doMedianWindow[WINDOW_SIZE];
uint16_t doLastStableVoltage = 0;
uint8_t doStabilityCounter = 0;

// Flag untuk data ready
bool dataReady = false;

// ================== FUNGSI DO SENSOR ====================

// Fungsi perhitungan DO
int16_t readDO(uint32_t voltage_mv, uint8_t temperature_c) {
#if TWO_POINT_CALIBRATION == 0
  // Single point calibration
  uint16_t V_saturation = (uint32_t)CAL1_V + 35 * temperature_c - CAL1_T * 35;
  return (voltage_mv * DO_Table[temperature_c] / V_saturation);
#else
  // Two point calibration
  uint16_t V_saturation = ((int16_t)(temperature_c - CAL2_T)) * (CAL1_V - CAL2_V) / (CAL1_T - CAL2_T) + CAL2_V;
  return (voltage_mv * DO_Table[temperature_c] / V_saturation);
#endif
}

// Fungsi untuk mengambil sampel ADC dari sensor DO
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

// Fungsi untuk menampilkan status DO
void tampilStatus(float doVal) {
  if (doVal >= 6.0) {
    Serial.println("Bagus");
  } else if (doVal >= 4.0 && doVal < 6.0) {
    Serial.println("Kurang");
  } else {
    Serial.println("Buruk");
  }
}

// Fungsi untuk update sensor DO
void updateDOSensor() {
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
  DO_ADC_Voltage = (uint32_t)VREF * DO_ADC_Raw / ADC_RES;
  
  // Periksa stabilitas pembacaan
  bool stable = isDOStable(DO_ADC_Voltage);
  
  // Hitung nilai DO
  DO_Value = readDO(DO_ADC_Voltage, doTemperature);
  DO_mgL = DO_Value / 1000.0;
  
  // Set flag data ready
  dataReady = true;
  
  // Log ke serial
  Serial.print("Suhu (°C): "); Serial.print(doTemperature);
  Serial.print(" | ADC: "); Serial.print(DO_ADC_Raw);
  Serial.print(" | Tegangan (mV): "); Serial.print(DO_ADC_Voltage);
  Serial.print(" | DO (mg/L): "); Serial.print(DO_mgL, 2);
  Serial.print(" | Status: ");
  tampilStatus(DO_mgL);
  Serial.print(" | Stable: "); Serial.println(stable ? "YES" : "NO");
}

// ================== I2C SLAVE FUNCTIONS ====================

// Callback ketika master meminta data
void onRequest() {
  if (dataReady) {
    // Kirim data DO sebagai 4 bytes (float)
    byte* floatBytes = (byte*)&DO_mgL;
    Wire.write(floatBytes, 4);
    
    Serial.print("Data sent via I2C: ");
    Serial.println(DO_mgL, 2);
  } else {
    // Kirim 0 jika data belum ready
    float zeroValue = 0.0;
    byte* floatBytes = (byte*)&zeroValue;
    Wire.write(floatBytes, 4);
    
    Serial.println("Data not ready, sent 0.0");
  }
}

// Callback ketika menerima data dari master (tidak digunakan dalam aplikasi ini)
void onReceive(int bytes) {
  // Optional: bisa digunakan untuk menerima perintah dari master
  while (Wire.available()) {
    Wire.read(); // Baca dan buang data
  }
}

// ================== SETUP DAN LOOP ====================

void setup() {
  Serial.begin(115200);
  Serial.println("ESP32 DO Sensor I2C Slave");
  Serial.print("I2C Address: 0x");
  Serial.println(I2C_SLAVE_ADDRESS, HEX);
  
  // Set resolusi ADC
  analogReadResolution(12);
  
  // Konfigurasi pin DO sensor
  pinMode(DO_PIN, INPUT);
  
  // Inisialisasi I2C sebagai slave
  Wire.begin(I2C_SLAVE_ADDRESS);
  Wire.onRequest(onRequest);   // Register callback untuk request
  Wire.onReceive(onReceive);   // Register callback untuk receive
  
  // Inisialisasi window median untuk DO sensor
  Serial.println("Initializing DO sensor...");
  for (int i = 0; i < WINDOW_SIZE; i++) {
    getDOSamples();
    doMedianWindow[i] = getMedian(doSamples, NUM_SAMPLES);
    delay(100);
  }
  
  Serial.println("DO Sensor I2C Slave ready");
  Serial.println("Pin DO: " + String(DO_PIN));
  Serial.println("Calibration: " + String(TWO_POINT_CALIBRATION ? "Two Point" : "Single Point"));
  Serial.println("Sample interval: " + String(SAMPLE_INTERVAL) + " ms");
  Serial.println("Number of samples: " + String(NUM_SAMPLES));
  Serial.println("Window size: " + String(WINDOW_SIZE));
  Serial.println("------------------------");
}

void loop() {
  // Update sensor DO setiap 1 detik
  static unsigned long lastUpdate = 0;
  unsigned long currentMillis = millis();
  
  if (currentMillis - lastUpdate >= 1000) {
    lastUpdate = currentMillis;
    
    // Update pembacaan sensor DO
    updateDOSensor();
  }
  
  // Delay kecil untuk stabilitas
  delay(10);
}