#include <Arduino.h>
#include <ModbusMaster.h>

// Definisi pin untuk komunikasi RS485
#define RX_PIN 17
#define TX_PIN 16
#define SERIAL_COMMUNICATION Serial2

// Alamat default untuk PZEM-017
#define PZEM_ADDR 0x01

// Instance untuk ModbusMaster
ModbusMaster node;

// Definisi pin dan parameter PWM
const int pwmPin = 5;        // Pin output PWM (GPIO5)
const int pwmChannel = 0;    // Channel PWM (0-15)
const int pwmFreq = 8500;    // Frekuensi PWM 8.5 kHz
const int pwmResolution = 12; // Resolusi PWM 12-bit (0-4095)
const int pwmMaxValue = 3000; // Nilai maksimum PWM dibatasi 500

// Variabel untuk menyimpan nilai PWM saat ini
int pwmValue = 0;

// Variabel untuk fuzzy control
float targetCurrent = 0.0;  // Target arus yang diinginkan

// Struktur untuk menyimpan data dari PZEM-017
struct PZEM_DATA {
  float voltage;
  float current;
  float power;
};

// Fungsi untuk logika fuzzy Sugeno berdasarkan error arus
int fuzzyPWMControl(float currentError) {
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
  int NBOutput = -10;   // Decrease PWM by 1
  int NSOutput = -1;   // Decrease PWM by 1
  int ZEOutput = 0;    // No change
  int PSOutput = 1;    // Increase PWM by 3 (lebih agresif naik)
  int PBOutput = 10;    // Increase PWM by 5 (jauh lebih agresif naik)
  
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
  Serial.println("ESP32-S3 - PZEM-017 DC Meter + Fuzzy Sugeno PWM Controller");
  
  // Inisialisasi Serial2 untuk komunikasi RS485
  SERIAL_COMMUNICATION.begin(9600, SERIAL_8N1, RX_PIN, TX_PIN);
  
  // Inisialisasi ModbusMaster
  node.begin(PZEM_ADDR, SERIAL_COMMUNICATION);
  
  // Konfigurasi PWM
  ledcSetup(pwmChannel, pwmFreq, pwmResolution);
  ledcAttachPin(pwmPin, pwmChannel);
  
  Serial.println("System initialized");
  Serial.println("Input nilai arus target langsung (misal: 1.5 untuk 1.5A)");
  Serial.println("PWM akan otomatis naik/turun dengan inkremen 1");
  delay(1000);
}

// Fungsi untuk membaca data dari PZEM-017
PZEM_DATA readPZEM() {
  PZEM_DATA data;
  
  // Inisialisasi data dengan nilai default
  data.voltage =.0;
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

void processCommand() {
  // Periksa apakah ada data serial yang masuk
  if (Serial.available() > 0) {
    // Baca input serial sebagai string
    String input = Serial.readStringUntil('\n');
    input.trim(); // Hapus whitespace
    
    // Jika input adalah numerik, gunakan sebagai target arus
    if (input.length() > 0) {
      // Coba konversi ke float
      float newCurrent = input.toFloat();
      
      // Update target arus jika nilai valid
      if (newCurrent >= 0) {
        targetCurrent = newCurrent;
        Serial.print("Target arus diatur ke: ");
        Serial.print(targetCurrent, 2);
        Serial.println(" A");
      } else {
        Serial.println("Error: Masukkan nilai arus positif");
      }
    }
  }
}

void loop() {
  // Proses perintah dari Serial
  processCommand();
  
  // Baca data dari PZEM-017
  PZEM_DATA data = readPZEM();
  
  // Hitung daya secara manual sebagai backup jika pembacaan dari sensor adalah 0
  float calculatedPower = calculatePower(data.voltage, data.current);
  
  // Jika daya dari sensor adalah 0 tapi tegangan dan arus tidak 0, gunakan daya yang dihitung
  if (data.power == 0 && data.voltage > 0 && data.current > 0) {
    data.power = calculatedPower;
  }
  
  // Update PWM berdasarkan fuzzy control
  float currentError = targetCurrent - data.current;
  int newPWM = fuzzyPWMControl(currentError);
    
  // Terapkan PWM baru jika ada perubahan
  if (newPWM != pwmValue) {
    pwmValue = newPWM;
    ledcWrite(pwmChannel, pwmValue);
  }
  
  // Jika target arus > 0.05 dan arus saat ini < 80% dari target dan PWM belum maksimal, paksa naikkan PWM
  if (targetCurrent > 0.05 && data.current < (targetCurrent * 0.8) && pwmValue < pwmMaxValue) {
    pwmValue += 1;  // Paksa naik 1 agar mencapai target
    ledcWrite(pwmChannel, pwmValue);
  }
  
  // Tampilkan data PZEM dan PWM
  Serial.println("------------------------");
  Serial.print("Tegangan: ");
  Serial.print(data.voltage, 2);
  Serial.println(" V");
  
  Serial.print("Arus: ");
  Serial.print(data.current, 2);
  Serial.print(" A");
  Serial.print(" (Target: ");
  Serial.print(targetCurrent, 2);
  Serial.print(" A)");
  Serial.println();
  
  Serial.print("Error Arus: ");
  Serial.print(currentError, 3);
  Serial.println(" A");
  
  Serial.print("Daya: ");
  Serial.print(data.power, 2);
  Serial.println(" W");
  
  Serial.print("PWM: ");
  Serial.print(pwmValue);
  Serial.print("/");
  Serial.print(pwmMaxValue);
  Serial.print(" (");
  Serial.print((float)pwmValue / pwmMaxValue * 100.0, 2);
  Serial.println("%)");
  
  // Tunggu sebelum membaca lagi
  delay(50);
}