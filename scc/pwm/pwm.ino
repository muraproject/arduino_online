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
const int pwmMaxValue = 4095; // Nilai maksimum untuk resolusi 12-bit

// Variabel untuk menyimpan nilai PWM saat ini
int pwmValue = 0;

// Struktur untuk menyimpan data dari PZEM-017
struct PZEM_DATA {
  float voltage;
  float current;
  float power;
};

void setup() {
  // Inisialisasi Serial untuk debugging
  Serial.begin(115200);
  Serial.println("ESP32-S3 - PZEM-017 DC Meter + PWM Controller");
  
  // Inisialisasi Serial2 untuk komunikasi RS485
  SERIAL_COMMUNICATION.begin(9600, SERIAL_8N1, RX_PIN, TX_PIN);
  
  // Inisialisasi ModbusMaster
  node.begin(PZEM_ADDR, SERIAL_COMMUNICATION);
  
  // Konfigurasi PWM
  ledcSetup(pwmChannel, pwmFreq, pwmResolution);
  ledcAttachPin(pwmPin, pwmChannel);
  
  Serial.println("System initialized");
  Serial.println("Kirim nilai 0-4095 untuk mengatur duty cycle PWM");
  delay(1000);
}

// Fungsi untuk membaca data dari PZEM-017
PZEM_DATA readPZEM() {
  PZEM_DATA data;
  
  // Inisialisasi data dengan nilai default
  data.voltage = 0;
  data.current = 0;
  data.power = 0;
  
  // Membaca semua register data dalam satu permintaan (alamat 0x0000, baca 3 register)
  if (node.readInputRegisters(0x0000, 3) == node.ku8MBSuccess) {
    // Register 0x0000: Tegangan
    data.voltage = node.getResponseBuffer(0) * 0.01; // Konversi ke volt
    
    // Register 0x0001: Arus
    data.current = node.getResponseBuffer(1) * 0.01; // Konversi ke ampere
    
    // Register 0x0002: Daya
    data.power = node.getResponseBuffer(2) * 0.1; // Konversi ke watt
  } else {
    // Serial.println("Gagal membaca data dari PZEM-017");
  }
  
  return data;
}

// Fungsi untuk menghitung daya secara manual
float calculatePower(float voltage, float current) {
  return voltage * current;
}

void processPWMCommand() {
  // Periksa apakah ada data serial yang masuk
  if (Serial.available() > 0) {
    // Baca input serial sebagai string
    String input = Serial.readStringUntil('\n');
    input.trim(); // Hapus whitespace
    
    // Konversi input menjadi integer
    int newValue = input.toInt();
    
    // Validasi input (pastikan dalam rentang 0-4095)
    if (newValue >= 0 && newValue <= pwmMaxValue) {
      // Perbarui nilai PWM
      pwmValue = newValue;
      
      // Terapkan nilai PWM ke pin
      ledcWrite(pwmChannel, pwmValue);
      
      // Feedback ke pengguna
      Serial.print("PWM diatur ke: ");
      Serial.println(pwmValue);
      
      // Tampilkan duty cycle dalam persentase
      float dutyCycle = (float)pwmValue / pwmMaxValue * 100.0;
      Serial.print("Duty Cycle: ");
      Serial.print(dutyCycle, 2); // Dua angka desimal
      Serial.println("%");
    } else {
      // Pesan error jika input tidak valid
      Serial.println("Error: Masukkan nilai antara 0-4095");
    }
  }
}

void loop() {
  // Proses perintah PWM dari Serial
  processPWMCommand();
  
  // Baca data dari PZEM-017
  PZEM_DATA data = readPZEM();
  
  // Hitung daya secara manual sebagai backup jika pembacaan dari sensor adalah 0
  float calculatedPower = calculatePower(data.voltage, data.current);
  
  // Jika daya dari sensor adalah 0 tapi tegangan dan arus tidak 0, gunakan daya yang dihitung
  if (data.power == 0 && data.voltage > 0 && data.current > 0) {
    data.power = calculatedPower;
  }
  
  // Tampilkan data PZEM dan PWM
  Serial.println("------------------------");
  Serial.print("Tegangan: ");
  Serial.print(data.voltage, 2);
  Serial.println(" V");
  
  Serial.print("Arus: ");
  Serial.print(data.current, 2);
  Serial.println(" A");
  
  Serial.print("Daya: ");
  Serial.print(data.power, 2);
  Serial.println(" W");
  
  Serial.print("PWM: ");
  Serial.print(pwmValue);
  Serial.print(" (");
  Serial.print((float)pwmValue / pwmMaxValue * 100.0, 2);
  Serial.println("%)");
  
  // Tunggu sebelum membaca lagi
  delay(50);
}