#include <Wire.h>
#include <Adafruit_INA219.h>

// Inisialisasi sensor INA219
Adafruit_INA219 ina219;

// Definisi pin
const int pwmPin = 5;        // GPIO5 untuk ESP32/ESP8266
int pwmValue = 5;          // Nilai PWM awal (0-255)

// Variabel untuk PID
float targetVoltage = 0.93;   // Tegangan target yang diinginkan (V)
float Kp = 3.0;             // Konstanta Proportional
float Ki = 0.0;              // Konstanta Integral
float Kd = 1.0;              // Konstanta Derivative

float lastError = 0;         // Error sebelumnya
float integral = 0;          // Nilai integral
unsigned long lastTime = 0;  // Waktu pembacaan terakhir

// Batasan untuk output PWM
const int minPWM = 10;       // Nilai minimum PWM
const int maxPWM = 240;      // Nilai maksimum PWM

void setup() {
  // Inisialisasi komunikasi serial
  Serial.begin(9600);
  
  // Inisialisasi pin PWM
  pinMode(pwmPin, OUTPUT);
  
  // // Setel PWM ke nilai awal
  // analogWrite(pwmPin, pwmValue);
  
  // Inisialisasi INA219
  if (!ina219.begin()) {
    Serial.println("Gagal menemukan sensor INA219!");
    while (1) { delay(10); }
  }
  
  // Konfigurasi kalibrasi INA219 (opsional)
  // ina219.setCalibration_16V_400mA();  // Untuk akurasi lebih baik pada tegangan rendah
  
  // Pesan pembuka
  Serial.println("Pengukuran Buck Converter dengan Kontrol PID");
  Serial.println("Target Voltage: " + String(targetVoltage) + " V");
  Serial.println("----------------------------------------------------------");
  Serial.println("Tegangan(V), Arus(mA), PWM, Error");
  
  // Inisialisasi waktu
  lastTime = millis();
}

void loop() {
  // Baca nilai dari INA219
  float shuntVoltage = ina219.getShuntVoltage_mV();
  float busVoltage = ina219.getBusVoltage_V();
  float current = ina219.getCurrent_mA();
  float loadVoltage = busVoltage + (shuntVoltage / 1000);
  
  // Hitung waktu yang berlalu dalam detik
  unsigned long currentTime = millis();
  float deltaTime = (currentTime - lastTime) / 1000.0; // konversi ke detik
  lastTime = currentTime;
  
  // Hitung error
  float error = targetVoltage - loadVoltage;
  
  // Hitung nilai integral (dengan anti-windup)
  integral += error * deltaTime;
  integral = constrain(integral, -50, 50); // Batasi integral untuk mencegah windup
  
  // Hitung nilai derivative
  float derivative = (error - lastError) / deltaTime;
  lastError = error;
  
  // Hitung output PID
  float pidOutput = (Kp * error) + (Ki * integral) + (Kd * derivative);
  
  // Sesuaikan nilai PWM
  pwmValue = pwmValue + (int)pidOutput;
  
  // Batasi nilai PWM
  pwmValue = constrain(pwmValue, minPWM, maxPWM);
  
  // Terapkan nilai PWM ke pin
  analogWrite(pwmPin, pwmValue);
  
  // Tampilkan hasil
  Serial.print(loadVoltage, 2);
  // Serial.print(", ");
  // // Serial.print(current, 1);
  // Serial.print(", ");
  // Serial.println(pwmValue);
  Serial.print(", ");
  Serial.println("3");
  
  // Tunggu sebelum pengukuran berikutnya
  delay(100);
}