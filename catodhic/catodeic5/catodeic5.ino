/*
 * Program ESP32 untuk Cathodic Protection System
 * - Menggunakan pembagi tegangan 10k-10k di pin 34 (ADC1_CH6)
 * - PWM output di pin 5 dengan resolusi 12-bit (0-4095)
 * - Menggunakan PID untuk menjaga tegangan stabil di set point
 * - Monitoring arus dan tegangan menggunakan INA219
 * - Tampilan LCD tanpa kedipan menggunakan sprintf
 */

#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Adafruit_INA219.h>
#include <stdio.h>

// Inisialisasi LCD (alamat I2C 0x27, 20 kolom, 4 baris)
LiquidCrystal_I2C lcd(0x27, 20, 4);

// Inisialisasi INA219 sensor
Adafruit_INA219 ina219;

// Definisi pin
const int sensorPin = 34;    // Pin sensor analog ESP32 (ADC1_CH6)
const int pwmPin = 5;        // Pin output PWM untuk buck converter

// Definisi pin I2C untuk ESP32
const int SDA_PIN = 21;      // Default SDA pin untuk ESP32
const int SCL_PIN = 22;      // Default SCL pin untuk ESP32

// Definisi PWM untuk ESP32
const int PWM_CHANNEL = 0;   // PWM channel (0-15)
const int PWM_FREQ = 5000;   // Frekuensi PWM (5kHz)
const int PWM_RESOLUTION = 12; // Resolusi PWM 12-bit (0-4095)

// Variabel untuk pengukuran
int pwmValue = 800;          // Nilai PWM awal (0-4095 untuk 12-bit)
float vRef = 3.3;            // Tegangan referensi ESP32 (3.3V)
int numSamples = 100;        // Jumlah sampel untuk perhitungan RMS

// Parameter PID
float setPoint = 0.93;       // Set point tegangan output yang diinginkan (V)
float Kp = 100.0;            // Konstanta Proporsional
float Ki = 20.0;             // Konstanta Integral
float Kd = 10.0;             // Konstanta Derivatif

// Batas PWM
const int PWM_MIN = 100;     // Nilai minimum PWM
const int PWM_MAX = 4000;    // Nilai maksimum PWM

// Variabel untuk kalkulasi PID
float lastError = 0;         // Error sebelumnya
float integral = 0;          // Nilai integral error
unsigned long lastTime = 0;  // Waktu perhitungan PID terakhir
float outputVrms = 0;        // Nilai VRMS output terakhir

// Variabel untuk update LCD
unsigned long lastLcdUpdate = 0;
const long lcdUpdateInterval = 300; // Update LCD setiap 300ms

// Variabel untuk INA219
float busVoltage = 0;
float current_mA = 0;
float power_mW = 0;
float loadVoltage = 0;

// Buffer untuk LCD - Pastikan buffer penuh dengan spasi untuk menimpa karakter lama
char lcdLine1[21] = " CATHODIC PROTECTION";  // Judul tetap
char lcdLine2[21] = "Vout:0.00V Vref:0.00";
char lcdLine3[21] = "Iout:0.0mA          ";
char lcdLine4[21] = "PWM:0 (0.0%)        ";

// Variabel untuk menyimpan nilai sebelumnya
float prev_loadVoltage = 0;
float prev_outputVrms = 0;
float prev_current_mA = 0;
int prev_pwmValue = 0;
float prev_pwmPercent = 0;

// Variabel untuk mendeteksi waktu sejak baris 1 direfresh
unsigned long lastLine1Refresh = 0;
const long line1RefreshInterval = 2000; // Refresh judul setiap 2 detik

void setup() {
  // Inisialisasi komunikasi serial
  Serial.begin(9600);
  
  // Inisialisasi I2C dengan pin khusus ESP32
  Wire.begin(SDA_PIN, SCL_PIN);
  
  // Inisialisasi pin sensor
  pinMode(sensorPin, INPUT);
  
  // Konfigurasi PWM ESP32
  ledcSetup(PWM_CHANNEL, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(pwmPin, PWM_CHANNEL);
  
  // Setel PWM ke nilai awal
  ledcWrite(PWM_CHANNEL, pwmValue);
  
  // Inisialisasi LCD
  lcd.init();
  lcd.backlight();
  
  // Inisialisasi INA219
  if (!ina219.begin()) {
    Serial.println("Gagal menemukan chip INA219");
    // Tetap tampilkan judul pada baris 1
    lcd.setCursor(0, 0);
    lcd.print(lcdLine1);
    // Tampilkan error pada baris 2
    lcd.setCursor(0, 1);
    lcd.print("INA219 Error!       ");
  }
  else {
    Serial.println("INA219 terdeteksi");
    // Kalibrasi INA219 (32V, 2A range)
    ina219.setCalibration_32V_2A();
  }
  
  // Tampilkan pesan pembuka di LCD
  lcd.clear(); // Hanya clear di awal program
  
  // Baris 1 selalu "CATHODIC PROTECTION"
  lcd.setCursor(0, 0);
  lcd.print(lcdLine1);
  
  // Pesan pembuka pada baris lain
  lcd.setCursor(0, 1);
  lcd.print("SPt: ");
  lcd.print(setPoint, 2);
  lcd.print("V");
  lcd.setCursor(0, 2);
  lcd.print("12-bit PWM, 5kHz");
  lcd.setCursor(0, 3);
  lcd.print("Monitoring Mode");
  delay(2000); // Tahan pesan pembuka selama 2 detik
  
  // Set tampilan awal LCD
  lcd.setCursor(0, 0);
  lcd.print(lcdLine1);
  lcd.setCursor(0, 1);
  lcd.print(lcdLine2);
  lcd.setCursor(0, 2);
  lcd.print(lcdLine3);
  lcd.setCursor(0, 3);
  lcd.print(lcdLine4);
  
  // Pesan pembuka di Serial
  Serial.println("Sistem Cathodic Protection dengan PID dan Monitoring INA219");
  Serial.println("Set Point: " + String(setPoint) + "V");
  Serial.println("PWM Range: Min=" + String(PWM_MIN) + ", Max=" + String(PWM_MAX));
  Serial.println("----------------------------------------------------------");
  Serial.println("PWM, VRMS, Vout(INA219), Iout(mA), Power(mW)");
  
  // Inisialisasi waktu
  lastTime = millis();
  lastLcdUpdate = millis();
  lastLine1Refresh = millis();
}

void loop() {
  // Variabel untuk menyimpan total kuadrat
  float sumSquares = 0.0;
  float voltagePin = 0.0;
  float outputVoltage = 0.0;
  
  // Ambil beberapa sampel untuk perhitungan RMS
  for (int i = 0; i < numSamples; i++) {
    // Baca nilai ADC
    int adcValue = analogRead(sensorPin);
    
    // Konversi ke tegangan di pin sensor
    float voltage = (adcValue / 4095.0) * vRef;
    
    // Kompensasi pembagi tegangan untuk mendapatkan tegangan output sebenarnya
    float actualVoltage = voltage * 2.0;  // Karena pembagi tegangan 10k-10k (1:1)
    
    // Tambahkan kuadrat ke total
    sumSquares += actualVoltage * actualVoltage;
    
    // Simpan nilai terakhir untuk tampilan
    if (i == numSamples - 1) {
      voltagePin = voltage;
      outputVoltage = actualVoltage;
    }
    
    // Tunggu sedikit sebelum sampel berikutnya
    delayMicroseconds(100);
  }
  
  // Hitung VRMS
  float vrms = sqrt(sumSquares / numSamples) + 0.3;
  outputVrms = vrms;
  
  // Hitung nilai PID dan perbarui PWM
  pwmValue = calculatePID(vrms);
  
  // Pastikan PWM dalam rentang yang valid
  pwmValue = constrain(pwmValue, PWM_MIN, PWM_MAX);
  
  // Terapkan nilai PWM baru dengan ledcWrite untuk ESP32
  ledcWrite(PWM_CHANNEL, pwmValue);
  
  // Baca data dari INA219 (hanya untuk monitoring)
  readINA219();
  
  // Hitung persentase PWM
  float pwmPercent = (pwmValue / 4095.0) * 100.0;
  
  // Tampilkan hasil di Serial
  Serial.print(pwmValue);
  Serial.print(", ");
  Serial.print(vrms, 4);
  Serial.print(", ");
  Serial.print(loadVoltage, 4);
  Serial.print(", ");
  Serial.print(current_mA, 2);
  Serial.print(", ");
  Serial.println(power_mW, 2);
  
  // Update LCD jika waktunya
  unsigned long currentMillis = millis();
  
  // Update informasi data pada LCD
  if (currentMillis - lastLcdUpdate >= lcdUpdateInterval) {
    lastLcdUpdate = currentMillis;
    updateLCDNoFlicker(pwmValue, pwmPercent);
  }
  
  // Refresh judul secara berkala untuk mencegah perubahan
  if (currentMillis - lastLine1Refresh >= line1RefreshInterval) {
    lastLine1Refresh = currentMillis;
    lcd.setCursor(0, 0);
    lcd.print(lcdLine1);
  }
  
  // Tunggu sebelum pengukuran berikutnya
  delay(100);
}

// Fungsi untuk membaca data dari INA219
void readINA219() {
  busVoltage = ina219.getBusVoltage_V();
  current_mA = ina219.getCurrent_mA();
  power_mW = ina219.getPower_mW();
  
  // Tegangan pada beban = tegangan bus
  loadVoltage = busVoltage;
}

// Fungsi untuk menghitung output PID
int calculatePID(float currentValue) {
  // Hitung waktu delta
  unsigned long currentTime = millis();
  float deltaTime = (currentTime - lastTime) / 1000.0; // dalam detik
  lastTime = currentTime;
  
  // Hitung error
  float error = setPoint - currentValue;
  
  // Bagian proporsional
  float proportional = Kp * error;
  
  // Bagian integral
  integral += Ki * error * deltaTime;
  // Anti windup - batasi nilai integral
  integral = constrain(integral, -1000, 1000); // Diperluas untuk skala 12-bit
  
  // Bagian derivatif
  float derivative = 0;
  if (deltaTime > 0) {
    derivative = Kd * (error - lastError) / deltaTime;
  }
  lastError = error;
  
  // Hitung output PID
  float output = proportional + integral + derivative;
  
  // KOREKSI: Buck converter, nilai PWM lebih tinggi = duty cycle lebih tinggi = tegangan output lebih tinggi
  // Jadi jika error positif (tegangan terlalu rendah), kita MENAMBAH PWM
  int pwmOutput = pwmValue + (int)output;
  
  return pwmOutput;
}

// Fungsi untuk memperbarui tampilan LCD tanpa kedipan
void updateLCDNoFlicker(int pwm, float pwmPercent) {
  // Perbarui baris 2 hanya jika nilai berubah signifikan
  if (abs(loadVoltage - prev_loadVoltage) > 0.01 || abs(outputVrms - prev_outputVrms) > 0.01) {
    sprintf(lcdLine2, "Vout:%.2fV Vref:%.2fV", loadVoltage, outputVrms);
    lcd.setCursor(0, 1);
    lcd.print(lcdLine2);
    prev_loadVoltage = loadVoltage;
    prev_outputVrms = outputVrms;
  }
  
  // Perbarui baris 3 hanya jika nilai berubah signifikan
  if (abs(current_mA - prev_current_mA) > 0.1) {
    sprintf(lcdLine3, "Iout:%.1fmA          ", current_mA);
    lcd.setCursor(0, 2);
    lcd.print(lcdLine3);
    prev_current_mA = current_mA;
  }
  
  // Perbarui baris 4 hanya jika nilai berubah signifikan
  if (pwm != prev_pwmValue || abs(pwmPercent - prev_pwmPercent) > 0.1) {
    sprintf(lcdLine4, "PWM:%d (%.1f%%)       ", pwm, pwmPercent);
    lcd.setCursor(0, 3);
    lcd.print(lcdLine4);
    prev_pwmValue = pwm;
    prev_pwmPercent = pwmPercent;
  }
}