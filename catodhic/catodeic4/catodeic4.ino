/*
 * Program ESP32 untuk Mengukur VRMS dari Buck Converter
 * - Menggunakan pembagi tegangan 10k-10k di pin 34 (ADC1_CH6)
 * - PWM output di pin 5 dengan resolusi 12-bit (0-4095)
 * - Menggunakan PID untuk menjaga tegangan stabil di set point
 * - Menampilkan data pada LCD 20x4 dengan I2C
 */

#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// Inisialisasi LCD (alamat I2C 0x27, 20 kolom, 4 baris)
LiquidCrystal_I2C lcd(0x27, 20, 4);

// Definisi pin
const int sensorPin = 34;    // Pin sensor analog ESP32 (ADC1_CH6)
const int pwmPin = 5;        // Pin output PWM untuk buck converter

// Definisi pin I2C untuk ESP32 (opsional, karena Wire.begin() tanpa parameter akan menggunakan pin default)
const int SDA_PIN = 21;      // Default SDA pin untuk ESP32
const int SCL_PIN = 22;      // Default SCL pin untuk ESP32

// Definisi PWM untuk ESP32
const int PWM_CHANNEL = 0;   // PWM channel (0-15)
const int PWM_FREQ = 5000;  // Frekuensi PWM (25kHz)
const int PWM_RESOLUTION = 12; // Resolusi PWM 12-bit (0-4095)

// Variabel untuk pengukuran
int pwmValue = 800;          // Nilai PWM awal (0-4095 untuk 12-bit)
float vRef = 3.3;            // Tegangan referensi ESP32 (3.3V)
int numSamples = 100;        // Jumlah sampel untuk perhitungan RMS

// Parameter PID
float setPoint = 0.93;       // Set point tegangan output yang diinginkan (V)
float Kp = 100.0;            // Konstanta Proporsional (dinaikkan untuk 12-bit range)
float Ki = 20.0;             // Konstanta Integral (dinaikkan untuk 12-bit range)
float Kd = 10.0;             // Konstanta Derivatif (dinaikkan untuk 12-bit range)

// Batas PWM (sesuaikan dengan resolusi 12-bit)
const int PWM_MIN = 100;     // Nilai minimum PWM
const int PWM_MAX = 4000;    // Nilai maksimum PWM

// Variabel untuk kalkulasi PID
float lastError = 0;         // Error sebelumnya
float integral = 0;          // Nilai integral error
unsigned long lastTime = 0;  // Waktu perhitungan PID terakhir
float outputVrms = 0;        // Nilai VRMS output terakhir

// Variabel untuk update LCD
unsigned long lastLcdUpdate = 0;
const long lcdUpdateInterval = 500; // Update LCD setiap 500ms

void setup() {
  // Inisialisasi komunikasi serial
  Serial.begin(9600);
  
  // Inisialisasi I2C dengan pin khusus ESP32 (opsional)
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
  
  // Tampilkan pesan pembuka di LCD
  lcd.setCursor(0, 0);
  lcd.print("Buck Converter VRMS");
  lcd.setCursor(0, 1);
  lcd.print("Set Point: ");
  lcd.print(setPoint);
  lcd.print("V");
  lcd.setCursor(0, 2);
  lcd.print("12-bit PWM (0-4095)");
  lcd.setCursor(0, 3);
  lcd.print("PWM Freq: 25kHz");
  delay(2000); // Tahan pesan pembuka selama 2 detik
  
  // Pesan pembuka di Serial
  Serial.println("Pengukuran VRMS Buck Converter dengan Pembagi Tegangan 10k-10k & Kontrol PID");
  Serial.println("Resolusi PWM: 12-bit (0-4095)");
  Serial.println("Frekuensi PWM: 25kHz");
  Serial.println("Set Point: " + String(setPoint) + "V");
  Serial.println("PID Parameters: Kp=" + String(Kp) + ", Ki=" + String(Ki) + ", Kd=" + String(Kd));
  Serial.println("PWM Range: Min=" + String(PWM_MIN) + ", Max=" + String(PWM_MAX));
  Serial.println("----------------------------------------------------------");
  Serial.println("PWM, ADC Raw, Voltage Pin, Output Voltage, VRMS, Error");
  
  // Inisialisasi waktu
  lastTime = millis();
  lastLcdUpdate = millis();
  
  // Siapkan tampilan LCD
  updateLCD(pwmValue, 0, 0, 0);
}

void loop() {
  // Variabel untuk menyimpan total kuadrat
  float sumSquares = 0.0;
  int rawADC = 0;
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
      rawADC = adcValue;
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
  
  // Hitung error
  float error = setPoint - vrms;
  
  // Tampilkan hasil di Serial
  Serial.print(pwmValue);
  Serial.print(", ");
  Serial.print(rawADC);
  Serial.print(", ");
  Serial.print(voltagePin, 4);
  Serial.print(", ");
  Serial.print(outputVoltage, 4);
  Serial.print(", ");
  Serial.print(vrms, 4);
  Serial.print(", ");
  Serial.println(error, 4);
  
  // Update LCD jika waktunya
  unsigned long currentMillis = millis();
  if (currentMillis - lastLcdUpdate >= lcdUpdateInterval) {
    lastLcdUpdate = currentMillis;
    updateLCD(pwmValue, vrms, error, rawADC);
  }
  
  // Tunggu sebelum pengukuran berikutnya
  delay(100);
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

// Fungsi untuk memperbarui LCD
void updateLCD(int pwm, float vrms, float error, int adc) {
  // Baris 1: PWM dan VRMS
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("PWM: ");
  lcd.print(pwm);
  // Hitung persentase PWM dari nilai 12-bit
  float pwmPercent = (pwm / 4095.0) * 100.0;
  lcd.print(" (");
  lcd.print(pwmPercent, 1);
  lcd.print("%)");
  
  // Baris 2: VRMS dan Set Point
  lcd.setCursor(0, 1);
  lcd.print("VRMS:");
  lcd.print(vrms, 3);
  lcd.print("V ");
  lcd.setCursor(12, 1);
  lcd.print("SP:");
  lcd.print(setPoint, 2);
  lcd.print("V");
  
  // Baris 3: Error dan ADC Raw Value
  lcd.setCursor(0, 2);
  lcd.print("Err:");
  lcd.print(error, 3);
  lcd.setCursor(12, 2);
  lcd.print("ADC:");
  lcd.print(adc);
  
  // Baris 4: PID Parameters
  lcd.setCursor(0, 3);
  lcd.print("P:");
  lcd.print(Kp, 0);
  lcd.setCursor(6, 3);
  lcd.print("I:");
  lcd.print(Ki, 0);
  lcd.setCursor(12, 3);
  lcd.print("D:");
  lcd.print(Kd, 0);
}