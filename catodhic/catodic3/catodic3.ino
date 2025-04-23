/*
 * Program ESP32 untuk Mengukur VRMS dari Buck Converter
 * - Menggunakan pembagi tegangan 10k-10k di pin 34 (ADC1_CH6)
 * - PWM output di pin 5
 * - Menggunakan PID untuk menjaga tegangan stabil di set point
 */

// Definisi pin
const int sensorPin = 34;    // Pin sensor analog ESP32 (ADC1_CH6)
const int pwmPin = 5;        // Pin output PWM untuk buck converter

// Variabel untuk pengukuran
int pwmValue = 20;           // Nilai PWM awal (0-255)
float vRef = 3.3;            // Tegangan referensi ESP32 (3.3V)
int numSamples = 100;        // Jumlah sampel untuk perhitungan RMS

// Parameter PID
float setPoint = 0.93;        // Set point tegangan output yang diinginkan (V)
float Kp = 5.0;              // Konstanta Proporsional
float Ki = 1.0;              // Konstanta Integral
float Kd = 0.5;              // Konstanta Derivatif

// Batas PWM
const int PWM_MIN = 3;       // Nilai minimum PWM
const int PWM_MAX = 100;     // Nilai maksimum PWM

// Variabel untuk kalkulasi PID
float lastError = 0;         // Error sebelumnya
float integral = 0;          // Nilai integral error
unsigned long lastTime = 0;  // Waktu perhitungan PID terakhir
float outputVrms = 0;        // Nilai VRMS output terakhir

void setup() {
  // Inisialisasi komunikasi serial
  Serial.begin(9600);
  
  // Inisialisasi pin
  pinMode(sensorPin, INPUT);
  pinMode(pwmPin, OUTPUT);
  
  // Setel PWM ke nilai awal
  analogWrite(pwmPin, pwmValue);
  
  // Pesan pembuka
  Serial.println("Pengukuran VRMS Buck Converter dengan Pembagi Tegangan 10k-10k & Kontrol PID");
  Serial.println("Set Point: " + String(setPoint) + "V");
  Serial.println("PID Parameters: Kp=" + String(Kp) + ", Ki=" + String(Ki) + ", Kd=" + String(Kd));
  Serial.println("PWM Range: Min=" + String(PWM_MIN) + ", Max=" + String(PWM_MAX));
  Serial.println("----------------------------------------------------------");
  Serial.println("PWM, ADC Raw, Voltage Pin, Output Voltage, VRMS, Error");
  
  // Inisialisasi waktu
  lastTime = millis();
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
  
  // Terapkan nilai PWM baru
  analogWrite(pwmPin, pwmValue);
  
  // Hitung error
  float error = setPoint - vrms;
  
  // Tampilkan hasil
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
  integral = constrain(integral, -50, 50);
  
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