/*
 * Program Arduino Kendali Buck Converter 0-5V dengan PID
 * - Sensor: Pembagi tegangan 10k-10k di pin A0
 * - Output PWM: Pin 9
 * - Kontrol: Serial input untuk set tegangan target
 */

// Definisi pin
const int sensorPin = A0;    // Pin sensor pembagi tegangan
const int pwmPin = 9;        // Pin output PWM untuk buck converter

// Parameter PID
double Kp = 5.0;             // Konstanta Proportional
double Ki = 0.5;             // Konstanta Integral
double Kd = 0.5;             // Konstanta Derivative

// Variabel PID
double setpoint = 0.95;       // Nilai tegangan default yang diinginkan (0-5V)
double input = 0.0;          // Nilai tegangan terukur saat ini
double output = 0.0;         // Nilai output ke PWM (0-255)
double error = 0.0;          // Selisih antara setpoint dan input
double lastError = 0.0;      // Error pada iterasi sebelumnya
double errorSum = 0.0;       // Akumulasi error (untuk integral)
double errorDelta = 0.0;     // Perubahan error (untuk derivative)

// Batasan waktu
unsigned long lastTime = 0;
const unsigned long sampleTime = 50;  // Waktu sampling dalam ms

// Variabel komunikasi serial
String inputString = "";        // String untuk menyimpan input
boolean stringComplete = false; // Flag untuk menandakan input selesai

void setup() {
  // Inisialisasi komunikasi serial
  Serial.begin(9600);
  
  // Inisialisasi pin
  pinMode(pwmPin, OUTPUT);
  pinMode(sensorPin, INPUT);
  
  // Setel PWM ke nilai awal
  analogWrite(pwmPin, 0);
  
  // Inisialisasi string
  inputString.reserve(10);
  
  // Pesan pembuka
  Serial.println("Buck Converter Control 0-5V");
  Serial.println("Masukkan nilai tegangan target (0-5V):");
}

void loop() {
  // Baca nilai sensor
  int sensorValue = analogRead(sensorPin);
  
  // Konversi nilai ADC (0-1023) ke tegangan (0-5V)
  // Dengan pembagi tegangan 10k-10k, tegangan input = 2 * tegangan yang terbaca
  input = (sensorValue / 1023.0) * 5.0 * 2.0;
  
  // Waktu saat ini
  unsigned long now = millis();
  
  // Hitung PID pada interval tertentu
  if (now - lastTime >= sampleTime) {
    // Hitung PID
    computePID();
    
    // Terapkan output ke PWM
    analogWrite(pwmPin, output);
    
    // Update waktu terakhir
    lastTime = now;
    
    // Tampilkan informasi
    Serial.print("Target: ");
    Serial.print(setpoint);
    Serial.print("V, Terukur: ");
    Serial.print(input);
    Serial.print("V, PWM: ");
    Serial.println(output);
  }
  
  // Proses input serial jika ada
  if (stringComplete) {
    // Konversi string ke float
    float newSetpoint = inputString.toFloat();
    
    // Validasi input
    if (newSetpoint >= 0.0 && newSetpoint <= 5.0) {
      setpoint = newSetpoint;
      Serial.print("Target tegangan diubah ke: ");
      Serial.println(setpoint);
      
      // Reset error untuk PID
      errorSum = 0;
    } else {
      Serial.println("Error: Nilai harus antara 0-5V");
    }
    
    // Reset string dan flag
    inputString = "";
    stringComplete = false;
  }
  delay(50);
}

// Fungsi untuk menghitung PID
void computePID() {
  // Hitung error
  error = setpoint - input;
  
  // Hitung komponen P, I, dan D
  double P = Kp * error;
  
  // Akumulasi error untuk komponen I dengan anti-windup (batasi errorSum)
  errorSum += error;
  errorSum = constrain(errorSum, -100, 100);  // Anti-windup
  double I = Ki * errorSum;
  
  // Hitung perubahan error untuk komponen D
  errorDelta = error - lastError;
  double D = Kd * errorDelta;
  
  // Hitung output PID
  output = P + I + D;
  
  // Batasi output antara 0-255 (range PWM)
  output = constrain(output, 0, 255);
  
  // Simpan error saat ini untuk iterasi berikutnya
  lastError = error;
}

// Fungsi event untuk serial
void serialEvent() {
  while (Serial.available()) {
    // Baca karakter yang masuk
    char inChar = (char)Serial.read();
    
    // Jika new line, set flag stringComplete
    if (inChar == '\n') {
      stringComplete = true;
    } else {
      // Tambahkan karakter ke string
      inputString += inChar;
    }
  }
}