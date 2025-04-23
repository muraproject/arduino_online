/*
 * Program Arduino untuk Mengukur VRMS dari Buck Converter
 * - Menggunakan pembagi tegangan 10k-10k di pin A0
 * - PWM output di pin 9
 * - Nilai PWM tetap untuk pengujian
 */

// Definisi pin
const int sensorPin = A0;    // Pin sensor pembagi tegangan
const int pwmPin = 9;        // Pin output PWM untuk buck converter

// Variabel untuk pengukuran
int pwmValue = 40;           // Nilai PWM konstan (0-255)
float vRef = 5.0;            // Tegangan referensi Arduino (5V)
int numSamples = 100;        // Jumlah sampel untuk perhitungan RMS

void setup() {
  // Inisialisasi komunikasi serial
  Serial.begin(9600);
  
  // Inisialisasi pin
  pinMode(pwmPin, OUTPUT);
  pinMode(sensorPin, INPUT);
  
  // Setel PWM ke nilai awal
  analogWrite(pwmPin, pwmValue);
  
  // Pesan pembuka
  Serial.println("Pengukuran VRMS Buck Converter dengan Pembagi Tegangan 10k-10k");
  Serial.println("PWM Value: " + String(pwmValue));
  Serial.println("----------------------------------------------------------");
  Serial.println("ADC Raw, Voltage at A0, Output Voltage, VRMS");
}

void loop() {
  // Variabel untuk menyimpan total kuadrat
  float sumSquares = 0.0;
  int rawADC = 0;
  float voltageA0 = 0.0;
  float outputVoltage = 0.0;
  
  // Ambil beberapa sampel untuk perhitungan RMS
  for (int i = 0; i < numSamples; i++) {
    // Baca nilai ADC
    int adcValue = analogRead(sensorPin);
    
    // Konversi ke tegangan di pin A0
    float voltage = (adcValue / 1023.0) * vRef;
    
    // Kompensasi pembagi tegangan untuk mendapatkan tegangan output sebenarnya
    float actualVoltage = voltage * 2.0;  // Karena pembagi tegangan 10k-10k (1:1)
    
    // Tambahkan kuadrat ke total
    sumSquares += actualVoltage * actualVoltage;
    
    // Simpan nilai terakhir untuk tampilan
    if (i == numSamples - 1) {
      rawADC = adcValue;
      voltageA0 = voltage;
      outputVoltage = actualVoltage;
    }
    
    // Tunggu sedikit sebelum sampel berikutnya
    delayMicroseconds(100);
  }
  
  // Hitung VRMS
  float vrms = sqrt(sumSquares / numSamples);
  
  // Tampilkan hasil
  Serial.print(rawADC);
  Serial.print(", ");
  Serial.print(voltageA0, 4);
  Serial.print(", ");
  Serial.print(outputVoltage, 4);
  Serial.print(", ");
  Serial.println(vrms-0.3, 4);
  
  // Tunggu sebelum pengukuran berikutnya
  delay(100);
}