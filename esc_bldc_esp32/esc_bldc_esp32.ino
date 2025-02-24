#include <ESP32Servo.h>

Servo ESC;
const int escPin = 13;

// Konstanta untuk ESC dengan motor high KV
const int MIN_PULSE = 1500;     // Sinyal minimum
const int MAX_PULSE = 2000;     // Sinyal maximum
const int START_PULSE = 1040;   // Mulai dengan pulse sangat rendah karena KV tinggi
const int SAFE_MAX = 1200;      // Batas aman untuk testing

void setup() {
  Serial.begin(115200);
  Serial.println("Inisialisasi ESC untuk motor High KV...");
  
  // Konfigurasi PWM ESP32
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  pinMode(2, OUTPUT);
  // Attach ESC
  ESC.attach(escPin, MIN_PULSE, MAX_PULSE);
  
  // Prosedur arming spesifik untuk motor high KV
  Serial.println("Memulai prosedur arming...");
  
  // Step 1: Pastikan throttle di posisi minimum
  ESC.writeMicroseconds(MIN_PULSE);
  delay(3000);
  
  // Step 2: Kirim pulse maksimum sebentar
  digitalWrite(2, HIGH);
  ESC.writeMicroseconds(2100);
  delay(1000);
  
  digitalWrite(2, 0);
  ESC.writeMicroseconds(1950);
  delay(1000);

  digitalWrite(2, 1);
  ESC.writeMicroseconds(1850);
  delay(1000);

  digitalWrite(2, 0);
  ESC.writeMicroseconds(1750);
  delay(1000);

  digitalWrite(2, 1);
  ESC.writeMicroseconds(1650);
  delay(1000);

  digitalWrite(2, 0);
  ESC.writeMicroseconds(1600);
  delay(1000);

  digitalWrite(2, 1);
  ESC.writeMicroseconds(1550);
  delay(1000);

 
  // // Step 3: Kembali ke minimum, tunggu beep
  // ESC.writeMicroseconds(MIN_PULSE);
  // delay(2000);
  

  // // Step 2: Kirim pulse maksimum sebentar
  // ESC.writeMicroseconds(MAX_PULSE);
  // delay(2000);
  
  // // Step 3: Kembali ke minimum, tunggu beep
  // ESC.writeMicroseconds(MIN_PULSE);
  // delay(5000);
  Serial.println("Arming selesai!");
}

void loop() {
  // Test sequence sangat pelan untuk motor high KV
  
  // 1. Mulai dengan pulse sangat rendah
  // Serial.println("Test awal - pulse sangat rendah");
  // ESC.writeMicroseconds(START_PULSE);
  // delay(4000);
  
  // // 2. Stop
  // Serial.println("Stop");
  // ESC.writeMicroseconds(MIN_PULSE);
  // delay(3000);
  
  // // 3. Coba sedikit lebih tinggi
  // Serial.println("Test - pulse sedikit lebih tinggi");
  // ESC.writeMicroseconds(START_PULSE + 10);
  // delay(4000);
  
  // // 4. Stop lagi
  // Serial.println("Stop");
  // ESC.writeMicroseconds(MIN_PULSE);
  // delay(3000);
  
  // // 5. Tes dengan pulse lebih tinggi
  // Serial.println("Test - pulse medium");
  // ESC.writeMicroseconds(START_PULSE + 20);
  // delay(4000);
  
  // // 6. Kembali ke minimum
  // Serial.println("Kembali ke minimum");
  // ESC.writeMicroseconds(MIN_PULSE);
  // delay(5000);
}