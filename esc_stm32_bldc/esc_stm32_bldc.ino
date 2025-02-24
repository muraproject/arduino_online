#include <Servo.h>

Servo ESC;
const int escPin = PA0; // Sesuaikan pin dengan STM32 Nucleo Anda

// Konstanta untuk ESC dengan motor high KV
const int MIN_PULSE = 1500;     // Sinyal minimum
const int MAX_PULSE = 2000;     // Sinyal maximum 
const int START_PULSE = 1040;   // Mulai dengan pulse sangat rendah karena KV tinggi
const int SAFE_MAX = 1200;      // Batas aman untuk testing

void setup() {
  Serial.begin(115200);
  Serial.println("Inisialisasi ESC untuk motor High KV...");
  
  // Setup pin LED built-in
  pinMode(LED_BUILTIN, OUTPUT);
  
  // Attach ESC ke pin PWM
  ESC.attach(escPin, MIN_PULSE, MAX_PULSE);
  
  // Prosedur arming spesifik untuk motor high KV
  Serial.println("Memulai prosedur arming...");
  
  // Step 1: Pastikan throttle di posisi minimum
  ESC.writeMicroseconds(MIN_PULSE);
  delay(3000);
  
  // Step 2: Sequence arming
  digitalWrite(LED_BUILTIN, HIGH);
  ESC.writeMicroseconds(2100);
  delay(1000);
  
  digitalWrite(LED_BUILTIN, LOW);
  ESC.writeMicroseconds(1950);
  delay(1000);

  digitalWrite(LED_BUILTIN, HIGH);
  ESC.writeMicroseconds(1850);
  delay(1000);

  digitalWrite(LED_BUILTIN, LOW);
  ESC.writeMicroseconds(1750);
  delay(1000);

  digitalWrite(LED_BUILTIN, HIGH);
  ESC.writeMicroseconds(1650);
  delay(1000);

  digitalWrite(LED_BUILTIN, LOW);
  ESC.writeMicroseconds(1600);
  delay(1000);

  digitalWrite(LED_BUILTIN, HIGH);
  ESC.writeMicroseconds(1000);
  delay(1000);

  Serial.println("Arming selesai!");
}

void loop() {
  // Loop kosong karena sequence hanya pada setup
}