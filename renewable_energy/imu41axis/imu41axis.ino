/*
 * Solar Tracker 1-Axis - Mapping Jam Matahari
 * Fokus horizontal saja (Timur-Barat)
 * Kalibrasi: Bagi range menjadi 24 bagian untuk jam 6-18 WIB
 * Input: jam saja (6-18)
 */

#include <Wire.h>
#include <MPU6050.h>

MPU6050 mpu;

// Variabel IMU - Accelerometer only untuk stabilitas
float roll = 0;

// Pin motor horizontal saja
const int motorHPin1 = 8;
const int motorHPin2 = 9;

// Kalibrasi
unsigned long waktuMulai = 0;
byte stepKalibrasi = 0;
bool kalibrasiBeres = false;

// Nilai kalibrasi horizontal
float rollTimur, rollBarat;
float rollRange = 0;
float rollPerJam = 0;

// Tracking
bool sedangTracking = false;
int jamTarget = 12;
float rollTarget = 0;
const float toleransi = 1.0; // derajat

void setup() {
  Serial.begin(9600);
  Wire.begin();
  
  mpu.initialize();
  if (!mpu.testConnection()) {
    Serial.println(F("MPU6050 gagal"));
    while(1);
  }
  
  pinMode(motorHPin1, OUTPUT);
  pinMode(motorHPin2, OUTPUT);
  stopMotor();
  
  Serial.println(F("=== SOLAR TRACKER 1-AXIS ==="));
  Serial.println(F("Horizontal tracking (Timur-Barat)"));
  Serial.println(F("Kalibrasi: Timur 15s, Barat 15s"));
  Serial.println(F("Mapping: Jam 6-18 WIB (12 jam = 24 step)"));
  Serial.println(F("Input nanti: jam (6-18)"));
  Serial.println();
  
  waktuMulai = millis();
}

void loop() {
  bacaIMU();
  
  if (!kalibrasiBeres) {
    jalankanKalibrasi();
  } else {
    bacaInputSerial();
    if (sedangTracking) {
      jalankanTracking();
    }
    tampilkanStatus();
  }
  
  delay(100);
}

void bacaIMU() {
  int16_t ax, ay, az;
  mpu.getAcceleration(&ax, &ay, &az);
  
  float accX = ax / 16384.0;
  float accY = ay / 16384.0;
  float accZ = az / 16384.0;
  
  // Hitung roll dari accelerometer
  roll = atan2(-accX, sqrt(accY * accY + accZ * accZ)) * 57.3;
  
  // Simple filter untuk stabilitas
  static float lastRoll = 0;
  roll = 0.8 * lastRoll + 0.2 * roll;
  lastRoll = roll;
}

void jalankanKalibrasi() {
  unsigned long elapsed = millis() - waktuMulai;
  
  switch (stepKalibrasi) {
    case 0: // TIMUR - 15 detik
      if (elapsed < 15000) {
        motorTimur();
        if (elapsed % 3000 < 200) {
          Serial.print(F("TIMUR: "));
          Serial.print((15000 - elapsed) / 1000);
          Serial.print(F("s - Roll: "));
          Serial.println(roll, 1);
        }
      } else {
        stopMotor();
        rollTimur = roll;
        Serial.print(F("TIMUR selesai - Roll: "));
        Serial.println(rollTimur, 1);
        nextStep();
      }
      break;
      
    case 1: // BARAT - 15 detik
      if (elapsed < 15000) {
        motorBarat();
        if (elapsed % 3000 < 200) {
          Serial.print(F("BARAT: "));
          Serial.print((15000 - elapsed) / 1000);
          Serial.print(F("s - Roll: "));
          Serial.println(roll, 1);
        }
      } else {
        stopMotor();
        rollBarat = roll;
        Serial.print(F("BARAT selesai - Roll: "));
        Serial.println(rollBarat, 1);
        kalibrasiBeres = true;
        hitungMappingJam();
        tampilkanHasilKalibrasi();
      }
      break;
  }
}

void nextStep() {
  stepKalibrasi++;
  waktuMulai = millis();
  delay(1000);
}

void hitungMappingJam() {
  // Hitung range total dan per jam
  rollRange = abs(rollTimur - rollBarat);
  rollPerJam = rollRange / 12.0; // 12 jam dari 6-18
  
  Serial.println(F("\n=== MAPPING JAM MATAHARI ==="));
  Serial.print(F("Range total: "));
  Serial.print(rollRange, 1);
  Serial.println(F("°"));
  Serial.print(F("Per jam: "));
  Serial.print(rollPerJam, 1);
  Serial.println(F("°"));
  
  // Tampilkan mapping setiap jam
  for (int jam = 6; jam <= 18; jam++) {
    float rollJam = map(jam, 6, 18, rollBarat, rollTimur);
    Serial.print(F("Jam "));
    if (jam < 10) Serial.print(F("0"));
    Serial.print(jam);
    Serial.print(F(":00 = Roll "));
    Serial.println(rollJam, 1);
  }
  Serial.println(F("============================="));
}

void bacaInputSerial() {
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim();
    
    int jamBaru = input.toInt();
    
    if (jamBaru >= 6 && jamBaru <= 18) {
      jamTarget = jamBaru;
      
      // Hitung roll target berdasarkan jam
      rollTarget = map(jamTarget, 6, 18, rollBarat, rollTimur);
      
      Serial.print(F("Target jam: "));
      if (jamTarget < 10) Serial.print(F("0"));
      Serial.print(jamTarget);
      Serial.print(F(":00 - Roll target: "));
      Serial.println(rollTarget, 1);
      
      sedangTracking = true;
      Serial.println(F(">>> MULAI TRACKING <<<"));
    } else {
      Serial.println(F("Jam harus 6-18 WIB!"));
      Serial.println(F("Contoh: 6, 12, 15, 18"));
    }
  }
}

void jalankanTracking() {
  float errorRoll = rollTarget - roll;
  
  if (abs(errorRoll) > toleransi) {
    if (errorRoll > 0) {
      motorTimur();
      Serial.print(F(">>Timur "));
    } else {
      motorBarat();
      Serial.print(F(">>Barat "));
    }
    
    Serial.print(F("Target:"));
    Serial.print(rollTarget, 1);
    Serial.print(F(" Now:"));
    Serial.print(roll, 1);
    Serial.print(F(" Err:"));
    Serial.println(errorRoll, 1);
  } else {
    // Target tercapai
    stopMotor();
    sedangTracking = false;
    Serial.println(F(">>> POSISI TERCAPAI - TERKUNCI <<<"));
    Serial.print(F("Panel menghadap posisi jam "));
    if (jamTarget < 10) Serial.print(F("0"));
    Serial.print(jamTarget);
    Serial.println(F(":00 WIB"));
    Serial.println(F("Input jam baru untuk tracking lagi"));
  }
}

void tampilkanStatus() {
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint > 2000) {
    lastPrint = millis();
    
    Serial.print(F("Roll: "));
    Serial.print(roll, 1);
    Serial.print(F("° "));
    
    if (sedangTracking) {
      Serial.print(F("-> Jam "));
      if (jamTarget < 10) Serial.print(F("0"));
      Serial.print(jamTarget);
      Serial.println(F(" |TRACKING"));
    } else {
      // Estimasi jam berdasarkan posisi saat ini
      int jamSekarang = map(roll, rollBarat, rollTimur, 6, 18);
      jamSekarang = constrain(jamSekarang, 6, 18);
      Serial.print(F("~ Jam "));
      if (jamSekarang < 10) Serial.print(F("0"));
      Serial.print(jamSekarang);
      Serial.println(F(" |SIAP"));
    }
  }
}

void tampilkanHasilKalibrasi() {
  Serial.println(F("\n=== KALIBRASI SELESAI ==="));
  Serial.print(F("TIMUR (06:00): Roll "));
  Serial.println(rollTimur, 1);
  Serial.print(F("BARAT (18:00): Roll "));
  Serial.println(rollBarat, 1);
  Serial.print(F("Range: "));
  Serial.print(rollRange, 1);
  Serial.print(F("° ("));
  Serial.print(rollPerJam, 1);
  Serial.println(F("°/jam)"));
  Serial.println(F("=========================="));
  Serial.println(F("SIAP! Input jam: 6-18"));
  Serial.println(F("Contoh: 6, 9, 12, 15, 18"));
  Serial.println();
}

// Motor functions - hanya horizontal
void motorTimur() {
  digitalWrite(motorHPin1, LOW);
  digitalWrite(motorHPin2, HIGH);
}

void motorBarat() {
  digitalWrite(motorHPin1, HIGH);
  digitalWrite(motorHPin2, LOW);
}

void stopMotor() {
  digitalWrite(motorHPin1, LOW);
  digitalWrite(motorHPin2, LOW);
}