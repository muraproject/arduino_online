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
int menitTarget = 0;
float rollTarget = 0;
const float toleransi = 0.4; // derajat (lebih presisi untuk 15 menit)
bool baruMulaiTracking = false; // Flag untuk memaksa gerakan minimal

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
  Serial.println(F("Accelerometer only - Stabil & Akurat"));
  Serial.println(F("Horizontal tracking (Timur-Barat)"));
  Serial.println(F("Gerakan minimal 500ms untuk setiap perintah"));
  Serial.println(F("Kalibrasi: Timur 15s, Barat 15s"));
  Serial.println(F("Mapping: Step 15 menit (7:00-17:00)"));
  Serial.println(F("Input: jam,menit (contoh: 12,30)"));
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
  // Hitung range total dan per 15 menit
  rollRange = abs(rollTimur - rollBarat);
  float rollPer15Menit = rollRange / 40.0; // 40 interval 15-menit dari 7:00-17:00
  
  Serial.println(F("\n=== MAPPING SETIAP 15 MENIT ==="));
  Serial.print(F("Range total: "));
  Serial.print(rollRange, 1);
  Serial.println(F("°"));
  Serial.print(F("Per 15 menit: "));
  Serial.print(rollPer15Menit, 2);
  Serial.println(F("°"));
  
  // Tampilkan mapping setiap 15 menit (contoh beberapa)
  Serial.println(F("Contoh mapping:"));
  for (int jam = 7; jam <= 17; jam += 2) { // Tampilkan tiap 2 jam sebagai contoh
    for (int menit = 0; menit < 60; menit += 30) { // Tampilkan 00 dan 30 saja
      if (jam == 17 && menit > 0) break; // Stop di 17:00
      
      float totalMenit = (jam - 7) * 60 + menit; // Menit dari jam 7:00
      float rollWaktu = map(totalMenit, 0, 600, rollBarat, rollTimur); // 600 = 10 jam dalam menit
      
      Serial.print(F("  "));
      if (jam < 10) Serial.print(F("0"));
      Serial.print(jam);
      Serial.print(F(":"));
      if (menit < 10) Serial.print(F("0"));
      Serial.print(menit);
      Serial.print(F(" = Roll "));
      Serial.println(rollWaktu, 1);
    }
  }
  Serial.println(F("============================="));
}

void bacaInputSerial() {
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim();
    
    int commaIndex = input.indexOf(',');
    
    if (commaIndex > 0) {
      // Format: jam,menit
      int jamBaru = input.substring(0, commaIndex).toInt();
      int menitBaru = input.substring(commaIndex + 1).toInt();
      
      // Validasi input
      if (jamBaru >= 7 && jamBaru <= 17 && menitBaru >= 0 && menitBaru <= 59) {
        // Bulatkan menit ke kelipatan 15
        int menitRounded = (menitBaru / 15) * 15;
        
        // Validasi batas waktu
        if (jamBaru == 17 && menitRounded > 0) {
          jamBaru = 17;
          menitRounded = 0;
        }
        
        jamTarget = jamBaru;
        menitTarget = menitRounded;
        
        // Hitung roll target berdasarkan jam dan menit
        float totalMenit = (jamTarget - 7) * 60 + menitTarget; // Menit dari jam 7:00
        rollTarget = map(totalMenit, 0, 600, rollBarat, rollTimur); // 600 = 10 jam dalam menit
        
        Serial.print(F("Target waktu: "));
        if (jamTarget < 10) Serial.print(F("0"));
        Serial.print(jamTarget);
        Serial.print(F(":"));
        if (menitTarget < 10) Serial.print(F("0"));
        Serial.print(menitTarget);
        Serial.print(F(" - Roll target: "));
        Serial.println(rollTarget, 1);
        
        sedangTracking = true;
        baruMulaiTracking = true; // Flag untuk memaksa gerakan minimal
        Serial.println(F(">>> MULAI TRACKING <<<"));
      } else {
        Serial.println(F("Waktu harus 7:00-17:00 WIB!"));
      }
    } else {
      Serial.println(F("Format: jam,menit"));
      Serial.println(F("Contoh: 7,0 atau 12,30 atau 16,45"));
      Serial.println(F("Menit otomatis dibulatkan ke: 00, 15, 30, 45"));
    }
  }
}

void jalankanTracking() {
  float errorRoll = rollTarget - roll;
  
  // PAKSA GERAKAN MINIMAL 500ms saat baru mulai tracking
  if (baruMulaiTracking || abs(errorRoll) > toleransi) {
    
    if (baruMulaiTracking) {
      Serial.println(F(">>> GERAKAN MINIMAL DIPAKSA <<<"));
    }
    
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
    Serial.print(errorRoll, 2);
    
    if (baruMulaiTracking) {
      Serial.println(F(" [FORCED]"));
      // Gerakan minimal 500ms
      delay(100);
      stopMotor();
      baruMulaiTracking = false; // Reset flag setelah gerakan paksa
      Serial.println(F(">>> GERAKAN MINIMAL SELESAI <<<"));
    } else {
      Serial.println();
    }
    
  } else {
    // Target tercapai
    stopMotor();
    sedangTracking = false;
    baruMulaiTracking = false; // Reset flag
    Serial.println(F(">>> POSISI TERCAPAI - TERKUNCI <<<"));
    Serial.print(F("Panel menghadap posisi jam "));
    if (jamTarget < 10) Serial.print(F("0"));
    Serial.print(jamTarget);
    Serial.print(F(":"));
    if (menitTarget < 10) Serial.print(F("0"));
    Serial.print(menitTarget);
    Serial.println(F(" WIB"));
    Serial.println(F("Input waktu baru untuk tracking lagi"));
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
      Serial.print(F("-> "));
      if (jamTarget < 10) Serial.print(F("0"));
      Serial.print(jamTarget);
      Serial.print(F(":"));
      if (menitTarget < 10) Serial.print(F("0"));
      Serial.print(menitTarget);
      Serial.println(F(" |TRACKING"));
    } else {
      // Estimasi waktu berdasarkan posisi saat ini
      float totalMenitSekarang = map(roll, rollBarat, rollTimur, 0, 600);
      totalMenitSekarang = constrain(totalMenitSekarang, 0, 600);
      
      int jamSekarang = 7 + (int)(totalMenitSekarang / 60);
      int menitSekarang = ((int)totalMenitSekarang % 60) / 15 * 15; // Bulatkan ke 15 menit
      
      Serial.print(F("~ "));
      if (jamSekarang < 10) Serial.print(F("0"));
      Serial.print(jamSekarang);
      Serial.print(F(":"));
      if (menitSekarang < 10) Serial.print(F("0"));
      Serial.print(menitSekarang);
      Serial.println(F(" |SIAP"));
    }
  }
}

void tampilkanHasilKalibrasi() {
  Serial.println(F("\n=== KALIBRASI SELESAI ==="));
  Serial.print(F("TIMUR (07:00): Roll "));
  Serial.println(rollTimur, 1);
  Serial.print(F("BARAT (17:00): Roll "));
  Serial.println(rollBarat, 1);
  Serial.print(F("Range: "));
  Serial.print(rollRange, 1);
  Serial.print(F("° ("));
  Serial.print(rollPerJam, 1);
  Serial.println(F("°/jam)"));
  Serial.println(F("=========================="));
  Serial.println(F("SIAP! Input waktu: jam,menit"));
  Serial.println(F("Contoh: 7,0 / 10,15 / 12,30 / 15,45 / 17,0"));
  Serial.println(F("Menit otomatis dibulatkan: 00, 15, 30, 45"));
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