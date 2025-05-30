/*
 * Solar Tracker 2-Axis Sequential untuk Indonesia
 * ARAH MOTOR DIPERBAIKI - motorUtara dan motorSelatan ditukar
 * 1. Kalibrasi: Timur-Barat-Utara-Selatan (15s each)
 * 2. Input: jam,menit,bulan (contoh: 12,30,6)
 * 3. Sequential tracking: Horizontal -> Vertikal
 * 4. Elevasi disesuaikan untuk Indonesia (latitude ~-6°)
 */

#include <Wire.h>
#include <MPU6050.h>

MPU6050 mpu;

// Variabel IMU - Accelerometer only
float roll = 0, pitch = 0;

// Pin motor horizontal dan vertikal
const int motorHPin1 = 8;
const int motorHPin2 = 9;
const int motorVPin1 = 6;
const int motorVPin2 = 7;

// Kalibrasi
unsigned long waktuMulai = 0;
byte stepKalibrasi = 0;
bool kalibrasiBeres = false;

// Nilai kalibrasi horizontal dan vertikal
float rollTimur, rollBarat;
float pitchUtara, pitchSelatan;
float rollRange = 0, pitchRange = 0;

// Tracking
bool sedangTracking = false;
byte faseTracking = 0; // 0=horizontal, 1=vertikal, 2=selesai
int jamTarget = 12;
int menitTarget = 0;
int bulanTarget = 6;
float rollTarget = 0, pitchTarget = 0;
const float toleransi = 0.5; // derajat
bool baruMulaiTracking = false; // Flag untuk memaksa gerakan minimal

// Elevasi matahari per bulan untuk Indonesia (latitude ~-6°)
const float elevasiBulan[12] = {
  67.5,  // Jan - Matahari tinggi (musim hujan)
  75.0,  // Feb 
  82.0,  // Mar - Mendekati zenith
  87.0,  // Apr - Sangat tinggi
  88.0,  // Mei - Puncak (hampir zenith)
  87.5,  // Jun - Masih sangat tinggi
  87.0,  // Jul
  82.0,  // Agu
  75.0,  // Sep
  67.5,  // Okt
  62.0,  // Nov - Mulai rendah
  65.0   // Des
};

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
  pinMode(motorVPin1, OUTPUT);
  pinMode(motorVPin2, OUTPUT);
  stopMotor();
  
  Serial.println(F("=== SOLAR TRACKER 2-AXIS INDONESIA ==="));
  Serial.println(F("ARAH MOTOR DIPERBAIKI"));
  Serial.println(F("Accelerometer only - Stabil & Akurat"));
  Serial.println(F("Sequential: Horizontal -> Vertikal"));
  Serial.println(F("Gerakan minimal 500ms untuk setiap perintah"));
  Serial.println(F("Kalibrasi: T-B-U-S 15s each"));
  Serial.println(F("Mapping: Step 15 menit (7:00-17:00)"));
  Serial.println(F("Elevasi: Per bulan untuk Indonesia"));
  Serial.println(F("Input: jam,menit,bulan (contoh: 12,30,6)"));
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
  
  // Hitung roll dan pitch dari accelerometer
  roll = atan2(-accX, sqrt(accY * accY + accZ * accZ)) * 57.3;
  pitch = atan2(accY, sqrt(accX * accX + accZ * accZ)) * 57.3;
  
  // Simple filter untuk kedua sumbu
  static float lastRoll = 0, lastPitch = 0;
  roll = 0.8 * lastRoll + 0.2 * roll;
  pitch = 0.8 * lastPitch + 0.2 * pitch;
  lastRoll = roll;
  lastPitch = pitch;
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
        nextStep();
      }
      break;
      
    case 2: // UTARA - 15 detik
      if (elapsed < 15000) {
        motorUtara();
        if (elapsed % 3000 < 200) {
          Serial.print(F("UTARA: "));
          Serial.print((15000 - elapsed) / 1000);
          Serial.print(F("s - Pitch: "));
          Serial.println(pitch, 1);
        }
      } else {
        stopMotor();
        pitchUtara = pitch;
        Serial.print(F("UTARA selesai - Pitch: "));
        Serial.println(pitchUtara, 1);
        nextStep();
      }
      break;
      
    case 3: // SELATAN - 15 detik
      if (elapsed < 15000) {
        motorSelatan();
        if (elapsed % 3000 < 200) {
          Serial.print(F("SELATAN: "));
          Serial.print((15000 - elapsed) / 1000);
          Serial.print(F("s - Pitch: "));
          Serial.println(pitch, 1);
        }
      } else {
        stopMotor();
        pitchSelatan = pitch;
        Serial.print(F("SELATAN selesai - Pitch: "));
        Serial.println(pitchSelatan, 1);
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
  // Hitung range horizontal dan vertikal
  rollRange = abs(rollTimur - rollBarat);
  pitchRange = abs(pitchUtara - pitchSelatan);
  
  Serial.println(F("\n=== MAPPING 2-AXIS ==="));
  Serial.print(F("Range Horizontal: "));
  Serial.print(rollRange, 1);
  Serial.println(F("°"));
  Serial.print(F("Range Vertikal: "));
  Serial.print(pitchRange, 1);
  Serial.println(F("°"));
  
  // Peringatan jika range vertikal terlalu kecil
  if (pitchRange < 5.0) {
    Serial.println(F(">>> PERINGATAN: Range vertikal kecil! <<<"));
    Serial.println(F(">>> Cek arah motor atau beban mekanis <<<"));
  }
  
  Serial.println(F("Elevasi per bulan (Indonesia):"));
  for (int bulan = 1; bulan <= 12; bulan++) {
    Serial.print(F("  Bulan "));
    if (bulan < 10) Serial.print(F("0"));
    Serial.print(bulan);
    Serial.print(F(": "));
    Serial.print(elevasiBulan[bulan-1], 1);
    Serial.println(F("°"));
  }
  Serial.println(F("========================"));
}

void bacaInputSerial() {
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim();
    
    int comma1 = input.indexOf(',');
    int comma2 = input.indexOf(',', comma1 + 1);
    
    if (comma1 > 0 && comma2 > 0) {
      // Format: jam,menit,bulan
      int jamBaru = input.substring(0, comma1).toInt();
      int menitBaru = input.substring(comma1 + 1, comma2).toInt();
      int bulanBaru = input.substring(comma2 + 1).toInt();
      
      // Validasi input
      if (jamBaru >= 7 && jamBaru <= 17 && menitBaru >= 0 && menitBaru <= 59 && bulanBaru >= 1 && bulanBaru <= 12) {
        // Bulatkan menit ke kelipatan 15
        int menitRounded = (menitBaru / 15) * 15;
        
        // Validasi batas waktu
        if (jamBaru == 17 && menitRounded > 0) {
          jamBaru = 17;
          menitRounded = 0;
        }
        
        jamTarget = jamBaru;
        menitTarget = menitRounded;
        bulanTarget = bulanBaru;
        
        // Hitung roll target berdasarkan jam dan menit
        float totalMenit = (jamTarget - 7) * 60 + menitTarget; // Menit dari jam 7:00
        rollTarget = map(totalMenit, 0, 600, rollBarat, rollTimur); // 600 = 10 jam dalam menit
        
        // Hitung pitch target berdasarkan bulan (skip jika range vertikal terlalu kecil)
        if (pitchRange >= 5.0) {
          float elevasiTarget = elevasiBulan[bulanTarget - 1];
          pitchTarget = map(elevasiTarget, 0, 90, pitchSelatan, pitchUtara);
        } else {
          pitchTarget = pitch; // Gunakan posisi saat ini
          Serial.println(F(">>> VERTIKAL SKIP (Range terlalu kecil) <<<"));
        }
        
        Serial.print(F("Target: "));
        if (jamTarget < 10) Serial.print(F("0"));
        Serial.print(jamTarget);
        Serial.print(F(":"));
        if (menitTarget < 10) Serial.print(F("0"));
        Serial.print(menitTarget);
        Serial.print(F(" Bulan "));
        Serial.print(bulanTarget);
        Serial.print(F(" (El:"));
        Serial.print(elevasiBulan[bulanTarget - 1], 1);
        Serial.println(F("°)"));
        
        Serial.print(F("Roll target: "));
        Serial.print(rollTarget, 1);
        Serial.print(F("° Pitch target: "));
        Serial.println(pitchTarget, 1);
        
        sedangTracking = true;
        faseTracking = 0; // Mulai dari horizontal
        baruMulaiTracking = true;
        Serial.println(F(">>> MULAI FASE HORIZONTAL <<<"));
      } else {
        Serial.println(F("Input tidak valid!"));
        Serial.println(F("Jam: 7-17, Menit: 0-59, Bulan: 1-12"));
      }
    } else {
      Serial.println(F("Format: jam,menit,bulan"));
      Serial.println(F("Contoh: 12,30,6 atau 15,45,3"));
    }
  }
}

void jalankanTracking() {
  if (faseTracking == 0) {
    // FASE HORIZONTAL
    float errorRoll = rollTarget - roll;
    
    if (baruMulaiTracking || abs(errorRoll) > toleransi) {
      if (baruMulaiTracking) {
        Serial.println(F(">>> GERAKAN HORIZONTAL DIPAKSA <<<"));
      }
      
      if (errorRoll > 0) {
        motorTimur();
        Serial.print(F("H>>Timur "));
      } else {
        motorBarat();
        Serial.print(F("H>>Barat "));
      }
      
      Serial.print(F("Target:"));
      Serial.print(rollTarget, 1);
      Serial.print(F(" Now:"));
      Serial.print(roll, 1);
      Serial.print(F(" Err:"));
      Serial.print(errorRoll, 2);
      
      if (baruMulaiTracking) {
        Serial.println(F(" [FORCED]"));
        delay(500);
        stopMotor();
        baruMulaiTracking = false;
        Serial.println(F(">>> GERAKAN HORIZONTAL SELESAI <<<"));
      } else {
        Serial.println();
      }
    } else {
      // Horizontal selesai, cek apakah perlu vertikal
      stopMotor();
      if (pitchRange >= 5.0) {
        faseTracking = 1;
        baruMulaiTracking = true; // Reset untuk fase vertikal
        Serial.println(F(">>> HORIZONTAL LOCK <<<"));
        Serial.println(F(">>> MULAI FASE VERTIKAL <<<"));
        delay(1000);
      } else {
        // Skip vertikal jika range terlalu kecil
        faseTracking = 2;
        sedangTracking = false;
        Serial.println(F(">>> HORIZONTAL LOCK <<<"));
        Serial.println(F(">>> VERTIKAL SKIP (Range kecil) <<<"));
        Serial.println(F(">>> TRACKING SELESAI <<<"));
        Serial.print(F("Panel menghadap jam "));
        if (jamTarget < 10) Serial.print(F("0"));
        Serial.print(jamTarget);
        Serial.print(F(":"));
        if (menitTarget < 10) Serial.print(F("0"));
        Serial.print(menitTarget);
        Serial.println(F(" (horizontal only)"));
        Serial.println(F("Input waktu baru untuk tracking lagi"));
      }
    }
  }
  else if (faseTracking == 1) {
    // FASE VERTIKAL
    float errorPitch = pitchTarget - pitch;
    
    if (baruMulaiTracking || abs(errorPitch) > toleransi) {
      if (baruMulaiTracking) {
        Serial.println(F(">>> GERAKAN VERTIKAL DIPAKSA <<<"));
      }
      
      if (errorPitch > 0) {
        motorUtara();
        Serial.print(F("V>>Utara "));
      } else {
        motorSelatan();
        Serial.print(F("V>>Selatan "));
      }
      
      Serial.print(F("Target:"));
      Serial.print(pitchTarget, 1);
      Serial.print(F(" Now:"));
      Serial.print(pitch, 1);
      Serial.print(F(" Err:"));
      Serial.print(errorPitch, 2);
      
      if (baruMulaiTracking) {
        Serial.println(F(" [FORCED]"));
        delay(500);
        stopMotor();
        baruMulaiTracking = false;
        Serial.println(F(">>> GERAKAN VERTIKAL SELESAI <<<"));
      } else {
        Serial.println();
      }
    } else {
      // Tracking selesai total
      stopMotor();
      faseTracking = 2;
      sedangTracking = false;
      Serial.println(F(">>> VERTIKAL LOCK <<<"));
      Serial.println(F(">>> TRACKING 2-AXIS SELESAI <<<"));
      Serial.print(F("Panel menghadap jam "));
      if (jamTarget < 10) Serial.print(F("0"));
      Serial.print(jamTarget);
      Serial.print(F(":"));
      if (menitTarget < 10) Serial.print(F("0"));
      Serial.print(menitTarget);
      Serial.print(F(" bulan "));
      Serial.println(bulanTarget);
      Serial.println(F("Input waktu baru untuk tracking lagi"));
    }
  }
}

void tampilkanStatus() {
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint > 2000) {
    lastPrint = millis();
    
    Serial.print(F("Roll: "));
    Serial.print(roll, 1);
    Serial.print(F("° Pitch: "));
    Serial.print(pitch, 1);
    Serial.print(F("° "));
    
    if (sedangTracking) {
      if (faseTracking == 0) Serial.println(F("|TRACK-H"));
      else Serial.println(F("|TRACK-V"));
    } else {
      Serial.println(F("|SIAP"));
    }
  }
}

void tampilkanHasilKalibrasi() {
  Serial.println(F("\n=== KALIBRASI 2-AXIS SELESAI ==="));
  Serial.print(F("HORIZONTAL - Timur: "));
  Serial.print(rollTimur, 1);
  Serial.print(F("° Barat: "));
  Serial.print(rollBarat, 1);
  Serial.print(F("° Range: "));
  Serial.print(rollRange, 1);
  Serial.println(F("°"));
  
  Serial.print(F("VERTIKAL - Utara: "));
  Serial.print(pitchUtara, 1);
  Serial.print(F("° Selatan: "));
  Serial.print(pitchSelatan, 1);
  Serial.print(F("° Range: "));
  Serial.print(pitchRange, 1);
  Serial.println(F("°"));
  
  Serial.println(F("=================================="));
  Serial.println(F("SIAP! Input: jam,menit,bulan"));
  Serial.println(F("Contoh: 12,30,6 / 9,15,3 / 15,45,9"));
  Serial.println(F("Sequential: Horizontal -> Vertikal"));
  Serial.println();
}

// Motor functions - ARAH DIPERBAIKI
void motorTimur() {
  digitalWrite(motorHPin1, LOW);
  digitalWrite(motorHPin2, HIGH);
  digitalWrite(motorVPin1, LOW);  // Pastikan vertikal OFF
  digitalWrite(motorVPin2, LOW);
}

void motorBarat() {
  digitalWrite(motorHPin1, HIGH);
  digitalWrite(motorHPin2, LOW);
  digitalWrite(motorVPin1, LOW);  // Pastikan vertikal OFF
  digitalWrite(motorVPin2, LOW);
}

// DIPERBAIKI: Tukar arah motorUtara dan motorSelatan
void motorUtara() {
  digitalWrite(motorVPin1, LOW);   // DIPERBAIKI: Ditukar
  digitalWrite(motorVPin2, HIGH);  // DIPERBAIKI: Ditukar
  digitalWrite(motorHPin1, LOW);   // Pastikan horizontal OFF
  digitalWrite(motorHPin2, LOW);
}

void motorSelatan() {
  digitalWrite(motorVPin1, HIGH);  // DIPERBAIKI: Ditukar
  digitalWrite(motorVPin2, LOW);   // DIPERBAIKI: Ditukar
  digitalWrite(motorHPin1, LOW);   // Pastikan horizontal OFF
  digitalWrite(motorHPin2, LOW);
}

void stopMotor() {
  digitalWrite(motorHPin1, LOW);
  digitalWrite(motorHPin2, LOW);
  digitalWrite(motorVPin1, LOW);
  digitalWrite(motorVPin2, LOW);
}