/*
 * Solar Tracker dengan Kalibrasi IMU dan Input Waktu
 * 1. Kalibrasi otomatis saat startup
 * 2. Input waktu: jam,menit,tanggal,bulan 
 * 3. Bergerak ke posisi matahari dan terkunci
 */

#include <Wire.h>
#include <MPU6050.h>
#include <math.h>

MPU6050 mpu;

// Variabel IMU (accelerometer only)
float pitch = 0, roll = 0;

// Pin motor
const int motorHPin1 = 8;
const int motorHPin2 = 9;
const int motorVPin1 = 6;
const int motorVPin2 = 7;

// Variabel sequence kalibrasi
unsigned long waktuMulai = 0;
byte stepKalibrasi = 0;
bool kalibrasiBeres = false;
bool sedangTracking = false;

// Nilai kalibrasi
float pitchTimur, rollTimur;
float pitchBarat, rollBarat; 
float pitchUtara, rollUtara;
float pitchSelatan, rollSelatan;

// Variabel waktu dan posisi matahari
int jam = 12, menit = 0, tanggal = 15, bulan = 6;
float azimuthTarget = 0;
float elevasiTarget = 0;
float targetRoll = 0;      // Target roll berdasarkan proporsi
float targetPitch = 0;     // Target pitch berdasarkan proporsi
const float toleransi = 2.0;   // Toleransi untuk lock

// Variabel tracking sequential
bool horizontalSelesai = false;
bool vertikalSelesai = false;
unsigned long waktuMulaiStep = 0;
const unsigned long maxWaktuPerStep = 15000; // 15 detik per step

// Koordinat lokasi (Jakarta)
const float latitude = -6.2088;
const float longitude = 106.8456;

void setup() {
  Serial.begin(9600);
  Wire.begin();
  
  mpu.initialize();
  if (!mpu.testConnection()) {
    Serial.println(F("MPU6050 failed"));
    while(1);
  }
  
  pinMode(motorHPin1, OUTPUT);
  pinMode(motorHPin2, OUTPUT);
  pinMode(motorVPin1, OUTPUT);
  pinMode(motorVPin2, OUTPUT);
  stopMotor();
  
  Serial.println(F("=== SOLAR TRACKER PROPORSI WAKTU ==="));
  Serial.println(F("Sistem sederhana tanpa rumus astronomi"));
  Serial.println(F("Jam: 6 pagi=Timur, 6 sore=Barat"));
  Serial.println(F("Bulan: Jan/Des=Selatan, Jun/Jul=Utara"));
  Serial.println(F("Memulai kalibrasi otomatis..."));
  Serial.println(F("Durasi: 15 detik per arah (Total 60 detik)"));
  Serial.println(F("Setelah selesai, input: jam,menit,tanggal,bulan"));
  Serial.println(F("Contoh: 14,30,15,6"));
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
  
  pitch = atan2(accY, sqrt(accX * accX + accZ * accZ)) * 57.3;
  roll = atan2(-accX, sqrt(accY * accY + accZ * accZ)) * 57.3;
  
  // Low-pass filter
  static float lastPitch = 0, lastRoll = 0;
  pitch = 0.9 * lastPitch + 0.1 * pitch;
  roll = 0.9 * lastRoll + 0.1 * roll;
  lastPitch = pitch;
  lastRoll = roll;
}

void jalankanKalibrasi() {
  unsigned long elapsed = millis() - waktuMulai;
  
  switch (stepKalibrasi) {
    case 0: // TIMUR
      if (elapsed < 15000) {
        motorTimur();
        if (elapsed % 3000 < 200) {
          Serial.print(F("Kalibrasi TIMUR: "));
          Serial.println((15000 - elapsed) / 1000);
        }
      } else {
        stopMotor();
        pitchTimur = pitch;
        rollTimur = roll;
        Serial.print(F("TIMUR selesai - P:"));
        Serial.print(pitch, 1);
        Serial.print(F(" R:"));
        Serial.println(roll, 1);
        nextStep();
      }
      break;
      
    case 1: // BARAT
      if (elapsed < 15000) {
        motorBarat();
        if (elapsed % 3000 < 200) {
          Serial.print(F("Kalibrasi BARAT: "));
          Serial.println((15000 - elapsed) / 1000);
        }
      } else {
        stopMotor();
        pitchBarat = pitch;
        rollBarat = roll;
        Serial.print(F("BARAT selesai - P:"));
        Serial.print(pitch, 1);
        Serial.print(F(" R:"));
        Serial.println(roll, 1);
        nextStep();
      }
      break;
      
    case 2: // UTARA
      if (elapsed < 15000) {
        motorUtara();
        if (elapsed % 3000 < 200) {
          Serial.print(F("Kalibrasi UTARA: "));
          Serial.println((15000 - elapsed) / 1000);
        }
      } else {
        stopMotor();
        pitchUtara = pitch;
        rollUtara = roll;
        Serial.print(F("UTARA selesai - P:"));
        Serial.print(pitch, 1);
        Serial.print(F(" R:"));
        Serial.println(roll, 1);
        nextStep();
      }
      break;
      
    case 3: // SELATAN
      if (elapsed < 15000) {
        motorSelatan();
        if (elapsed % 3000 < 200) {
          Serial.print(F("Kalibrasi SELATAN: "));
          Serial.println((15000 - elapsed) / 1000);
        }
      } else {
        stopMotor();
        pitchSelatan = pitch;
        rollSelatan = roll;
        Serial.print(F("SELATAN selesai - P:"));
        Serial.print(pitch, 1);
        Serial.print(F(" R:"));
        Serial.println(roll, 1);
        kalibrasiBeres = true;
        tampilkanHasilKalibrasi();
      }
      break;
  }
}

void nextStep() {
  stepKalibrasi++;
  waktuMulai = millis();
  delay(1000); // Jeda antar step
}

void bacaInputSerial() {
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim();
    
    int comma1 = input.indexOf(',');
    int comma2 = input.indexOf(',', comma1 + 1);
    int comma3 = input.indexOf(',', comma2 + 1);
    
    if (comma1 > 0 && comma2 > 0 && comma3 > 0) {
      int jamBaru = input.substring(0, comma1).toInt();
      int menitBaru = input.substring(comma1 + 1, comma2).toInt();
      int tanggalBaru = input.substring(comma2 + 1, comma3).toInt();
      int bulanBaru = input.substring(comma3 + 1).toInt();
      
      if (jamBaru >= 0 && jamBaru <= 23 && menitBaru >= 0 && menitBaru <= 59 &&
          tanggalBaru >= 1 && tanggalBaru <= 31 && bulanBaru >= 1 && bulanBaru <= 12) {
        
        jam = jamBaru;
        menit = menitBaru;
        tanggal = tanggalBaru;
        bulan = bulanBaru;
        
        Serial.print(F("Waktu: "));
        Serial.print(jam); Serial.print(F(":"));
        if (menit < 10) Serial.print(F("0"));
        Serial.print(menit); Serial.print(F(" "));
        Serial.print(tanggal); Serial.print(F("/"));
        Serial.println(bulan);
        
        hitungPosisiMatahari();
        sedangTracking = true;
        horizontalSelesai = false;
        vertikalSelesai = false;
        waktuMulaiStep = millis();
        Serial.print(F("Target: Az="));
        Serial.print(azimuthTarget, 1);
        Serial.print(F("° El="));
        Serial.print(elevasiTarget, 1);
        Serial.println(F("°"));
        Serial.println(F("Step 1: Tracking HORIZONTAL (Proporsi Waktu)..."));
      } else {
        Serial.println(F("Waktu tidak valid!"));
      }
    } else {
      Serial.println(F("Format: jam,menit,tanggal,bulan"));
    }
  }
}

void hitungPosisiMatahari() {
  // Sistem sederhana berbasis proporsi waktu
  // Jam 6 pagi = Timur, Jam 6 sore = Barat
  // Bulan 1 (Januari) = Selatan, Bulan 12 (Desember) = Selatan 
  // Bulan 6-7 (Juni-Juli) = Utara
  
  // Hitung proporsi waktu dalam hari (6 pagi - 6 sore = 12 jam)
  float waktuDesimal = jam + (menit / 60.0);
  
  // Batasi jam antara 6 pagi sampai 6 sore
  if (waktuDesimal < 6.0) waktuDesimal = 6.0;
  if (waktuDesimal > 18.0) waktuDesimal = 18.0;
  
  // Proporsi horizontal: 6 pagi = 0%, 12 siang = 50%, 6 sore = 100%
  float proporsiHari = (waktuDesimal - 6.0) / 12.0; // 0.0 to 1.0
  
  // Proporsi vertikal berdasarkan bulan
  // Januari(1) = 0%, Juni(6) = 100%, Desember(12) = 0%
  float proporsiBulan;
  if (bulan <= 6) {
    proporsiBulan = (bulan - 1) / 5.0; // Januari-Juni: 0% to 100%
  } else {
    proporsiBulan = (12 - bulan + 1) / 6.0; // Juli-Desember: 100% to 0%
  }
  
  // Hitung target berdasarkan proporsi dan nilai kalibrasi
  // Horizontal: Timur (pagi) ke Barat (sore)
  targetRoll = rollTimur + (proporsiHari * (rollBarat - rollTimur));
  
  // Vertikal: Selatan (musim dingin) ke Utara (musim panas)
  targetPitch = pitchSelatan + (proporsiBulan * (pitchUtara - pitchSelatan));
  
  // Konversi ke azimuth/elevasi untuk display (opsional)
  azimuthTarget = 90 + (proporsiHari * 180); // 90° (Timur) to 270° (Barat)
  elevasiTarget = 20 + (proporsiBulan * 50);  // 20° (rendah) to 70° (tinggi)
  
  Serial.print(F("Waktu: ")); Serial.print(waktuDesimal, 1);
  Serial.print(F("h Proporsi - Hari:")); Serial.print(proporsiHari * 100, 0);
  Serial.print(F("% Bulan:")); Serial.print(proporsiBulan * 100, 0); Serial.println(F("%"));
  
  Serial.print(F("Kalibrasi - Timur R:")); Serial.print(rollTimur, 1);
  Serial.print(F(" Barat R:")); Serial.print(rollBarat, 1);
  Serial.print(F(" Utara P:")); Serial.print(pitchUtara, 1);
  Serial.print(F(" Selatan P:")); Serial.println(pitchSelatan, 1);
  
  Serial.print(F("Target Roll:")); Serial.print(targetRoll, 1);
  Serial.print(F(" Pitch:")); Serial.println(targetPitch, 1);
  
  Serial.print(F("IMU saat ini - Roll:")); Serial.print(roll, 1);
  Serial.print(F(" Pitch:")); Serial.println(pitch, 1);
  
  Serial.print(F("Error - Roll:")); Serial.print(targetRoll - roll, 1);
  Serial.print(F(" Pitch:")); Serial.println(targetPitch - pitch, 1);
}

void jalankanTracking() {
  // STEP 1: TRACKING HORIZONTAL (TIMUR-BARAT) - mengubah ROLL
  if (!horizontalSelesai) {
    // Timeout check untuk horizontal
    if (millis() - waktuMulaiStep > maxWaktuPerStep) {
      Serial.println(F("Horizontal timeout - lanjut vertikal"));
      horizontalSelesai = true;
      waktuMulaiStep = millis();
      Serial.println(F("Step 2: Tracking VERTIKAL (Proporsi Bulan)..."));
      return;
    }
    
    float errorRoll = targetRoll - roll;
    
    Serial.print(F("H: Roll=")); Serial.print(roll, 1);
    Serial.print(F(" Target=")); Serial.print(targetRoll, 1);
    Serial.print(F(" Err=")); Serial.println(errorRoll, 1);
    
    if (abs(errorRoll) > toleransi) {
      if (errorRoll > 0) {
        // Perlu menambah roll - gerak ke TIMUR
        digitalWrite(motorHPin1, LOW);
        digitalWrite(motorHPin2, HIGH);
        Serial.println(F(">> TIMUR (tambah roll)"));
      } else {
        // Perlu mengurangi roll - gerak ke BARAT
        digitalWrite(motorHPin1, HIGH);
        digitalWrite(motorHPin2, LOW);
        Serial.println(F(">> BARAT (kurangi roll)"));
      }
      digitalWrite(motorVPin1, LOW);  // Pastikan vertikal off
      digitalWrite(motorVPin2, LOW);
    } else {
      stopMotor();
      horizontalSelesai = true;
      waktuMulaiStep = millis();
      Serial.println(F(">>> HORIZONTAL SELESAI <<<"));
      Serial.println(F("Step 2: Tracking VERTIKAL..."));
    }
    return;
  }
  
  // STEP 2: TRACKING VERTIKAL (UTARA-SELATAN) - mengubah PITCH
  if (!vertikalSelesai) {
    // Timeout check untuk vertikal
    if (millis() - waktuMulaiStep > maxWaktuPerStep) {
      Serial.println(F("Vertikal timeout - tracking selesai"));
      vertikalSelesai = true;
      stopMotor();
      sedangTracking = false;
      Serial.println(F(">>> TIMEOUT - POSISI FINAL <<<"));
      return;
    }
    
    float errorPitch = targetPitch - pitch;
    
    Serial.print(F("V: Pitch=")); Serial.print(pitch, 1);
    Serial.print(F(" Target=")); Serial.print(targetPitch, 1);
    Serial.print(F(" Err=")); Serial.println(errorPitch, 1);
    
    if (abs(errorPitch) > toleransi) {
      if (errorPitch > 0) {
        // Perlu menambah pitch - gerak ke UTARA
        digitalWrite(motorVPin1, HIGH);
        digitalWrite(motorVPin2, LOW);
        Serial.println(F(">> UTARA (tambah pitch)"));
      } else {
        // Perlu mengurangi pitch - gerak ke SELATAN
        digitalWrite(motorVPin1, LOW);
        digitalWrite(motorVPin2, HIGH);
        Serial.println(F(">> SELATAN (kurangi pitch)"));
      }
      digitalWrite(motorHPin1, LOW);  // Pastikan horizontal off
      digitalWrite(motorHPin2, LOW);
    } else {
      stopMotor();
      vertikalSelesai = true;
      sedangTracking = false;
      Serial.println(F(">>> VERTIKAL SELESAI <<<"));
      Serial.println(F(">>> SEMUA TRACKING SELESAI - TERKUNCI <<<"));
      Serial.println(F("Kirim waktu baru untuk tracking lagi"));
    }
    return;
  }
}

void tampilkanStatus() {
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint > 2000) { // Print tiap 2 detik
    lastPrint = millis();
    
    Serial.print(F("IMU P:"));
    Serial.print(pitch, 1);
    Serial.print(F(" R:"));
    Serial.print(roll, 1);
    
    if (!sedangTracking) {
      Serial.println(F(" | SIAP - Input waktu"));
    } else {
      Serial.println(F(" | TRACKING..."));
    }
  }
}

void tampilkanHasilKalibrasi() {
  Serial.println(F("\n=== KALIBRASI SELESAI ==="));
  Serial.print(F("TIMUR   P:")); Serial.print(pitchTimur, 1); 
  Serial.print(F(" R:")); Serial.println(rollTimur, 1);
  Serial.print(F("BARAT   P:")); Serial.print(pitchBarat, 1); 
  Serial.print(F(" R:")); Serial.println(rollBarat, 1);
  Serial.print(F("UTARA   P:")); Serial.print(pitchUtara, 1); 
  Serial.print(F(" R:")); Serial.println(rollUtara, 1);
  Serial.print(F("SELATAN P:")); Serial.print(pitchSelatan, 1); 
  Serial.print(F(" R:")); Serial.println(rollSelatan, 1);
  
  Serial.print(F("Range H: ")); 
  Serial.print(abs(rollTimur - rollBarat), 1);
  Serial.print(F("° V: ")); 
  Serial.print(abs(pitchUtara - pitchSelatan), 1);
  Serial.println(F("°"));
  
  Serial.println(F("========================"));
  Serial.println(F("SIAP! Input: jam,menit,tanggal,bulan"));
  Serial.println();
}

// Motor functions
void motorTimur() {
  digitalWrite(motorHPin1, LOW);
  digitalWrite(motorHPin2, HIGH);
}

void motorBarat() {
  digitalWrite(motorHPin1, HIGH);
  digitalWrite(motorHPin2, LOW);
}

void motorUtara() {
  digitalWrite(motorVPin1, HIGH);
  digitalWrite(motorVPin2, LOW);
}

void motorSelatan() {
  digitalWrite(motorVPin1, LOW);
  digitalWrite(motorVPin2, HIGH);
}

void stopMotor() {
  digitalWrite(motorHPin1, LOW);
  digitalWrite(motorHPin2, LOW);
  digitalWrite(motorVPin1, LOW);
  digitalWrite(motorVPin2, LOW);
}