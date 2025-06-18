/*
 * Solar Tracker 2-Axis Sequential dengan RTC Auto-Tracking untuk Indonesia
 * ARAH MOTOR DIPERBAIKI - motorUtara dan motorSelatan ditukar
 * 1. Kalibrasi: Timur-Barat-Utara-Selatan (15s each)
 * 2. Auto-tracking berdasarkan RTC setiap perubahan 2 menit
 * 3. Input serial untuk mengubah waktu RTC: jam,menit,tanggal,bulan,tahun
 * 4. Sequential tracking: Horizontal -> Vertikal
 * 5. Elevasi disesuaikan untuk Indonesia (latitude ~-6°)
 */

#include <Wire.h>
#include <MPU6050.h>
#include "RTClib.h"

MPU6050 mpu;
RTC_DS1307 rtc;

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
float rollTarget = 0, pitchTarget = 0;
const float toleransi = 0.5; // derajat
bool baruMulaiTracking = false; // Flag untuk memaksa gerakan minimal

// RTC Auto-tracking
DateTime waktuTerakhir;
DateTime waktuSekarang;
bool rtcInisialisasi = false;
unsigned long lastRTCCheck = 0;
const unsigned long rtcCheckInterval = 1000; // Check RTC setiap 1 detik

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
  
  // Inisialisasi MPU6050 dengan konfigurasi eksplisit
  Serial.println(F("Inisialisasi MPU6050..."));
  mpu.initialize();
  delay(100);
  
  // Cek koneksi dan tampilkan info
  if (mpu.testConnection()) {
    Serial.println(F("MPU6050 terkoneksi pada 0x68"));
  } else {
    Serial.println(F("MPU6050 gagal test koneksi"));
    Serial.println(F("Mencoba konfigurasi manual..."));
  }
  
  // Konfigurasi manual MPU6050
  mpu.setClockSource(MPU6050_CLOCK_PLL_XGYRO);
  mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_250);
  mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
  mpu.setSleepEnabled(false);
  delay(100);
  
  // Test baca accelerometer
  int16_t ax, ay, az;
  mpu.getAcceleration(&ax, &ay, &az);
  
  if (ax == 0 && ay == 0 && az == 0) {
    Serial.println(F("MPU6050 tidak memberikan data - Cek koneksi!"));
    while(1);
  } else {
    Serial.println(F("MPU6050 berhasil diinisialisasi"));
    Serial.print(F("Test data - AX: "));
    Serial.print(ax);
    Serial.print(F(" AY: "));
    Serial.print(ay);
    Serial.print(F(" AZ: "));
    Serial.println(az);
  }
  
  // Inisialisasi RTC
  if (!rtc.begin()) {
    Serial.println(F("RTC tidak ditemukan!"));
    while(1);
  }
  
  if (!rtc.isrunning()) {
    Serial.println(F("RTC tidak berjalan, mengatur waktu..."));
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }
  
  pinMode(motorHPin1, OUTPUT);
  pinMode(motorHPin2, OUTPUT);
  pinMode(motorVPin1, OUTPUT);
  pinMode(motorVPin2, OUTPUT);
  stopMotor();
  
  Serial.println(F("=== SOLAR TRACKER 2-AXIS RTC AUTO-TRACKING ==="));
  Serial.println(F("ARAH MOTOR DIPERBAIKI"));
  Serial.println(F("Accelerometer only - Stabil & Akurat"));
  Serial.println(F("Sequential: Horizontal -> Vertikal"));
  Serial.println(F("AUTO-TRACKING: Setiap perubahan 2 menit RTC"));
  Serial.println(F("Kalibrasi: T-B-U-S 15s each"));
  Serial.println(F("Mapping: Step 15 menit (7:00-17:00)"));
  Serial.println(F("Elevasi: Per bulan untuk Indonesia"));
  Serial.println(F("Input RTC: jam,menit,tanggal,bulan,tahun"));
  Serial.println(F("Contoh: 12,30,18,6,2025"));
  Serial.println();
  
  waktuMulai = millis();
}

void loop() {
  bacaIMU();
  
  if (!kalibrasiBeres) {
    jalankanKalibrasi();
  } else {
    // Inisialisasi RTC setelah kalibrasi selesai
    if (!rtcInisialisasi) {
      waktuTerakhir = rtc.now();
      rtcInisialisasi = true;
      Serial.println(F("RTC Auto-tracking dimulai!"));
      tampilkanWaktuRTC();
    }
    
    bacaInputSerial();
    cekPerubahanWaktu();
    
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

void cekPerubahanWaktu() {
  if (millis() - lastRTCCheck > rtcCheckInterval) {
    lastRTCCheck = millis();
    waktuSekarang = rtc.now();
    
    // Hitung selisih waktu dalam menit
    long selisihDetik = waktuSekarang.unixtime() - waktuTerakhir.unixtime();
    long selisihMenit = abs(selisihDetik) / 60;
    
    // Jika selisih >= 2 menit, lakukan tracking
    if (selisihMenit >= 2) {
      Serial.println(F("\n>>> PERUBAHAN WAKTU TERDETEKSI <<<"));
      Serial.print(F("Selisih: "));
      Serial.print(selisihMenit);
      Serial.println(F(" menit"));
      
      // Cek apakah waktu dalam range tracking (7:00-17:00)
      int jamSekarang = waktuSekarang.hour();
      int menitSekarang = waktuSekarang.minute();
      int bulanSekarang = waktuSekarang.month();
      
      if (jamSekarang >= 7 && jamSekarang <= 17) {
        if (jamSekarang == 17 && menitSekarang > 0) {
          // Jika lewat jam 17:00, set ke 17:00
          jamSekarang = 17;
          menitSekarang = 0;
        }
        
        // Bulatkan menit ke kelipatan 15
        int menitRounded = (menitSekarang / 15) * 15;
        
        Serial.print(F("Auto-tracking untuk: "));
        if (jamSekarang < 10) Serial.print(F("0"));
        Serial.print(jamSekarang);
        Serial.print(F(":"));
        if (menitRounded < 10) Serial.print(F("0"));
        Serial.print(menitRounded);
        Serial.print(F(" Bulan "));
        Serial.println(bulanSekarang);
        
        startAutoTracking(jamSekarang, menitRounded, bulanSekarang);
      } else {
        Serial.println(F("Waktu di luar range tracking (7:00-17:00)"));
        Serial.println(F("Panel tetap di posisi terakhir"));
      }
      
      waktuTerakhir = waktuSekarang;
    }
  }
}

void startAutoTracking(int jam, int menit, int bulan) {
  // Hitung roll target berdasarkan jam dan menit
  float totalMenit = (jam - 7) * 60 + menit; // Menit dari jam 7:00
  rollTarget = map(totalMenit, 0, 600, rollBarat, rollTimur); // 600 = 10 jam dalam menit
  
  // Hitung pitch target berdasarkan bulan (skip jika range vertikal terlalu kecil)
  if (pitchRange >= 5.0) {
    float elevasiTarget = elevasiBulan[bulan - 1];
    pitchTarget = map(elevasiTarget, 0, 90, pitchSelatan, pitchUtara);
  } else {
    pitchTarget = pitch; // Gunakan posisi saat ini
    Serial.println(F(">>> VERTIKAL SKIP (Range terlalu kecil) <<<"));
  }
  
  Serial.print(F("Roll target: "));
  Serial.print(rollTarget, 1);
  Serial.print(F("° Pitch target: "));
  Serial.print(pitchTarget, 1);
  Serial.print(F("° (El:"));
  Serial.print(elevasiBulan[bulan - 1], 1);
  Serial.println(F("°)"));
  
  sedangTracking = true;
  faseTracking = 0; // Mulai dari horizontal
  baruMulaiTracking = true;
  Serial.println(F(">>> MULAI FASE HORIZONTAL <<<"));
}

void tampilkanWaktuRTC() {
  DateTime now = rtc.now();
  
  Serial.print(F("Waktu RTC: "));
  if (now.hour() < 10) Serial.print(F("0"));
  Serial.print(now.hour());
  Serial.print(F(":"));
  if (now.minute() < 10) Serial.print(F("0"));
  Serial.print(now.minute());
  Serial.print(F(":"));
  if (now.second() < 10) Serial.print(F("0"));
  Serial.print(now.second());
  Serial.print(F(" "));
  if (now.day() < 10) Serial.print(F("0"));
  Serial.print(now.day());
  Serial.print(F("/"));
  if (now.month() < 10) Serial.print(F("0"));
  Serial.print(now.month());
  Serial.print(F("/"));
  Serial.println(now.year());
}

void bacaInputSerial() {
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim();
    
    // Format: jam,menit,tanggal,bulan,tahun
    int comma1 = input.indexOf(',');
    int comma2 = input.indexOf(',', comma1 + 1);
    int comma3 = input.indexOf(',', comma2 + 1);
    int comma4 = input.indexOf(',', comma3 + 1);
    
    if (comma1 > 0 && comma2 > 0 && comma3 > 0 && comma4 > 0) {
      int jamBaru = input.substring(0, comma1).toInt();
      int menitBaru = input.substring(comma1 + 1, comma2).toInt();
      int tanggalBaru = input.substring(comma2 + 1, comma3).toInt();
      int bulanBaru = input.substring(comma3 + 1, comma4).toInt();
      int tahunBaru = input.substring(comma4 + 1).toInt();
      
      // Validasi input
      if (jamBaru >= 0 && jamBaru <= 23 && 
          menitBaru >= 0 && menitBaru <= 59 && 
          tanggalBaru >= 1 && tanggalBaru <= 31 && 
          bulanBaru >= 1 && bulanBaru <= 12 && 
          tahunBaru >= 2000) {
        
        // Set waktu RTC
        rtc.adjust(DateTime(tahunBaru, bulanBaru, tanggalBaru, jamBaru, menitBaru, 0));
        waktuTerakhir = rtc.now(); // Update waktu referensi
        
        Serial.println(F(">>> WAKTU RTC DIUBAH <<<"));
        tampilkanWaktuRTC();
        
        // Jika waktu dalam range tracking, langsung lakukan tracking
        if (jamBaru >= 7 && jamBaru <= 17) {
          if (jamBaru == 17 && menitBaru > 0) {
            jamBaru = 17;
            menitBaru = 0;
          }
          
          int menitRounded = (menitBaru / 15) * 15;
          Serial.println(F("Waktu dalam range tracking - memulai auto-tracking"));
          startAutoTracking(jamBaru, menitRounded, bulanBaru);
        } else {
          Serial.println(F("Waktu di luar range tracking (7:00-17:00)"));
        }
      } else {
        Serial.println(F("Input tidak valid!"));
        Serial.println(F("Format: jam,menit,tanggal,bulan,tahun"));
        Serial.println(F("Jam: 0-23, Menit: 0-59, Tanggal: 1-31, Bulan: 1-12, Tahun: >= 2000"));
      }
    } else {
      Serial.println(F("Format: jam,menit,tanggal,bulan,tahun"));
      Serial.println(F("Contoh: 12,30,18,6,2025 atau 9,15,25,3,2025"));
    }
  }
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
        tampilkanWaktuRTC();
        Serial.println(F("Menunggu perubahan waktu berikutnya..."));
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
      tampilkanWaktuRTC();
      Serial.println(F("Menunggu perubahan waktu berikutnya..."));
    }
  }
}

void tampilkanStatus() {
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint > 3000) {
    lastPrint = millis();
    
    Serial.print(F("Roll: "));
    Serial.print(roll, 1);
    Serial.print(F("° Pitch: "));
    Serial.print(pitch, 1);
    Serial.print(F("° "));
    
    if (sedangTracking) {
      if (faseTracking == 0) Serial.print(F("|TRACK-H "));
      else Serial.print(F("|TRACK-V "));
    } else {
      Serial.print(F("|SIAP "));
    }
    
    if (rtcInisialisasi) {
      DateTime now = rtc.now();
      if (now.hour() < 10) Serial.print(F("0"));
      Serial.print(now.hour());
      Serial.print(F(":"));
      if (now.minute() < 10) Serial.print(F("0"));
      Serial.print(now.minute());
    }
    Serial.println();
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
  Serial.println(F("AUTO-TRACKING SIAP!"));
  Serial.println(F("Input RTC: jam,menit,tanggal,bulan,tahun"));
  Serial.println(F("Contoh: 12,30,18,6,2025 / 9,15,25,3,2025"));
  Serial.println(F("Auto-tracking: Setiap perubahan 2 menit"));
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