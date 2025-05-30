#include <Wire.h>
#include <MPU6050.h>

MPU6050 mpu;

// Variabel untuk filter komplementer
float pitch = 0, roll = 0;
float pitchAcc, rollAcc;
float pitchGyro = 0, rollGyro = 0;
unsigned long lastTime = 0;

// Konstanta filter komplementer (0.98 untuk gyro, 0.02 untuk accelerometer)
const float alpha = 0.98;

// Variabel untuk averaging filter
const int numReadings = 10;
float pitchReadings[numReadings];
float rollReadings[numReadings];
int readIndex = 0;
float pitchTotal = 0, rollTotal = 0;
float pitchAverage = 0, rollAverage = 0;

// Threshold untuk mengurangi noise kecil
const float threshold = 0.5;
float lastStablePitch = 0, lastStableRoll = 0;

void setup() {
  Serial.begin(9600);
  Wire.begin();
  
  // Inisialisasi MPU6050
  mpu.initialize();
  
  // Cek koneksi sensor
  if (mpu.testConnection()) {
    Serial.println("MPU6050 connection successful");
  } else {
    Serial.println("MPU6050 connection failed");
    while(1);
  }
  
  // Konfigurasi sensor
  mpu.setFullScaleGyroRange(0);  // 0 = ±250°/s, 1 = ±500°/s, 2 = ±1000°/s, 3 = ±2000°/s
  mpu.setFullScaleAccelRange(0); // 0 = ±2g, 1 = ±4g, 2 = ±8g, 3 = ±16g
  
  // Inisialisasi array untuk averaging
  for (int i = 0; i < numReadings; i++) {
    pitchReadings[i] = 0;
    rollReadings[i] = 0;
  }
  
  // Kalibrasi awal - baca beberapa sampel untuk stabilisasi
  Serial.println("Kalibrasi sensor...");
  for (int i = 0; i < 100; i++) {
    readSensorData();
    delay(10);
  }
  Serial.println("Kalibrasi selesai");
  
  lastTime = millis();
}

void loop() {
  readSensorData();
  applyComplementaryFilter();
  applyAveragingFilter();
  applyThresholdFilter();
  
  // Tampilkan hasil
  Serial.print("Pitch: "); Serial.print(lastStablePitch, 2);
  Serial.print("°\tRoll: "); Serial.print(lastStableRoll, 2);
  Serial.println("°");
  
  // Untuk solar tracker, Anda bisa menambahkan kontrol servo di sini
  // controlServos(lastStablePitch, lastStableRoll);
  
  delay(50); // Update setiap 50ms
}

void readSensorData() {
  int16_t ax, ay, az;
  int16_t gx, gy, gz;
  
  // Baca data accelerometer dan gyroscope
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  
  // Konversi ke satuan fisik
  float accX = ax / 16384.0; // untuk ±2g
  float accY = ay / 16384.0;
  float accZ = az / 16384.0;
  
  float gyroX = gx / 131.0; // untuk ±250°/s
  float gyroY = gy / 131.0;
  
  // Hitung sudut dari accelerometer (dalam derajat)
  pitchAcc = atan2(accY, sqrt(accX * accX + accZ * accZ)) * 180.0 / PI;
  rollAcc = atan2(-accX, sqrt(accY * accY + accZ * accZ)) * 180.0 / PI;
  
  // Simpan nilai gyroscope (dalam derajat per detik)
  pitchGyro = gyroX;
  rollGyro = gyroY;
}

void applyComplementaryFilter() {
  unsigned long currentTime = millis();
  float dt = (currentTime - lastTime) / 1000.0; // dalam detik
  
  // Filter komplementer: gabungkan gyroscope dan accelerometer
  pitch = alpha * (pitch + pitchGyro * dt) + (1 - alpha) * pitchAcc;
  roll = alpha * (roll + rollGyro * dt) + (1 - alpha) * rollAcc;
  
  lastTime = currentTime;
}

void applyAveragingFilter() {
  // Kurangi total dengan nilai lama
  pitchTotal = pitchTotal - pitchReadings[readIndex];
  rollTotal = rollTotal - rollReadings[readIndex];
  
  // Tambahkan nilai baru
  pitchReadings[readIndex] = pitch;
  rollReadings[readIndex] = roll;
  pitchTotal = pitchTotal + pitchReadings[readIndex];
  rollTotal = rollTotal + rollReadings[readIndex];
  
  // Pindah ke index berikutnya
  readIndex = readIndex + 1;
  if (readIndex >= numReadings) {
    readIndex = 0;
  }
  
  // Hitung rata-rata
  pitchAverage = pitchTotal / numReadings;
  rollAverage = rollTotal / numReadings;
}

void applyThresholdFilter() {
  // Hanya update jika perubahan cukup signifikan
  if (abs(pitchAverage - lastStablePitch) > threshold) {
    lastStablePitch = pitchAverage;
  }
  
  if (abs(rollAverage - lastStableRoll) > threshold) {
    lastStableRoll = rollAverage;
  }
}

// Fungsi opsional untuk mengontrol servo (uncomment jika diperlukan)
/*
#include <Servo.h>
Servo servoPitch, servoRoll;

void setupServos() {
  servoPitch.attach(9);  // Pin servo untuk pitch
  servoRoll.attach(10);  // Pin servo untuk roll
}

void controlServos(float pitch, float roll) {
  // Konversi sudut ke posisi servo (0-180 derajat)
  int servoPitchPos = map(pitch, -90, 90, 0, 180);
  int servoRollPos = map(roll, -90, 90, 0, 180);
  
  // Batasi range servo
  servoPitchPos = constrain(servoPitchPos, 0, 180);
  servoRollPos = constrain(servoRollPos, 0, 180);
  
  // Gerakkan servo
  servoPitch.write(servoPitchPos);
  servoRoll.write(servoRollPos);
}
*/
