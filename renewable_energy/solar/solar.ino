/*
 * Solar Tracker dengan 4 Sensor LDR (Dual Axis)
 * Menggunakan driver motor L298N dan linear actuator
 * Pengecekan setiap 1 detik
 * Pin PWM langsung dihubungkan ke HIGH
 */

// Definisi pin untuk sensor LDR (axis horizontal)
const int ldrKiri = 5;  // Pin digital untuk sensor LDR kiri
const int ldrKanan = 4; // Pin digital untuk sensor LDR kanan

// Definisi pin untuk sensor LDR (axis vertikal)
const int ldrAtas = 2;  // Pin digital untuk sensor LDR atas
const int ldrBawah = 3; // Pin digital untuk sensor LDR bawah

// Definisi pin untuk driver motor L298N (axis horizontal)
const int motorHPin1 = 8;  // IN1 pada L298N untuk motor horizontal
const int motorHPin2 = 9;  // IN2 pada L298N untuk motor horizontal
// Pin ENA langsung dihubungkan ke HIGH di hardware

// Definisi pin untuk driver motor L298N (axis vertikal)
const int motorVPin1 = 6;  // IN3 pada L298N untuk motor vertikal
const int motorVPin2 = 7;  // IN4 pada L298N untuk motor vertikal
// Pin ENB langsung dihubungkan ke HIGH di hardware

// Variabel untuk kendali waktu
unsigned long waktuTerakhirCek = 0;
const unsigned long intervalCek = 1000; // 1 detik dalam milidetik

void setup() {
  Serial.begin(9600);
  
  // Setup pin sensor sebagai input
  pinMode(ldrKiri, INPUT);
  pinMode(ldrKanan, INPUT);
  pinMode(ldrAtas, INPUT);
  pinMode(ldrBawah, INPUT);
  
  // Setup pin motor driver horizontal sebagai output
  pinMode(motorHPin1, OUTPUT);
  pinMode(motorHPin2, OUTPUT);
  
  // Setup pin motor driver vertikal sebagai output
  pinMode(motorVPin1, OUTPUT);
  pinMode(motorVPin2, OUTPUT);
  
  // Inisialisasi motor tidak bergerak
  digitalWrite(motorHPin1, LOW);
  digitalWrite(motorHPin2, LOW);
  digitalWrite(motorVPin1, LOW);
  digitalWrite(motorVPin2, LOW);
  
  Serial.println("Solar Tracker Dual Axis siap beroperasi!");
  Serial.println("Mode: L298N tanpa PWM (ENA dan ENB terhubung ke HIGH)");
}

void loop() {
  // Cek apakah sudah waktunya untuk pengecekan sensor
  unsigned long waktuSekarang = millis();
  
  if (waktuSekarang - waktuTerakhirCek >= intervalCek) {
    waktuTerakhirCek = waktuSekarang;
    
    // Baca nilai sensor LDR horizontal
    int nilaiLdrKiri = digitalRead(ldrKiri);
    int nilaiLdrKanan = digitalRead(ldrKanan);
    
    // Baca nilai sensor LDR vertikal
    int nilaiLdrAtas = digitalRead(ldrAtas);
    int nilaiLdrBawah = digitalRead(ldrBawah);
    
    Serial.print("Horizontal - Kiri: ");
    Serial.print(nilaiLdrKiri);
    Serial.print(", Kanan: ");
    Serial.print(nilaiLdrKanan);
    Serial.print(" | Vertikal - Atas: ");
    Serial.print(nilaiLdrAtas);
    Serial.print(", Bawah: ");
    Serial.println(nilaiLdrBawah);
    
    // Kontrol axis horizontal
    kontrolHorizontal(nilaiLdrKiri, nilaiLdrKanan);
    kontrolVertikal(nilaiLdrAtas, nilaiLdrBawah);
    
    // Berikan jeda sebelum mengontrol axis vertikal
    // delay(2000);
    
    // Kontrol axis vertikal
    
  }
}

// Fungsi kontrol untuk axis horizontal
void kontrolHorizontal(int nilaiKiri, int nilaiKanan) {
  // Catatan: Nilai 1 berarti gelap, 0 berarti terang 
  // karena ini sensor digital
  
  // Jika kedua sensor mendeteksi gelap (1,1) - berhenti
  if (nilaiKiri == 1 && nilaiKanan == 1) {
    berhentiMotorHorizontal();
    Serial.println("Horizontal: Kedua sensor gelap, motor berhenti");
  }
  // Jika sensor kiri gelap dan kanan terang - gerak ke kanan
  else if (nilaiKiri == 1 && nilaiKanan == 0) {
    gerakKanan();
    Serial.println("Horizontal: Bergerak ke kanan");
  }
  // Jika sensor kiri terang dan kanan gelap - gerak ke kiri
  else if (nilaiKiri == 0 && nilaiKanan == 1) {
    gerakKiri();
    Serial.println("Horizontal: Bergerak ke kiri");
  }
  // Jika kedua sensor terang - posisi ideal matahari, berhenti
  else if (nilaiKiri == 0 && nilaiKanan == 0) {
    berhentiMotorHorizontal();
    Serial.println("Horizontal: Posisi ideal, motor berhenti");
  }
}

// Fungsi kontrol untuk axis vertikal
void kontrolVertikal(int nilaiAtas, int nilaiBawah) {
  // Catatan: Nilai 1 berarti gelap, 0 berarti terang 
  // karena ini sensor digital
  
  // Jika kedua sensor mendeteksi gelap (1,1) - berhenti
  if (nilaiAtas == 1 && nilaiBawah == 1) {
    berhentiMotorVertikal();
    Serial.println("Vertikal: Kedua sensor gelap, motor berhenti");
  }
  // Jika sensor atas gelap dan bawah terang - gerak ke bawah
  else if (nilaiAtas == 1 && nilaiBawah == 0) {
    gerakBawah();
    Serial.println("Vertikal: Bergerak ke bawah");
  }
  // Jika sensor atas terang dan bawah gelap - gerak ke atas
  else if (nilaiAtas == 0 && nilaiBawah == 1) {
    gerakAtas();
    Serial.println("Vertikal: Bergerak ke atas");
  }
  // Jika kedua sensor terang - posisi ideal matahari, berhenti
  else if (nilaiAtas == 0 && nilaiBawah == 0) {
    berhentiMotorVertikal();
    Serial.println("Vertikal: Posisi ideal, motor berhenti");
  }
}

// Fungsi untuk mengendalikan motor horizontal
void gerakKiri() {
  // Aktifkan motor untuk bergerak ke kiri
  digitalWrite(motorHPin1, HIGH);
  digitalWrite(motorHPin2, LOW);
  delay(1000); // Bergerak selama 1 detik
  berhentiMotorHorizontal();
}

void gerakKanan() {
  // Aktifkan motor untuk bergerak ke kanan
  digitalWrite(motorHPin1, LOW);
  digitalWrite(motorHPin2, HIGH);
  delay(1000); // Bergerak selama 1 detik
  berhentiMotorHorizontal();
}

void berhentiMotorHorizontal() {
  // Matikan motor horizontal
  digitalWrite(motorHPin1, LOW);
  digitalWrite(motorHPin2, LOW);
}

// Fungsi untuk mengendalikan motor vertikal
void gerakAtas() {
  // Aktifkan motor untuk bergerak ke atas
  digitalWrite(motorVPin1, HIGH);
  digitalWrite(motorVPin2, LOW);
  delay(1000); // Bergerak selama 1 detik
  berhentiMotorVertikal();
}

void gerakBawah() {
  // Aktifkan motor untuk bergerak ke bawah
  digitalWrite(motorVPin1, LOW);
  digitalWrite(motorVPin2, HIGH);
  delay(1000); // Bergerak selama 1 detik
  berhentiMotorVertikal();
}

void berhentiMotorVertikal() {
  // Matikan motor vertikal
  digitalWrite(motorVPin1, LOW);
  digitalWrite(motorVPin2, LOW);
}