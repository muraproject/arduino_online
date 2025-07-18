// Program ESP32 untuk membaca input tombol digital dan analog dengan LCD I2C
// Digital input pins: 23, 19, 18, 5, 4, 32, 33 (dengan pullup)
// Analog input pin: 35
// LCD 16x2 I2C

#include <LiquidCrystal_I2C.h>
#include <Wire.h>

// Definisi pin digital
const int digitalPins[] = {23, 19, 18, 5, 4, 32, 33};
const int numDigitalPins = sizeof(digitalPins) / sizeof(digitalPins[0]);

// Definisi pin analog
const int analogPin = 35;

// Setup LCD I2C (alamat 0x27, 16 kolom, 2 baris)
LiquidCrystal_I2C lcd(0x27, 16, 2);

// Buffer untuk sprintf
char lcdBuffer[17]; // 16 karakter + null terminator

void setup() {
  // Inisialisasi Serial Monitor
  Serial.begin(115200);
  Serial.println("ESP32 Digital & Analog Input Reader dengan LCD I2C");
  Serial.println("====================================================");
  
  // Inisialisasi LCD I2C
  lcd.init();
  lcd.backlight();
  
  // Tampilkan pesan startup di LCD
  sprintf(lcdBuffer, "ESP32 Input Read");
  lcd.setCursor(0, 0);
  lcd.print(lcdBuffer);
  sprintf(lcdBuffer, "Starting...     ");
  lcd.setCursor(0, 1);
  lcd.print(lcdBuffer);
  
  // Setup pin digital sebagai input dengan pullup
  for (int i = 0; i < numDigitalPins; i++) {
    pinMode(digitalPins[i], INPUT_PULLUP);
    Serial.print("Pin ");
    Serial.print(digitalPins[i]);
    Serial.println(" diatur sebagai INPUT_PULLUP");
  }
  
  // Setup pin analog (pin 35 adalah input-only, tidak perlu pinMode)
  Serial.print("Pin ");
  Serial.print(analogPin);
  Serial.println(" diatur sebagai INPUT ANALOG");
  
  Serial.println("Setup selesai!\n");
  delay(2000);
}

void loop() {
  Serial.println("=== Pembacaan Input ===");
  
  // Array untuk menyimpan status digital
  int digitalValues[numDigitalPins];
  
  // Baca semua pin digital
  Serial.println("Digital Inputs:");
  for (int i = 0; i < numDigitalPins; i++) {
    digitalValues[i] = digitalRead(digitalPins[i]);
    Serial.print("Pin ");
    Serial.print(digitalPins[i]);
    Serial.print(": ");
    Serial.print(digitalValues[i]);
    Serial.print(" (");
    Serial.print(digitalValues[i] == LOW ? "PRESSED" : "RELEASED");
    Serial.println(")");
  }
  
  // Baca pin analog
  int analogValue = analogRead(analogPin);
  float voltage = (analogValue * 3.3) / 4095.0; // Konversi ke voltage (ESP32 menggunakan 12-bit ADC)
  
  Serial.println("\nAnalog Input:");
  Serial.print("Pin ");
  Serial.print(analogPin);
  Serial.print(": ");
  Serial.print(analogValue);
  Serial.print(" (");
  Serial.print(voltage, 2);
  Serial.println("V)");
  
  // Tampilkan di LCD menggunakan sprintf
  // Baris 1: Status tombol dalam format binary string
  sprintf(lcdBuffer, "BTN:%d%d%d%d%d%d%d      ", 
          digitalValues[0], digitalValues[1], digitalValues[2], 
          digitalValues[3], digitalValues[4], digitalValues[5], 
          digitalValues[6]);
  lcd.setCursor(0, 0);
  lcd.print(lcdBuffer);
  
  // Baris 2: Nilai analog
  sprintf(lcdBuffer, "A35:%4d %4.1fV ", analogValue, voltage);
  lcd.setCursor(0, 1);
  lcd.print(lcdBuffer);
  
  Serial.println("========================\n");
  
  // Delay sebelum pembacaan berikutnya
  delay(200);
}