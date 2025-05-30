#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// Inisialisasi LCD (alamat I2C, kolom, baris)
LiquidCrystal_I2C lcd(0x27, 20, 4);

// Pin GPIO pada Wemos D1 Mini
int pushPin = D6;
int LEDPin = D5;

// Nada-nada naik 2 oktaf
int c = 1044;
int d = 1172;
int e = 1316;
int f = 1396;
int g = 1564;
int a = 1760;
int Bb = 1864;
int b = 1972;
int highc = 2092;

// Durasi not
int quarter = 500;
int half = 1000;

void setup() {
  pinMode(pushPin, INPUT);
  pinMode(LEDPin, OUTPUT);
  digitalWrite(LEDPin, LOW);

  lcd.init();              // Inisialisasi LCD
  lcd.backlight();         // Nyalakan lampu belakang LCD
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("  SELAMAT ULANG");
  lcd.setCursor(0, 1);
  lcd.print("      TAHUN!");
  delay(2000);             // Tampilkan pesan awal
  lcd.clear();
}

void loop() {
  digitalWrite(LEDPin, HIGH);
  
  lcd.setCursor(0, 0);
  lcd.print("Selamat ulang");
  tone(LEDPin, c, 300); delay(400);
  tone(LEDPin, c, 100); delay(100);
  tone(LEDPin, d, quarter); delay(quarter);
  tone(LEDPin, c, quarter); delay(quarter);
  tone(LEDPin, f, quarter); delay(quarter);
  tone(LEDPin, e, half); delay(half);

  lcd.setCursor(0, 1);
  lcd.print("tahun kepadamu");
  tone(LEDPin, c, 300); delay(400);
  tone(LEDPin, c, 100); delay(100);
  tone(LEDPin, d, quarter); delay(quarter);
  tone(LEDPin, c, quarter); delay(quarter);
  tone(LEDPin, g, quarter); delay(quarter);
  tone(LEDPin, f, half); delay(half);

  lcd.setCursor(0, 2);
  lcd.print("Selamat ulang");
  tone(LEDPin, c, 300); delay(400);
  tone(LEDPin, c, 100); delay(100);
  tone(LEDPin, highc, quarter); delay(quarter);
  tone(LEDPin, a, quarter); delay(quarter);
  tone(LEDPin, f, quarter); delay(quarter);
  tone(LEDPin, e, quarter); delay(quarter);
  tone(LEDPin, d, half); delay(half);

  lcd.setCursor(0, 3);
  lcd.print("tahun wahai kamu");
  tone(LEDPin, Bb, 300); delay(400);
  tone(LEDPin, Bb, 100); delay(100);
  tone(LEDPin, a, quarter); delay(quarter);
  tone(LEDPin, f, quarter); delay(quarter);
  tone(LEDPin, g, quarter); delay(quarter);
  tone(LEDPin, f, half); delay(half);

  digitalWrite(LEDPin, LOW);
  delay(1000);
  lcd.clear();

  lcd.setCursor(0, 0);
  lcd.print("Selamat ulang");
  tone(LEDPin, c, 300); delay(400);
  tone(LEDPin, c, 100); delay(100);
  tone(LEDPin, d, quarter); delay(quarter);
  tone(LEDPin, c, quarter); delay(quarter);
  tone(LEDPin, f, quarter); delay(quarter);
  tone(LEDPin, e, half); delay(half);

  lcd.setCursor(0, 1);
  lcd.print("tahun kepadamu");
  tone(LEDPin, c, 300); delay(400);
  tone(LEDPin, c, 100); delay(100);
  tone(LEDPin, d, quarter); delay(quarter);
  tone(LEDPin, c, quarter); delay(quarter);
  tone(LEDPin, g, quarter); delay(quarter);
  tone(LEDPin, f, half); delay(half);

  lcd.setCursor(0, 2);
  lcd.print("Selamat ulang");
  tone(LEDPin, c, 300); delay(400);
  tone(LEDPin, c, 100); delay(100);
  tone(LEDPin, highc, quarter); delay(quarter);
  tone(LEDPin, a, quarter); delay(quarter);
  tone(LEDPin, f, quarter); delay(quarter);
  tone(LEDPin, e, quarter); delay(quarter);
  tone(LEDPin, d, half); delay(half);

  lcd.setCursor(0, 3);
  lcd.print("tahun wahai kamu");
  tone(LEDPin, Bb, 300); delay(400);
  tone(LEDPin, Bb, 100); delay(100);
  tone(LEDPin, a, quarter); delay(quarter);
  tone(LEDPin, f, quarter); delay(quarter);
  tone(LEDPin, g, quarter); delay(quarter);
  tone(LEDPin, f, half); delay(half);

  digitalWrite(LEDPin, LOW);
  delay(1000);
  lcd.clear();
}
