#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// Pin definitions for joystick
#define JOYSTICK_X 34  // X-axis pin (ADC)
#define JOYSTICK_Y 35  // Y-axis pin (ADC)
#define JOYSTICK_SW 32 // Switch pin (Digital)

// LCD I2C address (umumnya 0x27 atau 0x3F)
#define LCD_ADDRESS 0x27
#define LCD_COLS 20
#define LCD_ROWS 4

// Inisialisasi objek LCD
LiquidCrystal_I2C lcd(LCD_ADDRESS, LCD_COLS, LCD_ROWS);

// Variabel untuk menyimpan nilai joystick
int xValue, yValue;
bool swValue;

// Variabel untuk kalibrasi
const int centerThreshold = 100;
String direction = "Center";

void setup() {
  // Inisialisasi Serial Monitor
  Serial.begin(115200);
  
  // Inisialisasi pin joystick
  pinMode(JOYSTICK_X, INPUT);
  pinMode(JOYSTICK_Y, INPUT);
  pinMode(JOYSTICK_SW, INPUT_PULLUP);
  
  // Inisialisasi LCD
  Wire.begin();
  lcd.init();
  lcd.backlight();
  
  // Tampilkan pesan awal
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Joystick Monitor");
  lcd.setCursor(0, 1);
  lcd.print("----------------");
  delay(2000);
}

void loop() {
  // Baca nilai joystick
  xValue = analogRead(JOYSTICK_X);
  yValue = analogRead(JOYSTICK_Y);
  swValue = !digitalRead(JOYSTICK_SW);  // Active LOW
  
  // Tentukan arah berdasarkan nilai joystick
  determineDirection();
  
  // Update tampilan LCD
  updateLCD();
  
  // Delay untuk stabilitas pembacaan
  delay(100);
}

void determineDirection() {
  // Konversi nilai ADC (0-4095) ke persentase (0-100)
  int xPercent = map(xValue, 0, 4095, 0, 100);
  int yPercent = map(yValue, 0, 4095, 0, 100);
  
  // Tentukan arah berdasarkan posisi joystick
  if (xPercent < 45) {
    direction = "Left ";
  } else if (xPercent > 55) {
    direction = "Right";
  } else if (yPercent < 45) {
    direction = "Down ";
  } else if (yPercent > 55) {
    direction = "Up   ";
  } else {
    direction = "Center";
  }
}

void updateLCD() {
  // Baris 1: Nilai X
  lcd.setCursor(0, 1);
  lcd.print("X: ");
  lcd.print(xValue);
  lcd.print("   ");
  
  // Baris 2: Nilai Y
  lcd.setCursor(0, 2);
  lcd.print("Y: ");
  lcd.print(yValue);
  lcd.print("   ");
  
  // Baris 3: Arah dan Status Switch
  lcd.setCursor(0, 3);
  lcd.print("Dir: ");
  lcd.print(direction);
  lcd.print(" SW:");
  lcd.print(swValue ? "ON " : "OFF");
  
  // Debug di Serial Monitor
  Serial.printf("X: %d, Y: %d, SW: %d, Dir: %s\n", 
                xValue, yValue, swValue, direction.c_str());
}