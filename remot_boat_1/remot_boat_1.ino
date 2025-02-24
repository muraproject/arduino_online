#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// Pin definitions for joystick
#define JOYSTICK_X 34
#define JOYSTICK_Y 35
#define JOYSTICK_SW 32

// LoRa E220 pins
#define M0 4
#define M1 2
#define AUX 5
#define RXD2 16  // LoRa TX to ESP32 RX2
#define TXD2 17  // LoRa RX to ESP32 TX2

// LCD I2C settings
#define LCD_ADDRESS 0x27
#define LCD_COLS 20
#define LCD_ROWS 4

LiquidCrystal_I2C lcd(LCD_ADDRESS, LCD_COLS, LCD_ROWS);

// Variables
int xValue, yValue;
bool swValue;
float receivedSpeed = 0.0;
String direction = "STOP";
String lastCommand = "";

void setup() {
  Serial.begin(115200);
  
  // Initialize LoRa Serial
  Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2);
  
  // Joystick setup
  pinMode(JOYSTICK_X, INPUT);
  pinMode(JOYSTICK_Y, INPUT);
  pinMode(JOYSTICK_SW, INPUT_PULLUP);
  
  // LoRa setup
  pinMode(M0, OUTPUT);
  pinMode(M1, OUTPUT);
  pinMode(AUX, INPUT);
  digitalWrite(M0, LOW);
  digitalWrite(M1, LOW);
  
  // LCD setup
  Wire.begin();
  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Remote Control Ready");
  delay(2000);
}

void loop() {
  // Read joystick values
  xValue = analogRead(JOYSTICK_X);
  yValue = analogRead(JOYSTICK_Y);
  swValue = !digitalRead(JOYSTICK_SW);
  
  // Determine direction and send command
  String command = getDirection();
  if (command != lastCommand) {
    Serial2.println(command);
    lastCommand = command;
  }
  
  // Read speed data from boat if available
  if (Serial2.available()) {
    String data = Serial2.readStringUntil('\n');
    if (data.startsWith("SPD:")) {
      receivedSpeed = data.substring(4).toFloat();
    }
  }
  
  // Update LCD
  updateLCD();
  delay(100);
}

String getDirection() {
  int xPercent = map(xValue, 0, 4095, 0, 100);
  int yPercent = map(yValue, 0, 4095, 0, 100);
  
  if (yPercent < 35) {
    direction = "FORWARD";
    return "FWD";
  } else if (yPercent > 65) {
    direction = "BACKWARD";
    return "BWD";
  } else if (xPercent < 35) {
    direction = "LEFT";
    return "LFT";
  } else if (xPercent > 65) {
    direction = "RIGHT";
    return "RGT";
  } else {
    direction = "STOP";
    return "STP";
  }
}

void updateLCD() {
  lcd.clear();
  
  // Line 1: Direction
  lcd.setCursor(0, 0);
  lcd.print("Direction: ");
  lcd.print(direction);
  
  // Line 2: Boat Speed
  lcd.setCursor(0, 1);
  lcd.print("Boat Speed: ");
  lcd.print(receivedSpeed, 1);
  lcd.print(" m/s");
  
  // Line 3: Control Status
  lcd.setCursor(0, 2);
  lcd.print("Command: ");
  lcd.print(lastCommand);
}