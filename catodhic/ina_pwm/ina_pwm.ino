#include <Wire.h>
#include <Adafruit_INA219.h>

// Inisialisasi sensor INA219
Adafruit_INA219 ina219;

// Definisi pin
const int pwmPin = 5;        // GPIO5 untuk ESP32/ESP8266
int pwmValue = 100;           // Nilai PWM (0-255)

void setup() {
  // Inisialisasi komunikasi serial
  Serial.begin(9600);
  
  // Inisialisasi pin PWM
  pinMode(pwmPin, OUTPUT);
  
  // Setel PWM ke nilai awal (menggunakan analogWrite yang umum)
  analogWrite(pwmPin, pwmValue);
  
  // Inisialisasi INA219
  if (!ina219.begin()) {
    Serial.println("Gagal menemukan sensor INA219!");
    while (1) { delay(10); }
  }
  
  // Konfigurasi kalibrasi INA219 (opsional)
  // ina219.setCalibration_16V_400mA();  // Untuk akurasi lebih baik pada tegangan rendah
  
  // Pesan pembuka
  Serial.println("Pengukuran Buck Converter dengan INA219");
  Serial.println("PWM Value: " + String(pwmValue));
  Serial.println("----------------------------------------------------------");
  Serial.println("Tegangan Bus(V), Tegangan Shunt(mV), Tegangan Beban(V), Arus(mA), Daya(mW)");
}

void loop() {
  // Baca nilai dari INA219
  float shuntVoltage = ina219.getShuntVoltage_mV();
  float busVoltage = ina219.getBusVoltage_V();
  float current = ina219.getCurrent_mA();
  float power = ina219.getPower_mW();
  float loadVoltage = busVoltage + (shuntVoltage / 1000);
  

  // Serial.print(busVoltage, 4);
  // Serial.print(", ");
  // Serial.print(shuntVoltage, 4);
  // Serial.print(", ");
  Serial.print(loadVoltage, 1);
  Serial.print(", ");
  Serial.println(current, 1);
  // Serial.print(", ");
  // Serial.println(power, 4);
  
  // Tunggu sebelum pengukuran berikutnya
  delay(100);
}