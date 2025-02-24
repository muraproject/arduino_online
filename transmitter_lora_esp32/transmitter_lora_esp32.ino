// TRANSMITTER CODE (ESP32)
// File: LoRa_LED_Control_TX.ino

#define M0 4
#define M1 2
#define AUX 5
#define RXD2 16
#define TXD2 17

// Variable untuk menyimpan status LED
bool ledState = false;

void setup() {
  // Inisialisasi Serial Monitor
  Serial.begin(9600);
  
  // Inisialisasi Serial2 untuk E220
  Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2);

  // Setup pin modes
  pinMode(M0, OUTPUT);
  pinMode(M1, OUTPUT);
   pinMode(AUX, INPUT);
  
  // Set ke normal mode
  digitalWrite(M0, LOW);
  digitalWrite(M1, LOW);
  
  delay(1000);
  Serial.println("LoRa E220 LED Control Transmitter Ready");
  Serial.println("Mengirim ON/OFF setiap 2 detik");
}

void loop() {
  // Tunggu sampai AUX high (modul siap)
  // while(digitalRead(AUX) == LOW);
  
  // Toggle state
  ledState = !ledState;
  
  // Siapkan perintah berdasarkan state
  String command = ledState ? "on" : "off";
  
  // Kirim perintah
  Serial2.println(command);
  Serial.println("Mengirim perintah: " + command);
  
  // Tunggu 2 detik
  delay(2000);
}
