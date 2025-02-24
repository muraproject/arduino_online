#define M0 4
#define M1 2
#define AUX 5
#define RXD2 16
#define TXD2 17

void setup() {
  // Inisialisasi Serial Monitor
  Serial.begin(9600);
  
  // Inisialisasi Serial2 untuk komunikasi dengan E220
  Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2);
  
  // Setup pin modes
  pinMode(M0, OUTPUT);
  pinMode(M1, OUTPUT);
  pinMode(AUX, INPUT);
  
  // Set ke normal mode
  digitalWrite(M0, LOW);
  digitalWrite(M1, LOW);
  
  delay(1000);
  Serial.println("LoRa E220 Receiver Siap");
}

void loop() {
  // Cek apakah ada data yang diterima
  if(Serial2.available()) {
    // Baca data sampai newline
    String data = Serial2.readStringUntil('\n');
    
    // Tampilkan data di Serial Monitor
    Serial.print("Menerima: ");
    Serial.println(data);
  }
}