#include <WiFi.h>
#include <HTTPClient.h>
#include <HardwareSerial.h>

// Konfigurasi WiFi
const char* ssid = "admin";
const char* password = "admin123";

// Konfigurasi Serial
#define RX_PIN 16
#define TX_PIN 17
#define BAUD_RATE 9600
HardwareSerial SerialLink(1); // Menggunakan Serial1 untuk pin khusus

// Fungsi untuk terhubung ke WiFi
void connectToWiFi() {
  WiFi.begin(ssid, password);
  Serial.println("Menghubungkan ke WiFi...");
  
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  
  Serial.println("");
  Serial.println("WiFi terhubung");
  Serial.print("Alamat IP: ");
  Serial.println(WiFi.localIP());
}

// Fungsi untuk mengirim request HTTP
void sendHttpRequest(String url) {
  HTTPClient http;
  
  Serial.println("Memulai request HTTP ke: " + url);
  http.begin(url);
  
  int httpResponseCode = http.GET();
  
  if (httpResponseCode > 0) {
    Serial.print("Kode respons HTTP: ");
    Serial.println(httpResponseCode);
    
    String payload = http.getString();
    Serial.println("Respons:");
    Serial.println(payload);
  } else {
    Serial.print("Error pada request - kode error: ");
    Serial.println(httpResponseCode);
  }
  
  http.end();
}

void setup() {
  // Inisialisasi Serial Monitor
  Serial.begin(115200);
  
  // Inisialisasi Serial Link di pin khusus
  SerialLink.begin(BAUD_RATE, SERIAL_8N1, RX_PIN, TX_PIN);
  
  // Hubungkan ke WiFi
  connectToWiFi();
  
  Serial.println("ESP32 siap menerima link");
}

void loop() {
  // Cek apakah ada data serial masuk
  if (SerialLink.available()) {
    String receivedLink = SerialLink.readStringUntil('\n');
    receivedLink.trim(); // Hapus spasi atau karakter whitespace di awal/akhir
    
    Serial.println("Link diterima: " + receivedLink);
    
    // Kirim request HTTP ke link
    sendHttpRequest(receivedLink);
    
    Serial.println("Menunggu link berikutnya...");
  }
  
  // Pastikan WiFi tetap terhubung
  if (WiFi.status() != WL_CONNECTED) {
    connectToWiFi();
  }
}