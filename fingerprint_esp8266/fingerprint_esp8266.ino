// Component         | ESP8266 Pin | Function
// -----------------|-------------|----------
// Fingerprint (RX) | D4          | Receive data
// Fingerprint (TX) | D3          | Transmit data
// LCD (SDA)        | D2          | I2C Data
// LCD (SCL)        | D1          | I2C Clock
// Mode Switch      | D6          | Switch between modes
// LED             | D8          | Output indicator
// Buzzer          | D5          | Sound output

#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <Adafruit_Fingerprint.h>
#include <SoftwareSerial.h>
#include <LiquidCrystal_I2C.h>
#include <ESP8266HTTPClient.h>
#include <ArduinoJson.h>

// Konfigurasi WiFi
const char* ssid = "DonBosco31";
const char* password = "12345678";
WiFiClientSecure client;

// Konfigurasi Google Script
const char* googleScriptId = "AKfycbz9lKWP83KD5nVXfFpMpSrZ_m3MJOgPO87COscVkKNK54mGZop3j7qCRUIm6AynWdwKJA";

// Inisialisasi komponen
SoftwareSerial mySerial(D4, D3);  // RX, TX                                         
Adafruit_Fingerprint finger = Adafruit_Fingerprint(&mySerial);
LiquidCrystal_I2C lcd(0x27, 16, 2);
ESP8266WebServer server(80);

// Mode operasi
//bool isHotspotMode = false;
bool isHotspotMode = false;
const int MODE_SWITCH_PIN = D6;  // Pin untuk switch mode

// HTML untuk halaman web (disimpan sebagai string)
const char INDEX_HTML[] PROGMEM = R"=====(
<!DOCTYPE HTML>
<html>
<head>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <style>
    body { font-family: Arial, sans-serif; text-align: center; }
    table { margin: 0 auto; border-collapse: collapse; }
    th, td { border: 1px solid black; padding: 5px; }
    .button { background-color: #4CAF50; border: none; color: white; padding: 10px 20px; 
              text-align: center; text-decoration: none; display: inline-block; 
              font-size: 16px; margin: 4px 2px; cursor: pointer; }
    .delete { background-color: #f44336; }
  </style>
</head>
<body>
  <h1>Fingerprint Management</h1>
  <table id="fingerprintTable">
    <tr>
      <th>ID</th>
      <th>Action</th>
    </tr>
  </table>
  <button class="button" onclick="addFinger()">Add New Fingerprint</button>
  <script>
    function loadFingerprints() {
      fetch('/list-fingerprints')
        .then(response => response.json())
        .then(data => {
          const table = document.getElementById('fingerprintTable');
          // Clear existing rows except header
          while (table.rows.length > 1) {
            table.deleteRow(1);
          }
          // Add new rows
          data.forEach(id => {
            const row = table.insertRow(-1);
            const cell1 = row.insertCell(0);
            const cell2 = row.insertCell(1);
            cell1.textContent = id;
            cell2.innerHTML = `<button class="button delete" onclick="deleteFinger(${id})">Delete</button>`;
          });
        });
    }

    function addFinger() {
      fetch('/add-finger', { method: 'POST' })
        .then(response => response.text())
        .then(data => {
          alert(data);
          loadFingerprints();
        });
    }

    function deleteFinger(id) {
      fetch('/delete-finger', {
        method: 'POST',
        headers: {'Content-Type': 'application/x-www-form-urlencoded'},
        body: 'id=' + id
      })
      .then(response => response.text())
      .then(data => {
        alert(data);
        loadFingerprints();
      });
    }

    // Load fingerprints when page loads
    loadFingerprints();
  </script>
</body>
</html>
)=====";

void handleListFingerprints() {
  DynamicJsonDocument doc(1024);
  JsonArray array = doc.to<JsonArray>();

  for (int i = 1; i < 128; i++) {
    if (finger.loadModel(i) == FINGERPRINT_OK) {
      array.add(i);
    }
  }

  String json;
  serializeJson(doc, json);
  server.send(200, "application/json", json);
}

void setup() {
  Serial.begin(9600);
  lcd.init();
  lcd.backlight();
  finger.begin(57600);
  delay(1000);
 
  
  // Cek koneksi sensor
  if (finger.verifyPassword()) {
    Serial.println("Sensor sidik jari terdeteksi!");
    lcd.print("Sensor OK");
  } else {
    Serial.println("Sensor sidik jari tidak terdeteksi :(");
    lcd.print("Sensor Error");
    while (1) { delay(1); }
  }

  // Setup WiFi
  WiFi.begin(ssid, password);
//  while (WiFi.status() != WL_CONNECTED) {
//    delay(500);
//    Serial.print(".");
//  }
  Serial.println("");
  Serial.println("WiFi connected");
  
  // Setup web server
  server.on("/", HTTP_GET, handleRoot);
  server.on("/add-finger", HTTP_POST, handleAddFinger);
  server.on("/delete-finger", HTTP_POST, handleDeleteFinger);
  server.on("/list-fingerprints", HTTP_GET, handleListFingerprints);
  server.begin();
  pinMode(D6,2);
  pinMode(D8,1);
  pinMode(D5,1);
}

void loop() {
  static bool lastPinState = HIGH;
  bool currentPinState = digitalRead(MODE_SWITCH_PIN);
  
  if (currentPinState != lastPinState) {
    if (currentPinState == LOW) {
      switchMode();
    }
    lastPinState = currentPinState;
  }

  // Jalankan mode yang sesuai
  if (isHotspotMode) {
    server.handleClient();
  } else {
   checkAndProcessFingerprint();
//    Serial.println("finger");
  }
  
  delay(50);
}
void switchMode() {
  isHotspotMode = !isHotspotMode;
  if (isHotspotMode) {
    // Aktifkan mode hotspot
    Serial.println("enter AP mode");
    WiFi.disconnect();
    WiFi.mode(WIFI_AP);
    WiFi.softAP("Fingerprint_AP", "12345678");
    IPAddress myIP = WiFi.softAPIP();
    Serial.print("AP IP address: ");
    Serial.println(myIP);
    lcd.clear();
    lcd.print("Hotspot Mode");
    lcd.setCursor(0, 1);
    lcd.print(myIP);
  } else {
    // Kembali ke mode normal
    Serial.println("enter STA mode");
    WiFi.softAPdisconnect(true);
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
      delay(500);
      Serial.print(".");
    }
    Serial.println("");
    Serial.println("WiFi connected");
    lcd.clear();
    lcd.print("Normal Mode");
  }
}

void handleRoot() {
  server.send(200, "text/html", INDEX_HTML);
}

void handleAddFinger() {
  int id = findEmptyId();
  if (id == -1) {
    server.send(200, "text/plain", "No empty slots available");
    return;
  }
  
  String result = enrollFingerprint(id);
  server.send(200, "text/plain", result);
}

int findEmptyId() {
  for (int i = 1; i < 128; i++) {
    if (finger.loadModel(i) != FINGERPRINT_OK) {
      return i;
    }
  }
  return -1; // No empty slots
}

void handleDeleteFinger() {
  int id = server.arg("id").toInt();
  String result = deleteFingerprint(id);
  server.send(200, "text/plain", result);
}

String enrollFingerprint(uint8_t id) {
  int p = -1;
  Serial.print("Waiting for valid finger to enroll as #"); Serial.println(id);
  
  // Ambil gambar sidik jari pertama
  while (p != FINGERPRINT_OK) {
    p = finger.getImage();
    switch (p) {
      case FINGERPRINT_OK:
        Serial.println("Image taken");
        break;
      case FINGERPRINT_NOFINGER:
        Serial.print(".");
        break;
      case FINGERPRINT_PACKETRECIEVEERR:
        return "Communication error";
      case FINGERPRINT_IMAGEFAIL:
        return "Imaging error";
      default:
        return "Unknown error";
    }
  }

  // Konversi gambar ke template
  p = finger.image2Tz(1);
  switch (p) {
    case FINGERPRINT_OK:
      Serial.println("Image converted");
      break;
    case FINGERPRINT_IMAGEMESS:
      return "Image too messy";
    case FINGERPRINT_PACKETRECIEVEERR:
      return "Communication error";
    case FINGERPRINT_FEATUREFAIL:
    case FINGERPRINT_INVALIDIMAGE:
      return "Could not find fingerprint features";
    default:
      return "Unknown error";
  }
  
  Serial.println("Remove finger");
  delay(2000);
  p = 0;
  while (p != FINGERPRINT_NOFINGER) {
    p = finger.getImage();
  }

  Serial.print("ID "); Serial.println(id);
  p = -1;
  Serial.println("Tempatkan jari yang sama lagi!");

  // Ambil gambar sidik jari kedua
  while (p != FINGERPRINT_OK) {
    p = finger.getImage();
    switch (p) {
      case FINGERPRINT_OK:
        Serial.println("Image taken");
        break;
      case FINGERPRINT_NOFINGER:
        Serial.print(".");
        break;
      case FINGERPRINT_PACKETRECIEVEERR:
        return "Communication error";
      case FINGERPRINT_IMAGEFAIL:
        return "Imaging error";
      default:
        return "Unknown error";
    }
  }

  // Konversi gambar kedua ke template
  p = finger.image2Tz(2);
  switch (p) {
    case FINGERPRINT_OK:
      Serial.println("Image converted");
      break;
    case FINGERPRINT_IMAGEMESS:
      return "Image too messy";
    case FINGERPRINT_PACKETRECIEVEERR:
      return "Communication error";
    case FINGERPRINT_FEATUREFAIL:
    case FINGERPRINT_INVALIDIMAGE:
      return "Could not find fingerprint features";
    default:
      return "Unknown error";
  }
  
  // Buat model sidik jari
  Serial.print("Creating model for #");  Serial.println(id);
  p = finger.createModel();
  if (p == FINGERPRINT_OK) {
    Serial.println("Fingerprint Sesuai!");
  } else if (p == FINGERPRINT_PACKETRECIEVEERR) {
    return "Communication error";
  } else if (p == FINGERPRINT_ENROLLMISMATCH) {
    return "Fingerprints tidak sesuai";
  } else {
    return "Unknown error";
  }   
  
  // Simpan model
  Serial.print("ID "); Serial.println(id);
  p = finger.storeModel(id);
  if (p == FINGERPRINT_OK) {
    return "Fingerprint enrolled successfully!";
  } else if (p == FINGERPRINT_PACKETRECIEVEERR) {
    return "Communication error";
  } else if (p == FINGERPRINT_BADLOCATION) {
    return "Tidak bisa menyimpan di penyimpanan.";
  } else if (p == FINGERPRINT_FLASHERR) {
    return "Error writing to flash";
  } else {
    return "Unknown error";
  }
}

String deleteFingerprint(uint8_t id) {
  uint8_t p = finger.deleteModel(id);
  buzzer();
  if (p == FINGERPRINT_OK) {
    return "Fingerprint deleted successfully!";
  } else if (p == FINGERPRINT_PACKETRECIEVEERR) {
    return "Communication error";
  } else if (p == FINGERPRINT_BADLOCATION) {
    return "Could not delete in that location";
  } else if (p == FINGERPRINT_FLASHERR) {
    return "Error writing to flash";
  } else {
    return "Unknown error";
  }
}

uint8_t getFingerprintID() {
  uint8_t p = finger.getImage();
  switch (p) {
    case FINGERPRINT_OK:
      Serial.println("Image taken");
      break;
    case FINGERPRINT_NOFINGER:
      Serial.println("No finger detected");
      return p;
    case FINGERPRINT_PACKETRECIEVEERR:
      Serial.println("Communication error");
      return p;
    case FINGERPRINT_IMAGEFAIL:
      Serial.println("Imaging error");
      return p;
    default:
      Serial.println("Unknown error");
      return p;
  }

  // OK success!
  p = finger.image2Tz();
  switch (p) {
    case FINGERPRINT_OK:
      Serial.println("Image converted");
      break;
    case FINGERPRINT_IMAGEMESS:
      Serial.println("Image too messy");
      return p;
    case FINGERPRINT_PACKETRECIEVEERR:
      Serial.println("Communication error");
      return p;
    case FINGERPRINT_FEATUREFAIL:
      Serial.println("Could not find fingerprint features");
      return p;
    case FINGERPRINT_INVALIDIMAGE:
      Serial.println("Could not find fingerprint features");
      return p;
    default:
      Serial.println("Unknown error");
      return p;
  }

  // OK converted!
  p = finger.fingerFastSearch();
  if (p == FINGERPRINT_OK) {
    Serial.println("Found a print match!");
  } else if (p == FINGERPRINT_PACKETRECIEVEERR) {
    Serial.println("Communication error");
    return p;
  } else if (p == FINGERPRINT_NOTFOUND) {
    Serial.println("Did not find a match");
    return p;
  } else {
    Serial.println("Unknown error");
    return p;
  }

  // found a match!
  Serial.print("Found ID #"); Serial.print(finger.fingerID);
  Serial.print(" with confidence of "); Serial.println(finger.confidence);
  return finger.fingerID;
}

int getFingerprintIDez() {
  uint8_t p = finger.getImage();
  if (p != FINGERPRINT_OK) {
    return -1;
  }

  p = finger.image2Tz();
  if (p != FINGERPRINT_OK) {
    return -3;
  }

  p = finger.fingerFastSearch();
  if (p == FINGERPRINT_OK) {
    return finger.fingerID;
  } else if (p == FINGERPRINT_NOTFOUND) {
    return -2;
  } else {
    return -3;
  }
}

// Fungsi untuk menggunakan dalam loop utama atau saat dipanggil dari web server
//void checkAndProcessFingerprint() {
//  int fingerprintID = getFingerprintIDez();
//  if (fingerprintID != -1) {
//    // Sidik jari terdeteksi
//    Serial.print("Sidik jari terdeteksi dengan ID: ");
//    Serial.println(fingerprintID);
//    
//    // Di sini Anda bisa menambahkan kode untuk mengirim data ke Google Sheets
//    // atau melakukan tindakan lain yang diperlukan
//    sendToGoogleSheets(fingerprintID);
//    
//    // Update LCD jika diperlukan
//    lcd.clear();
//    lcd.print("ID Terdeteksi: ");
//    lcd.print(fingerprintID);
//  }
//}

void sendToGoogleSheets(uint16_t fingerID) {
  WiFiClient client;
  HTTPClient http;

  // URL endpoint dengan parameter ID
  String url = "http://myproject123.com/telegram/absen.php?id=" + String(fingerID);
  
  Serial.print("Connecting to ");
  Serial.println(url);
   http.addHeader("User-Agent", "ESP8266-FingerPrint/1.0");
  
  // Tambahkan timeout yang lebih lama
  http.setTimeout(10000); // 10 detik timeout

  http.begin(client, url);
  
  // Kirim permintaan GET ke server PHP
  int httpResponseCode = http.GET();
  
  if (httpResponseCode > 0) {
    Serial.print("HTTP Response code: ");
    Serial.println(httpResponseCode);
    String payload = http.getString();
    Serial.println("Response from PHP server: " + payload);
    
    // Parsing respons JSON
    DynamicJsonDocument doc(1024);
    DeserializationError error = deserializeJson(doc, payload);
    
    if (error) {
      Serial.println("JSON parsing failed");
      lcd.clear();
      lcd.print("Error parsing");
    } else {
      String name = doc["name"].as<String>();
      String status = doc["status"].as<String>();
      String time = doc["time"].as<String>();
      
      lcd.clear();
      lcd.print(name);
      lcd.setCursor(0, 1);
      lcd.print(status + " " + time);
      
      Serial.println("Name: " + name);
      Serial.println("Status: " + status);
      Serial.println("Time: " + time);
    }
  } else {
    Serial.print("Error code: ");
    Serial.println(httpResponseCode);
    lcd.clear();
    lcd.print("Gagal kirim");
  }
  delay(100);
  http.end();
}

void checkAndProcessFingerprint() {
  int fingerprintID = getFingerprintIDez();
  
  switch (fingerprintID) {
    case -1:  // Tidak ada jari terdeteksi
//      Serial.println("Tidak ada jari terdeteksi.");
      lcd.clear();
      lcd.print("Tempelkan jari");
      break;
      
    case -2:  // Sidik jari tidak sesuai
      Serial.println("Sidik jari tidak sesuai.");
      lcd.clear();
      lcd.print("Sidik jari");
      lcd.setCursor(0, 1);
      lcd.print("tidak sesuai");
      buzzer();  // Uncomment jika Anda ingin memberikan feedback suara
      delay(2000);  // Tampilkan pesan selama 2 detik
      break;
      
    case -3:  // Error komunikasi atau sensor
      Serial.println("Error sensor atau komunikasi.");
      lcd.clear();
      lcd.print("Error sensor");
      // buzzerError();  // Uncomment jika Anda ingin memberikan feedback suara
      delay(2000);  // Tampilkan pesan selama 2 detik
      break;
      
    default:  // Sidik jari terdeteksi dan sesuai
      if (fingerprintID > 0) {
        Serial.print("Sidik jari terdeteksi dengan ID: ");
        Serial.println(fingerprintID);
        lcd.clear();
        lcd.print("ID Terdeteksi: ");
        lcd.print(fingerprintID);
        // buzzerSuccess();  // Uncomment jika Anda ingin memberikan feedback suara
        digitalWrite(D8,1);
        sendToPhpServerAlternative(fingerprintID);
        digitalWrite(D8,0);
        Serial.println("LED NYALA");
      } else {
        Serial.println("Error tidak dikenal.");
        lcd.clear();
        lcd.print("Error tidak");
        lcd.setCursor(0, 1);
        lcd.print("dikenal");
        delay(2000);  // Tampilkan pesan selama 2 detik
      }
      break;
  }
}

void sendToPhpServerAlternative(uint16_t fingerID) {
  WiFiClient client;
  String serverName = "myproject123.com";
  
  if (!client.connect(serverName, 80)) {
    Serial.println("Connection failed");
    return;
  }
  
  String url = "/telegram/absen.php?id=" + String(fingerID);
  
  client.print(String("GET ") + url + " HTTP/1.1\r\n" +
               "Host: " + serverName + "\r\n" +
               "User-Agent: ESP8266-FingerPrint/1.0\r\n" +
               "Connection: close\r\n\r\n");
               
  unsigned long timeout = millis();
  while (client.available() == 0) {
    if (millis() - timeout > 30000) {
      Serial.println(">>> Client Timeout !");
      client.stop();
      return;
    }
  }
  
  // Baca respons
  while(client.available()){
    String line = client.readStringUntil('\r');
//    buzzer();
    Serial.print(line);
  }
}


void buzzer(){
  Serial.println("buzzer nyala");
  digitalWrite(D5,1);
  delay(3000);
  digitalWrite(D5,0);
}