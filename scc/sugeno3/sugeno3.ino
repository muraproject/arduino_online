#include <Arduino.h>
#include <ModbusMaster.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>

// WiFi credentials - ganti dengan data jaringan WiFi Anda
const char* ssid = "Myrep";       // Nama WiFi yang ingin dikoneksikan
const char* password = "87654321";    // Password WiFi

// IP Address statis (opsional)
// Jika Anda ingin ESP32 memiliki IP tetap di jaringan
IPAddress local_IP(192, 168, 1, 200);
IPAddress gateway(192, 168, 1, 1);
IPAddress subnet(255, 255, 255, 0);
IPAddress primaryDNS(8, 8, 8, 8);

// Buat instance AsyncWebServer
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

// Variable untuk menyimpan perintah dari WebSocket
String wsCommand = "";

// Definisi pin untuk komunikasi RS485
#define RX_PIN 17
#define TX_PIN 16
#define SERIAL_COMMUNICATION Serial2

// Definisi pin untuk analog input
#define ANALOG_PIN 18

// Threshold untuk deteksi sumber
#define SOURCE_THRESHOLD 500  // Nilai ini bisa disesuaikan berdasarkan pembacaan sebenarnya

// Variabel untuk status sumber
bool sourceDetected = false;

// Alamat default untuk PZEM-017
#define PZEM_ADDR 0x01

// Buffer to store data for WebSocket clients
String dataBuffer = "";

// Instance untuk ModbusMaster
ModbusMaster node;

// Definisi pin dan parameter PWM
const int pwmPin = 5;        // Pin output PWM (GPIO5)
const int pwmChannel = 0;    // Channel PWM (0-15)
const int pwmFreq = 8500;    // Frekuensi PWM 8.5 kHz
const int pwmResolution = 12; // Resolusi PWM 12-bit (0-4095)
const int pwmMaxValue = 3000; // Nilai maksimum PWM dibatasi 500

// Variabel untuk menyimpan nilai PWM saat ini
int pwmValue = 0;

// Variabel untuk fuzzy control
float targetCurrent = 0.0;  // Target arus yang diinginkan

// Struktur untuk menyimpan data dari PZEM-017
struct PZEM_DATA {
  float voltage;
  float current;
  float power;
};

// Fungsi untuk logika fuzzy Sugeno berdasarkan error arus
int fuzzyPWMControl(float currentError) {
  // Fungsi keanggotaan fuzzy sederhana untuk error
  // NB: Negative Big, NS: Negative Small, ZE: Zero, PS_val: Positive Small, PB_val: Positive Big
  
  float NB = 0.0, NS = 0.0, ZE = 0.0, PS_val = 0.0, PB_val = 0.0;
  
  // Fuzzifikasi error dengan rentang yang lebih sensitif
  if (currentError <= -0.5) {
    NB = 1.0;  // Error negatif besar - kurangi PWM
  } else if (currentError > -0.5 && currentError < -0.1) {
    NB = (-0.1 - currentError) / 0.4;
    NS = (currentError + 0.5) / 0.4;
  } else if (currentError >= -0.1 && currentError < 0) {
    NS = (-currentError) / 0.1;
    ZE = (currentError + 0.1) / 0.1;
  } else if (currentError >= 0 && currentError < 0.1) {
    ZE = (0.1 - currentError) / 0.1;
    PS_val = currentError / 0.1;
  } else if (currentError >= 0.1 && currentError < 0.5) {
    PS_val = (0.5 - currentError) / 0.4;
    PB_val = (currentError - 0.1) / 0.4;
  } else {  // currentError >= 0.5
    PB_val = 1.0;  // Error positif besar - naikkan PWM agresif
  }
  
  // Output singleton untuk Sugeno - lebih agresif naik daripada turun
  int NBOutput = -10;   // Decrease PWM by 1
  int NSOutput = -1;   // Decrease PWM by 1
  int ZEOutput = 0;    // No change
  int PSOutput = 1;    // Increase PWM by 3 (lebih agresif naik)
  int PBOutput = 10;    // Increase PWM by 5 (jauh lebih agresif naik)
  
  // Defuzzifikasi (weighted average - Sugeno)
  float totalWeight = NB + NS + ZE + PS_val + PB_val;
  
  if (totalWeight > 0) {
    int deltaOutput = (int)((NB * NBOutput + NS * NSOutput + ZE * ZEOutput + PS_val * PSOutput + PB_val * PBOutput) / totalWeight);
    
    // Jika masih di bawah target dan error positif, minimal naik 1
    if (currentError > 0.05 && deltaOutput == 0) {
      deltaOutput = 1;  // Pastikan selalu naik jika target belum tercapai
    }
    
    // Hitung nilai PWM baru
    int newPWM = pwmValue + deltaOutput;
    
    // Batasi dalam rentang yang valid
    return constrain(newPWM, 0, pwmMaxValue);
  }
  
  return pwmValue; // Jika tidak ada aturan yang terpicu, pertahankan nilai lama
}

// Handler for WebSocket events
void onWebSocketEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, 
                     AwsEventType type, void *arg, uint8_t *data, size_t len) {
  switch (type) {
    case WS_EVT_CONNECT:
      // Client connected
      Serial.printf("WebSocket client #%u connected from %s\n", client->id(), client->remoteIP().toString().c_str());
      break;
    case WS_EVT_DISCONNECT:
      // Client disconnected
      Serial.printf("WebSocket client #%u disconnected\n", client->id());
      break;
    case WS_EVT_DATA:
      // Data received
      handleWebSocketMessage(arg, data, len);
      break;
    case WS_EVT_PONG:
    case WS_EVT_ERROR:
      break;
  }
}

// Handle WebSocket message
void handleWebSocketMessage(void *arg, uint8_t *data, size_t len) {
  AwsFrameInfo *info = (AwsFrameInfo*)arg;
  if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT) {
    // Ensure null termination
    data[len] = 0;
    wsCommand = String((char*)data);
    
    // Try to convert to float for target current
    float newCurrent = wsCommand.toFloat();
    
    // Update target current if value is valid
    if (newCurrent >= 0) {
      targetCurrent = newCurrent;
      String response = "Target arus diatur ke: " + String(targetCurrent, 2) + " A";
      Serial.println(response);
      ws.textAll(response);
    }
  }
}

// Simple HTML web interface
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML>
<html>
<head>
  <title>PZEM-017 Controller</title>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <style>
    body { font-family: Arial, sans-serif; margin: 0; padding: 20px; }
    .container { max-width: 600px; margin: 0 auto; }
    h1 { color: #333; }
    .card { background-color: white; box-shadow: 0 4px 8px 0 rgba(0,0,0,0.2); padding: 20px; margin-bottom: 20px; border-radius: 5px; }
    .button { background-color: #4CAF50; border: none; color: white; padding: 10px 20px; text-align: center; display: inline-block; font-size: 16px; margin: 4px 2px; cursor: pointer; border-radius: 5px; }
    .switch-container { display: flex; align-items: center; margin: 10px 0; }
    .switch { position: relative; display: inline-block; width: 60px; height: 34px; margin-right: 10px; }
    .switch input { opacity: 0; width: 0; height: 0; }
    .slider { position: absolute; cursor: pointer; top: 0; left: 0; right: 0; bottom: 0; background-color: #ccc; transition: .4s; border-radius: 34px; }
    .slider:before { position: absolute; content: ""; height: 26px; width: 26px; left: 4px; bottom: 4px; background-color: white; transition: .4s; border-radius: 50%; }
    input:checked + .slider { background-color: #2196F3; }
    input:checked + .slider:before { transform: translateX(26px); }
    input { width: 100%; padding: 12px; border: 1px solid #ccc; border-radius: 4px; margin: 10px 0; box-sizing: border-box; }
    #log { background-color: #f8f8f8; border: 1px solid #ddd; height: 300px; overflow-y: scroll; padding: 10px; white-space: pre-wrap; font-family: monospace; font-size: 14px; }
    .flex-between { display: flex; justify-content: space-between; align-items: center; flex-wrap: wrap; }
    .controls { display: flex; gap: 15px; }
    .timestamp { color: #888; font-size: 12px; }
    .status-indicator { padding: 5px 10px; border-radius: 4px; font-weight: bold; display: inline-block; margin-top: 10px; }
    .status-detected { background-color: #dff0d8; color: #3c763d; }
    .status-not-detected { background-color: #f2dede; color: #a94442; }
  </style>
</head>
<body>
  <div class="container">
    <h1>PZEM-017 Controller</h1>
    <div class="card">
      <h2>Set Target Current</h2>
      <input type="number" id="currentInput" placeholder="Enter target current (A)" step="0.1">
      <button class="button" onclick="setTargetCurrent()">Set</button>
      <div id="sourceStatus" class="status-not-detected">Sumber: Tidak Terdeteksi</div>
    </div>
    <div class="card">
      <div class="flex-between">
        <h2>System Monitor</h2>
        <div class="controls">
          <div class="switch-container">
            <label class="switch">
              <input type="checkbox" id="autoScrollToggle" checked>
              <span class="slider"></span>
            </label>
            <span>Auto-scroll</span>
          </div>
          <div class="switch-container">
            <label class="switch">
              <input type="checkbox" id="timestampToggle">
              <span class="slider"></span>
            </label>
            <span>Timestamp</span>
          </div>
        </div>
      </div>
      <div id="log">Connecting to device...</div>
    </div>
  </div>
  <script>
    var gateway = `ws://${window.location.hostname}/ws`;
    var websocket;
    var autoScroll = true;
    var showTimestamp = false;
    
    window.addEventListener('load', onLoad);
    
    function initWebSocket() {
      console.log('Trying to open a WebSocket connection...');
      websocket = new WebSocket(gateway);
      websocket.onopen = onOpen;
      websocket.onclose = onClose;
      websocket.onmessage = onMessage;
    }
    
    function onOpen(event) {
      console.log('Connection opened');
      document.getElementById('log').innerHTML = "Connected to device";
    }
    
    function onClose(event) {
      console.log('Connection closed');
      setTimeout(initWebSocket, 2000);
    }
    
    function onMessage(event) {
      var logElement = document.getElementById('log');
      var timestamp = '';
      
      if (showTimestamp) {
        var now = new Date();
        timestamp = '<span class="timestamp">[' + 
          now.getHours().toString().padStart(2, '0') + ':' + 
          now.getMinutes().toString().padStart(2, '0') + ':' + 
          now.getSeconds().toString().padStart(2, '0') + '.' +
          now.getMilliseconds().toString().padStart(3, '0') + '] </span>';
      }
      
      // Update source status if the message contains source information
      if (event.data.includes("Sumber:")) {
        var sourceStatusElement = document.getElementById('sourceStatus');
        if (event.data.includes("Terdeteksi")) {
          sourceStatusElement.className = "status-indicator status-detected";
          sourceStatusElement.innerHTML = "Sumber: Terdeteksi";
        } else if (event.data.includes("Tidak Terdeteksi")) {
          sourceStatusElement.className = "status-indicator status-not-detected";
          sourceStatusElement.innerHTML = "Sumber: Tidak Terdeteksi";
        }
      }
      
      // Tambah baris baru di BAWAH (seperti Arduino Serial Monitor)
      logElement.innerHTML = logElement.innerHTML + timestamp + event.data + '\n';
      
      // Scroll ke bawah jika auto-scroll diaktifkan
      if (autoScroll) {
        logElement.scrollTop = logElement.scrollHeight;
      }
    }
    
    function onLoad(event) {
      initWebSocket();
      
      // Set up auto-scroll toggle
      document.getElementById('autoScrollToggle').addEventListener('change', function() {
        autoScroll = this.checked;
        if (autoScroll) {
          document.getElementById('log').scrollTop = document.getElementById('log').scrollHeight;
        }
      });
      
      // Set up timestamp toggle
      document.getElementById('timestampToggle').addEventListener('change', function() {
        showTimestamp = this.checked;
      });
    }
    
    function setTargetCurrent() {
      var currentValue = document.getElementById('currentInput').value;
      if (currentValue !== '') {
        websocket.send(currentValue);
        document.getElementById('currentInput').value = '';
      }
    }
  </script>
</body>
</html>
)rawliteral";

void setup() {
  // Inisialisasi Serial untuk debugging
  Serial.begin(115200);
  Serial.println("ESP32-S3 - PZEM-017 DC Meter + Fuzzy Sugeno PWM Controller + Source Detection");
  
  // Koneksi ke WiFi
  Serial.print("Menghubungkan ke WiFi ");
  Serial.println(ssid);
  
  // Jika ingin menggunakan IP statis, uncomment baris berikut
  // if (!WiFi.config(local_IP, gateway, subnet, primaryDNS)) {
  //   Serial.println("Gagal mengkonfigurasi IP Statis");
  // }
  
  WiFi.begin(ssid, password);
  
  // Tunggu sampai terhubung
  int timeout_counter = 0;
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    timeout_counter++;
    
    // Jika tidak bisa terhubung dalam 20 detik (40 x 500ms), restart ESP
    if (timeout_counter >= 40) {
      Serial.println("Gagal terhubung ke WiFi, restarting...");
      ESP.restart();
    }
  }
  
  Serial.println("");
  Serial.println("WiFi terhubung!");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());
  
  // Konfigurasi pin analog
  pinMode(ANALOG_PIN, INPUT);
  
  // Initialize WebSocket
  ws.onEvent(onWebSocketEvent);
  server.addHandler(&ws);
  
  // Route for root / web page
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/html", index_html);
  });
  
  // Start server
  server.begin();
  
  // Inisialisasi Serial2 untuk komunikasi RS485
  SERIAL_COMMUNICATION.begin(9600, SERIAL_8N1, RX_PIN, TX_PIN);
  
  // Inisialisasi ModbusMaster
  node.begin(PZEM_ADDR, SERIAL_COMMUNICATION);
  
  // Konfigurasi PWM
  ledcSetup(pwmChannel, pwmFreq, pwmResolution);
  ledcAttachPin(pwmPin, pwmChannel);
  
  // Cetak panduan penggunaan ke Serial
  String helpMessage = "System initialized\nBuka http://" + WiFi.localIP().toString() + " di browser Anda untuk mengontrol sistem";
  Serial.println(helpMessage);
  
  delay(1000);
}

// Fungsi untuk membaca data dari PZEM-017
PZEM_DATA readPZEM() {
  PZEM_DATA data;
  
  // Inisialisasi data dengan nilai default
  data.voltage = 0.0;
  data.current = 0.0;
  data.power = 0.0;
  
  // Membaca semua register data dalam satu permintaan (alamat 0x0000, baca 3 register)
  if (node.readInputRegisters(0x0000, 3) == node.ku8MBSuccess) {
    // Register 0x0000: Tegangan
    data.voltage = node.getResponseBuffer(0) * 0.01; // Konversi ke volt
    
    // Register 0x0001: Arus
    data.current = node.getResponseBuffer(1) * 0.01; // Konversi ke ampere
    
    // Register 0x0002: Daya
    data.power = node.getResponseBuffer(2) * 0.1; // Konversi ke watt
  } else {
    // Silent fail - tidak menampilkan pesan error setiap kali
  }
  
  return data;
}

// Fungsi untuk menghitung daya secara manual
float calculatePower(float voltage, float current) {
  return voltage * current;
}

// Fungsi untuk membaca nilai analog dan menentukan status sumber
bool checkSourceDetection() {
  int analogValue = analogRead(ANALOG_PIN);
  
  // Periksa apakah nilai analog di atas threshold
  bool isSourceDetected = (analogValue > SOURCE_THRESHOLD);
  
  // Jika status berubah, kirim notifikasi
  if (isSourceDetected != sourceDetected) {
    sourceDetected = isSourceDetected;
    
    String statusMessage = "Sumber: " + String(sourceDetected ? "Terdeteksi" : "Tidak Terdeteksi");
    statusMessage += " (Nilai Analog: " + String(analogValue) + ")";
    
    Serial.println(statusMessage);
    if (ws.count() > 0) {
      ws.textAll(statusMessage);
    }
  }
  
  return sourceDetected;
}

void processCommand() {
  // Periksa apakah ada data serial yang masuk dari USB
  if (Serial.available() > 0) {
    // Baca input serial sebagai string
    String input = Serial.readStringUntil('\n');
    input.trim(); // Hapus whitespace
    
    // Jika input adalah numerik, gunakan sebagai target arus
    if (input.length() > 0) {
      // Coba konversi ke float
      float newCurrent = input.toFloat();
      
      // Update target arus jika nilai valid
      if (newCurrent >= 0) {
        targetCurrent = newCurrent;
        String response = "Target arus diatur ke: " + String(targetCurrent, 2) + " A";
        Serial.println(response);
        ws.textAll(response);
      } else {
        String errorMsg = "Error: Masukkan nilai arus positif";
        Serial.println(errorMsg);
        ws.textAll(errorMsg);
      }
    }
  }
  
  // WebSocket commands are handled in the WebSocket event handler
}

void loop() {
  // Periksa status koneksi WiFi
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("Koneksi WiFi terputus, menghubungkan kembali...");
    WiFi.begin(ssid, password);
    
    // Tunggu maksimal 10 detik untuk koneksi
    int retryCount = 0;
    while (WiFi.status() != WL_CONNECTED && retryCount < 20) {
      delay(500);
      Serial.print(".");
      retryCount++;
    }
    
    if (WiFi.status() == WL_CONNECTED) {
      Serial.println("\nWiFi terhubung kembali");
      Serial.print("IP Address: ");
      Serial.println(WiFi.localIP());
    } else {
      Serial.println("\nGagal menghubungkan kembali, melanjutkan operasi tanpa WiFi");
    }
  }
  
  // Proses perintah dari Serial
  processCommand();
  
  // Periksa deteksi sumber
  bool isSourceDetected = checkSourceDetection();
  
  // Baca data dari PZEM-017
  PZEM_DATA data = readPZEM();
  
  // Hitung daya secara manual sebagai backup jika pembacaan dari sensor adalah 0
  float calculatedPower = calculatePower(data.voltage, data.current);
  
  // Jika daya dari sensor adalah 0 tapi tegangan dan arus tidak 0, gunakan daya yang dihitung
  if (data.power == 0 && data.voltage > 0 && data.current > 0) {
    data.power = calculatedPower;
  }
  
  // Update PWM berdasarkan fuzzy control
  float currentError = targetCurrent - data.current;
  int newPWM = fuzzyPWMControl(currentError);
    
  // Terapkan PWM baru jika ada perubahan
  if (newPWM != pwmValue) {
    pwmValue = newPWM;
    ledcWrite(pwmChannel, pwmValue);
  }
  
  // Jika target arus > 0.05 dan arus saat ini < 80% dari target dan PWM belum maksimal, paksa naikkan PWM
  if (targetCurrent > 0.05 && data.current < (targetCurrent * 0.8) && pwmValue < pwmMaxValue) {
    pwmValue += 1;  // Paksa naik 1 agar mencapai target
    ledcWrite(pwmChannel, pwmValue);
  }
  
  // Buat string data untuk ditampilkan di Serial dan WebSocket
  String dataOutput = "------------------------\n";
  dataOutput += "Status Sumber: " + String(isSourceDetected ? "Terdeteksi" : "Tidak Terdeteksi") + "\n";
  dataOutput += "Tegangan: " + String(data.voltage, 2) + " V\n";
  dataOutput += "Arus: " + String(data.current, 2) + " A (Target: " + String(targetCurrent, 2) + " A)\n";
  dataOutput += "Error Arus: " + String(currentError, 3) + " A\n";
  dataOutput += "Daya: " + String(data.power, 2) + " W\n";
  dataOutput += "PWM: " + String(pwmValue) + "/" + String(pwmMaxValue) + " (" + String((float)pwmValue / pwmMaxValue * 100.0, 2) + "%)\n";
  
  // Kirim data ke Serial
  Serial.print(dataOutput);
  
  // Kirim data ke WebSocket clients jika ada yang terhubung
  if (ws.count() > 0) {
    ws.textAll(dataOutput);
  }
  
  // Clean up inactive WebSocket clients
  ws.cleanupClients();
  
  // Tunggu sebelum membaca lagi
  delay(100);
}