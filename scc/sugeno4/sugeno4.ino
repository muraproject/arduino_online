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
// IPAddress local_IP(192, 168, 1, 200);
// IPAddress gateway(192, 168, 1, 1);
// IPAddress subnet(255, 255, 255, 0);
// IPAddress primaryDNS(8, 8, 8, 8);

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

// Threshold untuk deteksi sumber - nilai ini mungkin perlu disesuaikan
// Berdasarkan tegangan yang Anda baca pada pin analog
#define SOURCE_THRESHOLD 100  // Nilai ini dikurangi dari 500 menjadi 100
#define SOURCE_HYSTERESIS 20   // Untuk mencegah false triggering

// Variabel untuk status sumber dan debouncing
bool sourceDetected = false;
int sourceDetectionCounter = 0;  // Counter untuk debouncing
#define SOURCE_DETECTION_DEBOUNCE 3  // Butuh 3 pembacaan berturut-turut untuk mengubah status

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
const int pwmMaxValue = 3000; // Nilai maksimum PWM dibatasi 3000

// Variabel untuk menyimpan nilai PWM saat ini
int pwmValue = 0;

// SCC parameters
const float MAX_CHARGING_CURRENT = 3.0;  // Target arus charging maksimal (A)
const float VOLTAGE_LIMIT = 13.6;        // Batas tegangan charging (V)
const float CURRENT_THRESHOLD = 0.1;     // Threshold arus minimal untuk mendeteksi charging

// Variabel untuk sistem charging
float targetCurrent = MAX_CHARGING_CURRENT;  // Target arus charging awal
bool isCharging = false;                     // Status charging aktif
enum ChargingMode {
  BULK,     // Mode arus konstan
  ABSORPTION // Mode tegangan konstan
};
ChargingMode chargingMode = BULK;

// Struktur untuk menyimpan data dari PZEM-017
struct PZEM_DATA {
  float voltage;
  float current;
  float power;
};

// Fungsi untuk logika fuzzy Sugeno berdasarkan error arus atau tegangan
int fuzzyPWMControl(float currentError, float voltageError) {
  // Jika sumber tidak terdeteksi, langsung return 0
  if (!sourceDetected) {
    return 0;
  }
  
  // Jika tegangan diatas batas, gunakan error tegangan
  if (voltageError < 0) {
    // Kondisi tegangan melebihi batas
    if (voltageError < -0.1) {
      // Lebih agresif menurunkan PWM jika tegangan terlalu tinggi
      return max(0, pwmValue - 5);
    } else {
      // Pelan-pelan turunkan PWM untuk menjaga tegangan
      return max(0, pwmValue - 1);
    }
  }
  
  // Jika tegangan masih dibawah batas, gunakan error arus
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
  int NBOutput = -10;   // Decrease PWM by 10
  int NSOutput = -1;    // Decrease PWM by 1
  int ZEOutput = 0;     // No change
  int PSOutput = 1;     // Increase PWM by 1
  int PBOutput = 10;    // Increase PWM by 10
  
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
    
    // Update target current if value is valid, but cap at MAX_CHARGING_CURRENT
    if (newCurrent >= 0) {
      // Batasi nilai maksimum ke MAX_CHARGING_CURRENT
      if (newCurrent > MAX_CHARGING_CURRENT) {
        newCurrent = MAX_CHARGING_CURRENT;
      }
      
      targetCurrent = newCurrent;
      String response = "Target arus diatur ke: " + String(targetCurrent, 2) + " A";
      if (newCurrent == MAX_CHARGING_CURRENT) {
        response += " (Maksimum)";
      }
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
  Serial.println("ESP32-S3 - Solar Charge Controller (SCC) with PZEM-017");
  
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
  
  // Konfigurasi pin analog dengan pull-down internal
  pinMode(ANALOG_PIN, INPUT_PULLDOWN);  // Gunakan resistor pull-down internal
  
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
  
  // Set target arus awal ke MAX_CHARGING_CURRENT
  targetCurrent = MAX_CHARGING_CURRENT;
  
  // Cetak panduan penggunaan ke Serial
  String helpMessage = "Solar Charge Controller initialized\n";
  helpMessage += "Max Charging Current: " + String(MAX_CHARGING_CURRENT, 1) + " A\n";
  helpMessage += "Voltage Limit: " + String(VOLTAGE_LIMIT, 1) + " V\n";
  helpMessage += "Source Threshold: " + String(SOURCE_THRESHOLD) + " (nilai analog)\n";
  helpMessage += "Buka http://" + WiFi.localIP().toString() + " di browser Anda untuk monitoring sistem";
  Serial.println(helpMessage);
  
  // Kirim ke WebSocket jika ada klien yang terhubung
  if (ws.count() > 0) {
    ws.textAll(helpMessage);
  }
  
  // Baca nilai awal pin analog untuk referensi
  int initialAnalogValue = analogRead(ANALOG_PIN);
  Serial.println("Nilai awal Pin 18: " + String(initialAnalogValue));
  
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

// Fungsi untuk membaca nilai analog dan menentukan status sumber dengan debouncing
bool checkSourceDetection() {
  int analogValue = analogRead(ANALOG_PIN);
  
  // Debug: Selalu tampilkan nilai analog untuk membantu penyesuaian threshold
  static int lastAnalogValue = -1;
  if (abs(analogValue - lastAnalogValue) > 5) {  // Hanya tampilkan jika berubah signifikan
    String debugMsg = "Nilai Analog Pin 18: " + String(analogValue);
    Serial.println(debugMsg);
    lastAnalogValue = analogValue;
  }
  
  // Logika deteksi dengan hysteresis untuk mencegah false triggering
  bool currentReading;
  if (sourceDetected) {
    // Jika sebelumnya terdeteksi, gunakan threshold yang lebih rendah untuk mati
    currentReading = (analogValue > (SOURCE_THRESHOLD - SOURCE_HYSTERESIS));
  } else {
    // Jika sebelumnya tidak terdeteksi, gunakan threshold yang lebih tinggi untuk aktif
    currentReading = (analogValue > (SOURCE_THRESHOLD + SOURCE_HYSTERESIS));
  }
  
  // Implementasi debouncing
  if (currentReading != sourceDetected) {
    sourceDetectionCounter++;
    if (sourceDetectionCounter >= SOURCE_DETECTION_DEBOUNCE) {
      // Status berubah setelah beberapa pembacaan konsisten
      sourceDetected = currentReading;
      sourceDetectionCounter = 0;
      
      // Kirim notifikasi perubahan status
      String statusMessage = "Sumber: " + String(sourceDetected ? "Terdeteksi" : "Tidak Terdeteksi");
      statusMessage += " (Nilai Analog: " + String(analogValue) + ")";
      
      Serial.println(statusMessage);
      if (ws.count() > 0) {
        ws.textAll(statusMessage);
      }
      
      // Jika sumber baru terdeteksi, kirim informasi tambahan
      if (sourceDetected) {
        String infoMsg = "Memulai charging dengan target arus " + String(targetCurrent, 2) + " A dan batas tegangan " + String(VOLTAGE_LIMIT, 2) + " V";
        Serial.println(infoMsg);
        if (ws.count() > 0) {
          ws.textAll(infoMsg);
        }
      } else {
        // Jika sumber hilang, reset PWM
        pwmValue = 0;
        ledcWrite(pwmChannel, pwmValue);
        
        String infoMsg = "Charging dihentikan karena sumber tidak terdeteksi";
        Serial.println(infoMsg);
        if (ws.count() > 0) {
          ws.textAll(infoMsg);
        }
      }
    }
  } else {
    // Reset counter jika pembacaan stabil
    sourceDetectionCounter = 0;
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
      
      // Update target arus jika nilai valid, tapi batasi maksimum ke MAX_CHARGING_CURRENT
      if (newCurrent >= 0) {
        // Batasi nilai maksimum ke MAX_CHARGING_CURRENT
        if (newCurrent > MAX_CHARGING_CURRENT) {
          newCurrent = MAX_CHARGING_CURRENT;
        }
        
        targetCurrent = newCurrent;
        String response = "Target arus diatur ke: " + String(targetCurrent, 2) + " A";
        if (newCurrent == MAX_CHARGING_CURRENT) {
          response += " (Maksimum)";
        }
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
  
  // Tentukan mode charging berdasarkan tegangan dan arus
  if (data.voltage >= VOLTAGE_LIMIT) {
    chargingMode = ABSORPTION; // Masuk ke mode tegangan konstan
  } else if (data.voltage < (VOLTAGE_LIMIT - 0.3)) {
    chargingMode = BULK; // Kembali ke mode arus konstan
  }
  
  // Set nilai PWM berdasarkan deteksi sumber dan mode charging
  if (!isSourceDetected) {
    // Jika sumber tidak terdeteksi, matikan PWM
    pwmValue = 0;
    isCharging = false;
  } else {
    // Sumber terdeteksi, tentukan PWM berdasarkan mode
    isCharging = true;
    
    // Hitung error berdasarkan mode
    float currentError = targetCurrent - data.current;
    float voltageError = VOLTAGE_LIMIT - data.voltage;
    
    // Update PWM berdasarkan fuzzy control dan mode charging
    int newPWM = fuzzyPWMControl(currentError, voltageError);
    
    // Terapkan PWM baru jika ada perubahan
    if (newPWM != pwmValue) {
      pwmValue = newPWM;
    }
    
    // Jika dalam mode BULK dan arus terlalu rendah, paksa naikkan PWM
    if (chargingMode == BULK && targetCurrent > CURRENT_THRESHOLD && 
        data.current < (targetCurrent * 0.8) && pwmValue < pwmMaxValue) {
      pwmValue += 1;  // Paksa naik 1 agar mencapai target
    }
  }
  
  // Terapkan nilai PWM
  ledcWrite(pwmChannel, pwmValue);
  
  // Tentukan status charging untuk tampilan
  String chargingStatus;
  if (!isSourceDetected) {
    chargingStatus = "Mati (Sumber Tidak Terdeteksi)";
  } else if (chargingMode == BULK) {
    chargingStatus = "Bulk Charging (Arus Konstan)";
  } else {
    chargingStatus = "Absorption Charging (Tegangan Konstan)";
  }
  
  // Buat string data untuk ditampilkan di Serial dan WebSocket
  String dataOutput = "------------------------\n";
  dataOutput += "Status Sumber: " + String(isSourceDetected ? "Terdeteksi" : "Tidak Terdeteksi") + "\n";
  dataOutput += "Mode Charging: " + chargingStatus + "\n";
  dataOutput += "Tegangan: " + String(data.voltage, 2) + " V (Limit: " + String(VOLTAGE_LIMIT, 1) + " V)\n";
  dataOutput += "Arus: " + String(data.current, 2) + " A (Target: " + String(targetCurrent, 2) + " A)\n";
  
  // Tampilkan error sesuai mode
  if (chargingMode == BULK) {
    dataOutput += "Error Arus: " + String(targetCurrent - data.current, 3) + " A\n";
  } else {
    dataOutput += "Error Tegangan: " + String(VOLTAGE_LIMIT - data.voltage, 3) + " V\n";
  }
  
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