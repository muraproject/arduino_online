#include <Arduino.h>
#include <ModbusMaster.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <Wire.h>               // Library untuk I2C
#include <LiquidCrystal_I2C.h>  // Library untuk LCD I2C

// WiFi credentials - ganti dengan data jaringan WiFi Anda
const char* ssid = "Myrep";       // Nama WiFi yang ingin dikoneksikan
const char* password = "87654321";    // Password WiFi

// Definisi pin
#define RX_PIN 17               // Pin RX untuk komunikasi RS485
#define TX_PIN 16               // Pin TX untuk komunikasi RS485
#define ANALOG_PIN 18           // Pin analog untuk deteksi sumber
#define DO_PIN 15               // Pin analog untuk sensor DO
#define RELAY_PIN 4             // Pin relay untuk kontrol aerator
#define SDA_PIN 8               // Pin SDA pada ESP32 S3
#define SCL_PIN 9               // Pin SCL pada ESP32 S3
#define PWM_PIN 5               // Pin output PWM (GPIO5)

// Definisi komunikasi serial
#define SERIAL_COMMUNICATION Serial2

// Konfigurasi PWM
#define PWM_CHANNEL 0           // Channel PWM (0-15)
#define PWM_FREQ 8500           // Frekuensi PWM 8.5 kHz
#define PWM_RESOLUTION 12       // Resolusi PWM 12-bit (0-4095)
#define PWM_MAX_VALUE 3000      // Nilai maksimum PWM dibatasi 3000

// Threshold dan parameter sistem
#define SOURCE_THRESHOLD 500    // Threshold untuk deteksi sumber
#define DO_THRESHOLD 1.0        // Threshold DO untuk mengaktifkan relay (mg/L)
#define PZEM_ADDR 0x01          // Alamat default untuk PZEM-017

// Parameter sensor DO
#define VREF 3300               // VREF untuk ESP32 (3.3V = 3300 mV)
#define ADC_RES 4095            // Resolusi ADC ESP32 (12-bit: 0-4095)
#define TWO_POINT_CALIBRATION 0 // Mode Kalibrasi 0=Single point, 1=Two point
#define READ_TEMP (25)          // Suhu air saat ini dalam ℃
#define CAL1_V (1600)           // mV
#define CAL1_T (25)             // ℃
#define CAL2_V (1300)           // mV
#define CAL2_T (15)             // ℃

// Parameter untuk stabilisasi pembacaan DO
#define SAMPLE_INTERVAL 2       // Interval antara sampel (ms) - lower for faster DO processing
#define NUM_SAMPLES 20          // Jumlah sampel untuk median - reduced for faster processing
#define WINDOW_SIZE 5           // Ukuran window untuk median bergerak - reduced for faster processing
#define STABILITY_THRESHOLD 5   // Ambang batas stabilitas (mV)
#define STABILITY_COUNT 3       // Berapa kali harus stabil sebelum dianggap valid - reduced for faster stabilization

// Parameter charging
#define MAX_CHARGING_CURRENT 3.0  // Target arus charging maksimal (A)
#define VOLTAGE_LIMIT 13.6        // Batas tegangan charging (V)
#define CURRENT_THRESHOLD 0.1     // Threshold arus minimal untuk deteksi charging

// Interval task dan update (only for Core 0)
#define LCD_UPDATE_INTERVAL 500   // Update LCD setiap 500ms
#define WS_UPDATE_INTERVAL 1000   // Update WebSocket setiap 1000ms

// DO solubility table (saturation DO dalam air pada tekanan 101.325 kPa, unit: μg/L)
const uint16_t DO_Table[41] = {
    14460, 14220, 13820, 13440, 13090, 12740, 12420, 12110, 11810, 11530,
    11260, 11010, 10770, 10530, 10300, 10080, 9860, 9660, 9460, 9270,
    9080, 8900, 8730, 8570, 8410, 8250, 8110, 7960, 7820, 7690,
    7560, 7430, 7300, 7180, 7070, 6950, 6840, 6730, 6630, 6530, 6410
};

// Server dan WebSocket
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");
String wsCommand = "";

// Mutex for task synchronization
portMUX_TYPE dataMutex = portMUX_INITIALIZER_UNLOCKED;

// Multicore task handles
TaskHandle_t Core0Task;

// Inisialisasi LCD
LiquidCrystal_I2C lcd(0x27, 20, 4);

// ModbusMaster instance
ModbusMaster node;

// Struktur untuk menyimpan data dari PZEM-017
struct PZEM_DATA {
  float voltage;
  float current;
  float power;
};

// Mode charging
enum ChargingMode {
  BULK,     // Mode arus konstan
  ABSORPTION // Mode tegangan konstan
};

// Shared variables between cores
struct SharedData {
  // PZEM data
  PZEM_DATA pzem;
  
  // DO Sensor data
  uint8_t do_temperature;
  uint16_t DO_ADC_Raw;
  uint16_t DO_ADC_Voltage;
  uint16_t DO_Value;
  float DO_mgL;
  bool do_isStable;
  
  // System status
  bool sourceDetected;
  ChargingMode chargingMode;
  float targetCurrent;
  int pwmValue;
  bool isCharging;
  bool relayState;
  
  // Task timing (only used by Core 0)
  unsigned long lastLcdUpdate;
  unsigned long lastWebSocketUpdate;
  
  // Flags for communication between cores
  bool dataUpdated;  // Flag to indicate new data is available for Core 0
};

// Initialize shared data
SharedData sharedData = {
  .pzem = {0.0, 0.0, 0.0},
  .do_temperature = READ_TEMP,
  .DO_ADC_Raw = 0,
  .DO_ADC_Voltage = 0,
  .DO_Value = 0,
  .DO_mgL = 0.0,
  .do_isStable = false,
  .sourceDetected = false,
  .chargingMode = BULK,
  .targetCurrent = MAX_CHARGING_CURRENT,
  .pwmValue = 0,
  .isCharging = false,
  .relayState = false,
  .lastLcdUpdate = 0,
  .lastWebSocketUpdate = 0,
  .dataUpdated = false
};

// DO sensor arrays
uint16_t do_samples[NUM_SAMPLES];
uint16_t do_medianWindow[WINDOW_SIZE];
uint16_t do_lastStableVoltage = 0;
uint8_t do_stabilityCounter = 0;

// Function prototypes
void updateDOSensor();
void updateLCD(PZEM_DATA data, bool isSourceDetected, ChargingMode mode, float DO_level, bool relayState);
PZEM_DATA readPZEM();
bool checkSourceDetection();
int fuzzyPWMControl(float currentError, float voltageError);
void processCommand();
void Core0Task_UI(void *pvParameters);
void handleWebSocketMessage(void *arg, uint8_t *data, size_t len);

// Web interface HTML
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML>
<html>
<head>
  <title>PZEM-017 Controller with DO Sensor</title>
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
    .sensor-data { margin-top: 10px; display: grid; grid-template-columns: 1fr 1fr; gap: 10px; }
    .sensor-item { background-color: #f8f8f8; padding: 10px; border-radius: 5px; }
    .active { background-color: #dff0d8; }
    .inactive { background-color: #f2dede; }
  </style>
</head>
<body>
  <div class="container">
    <h1>PZEM-017 Controller with DO Sensor</h1>
    <div class="card">
      <h2>Set Target Current</h2>
      <input type="number" id="currentInput" placeholder="Enter target current (A)" step="0.1">
      <button class="button" onclick="setTargetCurrent()">Set</button>
      <div id="sourceStatus" class="status-not-detected">Sumber: Tidak Terdeteksi</div>
      
      <div class="sensor-data">
        <div class="sensor-item">
          <h3>Dissolved Oxygen</h3>
          <div id="doValue">DO: -- mg/L</div>
          <div id="doStatus">Status: --</div>
        </div>
        <div class="sensor-item" id="relayContainer">
          <h3>Aerator Relay</h3>
          <div id="relayStatus" class="inactive">Status: OFF</div>
          <div>Triggers when DO < 1.0 mg/L</div>
        </div>
      </div>
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
      
      // Update DO sensor data if available
      if (event.data.includes("DO:")) {
        const doMatch = event.data.match(/DO: ([\d.]+)mg\/L/);
        if (doMatch && doMatch[1]) {
          document.getElementById('doValue').innerHTML = "DO: " + doMatch[1] + " mg/L";
          
          // Update relay status based on DO level
          const doValue = parseFloat(doMatch[1]);
          const relayStatusElement = document.getElementById('relayStatus');
          const relayContainer = document.getElementById('relayContainer');
          
          if (doValue < 1.0) {
            relayStatusElement.className = "active";
            relayStatusElement.innerHTML = "Status: ON (Aerator Active)";
            relayContainer.className = "sensor-item active";
          } else {
            relayStatusElement.className = "inactive";
            relayStatusElement.innerHTML = "Status: OFF";
            relayContainer.className = "sensor-item";
          }
        }
        
        const doStatusMatch = event.data.includes("STABIL") ? "Stabil" : "Stabilizing...";
        document.getElementById('doStatus').innerHTML = "Status: " + doStatusMatch;
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

// ==================== SENSOR DO FUNCTIONS ====================
// Fungsi untuk membaca sampel ADC DO sensor (optimized for speed)
void getDOSamples() {
  for (int i = 0; i < NUM_SAMPLES; i++) {
    do_samples[i] = analogRead(DO_PIN);
    delay(SAMPLE_INTERVAL); // Minimal delay for stability
  }
}

// Fungsi untuk mengurutkan array (untuk mendapatkan median)
void sortArray(uint16_t arr[], int size) {
  // Using a faster sorting algorithm for small arrays
  for (int i = 0; i < size - 1; i++) {
    int minIdx = i;
    for (int j = i + 1; j < size; j++) {
      if (arr[j] < arr[minIdx]) {
        minIdx = j;
      }
    }
    // Swap
    if (minIdx != i) {
      uint16_t temp = arr[i];
      arr[i] = arr[minIdx];
      arr[minIdx] = temp;
    }
  }
}

// Fungsi untuk mendapatkan nilai median dari array (optimized)
uint16_t getMedian(uint16_t arr[], int size) {
  // Create a copy of the array to avoid modifying the original
  uint16_t temp[size];
  memcpy(temp, arr, size * sizeof(uint16_t));
  
  // Sort array
  sortArray(temp, size);
  
  // Return median value
  if (size % 2 == 0) {
    return (temp[size/2] + temp[size/2 - 1]) / 2;
  } else {
    return temp[size/2];
  }
}

// Fungsi untuk memperbarui median window (optimized)
void updateDOMedianWindow(uint16_t newValue) {
  // Shift all values by one position
  memmove(&do_medianWindow[0], &do_medianWindow[1], (WINDOW_SIZE - 1) * sizeof(uint16_t));
  
  // Add new value at the end
  do_medianWindow[WINDOW_SIZE - 1] = newValue;
}

// Fungsi untuk menghitung DO (no change needed - already efficient)
int16_t readDO(uint32_t voltage_mv, uint8_t temperature_c) {
#if TWO_POINT_CALIBRATION == 0
  // Single point calibration
  uint16_t V_saturation = (uint32_t)CAL1_V + (uint32_t)35 * temperature_c - (uint32_t)CAL1_T * 35;
  return (voltage_mv * DO_Table[temperature_c] / V_saturation);
#else
  // Two point calibration
  uint16_t V_saturation = (int16_t)((int8_t)temperature_c - CAL2_T) * ((uint16_t)CAL1_V - CAL2_V) / ((uint8_t)CAL1_T - CAL2_T) + CAL2_V;
  return (voltage_mv * DO_Table[temperature_c] / V_saturation);
#endif
}

// Fungsi untuk memeriksa stabilitas pembacaan
bool checkDOStability(uint16_t currentVoltage) {
  if (abs((int)currentVoltage - (int)do_lastStableVoltage) <= STABILITY_THRESHOLD) {
    do_stabilityCounter++;
    if (do_stabilityCounter >= STABILITY_COUNT) {
      return true;
    }
  } else {
    do_lastStableVoltage = currentVoltage;
    do_stabilityCounter = 0;
  }
  return false;
}

// Fungsi untuk update data DO sensor
void updateDOSensor() {
  // Set suhu
  uint8_t temperature = (uint8_t)READ_TEMP;
  
  // Ambil sampel
  getDOSamples();
  
  // Hitung median dari sampel
  uint16_t medianADC = getMedian(do_samples, NUM_SAMPLES);
  
  // Update median window
  updateDOMedianWindow(medianADC);
  
  // Hitung median dari window
  uint16_t adcRaw = getMedian(do_medianWindow, WINDOW_SIZE);
  
  // Konversi ke tegangan (mV)
  uint16_t adcVoltage = uint32_t(VREF) * adcRaw / ADC_RES;
  
  // Periksa stabilitas pembacaan
  bool isStable = checkDOStability(adcVoltage);
  
  // Hitung nilai DO
  uint16_t doValue = readDO(adcVoltage, temperature);
  float doMgL = doValue / 1000.0;
  
  // Update shared data securely with mutex
  portENTER_CRITICAL(&dataMutex);
  sharedData.do_temperature = temperature;
  sharedData.DO_ADC_Raw = adcRaw;
  sharedData.DO_ADC_Voltage = adcVoltage;
  sharedData.DO_Value = doValue;
  sharedData.DO_mgL = doMgL;
  sharedData.do_isStable = isStable;
  
  // Check if DO is below threshold and control relay
  bool newRelayState = (doMgL < DO_THRESHOLD);
  if (newRelayState != sharedData.relayState) {
    sharedData.relayState = newRelayState;
    digitalWrite(RELAY_PIN, newRelayState ? LOW : HIGH); // LOW = Relay ON, HIGH = Relay OFF
  }
  sharedData.dataUpdated = true;
  portEXIT_CRITICAL(&dataMutex);
}

// Function untuk meng-update LCD
void updateLCD(PZEM_DATA data, bool isSourceDetected, ChargingMode mode, float DO_level, bool relayState) {
  // Hapus semua isi LCD
  lcd.clear();
  
  // Baris 1: Status sumber dan mode charging
  lcd.setCursor(0, 0);
  if (!isSourceDetected) {
    lcd.print("Sumber: TIDAK ADA");
  } else {
    lcd.print("Sumber: ADA ");
    
    // Tampilkan mode charging
    if (mode == BULK) {
      lcd.print("BULK");
    } else {
      lcd.print("ABSRP");
    }
  }
  
  // Baris 2: Tegangan
  lcd.setCursor(0, 1);
  lcd.print("V:");
  lcd.print(data.voltage, 2);
  lcd.print("V Lim:");
  lcd.print(VOLTAGE_LIMIT, 1);
  lcd.print("V");
  
  // Baris 3: Arus
  lcd.setCursor(0, 2);
  lcd.print("I:");
  lcd.print(data.current, 2);
  lcd.print("A Set:");
  lcd.print(sharedData.targetCurrent, 1);
  lcd.print("A");
  
  // Baris 4: DO dan status relay
  lcd.setCursor(0, 3);
  lcd.print("DO:");
  lcd.print(DO_level, 1);
  lcd.print("mg/L Aer:");
  lcd.print(relayState ? "ON " : "OFF");
}

// Fungsi untuk membaca data dari PZEM-017 (optimized for no delay)
PZEM_DATA readPZEM() {
  PZEM_DATA data;
  
  // Initialize data with default values
  data.voltage = 0.0;
  data.current = 0.0;
  data.power = 0.0;
  
  // Read all data registers in one request (address 0x0000, read 3 registers)
  if (node.readInputRegisters(0x0000, 3) == node.ku8MBSuccess) {
    // Register 0x0000: Voltage
    data.voltage = node.getResponseBuffer(0) * 0.01; // Convert to volts
    
    // Register 0x0001: Current
    data.current = node.getResponseBuffer(1) * 0.01; // Convert to amperes
    
    // Register 0x0002: Power
    data.power = node.getResponseBuffer(2) * 0.1; // Convert to watts
  }
  
  return data;
}

// Fungsi untuk menghitung daya secara manual
float calculatePower(float voltage, float current) {
  return voltage * current;
}

// Fungsi untuk membaca nilai analog dan menentukan status sumber (optimized)
bool checkSourceDetection() {
  int analogValue = analogRead(ANALOG_PIN);
  bool isSourceDetected = (analogValue > SOURCE_THRESHOLD);
  
  // Only send notifications if the status changed
  if (isSourceDetected != sharedData.sourceDetected) {
    portENTER_CRITICAL(&dataMutex);
    sharedData.sourceDetected = isSourceDetected;
    portEXIT_CRITICAL(&dataMutex);
    
    // The notifications will be handled by Core 0
    return isSourceDetected;
  }
  
  return isSourceDetected;
}

// Fungsi untuk logika fuzzy Sugeno berdasarkan error arus atau tegangan (optimized)
int fuzzyPWMControl(float currentError, float voltageError) {
  // Read current PWM value and source status
  int currentPwmValue = sharedData.pwmValue;
  bool isSourceDetected = sharedData.sourceDetected;
  
  // If source not detected, return 0 immediately
  if (!isSourceDetected) {
    return 0;
  }
  
  // If voltage above limit, use voltage error
  if (voltageError < 0) {
    // Voltage exceeds limit
    if (voltageError < -0.1) {
      // More aggressive PWM reduction if voltage too high
      return max(0, currentPwmValue - 5);
    } else {
      // Slowly reduce PWM to maintain voltage
      return max(0, currentPwmValue - 1);
    }
  }
  
  // If voltage below limit, use current error
  // Simple fuzzy membership functions for error
  // NB: Negative Big, NS: Negative Small, ZE: Zero, PS_val: Positive Small, PB_val: Positive Big
  float NB = 0.0, NS = 0.0, ZE = 0.0, PS_val = 0.0, PB_val = 0.0;
  
  // Fuzzification with more sensitive ranges
  if (currentError <= -0.5) {
    NB = 1.0;  // Large negative error - reduce PWM
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
    PB_val = 1.0;  // Large positive error - increase PWM aggressively
  }
  
  // Singleton outputs for Sugeno - more aggressive increase than decrease
  int NBOutput = -10;   // Decrease PWM by 10
  int NSOutput = -1;    // Decrease PWM by 1
  int ZEOutput = 0;     // No change
  int PSOutput = 1;     // Increase PWM by 1
  int PBOutput = 10;    // Increase PWM by 10
  
  // Defuzzification (weighted average - Sugeno)
  float totalWeight = NB + NS + ZE + PS_val + PB_val;
  
  if (totalWeight > 0) {
    int deltaOutput = (int)((NB * NBOutput + NS * NSOutput + ZE * ZEOutput + PS_val * PSOutput + PB_val * PBOutput) / totalWeight);
    
    // If still below target and error positive, force at least +1
    if (currentError > 0.05 && deltaOutput == 0) {
      deltaOutput = 1;  // Ensure we always increase if target not reached
    }
    
    // Calculate new PWM value
    int newPWM = currentPwmValue + deltaOutput;
    
    // Constrain to valid range
    return constrain(newPWM, 0, PWM_MAX_VALUE);
  }
  
  return currentPwmValue; // If no rules triggered, maintain current value
}

// Handle WebSocket events (moved to Core 0)
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

// Handle WebSocket message (optimized)
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
      // Cap at maximum value
      if (newCurrent > MAX_CHARGING_CURRENT) {
        newCurrent = MAX_CHARGING_CURRENT;
      }
      
      // Update shared data with mutex protection
      portENTER_CRITICAL(&dataMutex);
      sharedData.targetCurrent = newCurrent;
      portEXIT_CRITICAL(&dataMutex);
      
      String response = "Target arus diatur ke: " + String(newCurrent, 2) + " A";
      if (newCurrent == MAX_CHARGING_CURRENT) {
        response += " (Maksimum)";
      }
      Serial.println(response);
      ws.textAll(response);
    }
  }
}

void processCommand() {
  // Check for serial input from USB
  if (Serial.available() > 0) {
    // Read serial input as string
    String input = Serial.readStringUntil('\n');
    input.trim(); // Remove whitespace
    
    // If input is numeric, use as target current
    if (input.length() > 0) {
      // Try to convert to float
      float newCurrent = input.toFloat();
      
      // Update target current if valid, but cap at MAX_CHARGING_CURRENT
      if (newCurrent >= 0) {
        // Cap at maximum value
        if (newCurrent > MAX_CHARGING_CURRENT) {
          newCurrent = MAX_CHARGING_CURRENT;
        }
        
        // Update shared data with mutex protection
        portENTER_CRITICAL(&dataMutex);
        sharedData.targetCurrent = newCurrent;
        portEXIT_CRITICAL(&dataMutex);
        
        String response = "Target arus diatur ke: " + String(newCurrent, 2) + " A";
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
}

// Core 0 task: handles WiFi, WebSocket, LCD and user interface
// This runs on the same core as WiFi to avoid conflicts
void Core0Task_UI(void *pvParameters) {
  // Set up WebSocket
  ws.onEvent(onWebSocketEvent);
  server.addHandler(&ws);
  
  // Set up web server route
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/html", index_html);
  });
  
  // Start server
  server.begin();
  
  // Main task loop
  for (;;) {
    // Get current time
    unsigned long currentMillis = millis();
    
    // Check WiFi connection
    if (WiFi.status() != WL_CONNECTED) {
      Serial.println("Koneksi WiFi terputus, menghubungkan kembali...");
      WiFi.begin(ssid, password);
      
      // Wait for connection (max 10 seconds)
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
    
    // WebSocket cleanup
    ws.cleanupClients();
    
    // Process serial commands
    processCommand();
    
    // Check source status changes and send notifications if needed
    bool localSourceDetected;
    portENTER_CRITICAL(&dataMutex);
    localSourceDetected = sharedData.sourceDetected;
    bool dataChanged = sharedData.dataUpdated;
    sharedData.dataUpdated = false;
    portEXIT_CRITICAL(&dataMutex);
    
    if (dataChanged) {
      float targetCurrent;
      float voltage;
      
      portENTER_CRITICAL(&dataMutex);
      targetCurrent = sharedData.targetCurrent;
      voltage = sharedData.pzem.voltage;
      portEXIT_CRITICAL(&dataMutex);
      
      // Send source change notifications
      if (localSourceDetected) {
        String infoMsg = "Memulai charging dengan target arus " + String(targetCurrent, 2) + " A dan batas tegangan " + String(VOLTAGE_LIMIT, 2) + " V";
        Serial.println(infoMsg);
        if (ws.count() > 0) {
          ws.textAll(infoMsg);
        }
      } else {
        String infoMsg = "Charging dihentikan karena sumber tidak terdeteksi";
        Serial.println(infoMsg);
        if (ws.count() > 0) {
          ws.textAll(infoMsg);
        }
      }
    }
    
    // Check if it's time to update LCD
    if (currentMillis - sharedData.lastLcdUpdate >= LCD_UPDATE_INTERVAL) {
      // Get a local copy of shared data (with mutex protection)
      PZEM_DATA localPzem;
      bool localSourceDetected;
      ChargingMode localChargingMode;
      float localDoLevel;
      bool localRelayState;
      
      portENTER_CRITICAL(&dataMutex);
      localPzem = sharedData.pzem;
      localSourceDetected = sharedData.sourceDetected;
      localChargingMode = sharedData.chargingMode;
      localDoLevel = sharedData.DO_mgL;
      localRelayState = sharedData.relayState;
      sharedData.lastLcdUpdate = currentMillis;
      portEXIT_CRITICAL(&dataMutex);
      
      // Update LCD with local data
      updateLCD(localPzem, localSourceDetected, localChargingMode, localDoLevel, localRelayState);
    }
    
    // Check if it's time to send WebSocket update
    if (currentMillis - sharedData.lastWebSocketUpdate >= WS_UPDATE_INTERVAL) {
      // Get a local copy of all relevant data
      PZEM_DATA localPzem;
      bool localSourceDetected;
      ChargingMode localChargingMode;
      float localTargetCurrent;
      int localPwmValue;
      float localDoLevel;
      uint16_t localDOAdcRaw;
      uint16_t localDOAdcVoltage;
      uint8_t localDoTemp;
      bool localDoStable;
      bool localRelayState;
      
      portENTER_CRITICAL(&dataMutex);
      localPzem = sharedData.pzem;
      localSourceDetected = sharedData.sourceDetected;
      localChargingMode = sharedData.chargingMode;
      localTargetCurrent = sharedData.targetCurrent;
      localPwmValue = sharedData.pwmValue;
      localDoLevel = sharedData.DO_mgL;
      localDOAdcRaw = sharedData.DO_ADC_Raw;
      localDOAdcVoltage = sharedData.DO_ADC_Voltage;
      localDoTemp = sharedData.do_temperature;
      localDoStable = sharedData.do_isStable;
      localRelayState = sharedData.relayState;
      sharedData.lastWebSocketUpdate = currentMillis;
      portEXIT_CRITICAL(&dataMutex);
      
      // Prepare system status message
      String chargingStatus;
      if (!localSourceDetected) {
        chargingStatus = "Mati (Sumber Tidak Terdeteksi)";
      } else if (localChargingMode == BULK) {
        chargingStatus = "Bulk Charging (Arus Konstan)";
      } else {
        chargingStatus = "Absorption Charging (Tegangan Konstan)";
      }
      
      // Build data message
      String dataOutput = "------------------------\n";
      dataOutput += "Status Sumber: " + String(localSourceDetected ? "Terdeteksi" : "Tidak Terdeteksi") + "\n";
      dataOutput += "Mode Charging: " + chargingStatus + "\n";
      dataOutput += "Tegangan: " + String(localPzem.voltage, 2) + " V (Limit: " + String(VOLTAGE_LIMIT, 1) + " V)\n";
      dataOutput += "Arus: " + String(localPzem.current, 2) + " A (Target: " + String(localTargetCurrent, 2) + " A)\n";
      
      // Add error info
      if (localChargingMode == BULK) {
        dataOutput += "Error Arus: " + String(localTargetCurrent - localPzem.current, 3) + " A\n";
      } else {
        dataOutput += "Error Tegangan: " + String(VOLTAGE_LIMIT - localPzem.voltage, 3) + " V\n";
      }
      
      dataOutput += "Daya: " + String(localPzem.power, 2) + " W\n";
      dataOutput += "PWM: " + String(localPwmValue) + "/" + String(PWM_MAX_VALUE) + " (" + String((float)localPwmValue / PWM_MAX_VALUE * 100.0, 2) + "%)\n";
      
      // Add DO sensor info
      dataOutput += "DO: " + String(localDoLevel, 2) + "mg/L (Temp: " + String(localDoTemp) + "°C) | ";
      dataOutput += localDoStable ? "STABIL" : "Stabilizing...";
      dataOutput += "\n";
      dataOutput += "DO ADC: " + String(localDOAdcRaw) + " | Voltage: " + String(localDOAdcVoltage) + "mV\n";
      
      // Add relay info
      dataOutput += "Relay Aerator: " + String(localRelayState ? "ON (DO < 1.0mg/L)" : "OFF") + "\n";
      
      // Send to all connected clients
      if (ws.count() > 0) {
        ws.textAll(dataOutput);
      }
      
      // Also send to Serial for debugging
      Serial.print(dataOutput);
    }
    
    // Small delay to prevent watchdog timeout and give CPU time to other tasks
    delay(10);
  }
}

void setup() {
  // Initialize serial for debugging
  Serial.begin(115200);
  
  // Configure pins
  pinMode(RELAY_PIN, OUTPUT);
  pinMode(ANALOG_PIN, INPUT);
  pinMode(DO_PIN, INPUT);
  
  // Initialize relay to OFF
  digitalWrite(RELAY_PIN, HIGH); // HIGH = OFF for most relay modules
  
  // Initialize I2C for LCD
  Wire.begin(SDA_PIN, SCL_PIN);
  
  // Initialize LCD
  lcd.init();
  lcd.backlight();
  lcd.clear();
  
  // Print startup message
  lcd.setCursor(0, 0);
  lcd.print("Solar Charge Control");
  lcd.setCursor(0, 1);
  lcd.print("Starting...");
  lcd.setCursor(0, 2);
  lcd.print("DO Sensor + Aerator");
  lcd.setCursor(0, 3);
  lcd.print("System v2.0");
  
  // Connect to WiFi
  Serial.print("Connecting to WiFi: ");
  Serial.println(ssid);
  
  WiFi.begin(ssid, password);
  
  // Wait for connection with timeout
  int timeout_counter = 0;
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    
    // Update connection message on LCD
    lcd.setCursor(11, 1);
    lcd.print("      ");
    lcd.setCursor(11, 1);
    for (int i=0; i < (timeout_counter % 6); i++) {
      lcd.print(".");
    }
    
    timeout_counter++;
    
    // Restart ESP if can't connect within 20 seconds
    if (timeout_counter >= 40) {
      Serial.println("Failed to connect to WiFi, restarting...");
      lcd.setCursor(0, 1);
      lcd.print("WiFi Error! Restart");
      ESP.restart();
    }
  }
  
  Serial.println("");
  Serial.println("WiFi connected!");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());
  
  // Show WiFi info on LCD
  lcd.setCursor(0, 1);
  lcd.print("WiFi: Connected    ");
  lcd.setCursor(0, 2);
  lcd.print("IP:                ");
  lcd.setCursor(4, 2);
  lcd.print(WiFi.localIP().toString());
  
  // Initialize RS485 communication
  SERIAL_COMMUNICATION.begin(9600, SERIAL_8N1, RX_PIN, TX_PIN);
  
  // Initialize ModbusMaster
  node.begin(PZEM_ADDR, SERIAL_COMMUNICATION);
  
  // Configure PWM
  ledcSetup(PWM_CHANNEL, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(PWM_PIN, PWM_CHANNEL);
  ledcWrite(PWM_CHANNEL, 0); // Start with PWM off
  
  // Initialize DO sensor median window
  for (int i = 0; i < WINDOW_SIZE; i++) {
    getDOSamples();
    do_medianWindow[i] = getMedian(do_samples, NUM_SAMPLES);
    // Minimal delay
    delay(10);
  }
  
  Serial.println("Warming up DO sensor... Please wait");
  delay(1000);  // Reduced warm-up time for faster startup
  
  // Print usage guide to Serial
  String helpMessage = "==== Solar Charge Controller with DO Sensor and Aerator Control ====\n";
  helpMessage += "Max Charging Current: " + String(MAX_CHARGING_CURRENT, 1) + " A\n";
  helpMessage += "Voltage Limit: " + String(VOLTAGE_LIMIT, 1) + " V\n";
  helpMessage += "DO Sensor: Enabled (Pin " + String(DO_PIN) + ")\n";
  helpMessage += "Aerator Relay: Enabled (Pin " + String(RELAY_PIN) + ") - Activates when DO < " + String(DO_THRESHOLD) + " mg/L\n";
  helpMessage += "Open http://" + WiFi.localIP().toString() + " in your browser for system monitoring\n";
  helpMessage += "================================================================";
  Serial.println(helpMessage);
  
  // Display startup complete message on LCD
  lcd.setCursor(0, 3);
  lcd.print("System Ready!      ");
  
  // Create task for Core 0 (UI and communication)
  xTaskCreatePinnedToCore(
    Core0Task_UI,          // Task function
    "UiTask",              // Task name
    10000,                 // Stack size (bytes)
    NULL,                  // Task parameters
    1,                     // Task priority
    &Core0Task,            // Task handle
    0                      // Core (0 = same as WiFi & BT)
  );
  
  // Let Core0Task start
  delay(500);
}

void loop() {
  // This runs on Core 1 (main application core)
  // Focus on time-critical operations: PZEM readings, DO sensor, and control logic
  // NO DELAYS in this core to ensure continuous operation
  
  // Check source detection
  bool isSourceDetected = checkSourceDetection();
  
  // Update DO sensor (fast operation)
  static unsigned long lastDoUpdate = 0;
  unsigned long currentMillis = millis();
  
  // Update DO sensor every 500ms without blocking the main loop
  if (currentMillis - lastDoUpdate >= 500) {
    lastDoUpdate = currentMillis;
    updateDOSensor();
  }
  
  // Read data from PZEM-017 (this is the continuous operation)
  PZEM_DATA data = readPZEM();
  
  // Calculate power as backup if sensor reading is 0
  float calculatedPower = data.voltage * data.current;
  if (data.power == 0 && data.voltage > 0 && data.current > 0) {
    data.power = calculatedPower;
  }
  
  // Update shared PZEM data with mutex protection
  portENTER_CRITICAL(&dataMutex);
  sharedData.pzem = data;
  portEXIT_CRITICAL(&dataMutex);
  
  // Determine charging mode based on voltage
  ChargingMode currentMode;
  if (data.voltage >= VOLTAGE_LIMIT) {
    currentMode = ABSORPTION; // Enter constant voltage mode
  } else if (data.voltage < (VOLTAGE_LIMIT - 0.3)) {
    currentMode = BULK;       // Return to constant current mode
  } else {
    // Read the current mode if voltage is in the transition range
    portENTER_CRITICAL(&dataMutex);
    currentMode = sharedData.chargingMode;
    portEXIT_CRITICAL(&dataMutex);
  }
  
  // Update charging mode if changed
  if (currentMode != sharedData.chargingMode) {
    portENTER_CRITICAL(&dataMutex);
    sharedData.chargingMode = currentMode;
    portEXIT_CRITICAL(&dataMutex);
  }
  
  // Set PWM based on source detection and charging mode
  int newPwmValue = 0;
  
  if (isSourceDetected) {
    // Source detected, determine PWM based on mode
    float targetCurrent;
    
    portENTER_CRITICAL(&dataMutex);
    targetCurrent = sharedData.targetCurrent;
    sharedData.isCharging = true;
    portEXIT_CRITICAL(&dataMutex);
    
    // Calculate error based on mode
    float currentError = targetCurrent - data.current;
    float voltageError = VOLTAGE_LIMIT - data.voltage;
    
    // Update PWM based on fuzzy control and charging mode
    newPwmValue = fuzzyPWMControl(currentError, voltageError);
    
    // Force PWM increase if in BULK mode and current too low
    if (currentMode == BULK && targetCurrent > CURRENT_THRESHOLD && 
        data.current < (targetCurrent * 0.8) && newPwmValue < PWM_MAX_VALUE) {
      newPwmValue += 1;  // Force increase by 1 to reach target
    }
  } else {
    // Source not detected, turn off PWM
    portENTER_CRITICAL(&dataMutex);
    sharedData.isCharging = false;
    portEXIT_CRITICAL(&dataMutex);
  }
  
  // Update PWM if value changed
  if (newPwmValue != sharedData.pwmValue) {
    portENTER_CRITICAL(&dataMutex);
    sharedData.pwmValue = newPwmValue;
    portEXIT_CRITICAL(&dataMutex);
    ledcWrite(PWM_CHANNEL, newPwmValue);
  }
  
  // No delay here - keep the loop running as fast as possible
  // This ensures continuous PZEM readings and fast PWM control
}