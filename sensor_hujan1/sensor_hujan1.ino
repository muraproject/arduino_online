#include <WiFi.h>
#include <HTTPClient.h>
#include <Arduino.h>

// WiFi credentials
const char* ssid = "admin";
const char* password = "admin123";

// Server API endpoint
const char* serverURL = "http://main.iotweb.my.id:3311/pc/zulfan/sensor_data.php";

// Rain gauge sensor configuration
const int pin_interrupt = 14;
volatile unsigned long jumlah_tip = 0;
float curah_hujan = 0.0;
const float milimeter_per_tip = 0.2;

// LED pin
const int ledPin = 2;

// Data transmission interval (2 minutes)
const unsigned long uploadInterval = 120000;
unsigned long lastUploadTime = 0;

// Status update interval (3 seconds)
const unsigned long statusInterval = 3000;
unsigned long lastStatusTime = 0;

// Auto-restart interval (10 minutes)
const unsigned long restartInterval = 605000;
unsigned long startTime = 0;

// WiFi connection status
bool wifiConnected = false;

// Function declarations
void IRAM_ATTR tipCounter();
void connectToWiFi();
void sendRainData();
void printStatus();
void checkRestartTime();

// Task handles
TaskHandle_t wifiTask;

// WiFi task running on Core 0
void wifiTaskCode(void * parameter) {
  // Setup WiFi on Core 0
  WiFi.setSleep(false); // Disable WiFi power save mode
  connectToWiFi();
  
  for(;;) {
    // Current time
    unsigned long currentMillis = millis();
    
    // Check and maintain WiFi connection
    if (WiFi.status() != WL_CONNECTED) {
      wifiConnected = false;
      connectToWiFi();
    } else {
      wifiConnected = true;
    }
    
    // Send data at regular intervals
    if (currentMillis - lastUploadTime >= uploadInterval) {
      if (wifiConnected) {
        sendRainData();
      } else {
        Serial.println("Cannot send data - WiFi not connected");
      }
      lastUploadTime = currentMillis;
    }
    
    // Check for auto-restart
    if (currentMillis - startTime >= restartInterval) {
      Serial.println("\n*** AUTO-RESTART TRIGGERED ***");
      Serial.println("Device has been running for 10 minutes");
      Serial.println("Restarting ESP32...");
      delay(500);
      ESP.restart();
    }
    
    delay(100); // Short delay to prevent watchdog issues
  }
}

void setup() {
  // Initialize serial communication
  Serial.begin(115200);
  delay(1000);
  
  // Record the start time
  startTime = millis();
  lastUploadTime = millis();
  lastStatusTime = millis();
  
  Serial.println("\n\n--- Rain Gauge Monitoring System ---");
  Serial.println("System will auto-restart every 10 minutes");
  
  // Setup LED pin
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, LOW);
  
  // Setup rain gauge sensor
  pinMode(pin_interrupt, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(pin_interrupt), tipCounter, FALLING);
  
  // Create WiFi task on Core 0
  xTaskCreatePinnedToCore(
    wifiTaskCode,   /* Task function */
    "WiFiTask",     /* Task name */
    10000,          /* Stack size */
    NULL,           /* Parameter */
    1,              /* Priority */
    &wifiTask,      /* Task handle */
    0);             /* Run on Core 0 */
}

void loop() {
  // Main code runs on Core 1
  unsigned long currentMillis = millis();
  
  // Update status at regular intervals
  if (currentMillis - lastStatusTime >= statusInterval) {
    printStatus();
    lastStatusTime = currentMillis;
  }
  
  // Flash LED if new tip detected
  if (jumlah_tip > 0) {
    digitalWrite(ledPin, HIGH);
    delay(50);
    digitalWrite(ledPin, LOW);
  }
  
  // Calculate rainfall from tip count
  curah_hujan = jumlah_tip * milimeter_per_tip;
  
  delay(50); // Short delay to prevent watchdog issues
}

// Interrupt handler for rain gauge
void IRAM_ATTR tipCounter() {
  static unsigned long last_interrupt_time = 0;
  unsigned long interrupt_time = millis();
  
  // Debounce - ignore interrupts too close together
  if (interrupt_time - last_interrupt_time > 200) {
    jumlah_tip++;
    Serial.print("*");
  }
  
  last_interrupt_time = interrupt_time;
}

// Connect to WiFi with simplified approach
void connectToWiFi() {
  Serial.println("\nAttempting to connect to WiFi...");
  
  // Disconnect first to clear any previous state
  WiFi.disconnect(true);
  delay(1000);
  
  // Set to station mode
  WiFi.mode(WIFI_STA);
  delay(1000);
  
  // Start connection
  WiFi.begin(ssid, password);
  
  // Wait up to 20 seconds for connection
  unsigned long connectionStart = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - connectionStart < 20000) {
    Serial.print(".");
    digitalWrite(ledPin, !digitalRead(ledPin)); // Flash LED while connecting
    delay(1000);
  }
  
  // Check if connected
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nConnected to WiFi!");
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());
    wifiConnected = true;
    
    // Indicate success with LED
    digitalWrite(ledPin, HIGH);
    delay(1000);
    digitalWrite(ledPin, LOW);
  } else {
    Serial.println("\nFailed to connect to WiFi");
    Serial.print("Status code: ");
    Serial.println(WiFi.status());
    wifiConnected = false;
  }
}

// Print status information
void printStatus() {
  Serial.println("\n----- STATUS UPDATE -----");
  Serial.print("Jumlah tip = ");
  Serial.print(jumlah_tip);
  Serial.println(" kali");
  
  Serial.print("Curah hujan = ");
  Serial.print(curah_hujan, 1);
  Serial.println(" mm");
  
  Serial.print("WiFi status: ");
  Serial.println(wifiConnected ? "Connected" : "Disconnected");
  
  // Print uptime
  unsigned long uptime = millis() - startTime;
  unsigned long seconds = uptime / 1000;
  unsigned long minutes = seconds / 60;
  unsigned long hours = minutes / 60;
  
  Serial.print("Device uptime: ");
  if (hours > 0) {
    Serial.print(hours);
    Serial.print(" hours, ");
  }
  Serial.print(minutes % 60);
  Serial.print(" minutes, ");
  Serial.print(seconds % 60);
  Serial.println(" seconds");
  
  // Print time until restart
  unsigned long timeToRestart = restartInterval - uptime;
  unsigned long restartMinutes = (timeToRestart / 1000) / 60;
  unsigned long restartSeconds = (timeToRestart / 1000) % 60;
  
  Serial.print("Auto-restart in: ");
  Serial.print(restartMinutes);
  Serial.print(" minutes, ");
  Serial.print(restartSeconds);
  Serial.println(" seconds");
  Serial.println("-------------------------");
}

// Send data to the server
void sendRainData() {
  HTTPClient http;
  
  Serial.println("\n===== SENDING DATA =====");
  Serial.println("URL: " + String(serverURL));
  
  http.begin(serverURL);
  http.addHeader("Content-Type", "application/x-www-form-urlencoded");
  
  // Prepare POST data - no timestamp, let the server handle it
  String postData = "sensor_type=rain";
  postData += "&jumlah_tip=" + String(jumlah_tip);
  postData += "&curah_hujan=" + String(curah_hujan, 1);
  
  Serial.println("Data: " + postData);
  
  // Send the request
  int httpResponseCode = http.POST(postData);
  
  if (httpResponseCode > 0) {
    String response = http.getString();
    Serial.println("HTTP Response code: " + String(httpResponseCode));
    Serial.println("Response: " + response);
    
    // Reset tip counter after successful transmission
    jumlah_tip = 0;
    curah_hujan = 0.0;
  } else {
    Serial.println("Error on HTTP request: " + String(httpResponseCode));
  }
  
  Serial.println("=======================");
  http.end();
}