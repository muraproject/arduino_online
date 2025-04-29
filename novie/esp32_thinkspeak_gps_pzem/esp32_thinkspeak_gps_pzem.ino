#include <TinyGPS++.h>
#include <PZEM004Tv30.h>
#include <WiFi.h>
#include <HTTPClient.h>

// WiFi credentials
const char* ssid = "admin";      // Replace with your WiFi name
const char* password = "admin123"; // Replace with your WiFi password

// ThingSpeak settings
const char* thingSpeakApiKey = "JLMU0HFWADBC70WJ"; // Your ThingSpeak Write API Key
const char* thingSpeakServer = "api.thingspeak.com";
const unsigned long channelID = 2692520;  // Your ThingSpeak Channel ID

// Define pins for GPS Serial2
#define GPS_RX 25  // GPS TX will connect to ESP32 GPIO25
#define GPS_TX 26  // GPS RX will connect to ESP32 GPIO26

// Define pins for PZEM Serial2
#define PZEM_RX 16
#define PZEM_TX 17

// Create objects
TinyGPSPlus gps;
PZEM004Tv30 pzem_r(Serial2, PZEM_RX, PZEM_TX);

// Variables for PZEM readings
float vr;
float ir;
float freq;
float pf_r;
float energy;
float power;

// Variables for GPS readings
float latitude = 0;
float longitude = 0;
bool gpsValid = false;

// Variables for timing
unsigned long lastGPSRead = 0;
unsigned long lastPZEMRead = 0;
unsigned long lastThingSpeakUpdate = 0;
const unsigned long GPS_INTERVAL = 1000;  // Read GPS every 1 second
const unsigned long PZEM_INTERVAL = 2000; // Read PZEM every 2 seconds
const unsigned long THINGSPEAK_INTERVAL = 15000; // Update ThingSpeak every 15 seconds

void setup() {
  // Start Serial Monitor
  Serial.begin(115200);
  
  // Initialize Serial2 for GPS first
  Serial2.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX);
  
  Serial.println("GPS and PZEM004T with ThingSpeak");
  
  // Connect to WiFi
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  
  Serial.println();
  Serial.println("WiFi connected!");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());
}

void loop() {
  unsigned long currentMillis = millis();
  
  // Handle GPS readings
  if (currentMillis - lastGPSRead >= GPS_INTERVAL) {
    readGPS();
    lastGPSRead = currentMillis;
  }
  
  // Handle PZEM readings
  if (currentMillis - lastPZEMRead >= PZEM_INTERVAL) {
    // Reconfigure Serial2 for PZEM
    Serial2.end();
    Serial2.begin(9600, SERIAL_8N1, PZEM_RX, PZEM_TX);
    readPZEM();
    // Reconfigure Serial2 for GPS
    Serial2.end();
    Serial2.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX);
    lastPZEMRead = currentMillis;
  }
  
  // Send data to ThingSpeak
  if (currentMillis - lastThingSpeakUpdate >= THINGSPEAK_INTERVAL) {
    sendToThingSpeak();
    lastThingSpeakUpdate = currentMillis;
  }
}

void readGPS() {
  while (Serial2.available() > 0) {
    if (gps.encode(Serial2.read())) {
      displayGPSInfo();
    }
  }
  
  if (millis() > 5000 && gps.charsProcessed() < 10) {
    Serial.println("No GPS detected");
  }
}

void displayGPSInfo() {
  Serial.println("-------- GPS Data --------");
  
  if (gps.location.isValid()) {
    latitude = gps.location.lat();
    longitude = gps.location.lng();
    gpsValid = true;
    
    Serial.print("Latitude: ");
    Serial.println(latitude, 6);
    Serial.print("Longitude: ");
    Serial.println(longitude, 6);
  } else {
    Serial.println("Location: Not Available");
    gpsValid = false;
  }
  
  if (gps.altitude.isValid()) {
    Serial.print("Altitude: ");
    Serial.print(gps.altitude.meters());
    Serial.println(" meters");
  }
  
  if (gps.date.isValid()) {
    Serial.print("Date: ");
    Serial.print(gps.date.day());
    Serial.print("/");
    Serial.print(gps.date.month());
    Serial.print("/");
    Serial.println(gps.date.year());
  } else {
    Serial.println("Date: Not Available");
  }
  
  if (gps.time.isValid()) {
    Serial.print("Time: ");
    if (gps.time.hour() < 10) Serial.print("0");
    Serial.print(gps.time.hour());
    Serial.print(":");
    if (gps.time.minute() < 10) Serial.print("0");
    Serial.print(gps.time.minute());
    Serial.print(":");
    if (gps.time.second() < 10) Serial.print("0");
    Serial.println(gps.time.second());
  } else {
    Serial.println("Time: Not Available");
  }
  
  Serial.print("Satellites: ");
  Serial.println(gps.satellites.value());
  Serial.println("------------------------");
}

void readPZEM() {
  vr = pzem_r.voltage();
  ir = pzem_r.current();
  freq = pzem_r.frequency();
  pf_r = pzem_r.pf();
  power = pzem_r.power();
  energy = pzem_r.energy();
  
  Serial.println("-------- PZEM Data --------");
  Serial.print("Voltage: "); Serial.print(vr, 2); Serial.println(" V");
  Serial.print("Current: "); Serial.print(ir, 3); Serial.println(" A");
  Serial.print("Power Factor: "); Serial.print(pf_r); Serial.println("%");
  Serial.print("Power: "); Serial.print(power); Serial.println(" W");
  Serial.print("Energy: "); Serial.print(energy, 3); Serial.println(" kWh");
  Serial.print("Frequency: "); Serial.print(freq, 1); Serial.println(" Hz");
  Serial.println("--------------------------");
}

void sendToThingSpeak() {
  // Check WiFi connection
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi disconnected! Cannot send data.");
    return;
  }
  
  // Create HTTP client
  HTTPClient http;
  
  // Construct URL with parameters
  String url = "http://api.thingspeak.com/update?api_key=";
  url += thingSpeakApiKey;
  url += "&field1=";
  url += String(vr, 2);  // Voltage
  url += "&field2=";
  url += String(ir, 3);  // Current
  url += "&field3=";
  url += String(power);  // Power
  
  // Add GPS data if valid
  if (gpsValid) {
    url += "&field4=";
    url += String(latitude, 6);  // Latitude
    url += "&field5=";
    url += String(longitude, 6); // Longitude
  }
  
  // Optional: can add more fields if needed (ThingSpeak allows 8 fields)
  // url += "&field6=" + String(freq, 1);  // Frequency
  // url += "&field7=" + String(pf_r);     // Power Factor
  // url += "&field8=" + String(energy, 3); // Energy
  
  // Print the URL
  Serial.println("ThingSpeak URL:");
  Serial.println(url);
  
  // Begin HTTP connection
  http.begin(url);
  
  // Send HTTP GET request
  int httpResponseCode = http.GET();
  
  // Handle response
  if (httpResponseCode > 0) {
    String response = http.getString();
    Serial.print("HTTP Response code: ");
    Serial.println(httpResponseCode);
    Serial.print("Server response: ");
    Serial.println(response);
  } else {
    Serial.print("HTTP Error: ");
    Serial.println(httpResponseCode);
  }
  
  // Free resources
  http.end();
}