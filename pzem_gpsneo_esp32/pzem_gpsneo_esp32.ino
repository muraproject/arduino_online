#include <TinyGPS++.h>
#include <PZEM004Tv30.h>

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

// Variables for timing
unsigned long lastGPSRead = 0;
unsigned long lastPZEMRead = 0;
const unsigned long GPS_INTERVAL = 1000;  // Read GPS every 1 second
const unsigned long PZEM_INTERVAL = 2000; // Read PZEM every 2 seconds

void setup() {
  // Start Serial Monitor
  Serial.begin(115200);
  
  // Initialize both serial connections
  Serial2.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX);
  
  Serial.println("GPS and PZEM004T Test");
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
    Serial.print("Latitude: ");
    Serial.println(gps.location.lat(), 6);
    Serial.print("Longitude: ");
    Serial.println(gps.location.lng(), 6);
  } else {
    Serial.println("Location: Not Available");
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