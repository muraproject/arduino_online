#include <TinyGPS++.h>

// Define pins for Hardware Serial2
#define GPS_RX 25  // GPS TX will connect to ESP32 GPIO25
#define GPS_TX 26  // GPS RX will connect to ESP32 GPIO26

// Create a TinyGPS++ object
TinyGPSPlus gps;

void setup() {
  // Start Serial Monitor
  Serial.begin(115200);
  
  // Start Hardware Serial2 for GPS
  Serial2.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX);

  Serial.println("GPS Neo 6M Test");
}

void loop() {
  // Read data from GPS module
  while (Serial2.available() > 0) {
    if (gps.encode(Serial2.read())) {
      displayInfo();
    }
  }

  // If no data received for 5 seconds, check for issues
  if (millis() > 5000 && gps.charsProcessed() < 10) {
    Serial.println("No GPS detected");
    while(true);
  }
}

void displayInfo() {
  Serial.println("--------------------");
  
  // Display location if available
  if (gps.location.isValid()) {
    Serial.print("Latitude: ");
    Serial.println(gps.location.lat(), 6);
    Serial.print("Longitude: ");
    Serial.println(gps.location.lng(), 6);
  } else {
    Serial.println("Location: Not Available");
  }

  // Display altitude if available
  if (gps.altitude.isValid()) {
    Serial.print("Altitude: ");
    Serial.print(gps.altitude.meters());
    Serial.println(" meters");
  } else {
    Serial.println("Altitude: Not Available");
  }

  // Display date/time if available
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

  // Display number of satellites
  Serial.print("Satellites: ");
  Serial.println(gps.satellites.value());

  Serial.println("--------------------");
  Serial.println();
  
  delay(1000);
}