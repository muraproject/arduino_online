#include <HardwareSerial.h>
#include <mavlink.h>
#include "BluetoothSerial.h"

// Menggunakan HardwareSerial2 untuk ESP32 (pin 16 RX, 17 TX)
HardwareSerial pixhawkSerial(2); // UART2 pada ESP32

// Inisialisasi Bluetooth Serial
BluetoothSerial SerialBT;

// Variabel global untuk menyimpan data terbaru
float groundSpeed_kmh = 0;
float heading_deg = 0;
unsigned long lastDataPrint = 0;

void setup() {
  Serial.begin(57600);
  while (!Serial) {
    ;  // Tunggu koneksi serial
  }

  // Inisialisasi serial ke Pixhawk menggunakan pin 16 (RX) dan 17 (TX)
  pixhawkSerial.begin(57600, SERIAL_8N1, 16, 17);

  // Inisialisasi Bluetooth dengan nama device
  SerialBT.begin("ESP32-MAVLink-Monitor"); // Nama Bluetooth device
  
  Serial.println("MAVLink Speed & Heading Reader Started (ESP32)");
  SerialBT.println("MAVLink Speed & Heading Reader Started (ESP32)");
  Serial.println("Bluetooth device name: ESP32-MAVLink-Monitor");
  SerialBT.println("Bluetooth connected successfully!");
  
  delay(1500);

  request_data();
}

void loop() {
  mavlink_message_t msg;
  mavlink_status_t status;

  while (pixhawkSerial.available()) {
    uint8_t c = pixhawkSerial.read();

    if (mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) {
      // Proses pesan VFR_HUD untuk ground speed
      if (msg.msgid == MAVLINK_MSG_ID_VFR_HUD) {
        mavlink_vfr_hud_t hud;
        mavlink_msg_vfr_hud_decode(&msg, &hud);
        
        // Konversi ground speed dari m/s ke km/h
        groundSpeed_kmh = hud.groundspeed * 3.6;
        
        // Heading juga tersedia di VFR_HUD
        heading_deg = hud.heading;
      }
      
      // Proses pesan ATTITUDE untuk heading jika tidak tersedia di VFR_HUD
      else if (msg.msgid == MAVLINK_MSG_ID_ATTITUDE) {
        mavlink_attitude_t attitude;
        mavlink_msg_attitude_decode(&msg, &attitude);
        
        // Konversi yaw dari radian ke derajat (0-360)
        float yaw_deg = attitude.yaw * 180.0 / M_PI;
        if (yaw_deg < 0) {
          yaw_deg += 360.0;
        }
        heading_deg = yaw_deg;
      }
      
      // Alternative source for ground speed from GLOBAL_POSITION_INT
      else if (msg.msgid == MAVLINK_MSG_ID_GLOBAL_POSITION_INT) {
        mavlink_global_position_int_t global_pos;
        mavlink_msg_global_position_int_decode(&msg, &global_pos);
        
        // Hitung ground speed dari velocity components
        float vx = float(global_pos.vx) / 100.0; // cm/s to m/s
        float vy = float(global_pos.vy) / 100.0; // cm/s to m/s
        float groundspeed_ms = sqrt(vx * vx + vy * vy);
        groundSpeed_kmh = groundspeed_ms * 3.6; // m/s to km/h
      }
    }
  }

  // Cek jika ada perintah dari Bluetooth
  if (SerialBT.available()) {
    String command = SerialBT.readString();
    command.trim();
    
    if (command == "status") {
      SerialBT.println("=== Status MAVLink ===");
      SerialBT.print("Kecepatan: ");
      SerialBT.print(groundSpeed_kmh, 1);
      SerialBT.print(" km/h | Arah: ");
      SerialBT.print(heading_deg, 0);
      SerialBT.println(" derajat");
      SerialBT.println("=====================");
    }
    else if (command == "help") {
      SerialBT.println("=== Perintah Tersedia ===");
      SerialBT.println("status - Tampilkan status saat ini");
      SerialBT.println("help - Tampilkan bantuan ini");
      SerialBT.println("========================");
    }
  }

  // Print data setiap 500ms
  if (millis() - lastDataPrint >= 500) {
    printSpeedAndHeading();
    lastDataPrint = millis();
  }

  // Permintaan data setiap 2 detik
  static unsigned long lastRequest = 0;
  if (millis() - lastRequest >= 2000) {
    request_data();
    lastRequest = millis();
  }
}

void printSpeedAndHeading() {
  // Kirim ke Serial Monitor
  Serial.print("Kecepatan: ");
  Serial.print(groundSpeed_kmh, 1); // 1 digit desimal
  Serial.print(" km/h | Arah: ");
  Serial.print(heading_deg, 0); // Tanpa desimal
  Serial.println(" derajat");
  
  // Kirim ke Bluetooth
  SerialBT.print("Kecepatan: ");
  SerialBT.print(groundSpeed_kmh, 1); // 1 digit desimal
  SerialBT.print(" km/h | Arah: ");
  SerialBT.print(heading_deg, 0); // Tanpa desimal
  SerialBT.println(" derajat");
}

void request_data() {
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  // Request EXTRA1 untuk ground speed (VFR_HUD) dan attitude (ATTITUDE)
  mavlink_msg_request_data_stream_pack(255, 190, &msg,
                                       1,  // target system
                                       0,  // target component
                                       MAV_DATA_STREAM_EXTRA1,
                                       10,  // 10 Hz rate
                                       1);  // Start

  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  for (uint16_t i = 0; i < len; i++) {
    pixhawkSerial.write(buf[i]);
  }

  delay(10);

  // Request POSITION untuk alternatif ground speed
  mavlink_msg_request_data_stream_pack(255, 190, &msg,
                                       1,  // target system
                                       0,  // target component
                                       MAV_DATA_STREAM_POSITION,
                                       10,  // 10 Hz rate
                                       1);  // Start

  len = mavlink_msg_to_send_buffer(buf, &msg);
  for (uint16_t i = 0; i < len; i++) {
    pixhawkSerial.write(buf[i]);
  }
}