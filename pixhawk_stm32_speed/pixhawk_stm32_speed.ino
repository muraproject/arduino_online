#include <SoftwareSerial.h>
#include <mavlink.h>

SoftwareSerial pixhawkSerial(10, 11);  // RX, TX

void setup() {
  Serial.begin(57600);
  while (!Serial) {
    ;  // Tunggu koneksi serial
  }

  pixhawkSerial.begin(57600);

  Serial.println("MAVLink IMU & GPS Reader Started");
  delay(1500);

  request_data();
}

void loop() {
  mavlink_message_t msg;
  mavlink_status_t status;

  while (pixhawkSerial.available()) {
    uint8_t c = pixhawkSerial.read();

    if (mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) {
      // Proses pesan IMU
      if (msg.msgid == MAVLINK_MSG_ID_RAW_IMU) {
        mavlink_raw_imu_t imu;
        mavlink_msg_raw_imu_decode(&msg, &imu);

        Serial.println("\n=== IMU DATA ===");
        Serial.print("Accelerometer (mg) -> X: ");
        Serial.print(imu.xacc);
        Serial.print(" Y: ");
        Serial.print(imu.yacc);
        Serial.print(" Z: ");
        Serial.println(imu.zacc);

        Serial.print("Gyroscope (mrad/s) -> X: ");
        Serial.print(imu.xgyro);
        Serial.print(" Y: ");
        Serial.print(imu.ygyro);
        Serial.print(" Z: ");
        Serial.println(imu.zgyro);

        Serial.print("Magnetometer (mGa) -> X: ");
        Serial.print(imu.xmag);
        Serial.print(" Y: ");
        Serial.print(imu.ymag);
        Serial.print(" Z: ");
        Serial.println(imu.zmag);
      }


      // Proses pesan VFR_HUD untuk ground speed
      else if (msg.msgid == MAVLINK_MSG_ID_VFR_HUD) {
        mavlink_vfr_hud_t hud;
        mavlink_msg_vfr_hud_decode(&msg, &hud);

        Serial.println("\n=== GROUND SPEED DATA ===");
        Serial.print("Ground Speed (m/s): ");
        Serial.println(hud.groundspeed);
      }
      // Proses pesan ATTITUDE untuk orientasi
      else if (msg.msgid == MAVLINK_MSG_ID_ATTITUDE) {
        mavlink_attitude_t attitude;
        mavlink_msg_attitude_decode(&msg, &attitude);

        Serial.println("\n=== ATTITUDE DATA ===");
        Serial.print("Roll (rad): ");
        Serial.println(attitude.roll);
        Serial.print("Pitch (rad): ");
        Serial.println(attitude.pitch);
        Serial.print("Yaw (rad): ");
        Serial.println(attitude.yaw);
      }



      // Debug pesan lain
      else {
        // Serial.print("\nPesan ID tidak dikenal: ");
        // Serial.println(msg.msgid);
        // Proses pesan GPS dengan koordinat yang benar
        // Proses pesan GPS tanpa konversi langsung
        if (msg.msgid == 33) {  // GLOBAL_POSITION_INT
          mavlink_global_position_int_t global_pos;
          mavlink_msg_global_position_int_decode(&msg, &global_pos);

          // Tampilkan data mentah dulu
          int32_t lat_deg = global_pos.lat / 10000000;  // Integer division
          int32_t lon_deg = global_pos.lon / 10000000;
          int32_t speed_ms = global_pos.vx;  // Convert cm/s to m/s
          

          Serial.print("Lat (deg): ");
          Serial.println(lat_deg, 7);
          Serial.print("Lon (deg): ");
          Serial.println(lon_deg, 7);
          Serial.print("Speed (m/s): ");
          // Serial.println(speed_ms);


          float vx = float(global_pos.vx) / 100.0;
          float vy = float(global_pos.vy) / 100.0;
          float groundspeed = sqrt(vx * vx + vy * vy);
          Serial.print(groundspeed);
          Serial.println(" m/s");
          // float lat_deg = lat / 10000000.0;
          // Serial.println(lat_deg, 7);

          // Serial.print("Lon Converted: ");
          // float lon_deg = lon / 10000000.0;
          // Serial.println(lon_deg, 7);
        }

        if (msg.msgid == 32) {
          mavlink_local_position_ned_t local_pos;
          mavlink_msg_local_position_ned_decode(&msg, &local_pos);

          Serial.println("\n=== SPEED DATA ===");
          Serial.print("North Speed (m/s): ");
          Serial.println(local_pos.vx);
          Serial.print("East Speed (m/s): ");
          Serial.println(local_pos.vy);
          Serial.print("Vertical Speed (m/s): ");
          Serial.println(local_pos.vz);
        }
      }
    }
  }

  // Permintaan data setiap 2 detik
  static unsigned long lastRequest = 0;
  if (millis() - lastRequest >= 2000) {
    request_data();
    lastRequest = millis();
  }
}

void request_data() {
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  // Request RAW_SENSORS untuk IMU
  mavlink_msg_request_data_stream_pack(255, 190, &msg,
                                       1,  // target system
                                       0,  // target component
                                       MAV_DATA_STREAM_RAW_SENSORS,
                                       10,  // 10 Hz rate
                                       1);  // Start

  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  for (uint16_t i = 0; i < len; i++) {
    pixhawkSerial.write(buf[i]);
  }

  delay(10);

  // Request EXTRA1 untuk ground speed (VFR_HUD)
  mavlink_msg_request_data_stream_pack(255, 190, &msg,
                                       1,  // target system
                                       0,  // target component
                                       MAV_DATA_STREAM_EXTRA1,
                                       10,  // 10 Hz rate
                                       1);  // Start

  len = mavlink_msg_to_send_buffer(buf, &msg);
  for (uint16_t i = 0; i < len; i++) {
    pixhawkSerial.write(buf[i]);
  }

  // Request EXTend
  mavlink_msg_request_data_stream_pack(255, 190, &msg,
                                       1,  // target system
                                       0,  // target component
                                       MAV_DATA_STREAM_EXTENDED_STATUS,
                                       10,  // 10 Hz rate
                                       1);  // Start

  len = mavlink_msg_to_send_buffer(buf, &msg);
  for (uint16_t i = 0; i < len; i++) {
    pixhawkSerial.write(buf[i]);
  }

  delay(10);

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

  delay(10);
  // Request ATTITUDE
  mavlink_msg_request_data_stream_pack(255, 190, &msg,
                                       1,  // target system
                                       0,  // target component
                                       MAV_DATA_STREAM_EXTRA1,
                                       10,  // 10 Hz rate
                                       1);  // Start

  len = mavlink_msg_to_send_buffer(buf, &msg);
  for (uint16_t i = 0; i < len; i++) {
    pixhawkSerial.write(buf[i]);
  }
}
