#include <SoftwareSerial.h>
#include <mavlink.h>

SoftwareSerial pixhawkSerial(10, 11,128);  // RX, TX

// LoRa on Hardware Serial
HardwareSerial LoRaSerial(PA10, PA9);  // RX, TX for STM32

// LoRa control pins
#define M0_PIN PB5
#define M1_PIN PB4
#define AUX_PIN PB3

String currentCommand1 = "STP";
unsigned long messageCount1 = 0;
unsigned long lastMessageTime1 = 0;

void setup() {
  // Debug Serial
  Serial.begin(9600);
  while (!Serial) {}
  delay(1000);

  Serial.println("\n--- LoRa Debug Test Started ---");

  // Debug pin status
  Serial.println("\nChecking Pins:");
  Serial.print("M0_PIN (PB5): ");
  Serial.println(digitalRead(M0_PIN));
  Serial.print("M1_PIN (PB4): ");
  Serial.println(digitalRead(M1_PIN));
  Serial.print("AUX_PIN (PB3): ");
  Serial.println(digitalRead(AUX_PIN));

  // LoRa Setup
  Serial.println("\nInitializing LoRa...");

  // LoRa Control Pins
  pinMode(M0_PIN, OUTPUT);
  pinMode(M1_PIN, OUTPUT);
  pinMode(AUX_PIN, INPUT);

  // Set Normal Mode
  digitalWrite(M0_PIN, LOW);
  digitalWrite(M1_PIN, LOW);

  Serial.println("Waiting for AUX to stabilize...");
  delay(100);  // Wait for AUX to stabilize

  Serial.print("AUX Status: ");
  Serial.println(digitalRead(AUX_PIN));

  // Initialize LoRa Serial
  LoRaSerial.begin(9600);
  Serial.println("LoRa Serial Initialized");

  // Send test message
  delay(1000);
  Serial.println("\nSending test message...");
  LoRaSerial.println("TEST:INIT");

  pixhawkSerial.begin(57600);

  Serial.println("MAVLink IMU & GPS Reader Started");
  delay(1500);

  printStatus();
}

void loop() {
  static unsigned long lastStatus = 0;

  // Print status every 5 seconds
  if (millis() - lastStatus >= 5000) {
    printStatus();
    lastStatus = millis();

    // Send periodic test message
    String sent = "SPD:";
    sent += lastStatus / 1000;
    LoRaSerial.println(sent);
    Serial.println("Sent: SPD:67");
    request_data();
  }

  delay(10);
  read_pixhawk();
  // Check for incoming data
  if (LoRaSerial.available()) {
    String data = LoRaSerial.readStringUntil('\n');
    data.trim();
    messageCount1++;
    lastMessageTime1 = millis();

    Serial.println("\n--- Received Message ---");
    Serial.print("Data: ");
    Serial.println(data);
    Serial.print("Message Count: ");
    Serial.println(messageCount1);
    Serial.print("RSSI: N/A");  // E220 doesn't provide RSSI
    Serial.print("AUX Pin: ");
    Serial.println(digitalRead(AUX_PIN));

    // Echo back
    LoRaSerial.print("ACK:");
    LoRaSerial.println(data);
    Serial.print("Sent ACK: ");
    Serial.println(data);
  }

  // Check for any Serial input for testing
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    Serial.print("Sending manual command: ");
    Serial.println(command);
    LoRaSerial.println(command);
  }
}

void printStatus() {
  Serial.println("\n=== LoRa Status ===");
  Serial.print("Runtime (s): ");
  Serial.println(millis() / 1000);
  Serial.print("Messages Received: ");
  Serial.println(messageCount1);
  Serial.print("Last Message (ms ago): ");
  if (lastMessageTime1 > 0) {
    Serial.println(millis() - lastMessageTime1);
  } else {
    Serial.println("No messages yet");
  }
  Serial.print("AUX Pin: ");
  Serial.println(digitalRead(AUX_PIN));
  Serial.print("M0 Pin: ");
  Serial.println(digitalRead(M0_PIN));
  Serial.print("M1 Pin: ");
  Serial.println(digitalRead(M1_PIN));
  Serial.println("===================\n");
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


void read_pixhawk() {
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
}