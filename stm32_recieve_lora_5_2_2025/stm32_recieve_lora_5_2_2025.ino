#include <SoftwareSerial.h>
#include <mavlink.h>

SoftwareSerial pixhawkSerial(10, 11);  // RX, TX
HardwareSerial LoRaSerial(PA10, PA9);  // RX, TX for STM32

String currentCommand1 = "STP";
unsigned long messageCount1 = 0;
unsigned long lastMessageTime1 = 0;
unsigned long lastSpeedCheck = 0;  // Untuk timing check ground speed
unsigned long lastRequest = 0;

void setup() {
  Serial.begin(9600);
  while (!Serial) {
    ;  // Tunggu koneksi serial
  }

  pixhawkSerial.begin(57600);
  LoRaSerial.begin(9600);
  
  Serial.println("MAVLink IMU & GPS Reader Started");
  delay(1500);
  
  request_data();
}

void loop() {
  unsigned long currentMillis = millis();
  
  // Check ground speed setiap 1 detik
  if (currentMillis - lastSpeedCheck >= 50) {
    checkGroundSpeed();
    lastSpeedCheck = currentMillis;
  }
  
  // Selalu check untuk perintah remote
  checkRemoteCommands();
  
  // Request data setiap 2 detik
  
  if (currentMillis - lastRequest >= 100) {
    request_data();
    lastRequest = currentMillis;
  }
}

void checkGroundSpeed() {
  mavlink_message_t msg;
  mavlink_status_t status;
  unsigned long startTime = millis();
  
  // Beri batas waktu 100ms untuk mencari data ground speed
  while (pixhawkSerial.available() && (millis() - startTime < 100)) {
    uint8_t c = pixhawkSerial.read();
    
    if (mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) {
      if (msg.msgid == 33) {  // GLOBAL_POSITION_INT
        mavlink_global_position_int_t global_pos;
        mavlink_msg_global_position_int_decode(&msg, &global_pos);

        float vx = float(global_pos.vx) / 100.0;
        float vy = float(global_pos.vy) / 100.0;
        float groundspeed = sqrt(vx * vx + vy * vy);
        
        String speedMsg = "SPD:" + String(int(groundspeed*100));
        LoRaSerial.println(speedMsg);
        Serial.println(speedMsg);
        
        return;  // Keluar dari fungsi setelah dapat data
      }
    }
  }
}

void checkRemoteCommands() {
  if (LoRaSerial.available()) {
    String data = LoRaSerial.readStringUntil('\n');
    data.trim();
    
    if (data.length() > 0) {
      messageCount1++;
      lastMessageTime1 = millis();
      
      Serial.println("\n--- Received Message ---");
      Serial.print("Data: ");
      Serial.println(data);
      Serial.print("Message Count: ");
      Serial.println(messageCount1);
      
      // Proses perintah remote di sini
      // ...
    }
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
