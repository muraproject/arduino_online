#include <SoftwareSerial.h>
#include <mavlink.h>

SoftwareSerial pixhawkSerial(10, 11); // RX, TX

void setup() {
  // Debug Serial
  Serial.begin(57600);
  while (!Serial) {
    ; // wait for serial port to connect
  }

  // Pixhawk Serial
  pixhawkSerial.begin(57600);
  
  Serial.println("MAVLink IMU Reader Started");
  delay(1500); // Wait for system to stabilize
  
  // Request IMU data
  request_imu_data();
}

void loop() {
  mavlink_message_t msg;
  mavlink_status_t status;

  // Read incoming messages
  while (pixhawkSerial.available()) {
    uint8_t c = pixhawkSerial.read();
    
    if (mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) {
      // Only process RAW_IMU messages
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
      }
    }
  }

  // Request IMU data every 2 seconds
  static unsigned long lastRequest = 0;
  if (millis() - lastRequest >= 2000) {
    request_imu_data();
    lastRequest = millis();
  }
}

void request_imu_data() {
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  
  // Request RAW_SENSORS stream
  mavlink_msg_request_data_stream_pack(255, 190, &msg,
                                     1,    // target system
                                     0,    // target component
                                     MAV_DATA_STREAM_RAW_SENSORS,  // Only request IMU data
                                     10,   // 10 Hz rate
                                     1);   // Start
                                     
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  
  // Send request
  for(uint16_t i = 0; i < len; i++) {
    pixhawkSerial.write(buf[i]);
  }
}