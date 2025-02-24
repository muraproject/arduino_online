#include <SoftwareSerial.h>
#include <mavlink.h>

SoftwareSerial pixhawkSerial(10, 11); // RX, TX

// Struktur untuk menyimpan data sensor
struct {
  // IMU
  int16_t accX, accY, accZ;
  int16_t gyroX, gyroY, gyroZ;
  // GPS
  int32_t lat, lon;
  int32_t alt;
  uint8_t fix_type;
  uint8_t satellites;
  float ground_speed;
  // Attitude
  float roll, pitch, yaw;
  // Battery
  uint16_t voltage;
  int16_t current;
} sensorData;

// Timer variables
unsigned long lastHeartbeat = 0;
unsigned long lastDisplay = 0;
const unsigned long DISPLAY_INTERVAL = 500; // Update display every 500ms

void setup() {
  Serial.begin(57600);
  pixhawkSerial.begin(57600);
  
  Serial.println("=== MAVLink Sensor Reader ===");
}

void loop() {
  // Send heartbeat every second
  if (millis() - lastHeartbeat >= 1000) {
    send_heartbeat();
    request_data_stream();
    lastHeartbeat = millis();
  }

  // Read incoming messages
  read_messages();

  // Display data periodically
  if (millis() - lastDisplay >= DISPLAY_INTERVAL) {
    display_data();
    lastDisplay = millis();
  }
}

void send_heartbeat() {
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  
  mavlink_msg_heartbeat_pack(255, 190, &msg,
                            MAV_TYPE_GCS,
                            MAV_AUTOPILOT_INVALID,
                            MAV_MODE_FLAG_SAFETY_ARMED,
                            0,
                            MAV_STATE_ACTIVE);
                            
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  for(uint16_t i = 0; i < len; i++) {
    pixhawkSerial.write(buf[i]);
  }
}

void request_data_stream() {
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  
  const uint8_t streams[] = {
    MAV_DATA_STREAM_RAW_SENSORS,
    MAV_DATA_STREAM_EXTENDED_STATUS,
    MAV_DATA_STREAM_POSITION,
    MAV_DATA_STREAM_EXTRA1
  };
  
  for(uint8_t i = 0; i < sizeof(streams); i++) {
    mavlink_msg_request_data_stream_pack(255, 190, &msg,
                                       1, // target system
                                       0, // target component
                                       streams[i],
                                       10,  // 10 Hz
                                       1);  // Start
    
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    for(uint16_t j = 0; j < len; j++) {
      pixhawkSerial.write(buf[j]);
    }
    delay(10);
  }
}

void read_messages() {
  mavlink_message_t msg;
  mavlink_status_t status;
  
  while(pixhawkSerial.available()) {
    uint8_t c = pixhawkSerial.read();
    
    if(mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) {
      switch(msg.msgid) {
        case MAVLINK_MSG_ID_RAW_IMU: {
          mavlink_raw_imu_t imu;
          mavlink_msg_raw_imu_decode(&msg, &imu);
          sensorData.accX = imu.xacc;
          sensorData.accY = imu.yacc;
          sensorData.accZ = imu.zacc;
          sensorData.gyroX = imu.xgyro;
          sensorData.gyroY = imu.ygyro;
          sensorData.gyroZ = imu.zgyro;
          break;
        }
        
        case MAVLINK_MSG_ID_GPS_RAW_INT: {
          mavlink_gps_raw_int_t gps;
          mavlink_msg_gps_raw_int_decode(&msg, &gps);
          sensorData.lat = gps.lat;
          sensorData.lon = gps.lon;
          sensorData.alt = gps.alt;
          sensorData.fix_type = gps.fix_type;
          sensorData.satellites = gps.satellites_visible;
          sensorData.ground_speed = gps.vel / 100.0; // Convert from cm/s to m/s
          break;
        }
        
        case MAVLINK_MSG_ID_ATTITUDE: {
          mavlink_attitude_t attitude;
          mavlink_msg_attitude_decode(&msg, &attitude);
          sensorData.roll = attitude.roll * 180.0 / M_PI;
          sensorData.pitch = attitude.pitch * 180.0 / M_PI;
          sensorData.yaw = attitude.yaw * 180.0 / M_PI;
          break;
        }
        
        case MAVLINK_MSG_ID_SYS_STATUS: {
          mavlink_sys_status_t sys_status;
          mavlink_msg_sys_status_decode(&msg, &sys_status);
          sensorData.voltage = sys_status.voltage_battery;
          sensorData.current = sys_status.current_battery;
          break;
        }
      }
    }
  }
}

void display_data() {
  Serial.println("\n=== Sensor Data ===");
  
  // IMU Data
  Serial.println("IMU:");
  Serial.print("Acc (mg): X="); Serial.print(sensorData.accX);
  Serial.print(" Y="); Serial.print(sensorData.accY);
  Serial.print(" Z="); Serial.println(sensorData.accZ);
  
  Serial.print("Gyro: X="); Serial.print(sensorData.gyroX);
  Serial.print(" Y="); Serial.print(sensorData.gyroY);
  Serial.print(" Z="); Serial.println(sensorData.gyroZ);
  
  // Attitude
  Serial.println("\nAttitude (deg):");
  Serial.print("Roll="); Serial.print(sensorData.roll, 1);
  Serial.print(" Pitch="); Serial.print(sensorData.pitch, 1);
  Serial.print(" Yaw="); Serial.println(sensorData.yaw, 1);
  
  // GPS
  Serial.println("\nGPS:");
  Serial.print("Fix: "); Serial.print(sensorData.fix_type);
  Serial.print(" Sats: "); Serial.println(sensorData.satellites);
  if(sensorData.fix_type > 1) {
    Serial.print("Lat: "); Serial.print(sensorData.lat / 1e7, 7);
    Serial.print(" Lon: "); Serial.print(sensorData.lon / 1e7, 7);
    Serial.print(" Alt: "); Serial.print(sensorData.alt / 1000.0);
    Serial.print("m Speed: "); Serial.print(sensorData.ground_speed);
    Serial.println(" m/s");
  }
  
  // Battery
  Serial.println("\nBattery:");
  Serial.print("Voltage: "); Serial.print(sensorData.voltage / 1000.0);
  Serial.print("V Current: "); Serial.print(sensorData.current / 100.0);
  Serial.println("A");
  
  Serial.println("==================");
}