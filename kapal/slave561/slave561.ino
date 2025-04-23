/*********
  I2C Slave 2 program for ESP32 with MAVLink Speed & Heading
  Communicates with Pixhawk via UART and responds to I2C master requests
*********/

#include <Wire.h>
#include <HardwareSerial.h>
#include <mavlink.h>

// I2C slave address
#define I2C_DEV_ADDR 0x56

// MAVLink communication
HardwareSerial pixhawkSerial(2); // UART2 on ESP32 (pin 16 RX, 17 TX)

// Global variables for latest data
volatile float groundSpeed_kmh = 0;
volatile float heading_deg = 0;
TaskHandle_t MAVLinkTask;

// Function called when master requests data from this slave
void onRequest() {
  // Create response string with current speed and heading
  char response[32];
  sprintf(response, "SPD:%.1f HDG:%.0f", groundSpeed_kmh, heading_deg);
  
  // Send the response
  Wire.print(response);
  
  Serial.print("I2C Request - Sent: ");
  Serial.println(response);
}

// Function called when master sends data to this slave
void onReceive(int len) {
  String command = "";
  
  Serial.printf("I2C Receive[%d]: ", len);
  
  // Read the command
  while (Wire.available()) {
    char c = Wire.read();
    command += c;
    Serial.write(c);
  }
  Serial.println();
}

// Request MAVLink data streams
void request_data() {
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  // Request EXTRA1 for ground speed (VFR_HUD) and attitude (ATTITUDE)
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

  // Request POSITION for alternative ground speed
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

// MAVLink processing task on Core 0
void MAVLinkHandler(void* parameter) {
  Serial.println("MAVLink task started on Core 0");
  
  unsigned long lastRequest = 0;
  unsigned long lastDataPrint = 0;
  
  // Initial data request
  request_data();
  
  while (true) {
    mavlink_message_t msg;
    mavlink_status_t status;

    // Process incoming MAVLink messages
    while (pixhawkSerial.available()) {
      uint8_t c = pixhawkSerial.read();

      if (mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) {
        // Process VFR_HUD message for ground speed
        if (msg.msgid == MAVLINK_MSG_ID_VFR_HUD) {
          mavlink_vfr_hud_t hud;
          mavlink_msg_vfr_hud_decode(&msg, &hud);
          
          // Convert ground speed from m/s to km/h
          groundSpeed_kmh = hud.groundspeed * 3.6;
          
          // Heading also available in VFR_HUD
          heading_deg = hud.heading;
        }
        
        // Process ATTITUDE message for heading if not available in VFR_HUD
        else if (msg.msgid == MAVLINK_MSG_ID_ATTITUDE) {
          mavlink_attitude_t attitude;
          mavlink_msg_attitude_decode(&msg, &attitude);
          
          // Convert yaw from radians to degrees (0-360)
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
          
          // Calculate ground speed from velocity components
          float vx = float(global_pos.vx) / 100.0; // cm/s to m/s
          float vy = float(global_pos.vy) / 100.0; // cm/s to m/s
          float groundspeed_ms = sqrt(vx * vx + vy * vy);
          groundSpeed_kmh = groundspeed_ms * 3.6; // m/s to km/h
        }
      }
    }

    // Print data every 500ms for debugging
    if (millis() - lastDataPrint >= 500) {
      Serial.print("Speed: ");
      Serial.print(groundSpeed_kmh, 1); // 1 decimal place
      Serial.print(" km/h | Heading: ");
      Serial.print(heading_deg, 0); // No decimal
      Serial.println(" degrees");
      
      lastDataPrint = millis();
    }

    // Request data every 2 seconds
    if (millis() - lastRequest >= 2000) {
      request_data();
      lastRequest = millis();
    }
    
    // Small delay to prevent watchdog issues
    vTaskDelay(5 / portTICK_PERIOD_MS);
  }
}

void setup() {
  // Initialize serial for debugging
  Serial.begin(115200);
  
  // Initialize serial communication with Pixhawk
  pixhawkSerial.begin(57600, SERIAL_8N1, 16, 17);
  
  Serial.println("ESP32 I2C Slave 2 with MAVLink Speed & Heading Starting...");
  delay(1000);
  
  // Setup I2C slave on Core 1
  Wire.onReceive(onReceive);
  Wire.onRequest(onRequest);
  Wire.begin((uint8_t)I2C_DEV_ADDR);
  
  Serial.printf("ESP32 I2C Slave 2 initialized at address: 0x%X\n", I2C_DEV_ADDR);
  
  // Create MAVLink processing task on Core 0
  xTaskCreatePinnedToCore(
    MAVLinkHandler,
    "MAVLinkTask",
    10000,
    NULL,
    1,
    &MAVLinkTask,
    0
  );
  
  Serial.println("MAVLink handler task created on Core 0");
  Serial.println("I2C communication running on Core 1");
}

void loop() {
  // Main loop runs on Core 1 and handles I2C communication
  // The Wire library events (onReceive/onRequest) are automatically processed
  
  // Loop can be kept minimal as the MAVLink processing is in a separate task
  delay(100);
}