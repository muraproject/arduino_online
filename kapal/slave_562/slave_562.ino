/*
 * Integrated Program for ESP32:
 * - Reading Current Values from two ACS758 100A Current Sensors
 * - Reading Voltage Values from two voltage divider circuits
 * - Reading Speed and Heading data from Pixhawk via MAVLink
 * - Operating as I2C slave to provide data to a master device
 */

#include <Wire.h>
#include <HardwareSerial.h>
#include <mavlink.h>

// I2C slave address
#define I2C_DEV_ADDR 0x56

// Pins connected to the ACS758 current sensor outputs
const int ACS_PIN_1 = 15;  // First current sensor connected to GPIO15
const int ACS_PIN_2 = 2;   // Second current sensor connected to GPIO2

// Pins connected to voltage divider outputs
const int VOLTAGE_PIN_1 = 35;  // First voltage divider connected to GPIO35
const int VOLTAGE_PIN_2 = 32;  // Second voltage divider connected to GPIO32

// ACS758 100A sensor parameters
const float VOLTAGE_REF = 3.3;       // ESP32 ADC reference voltage in volts
const float SENSITIVITY = 10.0;      // 20 mV/A for ACS758 100A model
const float ZERO_CURRENT_OFFSET_1 = 1.548; // Adjusted based on no-load readings for sensor 1
const float ZERO_CURRENT_OFFSET_2 = 1.555; // Adjusted based on no-load readings for sensor 2

// Voltage divider parameters
const float VOLTAGE_DIVIDER_RATIO = 5.0;  // For 10kΩ/2kΩ voltage divider (1/5 ratio)

// Variables for calculations - made volatile as they're shared between cores
volatile float voltage1, voltage2;
volatile float current1, current2;
volatile float externalVoltage1, externalVoltage2;  // Voltages measured by the dividers

// ADC resolution (ESP32 has 12-bit ADC: 0-4095)
const float ADC_RESOLUTION = 4095.0;

// MAVLink communication
HardwareSerial pixhawkSerial(2); // UART2 on ESP32 (pin 16 RX, 17 TX)

// Global variables for latest data
volatile float groundSpeed_kmh = 0;
volatile float heading_deg = 0;
volatile float rmsCurrent1 = 0;
volatile float rmsCurrent2 = 0;

// Task handles
TaskHandle_t MAVLinkTask;
TaskHandle_t SensorTask;

// Function called when master requests data from this slave
void onRequest() {
  // Create response string with current data from all sensors
  char response[128];
  sprintf(response, "SPD:%.1f,HDG:%.0f,V1:%.1f,V2:%.1f,I1:%.1f,I2:%.1f", 
          groundSpeed_kmh, heading_deg, 
          externalVoltage1, externalVoltage2,
          rmsCurrent1, rmsCurrent2);
  
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
  
  // Process any commands from the master here
  // (Future expansion capability)
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

// Function to read current sensor
void readCurrentSensor(int sensorPin, float zeroOffset, int sensorNumber) {
  // Variables for averaging and RMS calculation
  long sumSensorValue = 0;
  float sumVoltage = 0;
  float sumCurrent = 0;
  float sumCurrentSquared = 0;  // For RMS calculation
  const int numSamples = 30;    // Average 30 readings
  
  // Collect samples
  for (int i = 0; i < numSamples; i++) {
    // Read the sensor value
    int sensorValue = analogRead(sensorPin);
    sumSensorValue += sensorValue;
    
    // Convert the analog reading to voltage
    float sampleVoltage = (sensorValue / ADC_RESOLUTION) * VOLTAGE_REF;
    sumVoltage += sampleVoltage;
    
    // Calculate current based on sensitivity and zero current offset
    float sampleCurrent = (sampleVoltage - zeroOffset) / (SENSITIVITY / 1000.0);
    sumCurrent += sampleCurrent;
    
    // Square the current for RMS calculation
    sumCurrentSquared += sampleCurrent * sampleCurrent;
    
    delay(1); // Small delay between readings
  }
  
  // Calculate averages
  int avgSensorValue = sumSensorValue / numSamples;
  float voltage = sumVoltage / numSamples;
  float current = sumCurrent / numSamples;
  
  // Calculate RMS current
  float rmsCurrentSquared = sumCurrentSquared / numSamples;
  float rmsCurrent = sqrt(rmsCurrentSquared);
  
  // Store the results in the appropriate global variables
  if (sensorNumber == 1) {
    voltage1 = voltage;
    current1 = current;
    rmsCurrent1 = rmsCurrent;
  } else {
    voltage2 = voltage;
    current2 = current;
    rmsCurrent2 = rmsCurrent;
  }
}

// Function to read voltage sensor
void readVoltageSensor(int sensorPin, int sensorNumber) {
  // Variables for averaging
  long sumSensorValue = 0;
  float sumVoltage = 0;
  float sumExternalVoltage = 0;
  const int numSamples = 30;  // Average 30 readings
  
  // Collect samples
  for (int i = 0; i < numSamples; i++) {
    // Read the sensor value
    int sensorValue = analogRead(sensorPin);
    sumSensorValue += sensorValue;
    
    // Convert the analog reading to voltage at the ADC input
    float sampleVoltage = (sensorValue / ADC_RESOLUTION) * VOLTAGE_REF;
    sumVoltage += sampleVoltage;
    
    // Calculate the actual external voltage using the voltage divider ratio
    float sampleExternalVoltage = sampleVoltage * VOLTAGE_DIVIDER_RATIO;
    sumExternalVoltage += sampleExternalVoltage;
    
    delay(1); // Small delay between readings
  }
  
  // Calculate averages
  float adcVoltage = sumVoltage / numSamples;
  float externalVoltage = sumExternalVoltage / numSamples;
  
  // Store the results in the appropriate global variables
  if (sensorNumber == 1) {
    externalVoltage1 = externalVoltage;
  } else {
    externalVoltage2 = externalVoltage;
  }
}

// Sensor reading task on Core 0, runs alongside MAVLink task
void SensorHandler(void* parameter) {
  Serial.println("Sensor task started on Core 0");
  
  unsigned long lastPrint = 0;
  
  while (true) {
    // Read both current sensors
    readCurrentSensor(ACS_PIN_1, ZERO_CURRENT_OFFSET_1, 1);
    readCurrentSensor(ACS_PIN_2, ZERO_CURRENT_OFFSET_2, 2);
    
    // Read both voltage sensors
    readVoltageSensor(VOLTAGE_PIN_1, 1);
    readVoltageSensor(VOLTAGE_PIN_2, 2);
    
    // Print sensor data every 500ms for debugging
    if (millis() - lastPrint >= 500) {
      Serial.printf("Voltage 1: %.1fV | Voltage 2: %.1fV | ", externalVoltage1, externalVoltage2);
      Serial.printf("Current 1: %.1fA | Current 2: %.1fA\n", rmsCurrent1, rmsCurrent2);
      lastPrint = millis();
    }
    
    // Delay to prevent watchdog issues and allow time for other tasks
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}

void setup() {
  // Initialize serial for debugging
  Serial.begin(115200);
  
  // Set ADC resolution to 12 bits (0-4095)
  analogReadResolution(12);
  
  // Configure ADC
  analogSetAttenuation(ADC_11db);  // Set ADC attenuation for full 3.3V range
  
  // Initialize serial communication with Pixhawk
  pixhawkSerial.begin(57600, SERIAL_8N1, 16, 17);
  
  Serial.println("ESP32 Integrated Sensor System with I2C Slave Interface Starting...");
  delay(1000);
  
  // Setup I2C slave on Core 1
  Wire.onReceive(onReceive);
  Wire.onRequest(onRequest);
  Wire.begin((uint8_t)I2C_DEV_ADDR);
  
  Serial.printf("ESP32 I2C Slave initialized at address: 0x%X\n", I2C_DEV_ADDR);
  
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
  
  // Create Sensor reading task on Core 0
  xTaskCreatePinnedToCore(
    SensorHandler,
    "SensorTask",
    10000,
    NULL,
    2,  // Higher priority than MAVLink task
    &SensorTask,
    0
  );
  
  Serial.println("MAVLink and Sensor tasks created on Core 0");
  Serial.println("I2C communication running on Core 1");
}

void loop() {
  // Main loop runs on Core 1 and handles I2C communication
  // The Wire library events (onReceive/onRequest) are automatically processed
  
  // Loop can be kept minimal as the task processing is in separate tasks
  delay(100);
}