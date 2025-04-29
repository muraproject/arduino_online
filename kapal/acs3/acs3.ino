/*
 * Program for Reading Current and Voltage Values with ESP32
 * 
 * This program reads:
 * - Current values from two ACS758 100A current sensors (pins 15 and 2)
 * - Voltage values from two voltage divider circuits (pins 35 and 32)
 * 
 * The voltage divider consists of:
 * - Upper resistor: 10kΩ
 * - Lower resistor: 2kΩ
 * - Ratio: 1/5 (measures up to 5x reference voltage)
 */

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

// Variables for calculations
float voltage1, voltage2;
float current1, current2;
float externalVoltage1, externalVoltage2;  // Voltages measured by the dividers

// ADC resolution (ESP32 has 12-bit ADC: 0-4095)
const float ADC_RESOLUTION = 4095.0;

void setup() {
  // Initialize serial communication
  Serial.begin(115200);
  
  // Set ADC resolution to 12 bits (0-4095)
  analogReadResolution(12);
  
  // Configure ADC
  analogSetAttenuation(ADC_11db);  // Set ADC attenuation for full 3.3V range
  
  Serial.println("Current and Voltage Sensors Reading Program");
  Serial.println("===========================================");
  delay(1000);
}

void loop() {
  // Read and process current sensors
  readCurrentSensor(ACS_PIN_1, ZERO_CURRENT_OFFSET_1, "Current Sensor 1 (Pin 15)");
  readCurrentSensor(ACS_PIN_2, ZERO_CURRENT_OFFSET_2, "Current Sensor 2 (Pin 2)");
  
  // Read and process voltage sensors
  readVoltageSensor(VOLTAGE_PIN_1, "Voltage Sensor 1 (Pin 35)");
  readVoltageSensor(VOLTAGE_PIN_2, "Voltage Sensor 2 (Pin 32)");
  
  // Print a separator line
  Serial.println("----------------------------------------------");
  
  delay(1000); // Update every 1 second
}

void readCurrentSensor(int sensorPin, float zeroOffset, const char* sensorName) {
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
    
    delay(2); // Small delay between readings
  }
  
  // Calculate averages
  int avgSensorValue = sumSensorValue / numSamples;
  float voltage = sumVoltage / numSamples;
  float current = sumCurrent / numSamples;
  
  // Calculate RMS current
  float rmsCurrentSquared = sumCurrentSquared / numSamples;
  float rmsCurrent = sqrt(rmsCurrentSquared);
  
  // Print the results
  Serial.print(sensorName);
  Serial.print(" | ADC Value: ");
  Serial.print(avgSensorValue);
  Serial.print(" | Voltage: ");
  Serial.print(voltage, 3);
  Serial.print(" V | Average Current: ");
  Serial.print(current, 2);
  Serial.print(" A | RMS Current: ");
  Serial.print(rmsCurrent, 2);
  Serial.println(" A");
}

void readVoltageSensor(int sensorPin, const char* sensorName) {
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
    
    delay(2); // Small delay between readings
  }
  
  // Calculate averages
  int avgSensorValue = sumSensorValue / numSamples;
  float adcVoltage = sumVoltage / numSamples;
  float externalVoltage = sumExternalVoltage / numSamples;
  
  // Print the results
  Serial.print(sensorName);
  Serial.print(" | ADC Value: ");
  Serial.print(avgSensorValue);
  Serial.print(" | ADC Voltage: ");
  Serial.print(adcVoltage, 3);
  Serial.print(" V | External Voltage: ");
  Serial.print(externalVoltage, 3);
  Serial.println(" V");
}