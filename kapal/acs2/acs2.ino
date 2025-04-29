/*
 * Program for Reading Current Values with Two ACS758 100A Current Sensors and ESP32
 * 
 * This program reads current values from two ACS758 100A current sensors
 * connected to an ESP32 and displays the results on the Serial Monitor.
 */

// Pins connected to the ACS758 sensor outputs
const int ACS_PIN_1 = 15;  // First sensor connected to GPIO15
const int ACS_PIN_2 = 2;   // Second sensor connected to GPIO2

// ACS758 100A sensor parameters
const float VOLTAGE_REF = 3.3;       // ESP32 ADC reference voltage in volts
const float SENSITIVITY = 10.0;      // 20 mV/A for ACS758 100A model
const float ZERO_CURRENT_OFFSET_1 = 1.548; // Adjusted based on no-load readings for sensor 1
const float ZERO_CURRENT_OFFSET_2 = 1.555; // Adjusted based on no-load readings for sensor 2

// Variables for calculations
float voltage1, voltage2;
float current1, current2;

// ADC resolution (ESP32 has 12-bit ADC: 0-4095)
const float ADC_RESOLUTION = 4095.0;

void setup() {
  // Initialize serial communication
  Serial.begin(115200);
  
  // Set ADC resolution to 12 bits (0-4095)
  analogReadResolution(12);
  
  // Configure ADC
  analogSetAttenuation(ADC_11db);  // Set ADC attenuation for full 3.3V range
  
  Serial.println("Dual ACS758 100A Current Sensor Reading Program");
  Serial.println("==============================================");
  delay(1000);
}

void loop() {
  // Read and process sensor 1
  readSensor(ACS_PIN_1, ZERO_CURRENT_OFFSET_1, "Sensor 1 (Pin 15)");
  
  // Read and process sensor 2
  readSensor(ACS_PIN_2, ZERO_CURRENT_OFFSET_2, "Sensor 2 (Pin 2)");
  
  // Print a separator line
  Serial.println("----------------------------------------------");
  
  delay(1000); // Update every 1 second
}

void readSensor(int sensorPin, float zeroOffset, const char* sensorName) {
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
