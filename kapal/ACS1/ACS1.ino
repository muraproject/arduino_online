/*
 * Program for Reading Current Values with ACS758 100A Current Sensor and ESP32
 * 
 * This program reads current values from an ACS758 100A current sensor
 * connected to an ESP32 and displays the results on the Serial Monitor.
 */

// Pin connected to the ACS758 sensor output
const int ACS_PIN = 34;  // Using GPIO34 which is an ADC pin on ESP32

// ACS758 100A sensor parameters
const float VOLTAGE_REF = 3.3;       // ESP32 ADC reference voltage in volts
const float SENSITIVITY = 20.0;      // 20 mV/A for ACS758 100A model
const float ZERO_CURRENT_OFFSET = 1.450; // Adjusted based on no-load readings

// Variables for calculations
float voltage;
float current;

// ADC resolution (ESP32 has 12-bit ADC: 0-4095)
const float ADC_RESOLUTION = 4095.0;

void setup() {
  // Initialize serial communication
  Serial.begin(115200);
  
  // Set ADC resolution to 12 bits (0-4095)
  analogReadResolution(12);
  
  // Configure ADC
  analogSetAttenuation(ADC_11db);  // Set ADC attenuation for full 3.3V range
  
  Serial.println("ACS758 100A Current Sensor Reading Program");
  Serial.println("==========================================");
  delay(1000);
}

void loop() {
  // Variables for averaging and RMS calculation
  long sumSensorValue = 0;
  float sumVoltage = 0;
  float sumCurrent = 0;
  float sumCurrentSquared = 0;  // For RMS calculation
  const int numSamples = 30; // Average 30 readings
  
  // Collect samples
  for (int i = 0; i < numSamples; i++) {
    // Read the sensor value
    int sensorValue = analogRead(ACS_PIN);
    sumSensorValue += sensorValue;
    
    // Convert the analog reading to voltage
    float sampleVoltage = (sensorValue / ADC_RESOLUTION) * VOLTAGE_REF;
    sumVoltage += sampleVoltage;
    
    // Calculate current based on sensitivity and zero current offset
    float sampleCurrent = (sampleVoltage - ZERO_CURRENT_OFFSET) / (SENSITIVITY / 1000.0);
    sumCurrent += sampleCurrent;
    
    // Square the current for RMS calculation
    sumCurrentSquared += sampleCurrent * sampleCurrent;
    
    delay(2); // Small delay between readings
  }
  
  // Calculate averages
  int avgSensorValue = sumSensorValue / numSamples;
  voltage = sumVoltage / numSamples;
  current = sumCurrent / numSamples;
  
  // Calculate RMS current
  float rmsCurrentSquared = sumCurrentSquared / numSamples;
  float rmsCurrent = sqrt(rmsCurrentSquared);
  
  // Print the results
  Serial.print("Average of ");
  Serial.print(numSamples);
  Serial.print(" samples | ADC Value: ");
  Serial.print(avgSensorValue);
  Serial.print(" | Voltage: ");
  Serial.print(voltage, 3);
  Serial.print(" V | Average Current: ");
  Serial.print(current, 2);
  Serial.print(" A | RMS Current: ");
  Serial.print(rmsCurrent, 2);
  Serial.println(" A");
  
  delay(2000); // Update every 2 seconds
}