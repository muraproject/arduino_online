#ifndef VARIABLE_DEFINITIONS_H
#define VARIABLE_DEFINITIONS_H

#include "variables_settings.h"

// Servo/ESC instances
Servo ESC1;
Servo ESC2;
Servo ESC3;    // Servo ketiga
Servo RUDDER;  // Rudder servo

// Pin definitions
const int esc1Pin = PA0;
const int esc2Pin = PA1;
const int esc3Pin = PA4;    // Pin untuk servo ketiga
const int rudderPin = PA7;  // Pin untuk rudder

// Konstanta ESC untuk motor 1 & 2 (dalam microseconds)
const int MIN_PULSE = 1500;    // Minimum pulse width
const int START_PULSE = 1530;  // Starting pulse width
const int MAX_PULSE = 1950;    // Maximum pulse width
const int STEP_SIZE = 5;       // Step size (lebih halus)

// Konstanta ESC untuk motor 3 (dalam derajat)
const int MIN_PULSE3_DEG = 90;    // Minimum angle = 90 derajat
const int START_PULSE3_DEG = 98;  // Starting angle = 93 derajat
const int MAX_PULSE3_DEG = 115;   // Maximum angle = 105 derajat
const int STEP_SIZE3_DEG = 1;     // Step size dalam derajat

// Konstanta Rudder (microseconds)
const int RUDDER_MIN = 1000;     // Minimum rudder angle
const int RUDDER_CENTER = 1500;  // Center rudder position
const int RUDDER_MAX = 2000;     // Maximum rudder angle

// Variabel throttle motor 3 dalam derajat
int currentThrottle3_deg = MIN_PULSE3_DEG+8;

// Remote control variables
int remoteThrottle = 100;   // Throttle from remote (90-110)
int remoteRudderAngle = 0;  // Rudder angle from remote
int lastRemoteThrottle = -1;
int lastRemoteRudderAngle = -999;

// ESP32 komunikasi (tetap dipertahankan untuk kompatibilitas dengan LoRa)
HardwareSerial ESP32Serial(PA10, PA9);

// Throttle variables
int currentThrottle1 = MIN_PULSE;
int currentThrottle2 = MIN_PULSE;
int currentThrottle3 = MIN_PULSE3_DEG;  // Untuk kompatibilitas, sama dengan currentThrottle3_deg

// RPM variables
int targetRPM = 0;
int targetRPM3 = 0;   // Target RPM untuk motor ketiga
int currentRPM = 0;   // RPM dari sensor 1 (pin 35)
int currentRPM2 = 0;  // RPM dari sensor 2 (pin 34)
int currentRPM3 = 0;  // RPM dari sensor ketiga (pin 32)
unsigned long lastRPMCheck = 0;
const unsigned long RPM_CHECK_INTERVAL = 100;

// TAMBAHAN: Timing khusus untuk fuzzy control motor ketiga
unsigned long lastFuzzy3Check = 0;
const unsigned long FUZZY3_CHECK_INTERVAL = 2000;  // 2 detik = 2000ms

// GROUND SPEED CONTROL - TAMBAHAN BARU
bool groundSpeedControlActive = false;  // Flag untuk ground speed control
float targetGroundSpeed = 0.0;          // Target ground speed dalam km/h
unsigned long lastGroundSpeedCheck = 0;
const unsigned long GROUND_SPEED_CHECK_INTERVAL = 1000;  // 1 detik untuk update RPM target

// PID GROUND SPEED CONTROL - TAMBAHAN BARU
bool pidSpeedControlActive = false;  // Flag untuk PID ground speed control
float targetPIDSpeed = 0.0;          // Target PID speed dalam km/h
unsigned long lastPIDSpeedCheck = 0;
const unsigned long PID_SPEED_CHECK_INTERVAL = 1000;  // 300ms untuk update RPM target

// FUZZY PID GAIN SCHEDULING CONTROL - FITUR BARU
bool fuzzyPIDControlActive = false;  // Flag untuk Fuzzy PID Gain Scheduling control
float targetFuzzyPIDSpeed = 0.0;     // Target Fuzzy PID speed dalam km/h
unsigned long lastFuzzyPIDCheck = 0;
const unsigned long FUZZY_PID_CHECK_INTERVAL = 1000;  // 300ms untuk update RPM target

// Neural Network control variables
bool neuralControlActive = false;
float targetNeuralSpeed = 0.0;
unsigned long lastNeuralCheck = 0;
const unsigned long NEURAL_CHECK_INTERVAL = 500;  // 500ms neural control update
NeuralNetwork nn;

// NN Performance tracking
float nnPerformanceScore = 0.0;
int nnSuccessCount = 0;
int nnTotalPredictions = 0;

// PID Parameters untuk Ground Speed Control (Fixed)
const float PID_KP = 500.0;   // Proportional gain (RPM per km/h error)
const float PID_KI = 200.0;   // Integral gain
const float PID_KD = 50.0;   // Derivative gain

// FUZZY PID GAIN SCHEDULING Parameters - FITUR BARU
// Base parameters
const float FUZZY_PID_KI_BASE = 200.0;  // Ki tetap (tidak diubah fuzzy)

// Fuzzy Kp boundaries (berdasarkan error speed)
const float FUZZY_KP_MIN = 400.0;   // Minimum Kp (error kecil)
const float FUZZY_KP_MAX = 600.0;  // Maximum Kp (error besar)

// Fuzzy Kd boundaries (berdasarkan error speed)  
const float FUZZY_KD_MIN = 25.0;    // Minimum Kd (error besar)
const float FUZZY_KD_MAX = 75.0;   // Maximum Kd (error kecil)

// Error boundaries untuk fuzzy gain scheduling
const float GAIN_ERROR_SMALL = 1.0;   // Error kecil (< 1 km/h)
const float GAIN_ERROR_MEDIUM = 3.0;  // Error sedang (1-3 km/h)
const float GAIN_ERROR_LARGE = 5.0;   // Error besar (> 3 km/h)

// PID Variables (Fixed PID)
float pidSpeedError = 0.0;
float pidSpeedErrorPrev = 0.0;
float pidSpeedIntegral = 0.0;
float pidSpeedDerivative = 0.0;

// FUZZY PID Variables - FITUR BARU
float fuzzyPIDSpeedError = 0.0;
float fuzzyPIDSpeedErrorPrev = 0.0;
float fuzzyPIDSpeedIntegral = 0.0;
float fuzzyPIDSpeedDerivative = 0.0;
float adaptiveKp = PID_KP;  // Adaptive Kp dari fuzzy
float adaptiveKd = PID_KD;  // Adaptive Kd dari fuzzy

// Base RPM values untuk setiap motor (starting point)
int baseRPM1 = 2000;  // Base RPM untuk motor 1
int baseRPM2 = 2000;  // Base RPM untuk motor 2
int baseRPM3 = 1500;  // Base RPM untuk motor 3

// RPM adjustment limits per detik
const int RPM_ADJUST_MIN = 50;  // Minimum adjustment per detik
const int RPM_ADJUST_MAX = 500;  // Maximum adjustment per detik

// Additional variables for I2C data from slave2
float groundSpeed_kmh = 0.0;
float heading_deg = 0.0;
float voltage1 = 0.0;
float voltage2 = 0.0;
float current1 = 0.0;
float current2 = 0.0;

// I2C communication timing
unsigned long lastI2CRequestTime = 0;
const unsigned long I2C_REQUEST_INTERVAL = 100;  // 100ms = 10Hz

// Control Mode
bool controlActive = false;
bool control3Active = false;       // Flag untuk kontrol motor ketiga
bool remoteControlActive = false;  // Flag for remote control mode
ControlMode currentMode = NONE;

// SMC variables
float prevError1_SMC = 0;
float prevError2_SMC = 0;

// SMC params (adjusted for microsecond range)
const int LAMBDA_INT = 3;  // 0.03 * 100
const int ETA_INT = 50;    // 0.5 * 100
const int K_INT = 5;       // 0.05 * 100
const int BOUNDARY_INT = 800;

// Fuzzy Variables
int prevError1_Fuzzy = 0;
int prevError2_Fuzzy = 0;
int prevError3_Fuzzy = 0;  // Error sebelumnya untuk motor ketiga

// Parameter Fuzzy untuk Motor 1 & 2
const int ERROR_NEGL = -5000;
const int ERROR_NEGS = -2000;
const int ERROR_ZERO = 0;
const int ERROR_POSS = 2000;
const int ERROR_POSL = 5000;

// Parameter Fuzzy KHUSUS untuk Motor 3 (ESC3)
const int ERROR3_NEGL = -5000;  // Negative Large boundary untuk ESC3
const int ERROR3_NEGS = -1700;  // Negative Small boundary untuk ESC3
const int ERROR3_ZERO = 0;      // Zero boundary untuk ESC3
const int ERROR3_POSS = 1700;   // Positive Small boundary untuk ESC3
const int ERROR3_POSL = 5000;   // Positive Large boundary untuk ESC3

// Parameter Fuzzy untuk GROUND SPEED CONTROL
const float SPEED_ERROR_NEGL = -5.0;  // Negative Large boundary untuk speed (km/h)
const float SPEED_ERROR_NEGS = -1.0;   // Negative Small boundary untuk speed
const float SPEED_ERROR_ZERO = 0.0;    // Zero boundary untuk speed
const float SPEED_ERROR_POSS = 1.0;    // Positive Small boundary untuk speed
const float SPEED_ERROR_POSL = 5.0;   // Positive Large boundary untuk speed

unsigned long lastJSONSendTime = 0;
const unsigned long JSON_SEND_INTERVAL = 300;  // 500ms = 2Hz

// LoRa communication
unsigned long lastHeartbeatTime = 0;
const unsigned long HEARTBEAT_INTERVAL = 5000;  // 5 seconds
char loraBuffer[64];
int loraBufferIndex = 0;
unsigned long messageCount = 0;

// Buffer for I2C communication
char buffer[64];  // Buffer for receiving data

// Flag to track if we're received any data from slaves
bool rpmDataReceived = false;
bool sensorDataReceived = false;

#endif // VARIABLE_DEFINITIONS_H