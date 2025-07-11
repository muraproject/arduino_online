#ifndef VARIABLES_SETTINGS_H
#define VARIABLES_SETTINGS_H

#include <Servo.h>
#include <Wire.h>

// I2C Slave addresses (matching the slave programs)
#define SLAVE1_ADDR 0x55  // RPM slave
#define SLAVE2_ADDR 0x56  // Current, voltage, speed slave

// Set a slower I2C clock speed for better reliability
#define I2C_CLOCK_SPEED 50000  // 50 kHz

// Servo/ESC
extern Servo ESC1;
extern Servo ESC2;
extern Servo ESC3;    // Servo ketiga
extern Servo RUDDER;  // Rudder servo
extern const int esc1Pin;
extern const int esc2Pin;
extern const int esc3Pin;    // Pin untuk servo ketiga
extern const int rudderPin;  // Pin untuk rudder

// Konstanta ESC untuk motor 1 & 2 (dalam microseconds)
extern const int MIN_PULSE;    // Minimum pulse width
extern const int START_PULSE;  // Starting pulse width
extern const int MAX_PULSE;    // Maximum pulse width
extern const int STEP_SIZE;       // Step size (lebih halus)

// Konstanta ESC untuk motor 3 (dalam derajat)
extern const int MIN_PULSE3_DEG;    // Minimum angle = 90 derajat
extern const int START_PULSE3_DEG;  // Starting angle = 93 derajat
extern const int MAX_PULSE3_DEG;   // Maximum angle = 105 derajat
extern const int STEP_SIZE3_DEG;     // Step size dalam derajat

// Konstanta Rudder (microseconds)
extern const int RUDDER_MIN;     // Minimum rudder angle
extern const int RUDDER_CENTER;  // Center rudder position
extern const int RUDDER_MAX;     // Maximum rudder angle

// Variabel throttle motor 3 dalam derajat
extern int currentThrottle3_deg;

// Remote control variables
extern int remoteThrottle;   // Throttle from remote (90-110)
extern int remoteRudderAngle;  // Rudder angle from remote
extern int lastRemoteThrottle;
extern int lastRemoteRudderAngle;

// ESP32 komunikasi (tetap dipertahankan untuk kompatibilitas dengan LoRa)
extern HardwareSerial ESP32Serial;

// Throttle variables
extern int currentThrottle1;
extern int currentThrottle2;
extern int currentThrottle3;  // Untuk kompatibilitas, sama dengan currentThrottle3_deg

// RPM variables
extern int targetRPM;
extern int targetRPM3;   // Target RPM untuk motor ketiga
extern int currentRPM;   // RPM dari sensor 1 (pin 35)
extern int currentRPM2;  // RPM dari sensor 2 (pin 34)
extern int currentRPM3;  // RPM dari sensor ketiga (pin 32)
extern unsigned long lastRPMCheck;
extern const unsigned long RPM_CHECK_INTERVAL;

// TAMBAHAN: Timing khusus untuk fuzzy control motor ketiga
extern unsigned long lastFuzzy3Check;
extern const unsigned long FUZZY3_CHECK_INTERVAL;  // 2 detik = 2000ms

// GROUND SPEED CONTROL - TAMBAHAN BARU
extern bool groundSpeedControlActive;  // Flag untuk ground speed control
extern float targetGroundSpeed;          // Target ground speed dalam km/h
extern unsigned long lastGroundSpeedCheck;
extern const unsigned long GROUND_SPEED_CHECK_INTERVAL;  // 1 detik untuk update RPM target

// PID GROUND SPEED CONTROL - TAMBAHAN BARU
extern bool pidSpeedControlActive;  // Flag untuk PID ground speed control
extern float targetPIDSpeed;          // Target PID speed dalam km/h
extern unsigned long lastPIDSpeedCheck;
extern const unsigned long PID_SPEED_CHECK_INTERVAL;  // 300ms untuk update RPM target

// FUZZY PID GAIN SCHEDULING CONTROL - FITUR BARU
extern bool fuzzyPIDControlActive;  // Flag untuk Fuzzy PID Gain Scheduling control
extern float targetFuzzyPIDSpeed;     // Target Fuzzy PID speed dalam km/h
extern unsigned long lastFuzzyPIDCheck;
extern const unsigned long FUZZY_PID_CHECK_INTERVAL;  // 300ms untuk update RPM target

// ==================== NEURAL NETWORK VARIABLES - BARU ====================
// NN Structure: 5 inputs -> 8 hidden -> 3 outputs
#define NN_INPUT_SIZE 5
#define NN_HIDDEN_SIZE 8  
#define NN_OUTPUT_SIZE 3

struct NeuralNetwork {
  // Weights and biases (pre-trained from corrected data)
  float weights_input_hidden[NN_INPUT_SIZE][NN_HIDDEN_SIZE];
  float bias_hidden[NN_HIDDEN_SIZE];
  float weights_hidden_output[NN_HIDDEN_SIZE][NN_OUTPUT_SIZE];
  float bias_output[NN_OUTPUT_SIZE];
  
  // Activation arrays
  float hidden_layer[NN_HIDDEN_SIZE];
  float output_layer[NN_OUTPUT_SIZE];
  
  // Normalization parameters (from dataset analysis)
  float input_min[NN_INPUT_SIZE] = {4.0, 3.5, 0.0, 0.0, 0.0};      // target, speed, rpm1, rpm2, rpm3
  float input_max[NN_INPUT_SIZE] = {12.0, 12.3, 12000.0, 12000.0, 14700.0};
  float output_scale = 200.0; // Max RPM adjustment
};

// Neural Network control variables
extern bool neuralControlActive;
extern float targetNeuralSpeed;
extern unsigned long lastNeuralCheck;
extern const unsigned long NEURAL_CHECK_INTERVAL;  // 500ms neural control update
extern NeuralNetwork nn;

// NN Performance tracking
extern float nnPerformanceScore;
extern int nnSuccessCount;
extern int nnTotalPredictions;
// ================== END NEURAL NETWORK VARIABLES ==================

// PID Parameters untuk Ground Speed Control (Fixed)
extern const float PID_KP;   // Proportional gain (RPM per km/h error)
extern const float PID_KI;   // Integral gain
extern const float PID_KD;   // Derivative gain

// FUZZY PID GAIN SCHEDULING Parameters - FITUR BARU
// Base parameters
extern const float FUZZY_PID_KI_BASE;  // Ki tetap (tidak diubah fuzzy)

// Fuzzy Kp boundaries (berdasarkan error speed)
extern const float FUZZY_KP_MIN;   // Minimum Kp (error kecil)
extern const float FUZZY_KP_MAX;  // Maximum Kp (error besar)

// Fuzzy Kd boundaries (berdasarkan error speed)  
extern const float FUZZY_KD_MIN;    // Minimum Kd (error besar)
extern const float FUZZY_KD_MAX;   // Maximum Kd (error kecil)

// Error boundaries untuk fuzzy gain scheduling
extern const float GAIN_ERROR_SMALL;   // Error kecil (< 1 km/h)
extern const float GAIN_ERROR_MEDIUM;  // Error sedang (1-3 km/h)
extern const float GAIN_ERROR_LARGE;   // Error besar (> 3 km/h)

// PID Variables (Fixed PID)
extern float pidSpeedError;
extern float pidSpeedErrorPrev;
extern float pidSpeedIntegral;
extern float pidSpeedDerivative;

// FUZZY PID Variables - FITUR BARU
extern float fuzzyPIDSpeedError;
extern float fuzzyPIDSpeedErrorPrev;
extern float fuzzyPIDSpeedIntegral;
extern float fuzzyPIDSpeedDerivative;
extern float adaptiveKp;  // Adaptive Kp dari fuzzy
extern float adaptiveKd;  // Adaptive Kd dari fuzzy

// Base RPM values untuk setiap motor (starting point)
extern int baseRPM1;  // Base RPM untuk motor 1
extern int baseRPM2;  // Base RPM untuk motor 2
extern int baseRPM3;  // Base RPM untuk motor 3

// RPM adjustment limits per detik
extern const int RPM_ADJUST_MIN;  // Minimum adjustment per detik
extern const int RPM_ADJUST_MAX;  // Maximum adjustment per detik

// Additional variables for I2C data from slave2
extern float groundSpeed_kmh;
extern float heading_deg;
extern float voltage1;
extern float voltage2;
extern float current1;
extern float current2;

// I2C communication timing
extern unsigned long lastI2CRequestTime;
extern const unsigned long I2C_REQUEST_INTERVAL;  // 100ms = 10Hz

// Control Mode
extern bool controlActive;
extern bool control3Active;       // Flag untuk kontrol motor ketiga
extern bool remoteControlActive;  // Flag for remote control mode
enum ControlMode { NONE,
                   SMC,
                   FUZZY,
                   REMOTE,
                   GROUND_SPEED,
                   PID_SPEED,
                   FUZZY_PID_SPEED,
                   NEURAL_NETWORK };  // Tambahan mode Neural Network
extern ControlMode currentMode;

// SMC variables
extern float prevError1_SMC;
extern float prevError2_SMC;

// SMC params (adjusted for microsecond range)
extern const int LAMBDA_INT;  // 0.03 * 100
extern const int ETA_INT;    // 0.5 * 100
extern const int K_INT;       // 0.05 * 100
extern const int BOUNDARY_INT;

// Fuzzy Variables
extern int prevError1_Fuzzy;
extern int prevError2_Fuzzy;
extern int prevError3_Fuzzy;  // Error sebelumnya untuk motor ketiga

// Parameter Fuzzy untuk Motor 1 & 2
extern const int ERROR_NEGL;
extern const int ERROR_NEGS;
extern const int ERROR_ZERO;
extern const int ERROR_POSS;
extern const int ERROR_POSL;

// Parameter Fuzzy KHUSUS untuk Motor 3 (ESC3)
extern const int ERROR3_NEGL;  // Negative Large boundary untuk ESC3
extern const int ERROR3_NEGS;  // Negative Small boundary untuk ESC3
extern const int ERROR3_ZERO;      // Zero boundary untuk ESC3
extern const int ERROR3_POSS;   // Positive Small boundary untuk ESC3
extern const int ERROR3_POSL;   // Positive Large boundary untuk ESC3

// Parameter Fuzzy untuk GROUND SPEED CONTROL
extern const float SPEED_ERROR_NEGL;  // Negative Large boundary untuk speed (km/h)
extern const float SPEED_ERROR_NEGS;   // Negative Small boundary untuk speed
extern const float SPEED_ERROR_ZERO;    // Zero boundary untuk speed
extern const float SPEED_ERROR_POSS;    // Positive Small boundary untuk speed
extern const float SPEED_ERROR_POSL;   // Positive Large boundary untuk speed

extern unsigned long lastJSONSendTime;
extern const unsigned long JSON_SEND_INTERVAL;  // 500ms = 2Hz

// LoRa communication
extern unsigned long lastHeartbeatTime;
extern const unsigned long HEARTBEAT_INTERVAL;  // 5 seconds
extern char loraBuffer[64];
extern int loraBufferIndex;
extern unsigned long messageCount;

// Buffer for I2C communication
extern char buffer[64];  // Buffer for receiving data

// Flag to track if we're received any data from slaves
extern bool rpmDataReceived;
extern bool sensorDataReceived;

#endif // VARIABLES_SETTINGS_H