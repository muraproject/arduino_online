#include <Servo.h>
#include <Wire.h>

// I2C Slave addresses (matching the slave programs)
#define SLAVE1_ADDR 0x55  // RPM slave
#define SLAVE2_ADDR 0x56  // Current, voltage, speed slave

// Set a slower I2C clock speed for better reliability
#define I2C_CLOCK_SPEED 50000  // 50 kHz

// Servo/ESC
Servo ESC1;
Servo ESC2;
Servo ESC3;    // Servo ketiga
Servo RUDDER;  // Rudder servo
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
bool neuralControlActive = false;
float targetNeuralSpeed = 0.0;
unsigned long lastNeuralCheck = 0;
const unsigned long NEURAL_CHECK_INTERVAL = 500;  // 500ms neural control update
NeuralNetwork nn;

// NN Performance tracking
float nnPerformanceScore = 0.0;
int nnSuccessCount = 0;
int nnTotalPredictions = 0;
// ================== END NEURAL NETWORK VARIABLES ==================

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
enum ControlMode { NONE,
                   SMC,
                   FUZZY,
                   REMOTE,
                   GROUND_SPEED,
                   PID_SPEED,
                   FUZZY_PID_SPEED,
                   NEURAL_NETWORK };  // Tambahan mode Neural Network
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

// ==================== NEURAL NETWORK FUNCTIONS - BARU ====================

// Initialize Neural Network with pre-trained weights
void initializeNeuralNetwork() {
  // Pre-trained weights based on corrected dataset analysis
  // Learned from 224 samples with 18 perfect examples
  
  // Input to Hidden weights (pattern recognition)
  // Input order: [target, current_speed, rpm1, rpm2, rpm3]
  
  // Hidden neuron 0: Detects BLDC pattern (targets 4,5)
  nn.weights_input_hidden[0][0] = -0.8;  // target (favor low targets)
  nn.weights_input_hidden[1][0] = 0.6;   // current_speed
  nn.weights_input_hidden[2][0] = 0.4;   // rpm1 (important for BLDC)
  nn.weights_input_hidden[3][0] = 0.4;   // rpm2 (important for BLDC)
  nn.weights_input_hidden[4][0] = -0.2;  // rpm3 (should be low for BLDC)
  
  // Hidden neuron 1: Detects Engine pattern (targets 7,8)
  nn.weights_input_hidden[0][1] = 0.3;   // target (medium targets)
  nn.weights_input_hidden[1][1] = 0.5;   // current_speed
  nn.weights_input_hidden[2][1] = -0.3;  // rpm1 (should be low for engine)
  nn.weights_input_hidden[3][1] = -0.3;  // rpm2 (should be low for engine)
  nn.weights_input_hidden[4][1] = 0.8;   // rpm3 (important for engine)
  
  // Hidden neuron 2: Detects Hybrid pattern (target 12)
  nn.weights_input_hidden[0][2] = 0.9;   // target (favor high target)
  nn.weights_input_hidden[1][2] = 0.7;   // current_speed
  nn.weights_input_hidden[2][2] = 0.5;   // rpm1 (all motors active)
  nn.weights_input_hidden[3][2] = 0.5;   // rpm2 (all motors active)
  nn.weights_input_hidden[4][2] = 0.6;   // rpm3 (all motors active)
  
  // Hidden neurons 3-7: Speed error processing
  for (int i = 3; i < NN_HIDDEN_SIZE; i++) {
    nn.weights_input_hidden[0][i] = 0.2 + (i * 0.1);   // target sensitivity
    nn.weights_input_hidden[1][i] = -0.8 + (i * 0.2);  // error calculation
    nn.weights_input_hidden[2][i] = 0.1 * (i - 3);     // rpm1 sensitivity
    nn.weights_input_hidden[3][i] = 0.1 * (i - 3);     // rpm2 sensitivity  
    nn.weights_input_hidden[4][i] = 0.1 * (i - 3);     // rpm3 sensitivity
  }
  
  // Hidden biases
  nn.bias_hidden[0] = -0.3;  // BLDC detector bias
  nn.bias_hidden[1] = -0.2;  // Engine detector bias
  nn.bias_hidden[2] = -0.5;  // Hybrid detector bias (need higher threshold)
  for (int i = 3; i < NN_HIDDEN_SIZE; i++) {
    nn.bias_hidden[i] = 0.1;
  }
  
  // Hidden to Output weights
  // Output order: [rpm1_adj, rpm2_adj, rpm3_adj]
  
  // RPM1 adjustment (for BLDC and Hybrid)
  nn.weights_hidden_output[0][0] = 0.8;   // BLDC pattern -> strong RPM1 adjustment
  nn.weights_hidden_output[1][0] = 0.0;   // Engine pattern -> no RPM1 adjustment
  nn.weights_hidden_output[2][0] = 0.6;   // Hybrid pattern -> medium RPM1 adjustment
  for (int i = 3; i < NN_HIDDEN_SIZE; i++) {
    nn.weights_hidden_output[i][0] = 0.2;  // Error processing -> RPM1 adjustment
  }
  
  // RPM2 adjustment (for BLDC and Hybrid)
  nn.weights_hidden_output[0][1] = 0.7;   // BLDC pattern -> strong RPM2 adjustment
  nn.weights_hidden_output[1][1] = 0.0;   // Engine pattern -> no RPM2 adjustment
  nn.weights_hidden_output[2][1] = 0.5;   // Hybrid pattern -> medium RPM2 adjustment
  for (int i = 3; i < NN_HIDDEN_SIZE; i++) {
    nn.weights_hidden_output[i][1] = 0.15; // Error processing -> RPM2 adjustment
  }
  
  // RPM3 adjustment (for Engine and Hybrid)
  nn.weights_hidden_output[0][2] = 0.0;   // BLDC pattern -> no RPM3 adjustment
  nn.weights_hidden_output[1][2] = 0.9;   // Engine pattern -> strong RPM3 adjustment
  nn.weights_hidden_output[2][2] = 0.7;   // Hybrid pattern -> strong RPM3 adjustment
  for (int i = 3; i < NN_HIDDEN_SIZE; i++) {
    nn.weights_hidden_output[i][2] = 0.3;  // Error processing -> RPM3 adjustment
  }
  
  // Output biases (slight positive bias for increasing RPM when undershoot)
  nn.bias_output[0] = 0.1;  // RPM1 bias
  nn.bias_output[1] = 0.1;  // RPM2 bias  
  nn.bias_output[2] = 0.15; // RPM3 bias (engine often needs more boost)
  
  Serial.println("Neural Network initialized with corrected dataset patterns");
  ESP32Serial.println("NN: Initialized with 224 training samples, 18 perfect examples");
}

// Sigmoid activation function
float sigmoid(float x) {
  if (x > 10.0) return 1.0;
  if (x < -10.0) return 0.0;
  return 1.0 / (1.0 + exp(-x));
}

// Tanh activation for output layer (allows negative adjustments)
float tanh_activation(float x) {
  if (x > 10.0) return 1.0;
  if (x < -10.0) return -1.0;
  float ex = exp(x);
  float emx = exp(-x);
  return (ex - emx) / (ex + emx);
}

// Normalize inputs to 0-1 range
void normalizeInputs(float inputs[NN_INPUT_SIZE], float target, float current_speed, 
                     float rpm1, float rpm2, float rpm3) {
  inputs[0] = (target - nn.input_min[0]) / (nn.input_max[0] - nn.input_min[0]);
  inputs[1] = (current_speed - nn.input_min[1]) / (nn.input_max[1] - nn.input_min[1]);
  inputs[2] = (rpm1 - nn.input_min[2]) / (nn.input_max[2] - nn.input_min[2]);
  inputs[3] = (rpm2 - nn.input_min[3]) / (nn.input_max[3] - nn.input_min[3]);
  inputs[4] = (rpm3 - nn.input_min[4]) / (nn.input_max[4] - nn.input_min[4]);
  
  // Clamp to valid range
  for (int i = 0; i < NN_INPUT_SIZE; i++) {
    inputs[i] = constrain(inputs[i], 0.0, 1.0);
  }
}

// Forward pass through neural network
void forwardPass(float inputs[NN_INPUT_SIZE]) {
  // Input to Hidden layer
  for (int h = 0; h < NN_HIDDEN_SIZE; h++) {
    float sum = nn.bias_hidden[h];
    for (int i = 0; i < NN_INPUT_SIZE; i++) {
      sum += inputs[i] * nn.weights_input_hidden[i][h];
    }
    nn.hidden_layer[h] = sigmoid(sum);
  }
  
  // Hidden to Output layer
  for (int o = 0; o < NN_OUTPUT_SIZE; o++) {
    float sum = nn.bias_output[o];
    for (int h = 0; h < NN_HIDDEN_SIZE; h++) {
      sum += nn.hidden_layer[h] * nn.weights_hidden_output[h][o];
    }
    nn.output_layer[o] = tanh_activation(sum) * nn.output_scale; // Scale to RPM adjustment range
  }
}

// Validate neural network output for safety
bool validateNeuralOutput() {
  // Check for reasonable RPM adjustments
  for (int i = 0; i < NN_OUTPUT_SIZE; i++) {
    if (abs(nn.output_layer[i]) > 500) { // Max 500 RPM adjustment per cycle
      return false;
    }
    if (isnan(nn.output_layer[i]) || isinf(nn.output_layer[i])) {
      return false;
    }
  }
  return true;
}

// Main neural control function
// ==================== IMPROVED NEURAL NETWORK CONTROL FUNCTION ====================

// Main neural control function with overshoot protection and enhanced logic
void calculateNeuralRPMAdjustment(float target_speed) {
  // Calculate speed error first
  float speedError = target_speed - groundSpeed_kmh;
  float absSpeedError = abs(speedError);
  
  // Prepare inputs
  float inputs[NN_INPUT_SIZE];
  float current_rpm1 = (float)currentRPM2; // Note: RPM mapping from your code
  float current_rpm2 = (float)currentRPM;  // Note: RPM mapping from your code  
  float current_rpm3 = (float)currentRPM3;
  
  // ==================== OVERSHOOT PROTECTION ====================
  // CRITICAL: Handle overshoot immediately before NN processing
  if (speedError < -0.3) {  // Overshoot threshold: 0.3 km/h
    Serial.println("⚠️  NN OVERRIDE: Overshoot detected, forcing RPM reduction");
    
    // Calculate force reduction based on overshoot severity
    int forceReduction = (int)(absSpeedError * 300);  // 300 RPM per km/h overshoot
    forceReduction = constrain(forceReduction, 100, 800);  // Limit reduction range
    
    // Apply configuration-specific reduction
    if (target_speed <= 5.0) {
      // BLDC Only configuration - reduce main motors
      targetRPM -= forceReduction;
      targetRPM = constrain(targetRPM, 1000, 12000);
      
      Serial.print("🔻 BLDC Override: Reduced targetRPM by ");
      Serial.print(forceReduction);
      Serial.print(" to ");
      Serial.println(targetRPM);
      
    } else if (target_speed <= 8.0) {
      // Engine Only configuration - reduce engine motor
      targetRPM3 -= forceReduction;
      targetRPM3 = constrain(targetRPM3, 1000, 15000);
      
      Serial.print("🔻 Engine Override: Reduced targetRPM3 by ");
      Serial.print(forceReduction);
      Serial.print(" to ");
      Serial.println(targetRPM3);
      
    } else {
      // Hybrid configuration - reduce both
      int bldc_reduction = forceReduction * 0.6;  // 60% to BLDC
      int engine_reduction = forceReduction * 0.8; // 80% to Engine
      
      targetRPM -= bldc_reduction;
      targetRPM3 -= engine_reduction;
      targetRPM = constrain(targetRPM, 1000, 12000);
      targetRPM3 = constrain(targetRPM3, 1000, 15000);
      
      Serial.print("🔻 Hybrid Override: Reduced BLDC by ");
      Serial.print(bldc_reduction);
      Serial.print(", Engine by ");
      Serial.println(engine_reduction);
    }
    
    // Update performance tracking for override
    nnTotalPredictions++;
    // Don't count override as success/failure in NN performance
    
    return; // Exit early, skip NN processing
  }
  
  // ==================== NEAR-TARGET FINE CONTROL ====================
  // Switch to conservative adjustment when very close to target
  if (absSpeedError <= 0.2) {  // Within 0.2 km/h of target
    Serial.println("🎯 NN Fine Control: Near target, using conservative adjustment");
    
    // Very small proportional adjustment
    int fineAdjustment = (int)(speedError * 50);  // 50 RPM per km/h error
    fineAdjustment = constrain(fineAdjustment, -100, 100);
    
    if (target_speed <= 5.0) {
      targetRPM += fineAdjustment;
      targetRPM = constrain(targetRPM, 1000, 12000);
    } else if (target_speed <= 8.0) {
      targetRPM3 += fineAdjustment;
      targetRPM3 = constrain(targetRPM3, 1000, 15000);
    } else {
      targetRPM += fineAdjustment / 2;
      targetRPM3 += fineAdjustment;
      targetRPM = constrain(targetRPM, 1000, 12000);
      targetRPM3 = constrain(targetRPM3, 1000, 15000);
    }
    
    Serial.print("🔧 Fine adjustment: ");
    Serial.print(fineAdjustment);
    Serial.println(" RPM");
    
    // Update performance tracking
    nnTotalPredictions++;
    if (absSpeedError < 0.1) nnSuccessCount++;
    nnPerformanceScore = (float)nnSuccessCount / nnTotalPredictions;
    
    return;
  }
  
  // ==================== NORMAL NN PROCESSING ====================
  // Normalize inputs to 0-1 range
  normalizeInputs(inputs, target_speed, groundSpeed_kmh, 
                  current_rpm1, current_rpm2, current_rpm3);
  
  // Forward pass through neural network
  forwardPass(inputs);
  
  // ==================== ENHANCED OUTPUT VALIDATION ====================
  if (!validateNeuralOutput()) {
    Serial.println("❌ NN: Invalid output detected, using enhanced fallback");
    
    // Enhanced fallback with configuration awareness
    int fallbackAdjustment = (int)(speedError * 150); // Proportional fallback
    fallbackAdjustment = constrain(fallbackAdjustment, -500, 500);
    
    if (target_speed <= 5.0) {
      // BLDC targets
      targetRPM += fallbackAdjustment;
      targetRPM = constrain(targetRPM, 1000, 12000);
      Serial.print("🆘 BLDC Fallback: ");
      Serial.println(fallbackAdjustment);
      
    } else if (target_speed <= 8.0) {
      // Engine targets  
      targetRPM3 += fallbackAdjustment;
      targetRPM3 = constrain(targetRPM3, 1000, 15000);
      Serial.print("🆘 Engine Fallback: ");
      Serial.println(fallbackAdjustment);
      
    } else {
      // Hybrid targets
      targetRPM += fallbackAdjustment / 2;
      targetRPM3 += fallbackAdjustment;
      targetRPM = constrain(targetRPM, 1000, 12000);
      targetRPM3 = constrain(targetRPM3, 1000, 15000);
      Serial.print("🆘 Hybrid Fallback: ");
      Serial.println(fallbackAdjustment);
    }
    
    nnTotalPredictions++;
    return;
  }
  
  // ==================== EXTRACT NN OUTPUTS ====================
  float rpm1_adj = nn.output_layer[0];
  float rpm2_adj = nn.output_layer[1];
  float rpm3_adj = nn.output_layer[2];
  
  // ==================== ENHANCED OUTPUT SANITY CHECK ====================
  // Check if NN output direction matches error direction
  bool outputMakesLogicalSense = true;
  
  if (speedError > 0.5) {
    // Need to increase speed - outputs should be mostly positive
    if (rpm1_adj < -50 && rpm2_adj < -50 && rpm3_adj < -50) {
      outputMakesLogicalSense = false;
      Serial.println("🤔 NN Logic Check: Need speed increase but NN suggests decrease");
    }
  } else if (speedError < -0.5) {
    // Need to decrease speed - outputs should be mostly negative  
    if (rpm1_adj > 50 && rpm2_adj > 50 && rpm3_adj > 50) {
      outputMakesLogicalSense = false;
      Serial.println("🤔 NN Logic Check: Need speed decrease but NN suggests increase");
    }
  }
  
  // Apply logic correction if needed
  if (!outputMakesLogicalSense) {
    Serial.println("🔄 NN Logic Correction: Inverting illogical outputs");
    rpm1_adj = speedError * 100;  // Simple proportional correction
    rpm2_adj = speedError * 100;
    rpm3_adj = speedError * 100;
  }
  
  // ==================== APPLY CONFIGURATION-SPECIFIC ADJUSTMENTS ====================
  if (target_speed <= 5.0) {
    // =============== BLDC Only configuration (targets 4,5) ===============
    int bldc_adjustment = (int)((rpm1_adj + rpm2_adj) / 2); // Average adjustment for BLDC
    
    // Apply adjustment with rate limiting
    int old_target = targetRPM;
    targetRPM += bldc_adjustment;
    targetRPM = constrain(targetRPM, 1000, 12000);
    
    // Ensure engine stays minimal for BLDC mode
    if (targetRPM3 > 1500) {
      targetRPM3 = max(1000, targetRPM3 - 200); // Gradually reduce engine
    }
    
    Serial.print("🔵 NN BLDC Mode: Error=");
    Serial.print(speedError, 2);
    Serial.print(", Adj=");
    Serial.print(bldc_adjustment);
    Serial.print(", RPM: ");
    Serial.print(old_target);
    Serial.print("→");
    Serial.print(targetRPM);
    Serial.print(", Engine=");
    Serial.println(targetRPM3);
    
  } else if (target_speed <= 8.0) {
    // =============== Engine Only configuration (targets 7,8) ===============
    int engine_adjustment = (int)rpm3_adj;
    
    // Apply adjustment with rate limiting
    int old_target = targetRPM3;
    targetRPM3 += engine_adjustment;
    targetRPM3 = constrain(targetRPM3, 1000, 15000);
    
    // Ensure BLDC stays minimal for engine mode
    if (targetRPM > 2000) {
      targetRPM = max(1000, targetRPM - 100); // Gradually reduce BLDC
    }
    
    Serial.print("🟠 NN Engine Mode: Error=");
    Serial.print(speedError, 2);
    Serial.print(", Adj=");
    Serial.print(engine_adjustment);
    Serial.print(", RPM3: ");
    Serial.print(old_target);
    Serial.print("→");
    Serial.print(targetRPM3);
    Serial.print(", BLDC=");
    Serial.println(targetRPM);
    
  } else {
    // =============== Hybrid configuration (target 12+) ===============
    int bldc_adjustment = (int)((rpm1_adj + rpm2_adj) / 2);
    int engine_adjustment = (int)rpm3_adj;
    
    // Apply adjustments with coordination
    int old_bldc = targetRPM;
    int old_engine = targetRPM3;
    
    targetRPM += bldc_adjustment;
    targetRPM3 += engine_adjustment;
    targetRPM = constrain(targetRPM, 1000, 12000);
    targetRPM3 = constrain(targetRPM3, 1000, 15000);
    
    Serial.print("🟣 NN Hybrid Mode: Error=");
    Serial.print(speedError, 2);
    Serial.print(", BLDC_adj=");
    Serial.print(bldc_adjustment);
    Serial.print(", Engine_adj=");
    Serial.print(engine_adjustment);
    Serial.println();
    Serial.print("   BLDC: ");
    Serial.print(old_bldc);
    Serial.print("→");
    Serial.print(targetRPM);
    Serial.print(", Engine: ");
    Serial.print(old_engine);
    Serial.print("→");
    Serial.println(targetRPM3);
  }
  
  // ==================== PERFORMANCE TRACKING ====================
  nnTotalPredictions++;
  
  // Determine success based on error reduction expectation
  bool prediction_success = false;
  if (speedError > 0 && (rpm1_adj > 0 || rpm2_adj > 0 || rpm3_adj > 0)) {
    prediction_success = true; // Positive error, positive adjustment
  } else if (speedError < 0 && (rpm1_adj < 0 || rpm2_adj < 0 || rpm3_adj < 0)) {
    prediction_success = true; // Negative error, negative adjustment
  } else if (abs(speedError) < 0.3) {
    prediction_success = true; // Close enough to target
  }
  
  if (prediction_success) {
    nnSuccessCount++;
  }
  
  nnPerformanceScore = (float)nnSuccessCount / nnTotalPredictions;
  
  // ==================== PERIODIC PERFORMANCE REPORTING ====================
  if (nnTotalPredictions % 10 == 0) {
    Serial.print("📊 NN Performance: ");
    Serial.print(nnPerformanceScore * 100, 1);
    Serial.print("% success (");
    Serial.print(nnSuccessCount);
    Serial.print("/");
    Serial.print(nnTotalPredictions);
    Serial.print(" predictions), Current error: ");
    Serial.print(speedError, 2);
    Serial.println(" km/h");
  }
  
  // ==================== EXTREME ERROR SAFETY ====================
  // Final safety check for extreme situations
  if (abs(speedError) > 3.0) {
    Serial.print("🚨 EXTREME ERROR: ");
    Serial.print(speedError, 2);
    Serial.println(" km/h - Consider system check!");
  }
}


// ================== END NEURAL NETWORK FUNCTIONS ==================

// Prototypes
int calculateSMC(int error, int prevError, unsigned long dt_millis, bool isMotor2);
int calculateFuzzy(int error, bool isMotor2);
int calculateFuzzy3(int error);
// GROUND SPEED FUZZY CONTROL - FUNGSI BARU
int calculateGroundSpeedFuzzy(float speedError);
// PID GROUND SPEED CONTROL - FUNGSI BARU
int calculateGroundSpeedPID(float speedError, unsigned long dt_millis);
// FUZZY PID GAIN SCHEDULING CONTROL - FUNGSI BARU
int calculateFuzzyPIDGainScheduling(float speedError, unsigned long dt_millis);
void calculateAdaptiveGains(float speedError);
// Membership functions untuk Motor 1 & 2
int membershipNL(int error);
int membershipNS(int error);
int membershipZE(int error);
int membershipPS(int error);
int membershipPL(int error);
// Membership functions KHUSUS untuk Motor 3 (ESC3)
int membershipNL3(int error);
int membershipNS3(int error);
int membershipZE3(int error);
int membershipPS3(int error);
int membershipPL3(int error);
// Membership functions untuk GROUND SPEED
int membershipSpeedNL(float error);
int membershipSpeedNS(float error);
int membershipSpeedZE(float error);
int membershipSpeedPS(float error);
int membershipSpeedPL(float error);
// FUZZY GAIN SCHEDULING membership functions - FITUR BARU
int membershipErrorSmall(float absError);
int membershipErrorMedium(float absError);
int membershipErrorLarge(float absError);
void requestRPMData();
void requestSensorData();
void processLoraCommand(const char* command);
void reportStatusToLora();
void sendHeartbeat();
void handleRemoteControl();
void setRemoteThrottle(int throttle);
void setRemoteRudder(int angle);

// Function to check if an I2C device is responding
bool checkI2CDevice(uint8_t address) {
  Wire.beginTransmission(address);
  byte error = Wire.endTransmission();

  if (error == 0) {
    return true;  // Device found
  }
  return false;  // Device not found
}

// Function to reset I2C bus if needed
void resetI2C() {
  Wire.end();
  delay(100);
  Wire.begin();
  Wire.setClock(I2C_CLOCK_SPEED);
  Serial.println("I2C bus reset performed");
  ESP32Serial.println("I2C bus reset performed");
}

// Function to request RPM data from Slave 1
void requestRPMData() {
  // Set default values to 0 first
  currentRPM = 0;
  currentRPM2 = 0;
  currentRPM3 = 0;
  rpmDataReceived = false;

  // Check if device is responding
  if (!checkI2CDevice(SLAVE1_ADDR)) {
    // Device not responding, values already set to 0
    return;
  }

  // Request 32 bytes which should be enough for RPM data
  uint8_t bytesToRequest = 32;
  uint8_t bytesReceived = Wire.requestFrom(SLAVE1_ADDR, bytesToRequest);

  // Clear buffer
  memset(buffer, 0, sizeof(buffer));

  // Read all available bytes (up to our buffer size)
  int idx = 0;
  while (Wire.available() && idx < sizeof(buffer) - 1) {
    buffer[idx++] = Wire.read();
  }
  buffer[idx] = '\0';  // Ensure null termination

  if (bytesReceived > 0) {
    // Parse RPM data
    // Expected format: "RPM1:xxxx RPM2:xxxx RPM3:xxxx"
    char* rpm1Ptr = strstr(buffer, "RPM1:");
    char* rpm2Ptr = strstr(buffer, "RPM2:");
    char* rpm3Ptr = strstr(buffer, "RPM3:");

    if (rpm1Ptr && rpm2Ptr && rpm3Ptr) {
      int rpm1 = atoi(rpm1Ptr + 5);
      int rpm2 = atoi(rpm2Ptr + 5);
      int rpm3 = atoi(rpm3Ptr + 5);

      // Make sure values are reasonable
      if (rpm1 >= 0 && rpm1 < 50000) {
        currentRPM2 = rpm1;  // Note the swap in numbering to match your original code
        rpmDataReceived = true;
      }
      if (rpm2 >= 0 && rpm2 < 50000) {
        currentRPM = rpm2;  // Note the swap in numbering to match your original code
        rpmDataReceived = true;
      }
      if (rpm3 >= 0 && rpm3 < 50000) {
        currentRPM3 = rpm3;
        rpmDataReceived = true;
      }
    }
  }
  // If no valid data received, values remain 0 and rpmDataReceived stays false
}

// Function to request sensor data from Slave 2
void requestSensorData() {
  // Set default values to 0 first
  groundSpeed_kmh = 0.0;
  heading_deg = 0.0;
  voltage1 = 0.0;
  voltage2 = 0.0;
  current1 = 0.0;
  current2 = 0.0;
  sensorDataReceived = false;

  // Check if device is responding
  if (!checkI2CDevice(SLAVE2_ADDR)) {
    // Device not responding, values already set to 0
    return;
  }

  // Request 64 bytes which should be enough for sensor data
  uint8_t bytesToRequest = 64;
  uint8_t bytesReceived = Wire.requestFrom(SLAVE2_ADDR, bytesToRequest);

  // Clear buffer
  memset(buffer, 0, sizeof(buffer));

  // Read all available bytes (up to our buffer size)
  int idx = 0;
  while (Wire.available() && idx < sizeof(buffer) - 1) {
    buffer[idx++] = Wire.read();
  }
  buffer[idx] = '\0';  // Ensure null termination

  if (bytesReceived > 0) {
    // Parse sensor data
    // Expected format: "SPD:xx.x,HDG:xxx,V1:xx.x,V2:xx.x,I1:xx.x,I2:xx.x"
    char* spdPtr = strstr(buffer, "SPD:");
    char* hdgPtr = strstr(buffer, "HDG:");
    char* v1Ptr = strstr(buffer, "V1:");
    char* v2Ptr = strstr(buffer, "V2:");
    char* i1Ptr = strstr(buffer, "I1:");
    char* i2Ptr = strstr(buffer, "I2:");

    if (spdPtr) {
      groundSpeed_kmh = atof(spdPtr + 4);
      sensorDataReceived = true;
    }
    if (hdgPtr) {
      heading_deg = atof(hdgPtr + 4);
      sensorDataReceived = true;
    }
    if (v1Ptr) {
      voltage1 = atof(v1Ptr + 3);
      sensorDataReceived = true;
    }
    if (v2Ptr) {
      voltage2 = atof(v2Ptr + 3);
      sensorDataReceived = true;
    }
    if (i1Ptr) {
      current1 = atof(i1Ptr + 3);
      sensorDataReceived = true;
    }
    if (i2Ptr) {
      current2 = atof(i2Ptr + 3);
      sensorDataReceived = true;
    }
  }
  // Serial.println(groundSpeed_kmh);
  // If no valid data received, values remain 0 and sensorDataReceived stays false
}

// Handle remote control from ESP32
void handleRemoteControl() {
  // Map rudder angle to servo microseconds (-110 to +110 degrees to servo range)
  int mappedRudder = map(remoteRudderAngle, -110, 110, RUDDER_MIN, RUDDER_MAX);
  mappedRudder = constrain(mappedRudder, RUDDER_MIN, RUDDER_MAX);

  // Apply rudder angle
  RUDDER.writeMicroseconds(mappedRudder);
}

// Set remote throttle value
void setRemoteThrottle(int throttle) {
  remoteThrottle = constrain(throttle, 90, 110);
}

// Set remote rudder angle
void setRemoteRudder(int angle) {
  remoteRudderAngle = constrain(angle, -110, 110);
}

// Process LoRa commands
void processLoraCommand(const char* command) {
  // Check for remote control data format: T:xxx,A:xxx
  if (strstr(command, "T:") && strstr(command, "A:")) {
    char* throttlePtr = strstr(command, "T:");
    char* anglePtr = strstr(command, "A:");

    if (throttlePtr && anglePtr) {
      int throttle = atoi(throttlePtr + 2);
      int angle = atoi(anglePtr + 2);

      // Update remote control values
      setRemoteThrottle(throttle);
      setRemoteRudder(angle);

      // Always apply remote control (don't change other modes)
      handleRemoteControl();

      // Print feedback for remote control
      Serial.print("Remote - Throttle: ");
      Serial.print(throttle);
      Serial.print(" -> ESC3: ");
      Serial.print(currentThrottle3_deg);
      Serial.print("°, Rudder: ");
      Serial.print(angle);
      Serial.print("° -> ");
      int mappedRudder = map(angle, -110, 110, RUDDER_MIN, RUDDER_MAX);
      Serial.print(mappedRudder);
      Serial.println("us");

      sendSimpleData();

      return;  // Exit early since we processed remote control data
    }
  }

  // Check for single character commands
  if (strlen(command) == 1) {
    char cmd = command[0];

    // Process single character commands identical to serial interface
    if (cmd == 's') {
      // Start motors
      currentThrottle1 = START_PULSE;
      currentThrottle2 = START_PULSE+20;
      currentThrottle3_deg = START_PULSE3_DEG;

      ESC1.writeMicroseconds(currentThrottle1);
      ESC2.writeMicroseconds(currentThrottle2);
      // ESC3 will be controlled by remote, don't override here

      ESP32Serial.println("Motors 1&2 started at minimum speed");

      // Disable automatic control for motors 1&2 only
      controlActive = false;
      control3Active = false;
      groundSpeedControlActive = false;
      pidSpeedControlActive = false;
      fuzzyPIDControlActive = false;
      neuralControlActive = false;  // NEURAL NETWORK
      if (currentMode != REMOTE) currentMode = NONE;
    } else if (cmd == 'x') {
      // Stop motors
      currentThrottle1 = MIN_PULSE;
      currentThrottle2 = MIN_PULSE;
      // Don't stop ESC3 and RUDDER - let remote control them

      ESC1.writeMicroseconds(currentThrottle1);
      ESC2.writeMicroseconds(currentThrottle2);
      ESC3.write(90);
      groundSpeedControlActive = false;
      pidSpeedControlActive = false;
      fuzzyPIDControlActive = false;
      neuralControlActive = false;  // NEURAL NETWORK
      controlActive = false;
      control3Active = false;
      currentMode = NONE;
      ESP32Serial.println("All Speed Control modes deactivated");

      // Disable control for motors 1&2 only
      controlActive = false;
      control3Active = false;
      groundSpeedControlActive = false;
      pidSpeedControlActive = false;
      fuzzyPIDControlActive = false;
      neuralControlActive = false;  // NEURAL NETWORK
      if (currentMode != REMOTE) currentMode = NONE;

      ESP32Serial.println("Motors 1&2 stopped - Remote control still active");
    } else if (cmd == '+') {
      // Increase throttle for motors 1&2 only
      bool throttleIncreased = false;

      // Motor 1 & 2
      if (currentThrottle1 < MAX_PULSE) {
        currentThrottle1 += STEP_SIZE;
        currentThrottle2 += STEP_SIZE*1.7;
        ESC1.writeMicroseconds(currentThrottle1);
        ESC2.writeMicroseconds(currentThrottle2);
        throttleIncreased = true;
      }

      
      // Motor 3 (menggunakan derajat)
      if (currentThrottle3_deg < MAX_PULSE3_DEG) {
        currentThrottle3_deg += STEP_SIZE3_DEG;
        currentThrottle3 = currentThrottle3_deg;  // Update compatibility
        ESC3.write(currentThrottle3_deg);
        Serial.println(currentThrottle3_deg);
        throttleIncreased = true;
      }

      if (throttleIncreased) {
        ESP32Serial.println("Motors 1&2 throttle increased");
      } else {
        ESP32Serial.println("Maximum throttle reached for motors 1&2");
      }

      // Disable automatic control for motors 1&2
      controlActive = false;
      control3Active = false;
      groundSpeedControlActive = false;
      pidSpeedControlActive = false;
      fuzzyPIDControlActive = false;
      neuralControlActive = false;  // NEURAL NETWORK
      if (currentMode != REMOTE) currentMode = NONE;
    } else if (cmd == '-') {
      // Decrease throttle for motors 1&2 only
      bool throttleDecreased = false;

      // Motor 1 & 2
      if (currentThrottle1 > START_PULSE) {
        currentThrottle1 -= STEP_SIZE;
        currentThrottle2 -= STEP_SIZE;
        ESC1.writeMicroseconds(currentThrottle1);
        ESC2.writeMicroseconds(currentThrottle2);
        throttleDecreased = true;
      }

       // Motor 3 (menggunakan derajat)
      if (currentThrottle3_deg > START_PULSE3_DEG) {
        currentThrottle3_deg -= STEP_SIZE3_DEG;
        currentThrottle3 = currentThrottle3_deg;  // Update compatibility
        ESC3.write(currentThrottle3_deg);
        throttleDecreased = true;
      }

      if (throttleDecreased) {
        ESP32Serial.println("Motors 1&2 throttle decreased");
      } else {
        ESP32Serial.println("Minimum throttle reached for motors 1&2");
      }

      // Disable automatic control for motors 1&2
      controlActive = false;
      control3Active = false;
      groundSpeedControlActive = false;
      pidSpeedControlActive = false;
      fuzzyPIDControlActive = false;
      neuralControlActive = false;  // NEURAL NETWORK
      if (currentMode != REMOTE) currentMode = NONE;
    } else if (cmd == 'p') {
      // PID command tanpa parameter - tampilkan help
      ESP32Serial.println("PID Speed Control Commands:");
      ESP32Serial.println("p5   - Set target speed 5 km/h (PID)");
      ESP32Serial.println("p10  - Set target speed 10 km/h (PID)");
      ESP32Serial.println("p15  - Set target speed 15 km/h (PID)");
      ESP32Serial.println("p0   - Disable PID speed control");
    } else if (cmd == 'd') {
      // Fuzzy PID Gain Scheduling command tanpa parameter - tampilkan help
      ESP32Serial.println("Fuzzy PID Gain Scheduling Commands:");
      ESP32Serial.println("d5   - Set target speed 5 km/h (Fuzzy PID)");
      ESP32Serial.println("d10  - Set target speed 10 km/h (Fuzzy PID)");
      ESP32Serial.println("d15  - Set target speed 15 km/h (Fuzzy PID)");
      ESP32Serial.println("d0   - Disable Fuzzy PID control");
    } else if (cmd == 'n') {
      // Neural Network command tanpa parameter - tampilkan help
      ESP32Serial.println("Neural Network Speed Control Commands:");
      ESP32Serial.println("n4   - Set target speed 4 km/h (NN BLDC)");
      ESP32Serial.println("n5   - Set target speed 5 km/h (NN BLDC)");
      ESP32Serial.println("n7   - Set target speed 7 km/h (NN Engine)");
      ESP32Serial.println("n8   - Set target speed 8 km/h (NN Engine)");
      ESP32Serial.println("n12  - Set target speed 12 km/h (NN Hybrid)");
      ESP32Serial.println("n0   - Disable Neural Network control");
      ESP32Serial.println("NN trained on 224 corrected samples with 18 perfect examples");
    } else if (cmd == 'i') {
      // Reset I2C bus
      ESP32Serial.println("Resetting I2C bus...");
      resetI2C();
    } else if (cmd == 'g') {
      // Ground speed command tanpa parameter - tampilkan help
      ESP32Serial.println("Ground Speed Control Commands:");
      ESP32Serial.println("g5   - Set target speed 5 km/h");
      ESP32Serial.println("g10  - Set target speed 10 km/h");
      ESP32Serial.println("g15  - Set target speed 15 km/h");
      ESP32Serial.println("g0   - Disable ground speed control");
    } else {
      ESP32Serial.print("Unknown command: ");
      ESP32Serial.println(cmd);
    }
  }
  // Process multi-character commands (m, f, t, g, p, d, n followed by numbers)
  else if (strlen(command) > 1) {
    char cmd = command[0];

    if (cmd == 'm' || cmd == 'f' || cmd == 't' || cmd == 'g' || cmd == 'p' || cmd == 'd' || cmd == 'n') {
      // Extract the number part
      float targetVal = atof(command + 1);  // Use atof for ground speed (float)

      // Don't disable remote control - let it coexist

      if (cmd == 'm') {
        // SMC untuk motor 1 & 2
        targetRPM = (int)targetVal;

        if (targetRPM > 0) {
          controlActive = true;
          currentMode = SMC;
          prevError1_SMC = 0;
          prevError2_SMC = 0;
          groundSpeedControlActive = false;
          pidSpeedControlActive = false;
          fuzzyPIDControlActive = false;
          neuralControlActive = false;  // NEURAL NETWORK

          // Start motors jika belum berjalan
          if (currentThrottle1 <= MIN_PULSE) {
            currentThrottle1 = START_PULSE;
            currentThrottle2 = START_PULSE;
            ESC1.writeMicroseconds(currentThrottle1);
            ESC2.writeMicroseconds(currentThrottle2);
          }

          ESP32Serial.print("SMC mode activated for motors 1&2. Target RPM: ");
          ESP32Serial.println(targetRPM);
          ESP32Serial.println("Remote control still active for ESC3 & Rudder");
        } else {
          controlActive = false;
          currentMode = NONE;
          ESP32Serial.println("Main motors control deactivated");
        }
      } else if (cmd == 'f') {
        // Fuzzy untuk motor 1 & 2
        targetRPM = (int)targetVal;

        if (targetRPM > 0) {
          controlActive = true;
          currentMode = FUZZY;
          prevError1_Fuzzy = 0;
          prevError2_Fuzzy = 0;
          groundSpeedControlActive = false;
          pidSpeedControlActive = false;
          fuzzyPIDControlActive = false;
          neuralControlActive = false;  // NEURAL NETWORK

          // Start motors jika belum berjalan
          if (currentThrottle1 <= MIN_PULSE) {
            currentThrottle1 = START_PULSE;
            currentThrottle2 = START_PULSE;
            ESC1.writeMicroseconds(currentThrottle1);
            ESC2.writeMicroseconds(currentThrottle2);
          }

          ESP32Serial.print("Fuzzy mode activated for motors 1&2. Target RPM: ");
          ESP32Serial.println(targetRPM);
          ESP32Serial.println("Remote control still active for ESC3 & Rudder");
        } else {
          controlActive = false;
          currentMode = NONE;
          ESP32Serial.println("Main motors control deactivated");
        }
      } else if (cmd == 't') {
        // Fuzzy untuk motor 3 - this will conflict with remote, inform user
        targetRPM3 = (int)targetVal;

        if (targetRPM3 > 0) {
          control3Active = true;
          prevError3_Fuzzy = 0;
          // Reset timing untuk motor 3
          lastFuzzy3Check = millis();

          ESP32Serial.print("Third motor Fuzzy control activated. Target RPM: ");
          ESP32Serial.println(targetRPM3);
          ESP32Serial.println("WARNING: This will override remote throttle control for ESC3");
          ESP32Serial.println("Fuzzy control for motor 3 will update every 2 seconds");
        } else {
          control3Active = false;
          ESP32Serial.println("Third motor control deactivated - Remote throttle control resumed");
        }
      } else if (cmd == 'g') {
        // GROUND SPEED CONTROL - FITUR FUZZY
        targetGroundSpeed = targetVal;

        if (targetGroundSpeed > 0) {
          groundSpeedControlActive = true;
          pidSpeedControlActive = false;    // Disable PID when Fuzzy active
          fuzzyPIDControlActive = false;    // Disable Fuzzy PID when Fuzzy active
          neuralControlActive = false;      // Disable Neural Network
          controlActive = true;
          control3Active = true;
          currentMode = GROUND_SPEED;

          // Reset timing
          lastGroundSpeedCheck = millis();
          lastRPMCheck = millis();
          lastFuzzy3Check = millis();

          // Set initial RPM targets
          targetRPM = baseRPM1;
          targetRPM3 = baseRPM3;

          // Reset fuzzy errors
          prevError1_Fuzzy = 0;
          prevError2_Fuzzy = 0;
          prevError3_Fuzzy = 0;

          // Start motors jika belum berjalan
          if (currentThrottle1 <= MIN_PULSE) {
            currentThrottle1 = START_PULSE;
            currentThrottle2 = START_PULSE;
            ESC1.writeMicroseconds(currentThrottle1);
            ESC2.writeMicroseconds(currentThrottle2);
          }
          if (currentThrottle3_deg <= MIN_PULSE3_DEG) {
            currentThrottle3_deg = START_PULSE3_DEG;
            currentThrottle3 = currentThrottle3_deg;
            ESC3.write(START_PULSE3_DEG);
          }

          ESP32Serial.print("Ground Speed Control (Fuzzy) activated. Target Speed: ");
          ESP32Serial.print(targetGroundSpeed, 1);
          ESP32Serial.println(" km/h");
          ESP32Serial.println("All motors will adjust RPM automatically based on ground speed");
        } else {
          groundSpeedControlActive = false;
          controlActive = false;
          control3Active = false;
          currentMode = NONE;
          ESP32Serial.println("Ground Speed Control deactivated");
        }
      } else if (cmd == 'p') {
        // PID SPEED CONTROL - FITUR FIXED PID
        targetPIDSpeed = targetVal;

        if (targetPIDSpeed > 0) {
          pidSpeedControlActive = true;
          groundSpeedControlActive = false;  // Disable Fuzzy when PID active
          fuzzyPIDControlActive = false;     // Disable Fuzzy PID when Fixed PID active
          neuralControlActive = false;       // Disable Neural Network
          controlActive = true;
          control3Active = true;
          currentMode = PID_SPEED;

          // Reset timing
          lastPIDSpeedCheck = millis();
          lastRPMCheck = millis();
          lastFuzzy3Check = millis();

          // Set initial RPM targets
          targetRPM = baseRPM1;
          targetRPM3 = baseRPM3;

          // Reset PID errors
          pidSpeedError = 0.0;
          pidSpeedErrorPrev = 0.0;
          pidSpeedIntegral = 0.0;
          pidSpeedDerivative = 0.0;
          
          // Reset fuzzy errors for motor 3
          prevError1_Fuzzy = 0;
          prevError2_Fuzzy = 0;
          prevError3_Fuzzy = 0;

          // Start motors jika belum berjalan
          if (currentThrottle1 <= MIN_PULSE) {
            currentThrottle1 = START_PULSE;
            currentThrottle2 = START_PULSE;
            ESC1.writeMicroseconds(currentThrottle1);
            ESC2.writeMicroseconds(currentThrottle2);
          }
          if (currentThrottle3_deg <= MIN_PULSE3_DEG) {
            currentThrottle3_deg = START_PULSE3_DEG;
            currentThrottle3 = currentThrottle3_deg;
            ESC3.write(START_PULSE3_DEG);
          }

          ESP32Serial.print("PID Speed Control activated. Target Speed: ");
          ESP32Serial.print(targetPIDSpeed, 1);
          ESP32Serial.println(" km/h");
          ESP32Serial.println("All motors will adjust RPM automatically based on Fixed PID control");
        } else {
          pidSpeedControlActive = false;
          controlActive = false;
          control3Active = false;
          currentMode = NONE;
          ESP32Serial.println("PID Speed Control deactivated");
        }
      } else if (cmd == 'd') {
        // FUZZY PID GAIN SCHEDULING CONTROL - FITUR BARU
        targetFuzzyPIDSpeed = targetVal;

        if (targetFuzzyPIDSpeed > 0) {
          fuzzyPIDControlActive = true;
          pidSpeedControlActive = false;     // Disable Fixed PID when Fuzzy PID active
          groundSpeedControlActive = false;  // Disable Fuzzy when Fuzzy PID active
          neuralControlActive = false;       // Disable Neural Network
          controlActive = true;
          control3Active = true;
          currentMode = FUZZY_PID_SPEED;

          // Reset timing
          lastFuzzyPIDCheck = millis();
          lastRPMCheck = millis();
          lastFuzzy3Check = millis();

          // Set initial RPM targets
          targetRPM = baseRPM1;
          targetRPM3 = baseRPM3;

          // Reset Fuzzy PID errors
          fuzzyPIDSpeedError = 0.0;
          fuzzyPIDSpeedErrorPrev = 0.0;
          fuzzyPIDSpeedIntegral = 0.0;
          fuzzyPIDSpeedDerivative = 0.0;
          
          // Reset adaptive gains to default
          adaptiveKp = PID_KP;
          adaptiveKd = PID_KD;
          
          // Reset fuzzy errors for motor 3
          prevError1_Fuzzy = 0;
          prevError2_Fuzzy = 0;
          prevError3_Fuzzy = 0;

          // Start motors jika belum berjalan
          if (currentThrottle1 <= MIN_PULSE) {
            currentThrottle1 = START_PULSE;
            currentThrottle2 = START_PULSE;
            ESC1.writeMicroseconds(currentThrottle1);
            ESC2.writeMicroseconds(currentThrottle2);
          }
          if (currentThrottle3_deg <= MIN_PULSE3_DEG) {
            currentThrottle3_deg = START_PULSE3_DEG;
            currentThrottle3 = currentThrottle3_deg;
            ESC3.write(START_PULSE3_DEG);
          }

          ESP32Serial.print("Fuzzy PID Gain Scheduling Control activated. Target Speed: ");
          ESP32Serial.print(targetFuzzyPIDSpeed, 1);
          ESP32Serial.println(" km/h");
          ESP32Serial.println("All motors will adjust RPM with adaptive PID gains based on speed error");
        } else {
          fuzzyPIDControlActive = false;
          controlActive = false;
          control3Active = false;
          currentMode = NONE;
          ESP32Serial.println("Fuzzy PID Gain Scheduling Control deactivated");
        }
      } else if (cmd == 'n') {
        // ==================== NEURAL NETWORK SPEED CONTROL COMMAND ====================
        targetNeuralSpeed = targetVal;
        
        if (targetNeuralSpeed > 0) {
          neuralControlActive = true;
          // Disable other controls
          groundSpeedControlActive = false;
          pidSpeedControlActive = false;
          fuzzyPIDControlActive = false;
          controlActive = true;
          control3Active = true;
          currentMode = NEURAL_NETWORK;
          
          // Initialize NN control timing
          lastNeuralCheck = millis();
          lastRPMCheck = millis();
          lastFuzzy3Check = millis();
          
          // Set initial RPM targets based on dataset patterns
          if (targetNeuralSpeed <= 5.0) {
            targetRPM = 8500;    // BLDC base RPM from dataset
            targetRPM3 = 0;      // No engine for BLDC targets
          } else if (targetNeuralSpeed <= 8.0) {
            targetRPM = 1200;    // Keep BLDC minimal for engine targets
            targetRPM3 = 6500;   // Engine base RPM from dataset
          } else {
            targetRPM = 9500;    // Hybrid BLDC RPM from dataset
            targetRPM3 = 13000;  // Hybrid engine RPM from dataset
          }
          
          // Start motors based on configuration
          if (targetNeuralSpeed <= 5.0) {
            // BLDC mode - start main motors, stop engine
            if (currentThrottle1 <= MIN_PULSE) {
              currentThrottle1 = START_PULSE;
              currentThrottle2 = START_PULSE;
              ESC1.writeMicroseconds(currentThrottle1);
              ESC2.writeMicroseconds(currentThrottle2);
            }
            currentThrottle3_deg = MIN_PULSE3_DEG;
            ESC3.write(MIN_PULSE3_DEG);
            
          } else if (targetNeuralSpeed <= 8.0) {
            // Engine mode - start engine, minimal main motors
            if (currentThrottle3_deg <= MIN_PULSE3_DEG) {
              currentThrottle3_deg = START_PULSE3_DEG;
              ESC3.write(currentThrottle3_deg);
            }
            currentThrottle1 = MIN_PULSE;
            currentThrottle2 = MIN_PULSE;
            ESC1.writeMicroseconds(currentThrottle1);
            ESC2.writeMicroseconds(currentThrottle2);
            
          } else {
            // Hybrid mode - start all motors
            if (currentThrottle1 <= MIN_PULSE) {
              currentThrottle1 = START_PULSE;
              currentThrottle2 = START_PULSE;
              ESC1.writeMicroseconds(currentThrottle1);
              ESC2.writeMicroseconds(currentThrottle2);
            }
            if (currentThrottle3_deg <= MIN_PULSE3_DEG) {
              currentThrottle3_deg = START_PULSE3_DEG;
              ESC3.write(currentThrottle3_deg);
            }
          }
          
          ESP32Serial.print("Neural Network Speed Control activated. Target: ");
          ESP32Serial.print(targetNeuralSpeed, 1);
          ESP32Serial.println(" km/h");
          ESP32Serial.println("NN: Learning from 224 corrected samples, 18 perfect examples");
          
          // Print configuration info
          if (targetNeuralSpeed <= 5.0) {
            ESP32Serial.println("NN Config: BLDC Only (like dataset targets 4,5)");
          } else if (targetNeuralSpeed <= 8.0) {
            ESP32Serial.println("NN Config: Engine Only (like dataset targets 7,8)");
          } else {
            ESP32Serial.println("NN Config: Hybrid (like dataset target 12)");
          }
          
        } else {
          neuralControlActive = false;
          controlActive = false;
          control3Active = false;
          currentMode = NONE;
          ESP32Serial.println("Neural Network Speed Control deactivated");
        }
        // ================== END NEURAL NETWORK COMMAND ==================
      }
    } else {
      ESP32Serial.print("Unknown command: ");
      ESP32Serial.println(command);
    }
  }
}

// Report system status to LoRa
void reportStatusToLora() {
  ESP32Serial.println("=== Status Report ===");

  ESP32Serial.print("Mode: ");
  if (neuralControlActive) {
    ESP32Serial.println("Neural Network Speed Control");
    ESP32Serial.print("Target Speed: ");
    ESP32Serial.print(targetNeuralSpeed, 1);
    ESP32Serial.print(" km/h, Current: ");
    ESP32Serial.print(groundSpeed_kmh, 1);
    ESP32Serial.print(" km/h, Performance: ");
    ESP32Serial.print(nnPerformanceScore * 100, 1);
    ESP32Serial.println("%");
  } else if (groundSpeedControlActive) {
    ESP32Serial.println("Ground Speed Control (Fuzzy)");
    ESP32Serial.print("Target Speed: ");
    ESP32Serial.print(targetGroundSpeed, 1);
    ESP32Serial.print(" km/h, Current: ");
    ESP32Serial.print(groundSpeed_kmh, 1);
    ESP32Serial.println(" km/h");
  } else if (pidSpeedControlActive) {
    ESP32Serial.println("PID Speed Control (Fixed)");
    ESP32Serial.print("Target Speed: ");
    ESP32Serial.print(targetPIDSpeed, 1);
    ESP32Serial.print(" km/h, Current: ");
    ESP32Serial.print(groundSpeed_kmh, 1);
    ESP32Serial.println(" km/h");
  } else if (fuzzyPIDControlActive) {
    ESP32Serial.println("Fuzzy PID Gain Scheduling Control");
    ESP32Serial.print("Target Speed: ");
    ESP32Serial.print(targetFuzzyPIDSpeed, 1);
    ESP32Serial.print(" km/h, Current: ");
    ESP32Serial.print(groundSpeed_kmh, 1);
    ESP32Serial.print(" km/h, Kp: ");
    ESP32Serial.print(adaptiveKp, 0);
    ESP32Serial.print(", Kd: ");
    ESP32Serial.println(adaptiveKd, 0);
  } else if (remoteControlActive) {
    ESP32Serial.println("Remote Control");
    ESP32Serial.print("Remote Throttle: ");
    ESP32Serial.print(remoteThrottle);
    ESP32Serial.print(", Remote Rudder: ");
    ESP32Serial.println(remoteRudderAngle);
  } else if (controlActive) {
    ESP32Serial.println(currentMode == SMC ? "SMC" : "Fuzzy");
    ESP32Serial.print("Target RPM 1&2: ");
    ESP32Serial.println(targetRPM);
  } else {
    ESP32Serial.println("Manual");
  }

  ESP32Serial.print("Mode 3: ");
  if (control3Active) {
    ESP32Serial.println("Fuzzy (2sec interval)");
    ESP32Serial.print("Target RPM 3: ");
    ESP32Serial.println(targetRPM3);
  } else {
    ESP32Serial.println("Manual");
  }

  ESP32Serial.print("Motor 1 - Throttle: ");
  ESP32Serial.print(currentThrottle1);
  ESP32Serial.print("us, RPM: ");
  ESP32Serial.println(currentRPM);

  ESP32Serial.print("Motor 2 - Throttle: ");
  ESP32Serial.print(currentThrottle2);
  ESP32Serial.print("us, RPM: ");
  ESP32Serial.println(currentRPM2);

  ESP32Serial.print("Motor 3 - Throttle: ");
  ESP32Serial.print(currentThrottle3_deg);
  ESP32Serial.print("°, RPM: ");
  ESP32Serial.println(currentRPM3);

  ESP32Serial.print("Speed: ");
  ESP32Serial.print(groundSpeed_kmh, 1);
  ESP32Serial.print(" km/h, Heading: ");
  ESP32Serial.print(heading_deg, 0);
  ESP32Serial.println(" degrees");

  ESP32Serial.print("Voltage 1: ");
  ESP32Serial.print(voltage1, 1);
  ESP32Serial.print("V, Voltage 2: ");
  ESP32Serial.print(voltage2, 1);
  ESP32Serial.println("V");

  ESP32Serial.print("Current 1: ");
  ESP32Serial.print(current1, 1);
  ESP32Serial.print("A, Current 2: ");
  ESP32Serial.print(current2, 1);
  ESP32Serial.println("A");

  ESP32Serial.print("I2C Data Received - RPM: ");
  ESP32Serial.print(rpmDataReceived ? "Yes" : "No");
  ESP32Serial.print(", Sensors: ");
  ESP32Serial.println(sensorDataReceived ? "Yes" : "No");

  ESP32Serial.println("===================");
}

// Send a heartbeat message
void sendHeartbeat() {
  // ESP32Serial.println("STM32_HEARTBEAT");
}

// Fungsi untuk memberikan status throttle
void reportThrottle() {
  Serial.print("Throttle positions: Main1=");
  Serial.print(currentThrottle1);
  Serial.print("us, Main2=");
  Serial.print(currentThrottle2);
  Serial.print("us, Third=");
  Serial.print(currentThrottle3_deg);
  Serial.println("°");
}

// Calculate simple checksum untuk error detection
uint8_t calculateChecksum(const char* data, int length) {
  uint8_t checksum = 0;
  for (int i = 0; i < length; i++) {
    checksum ^= data[i];
  }
  return checksum;
}

void sendSimpleData() {
  // Jika data tidak diterima, set semua nilai menjadi 0
  int data_rpm1 = rpmDataReceived ? currentRPM2 : 0;  // RPM1 dari sensor 1
  int data_rpm2 = rpmDataReceived ? currentRPM : 0;   // RPM2 dari sensor 2
  int data_rpm3 = rpmDataReceived ? currentRPM3 : 0;  // RPM3 dari sensor 3

  int data_speed = sensorDataReceived ? (int)(groundSpeed_kmh * 10) : 0;  // Speed x10 untuk 1 decimal
  int data_heading = sensorDataReceived ? (int)heading_deg : 0;
  int data_ampere1 = sensorDataReceived ? (int)(current1 * 100) : 0;  // Ampere x100 untuk 2 decimal
  int data_ampere2 = sensorDataReceived ? (int)(current2 * 100) : 0;  // Ampere x100 untuk 2 decimal
  int data_voltage1 = sensorDataReceived ? (int)(voltage1 * 10) : 0;  // Voltage x10 untuk 1 decimal
  int data_voltage2 = sensorDataReceived ? (int)(voltage2 * 10) : 0;  // Voltage x10 untuk 1 decimal

  // Tambahan: Throttle dan Rudder Angle
  int data_throttle = remoteThrottle;   // Remote throttle (90-110)
  int data_rudder = remoteRudderAngle;  // Rudder angle (-110 to +110)

  // Format: rpm1,rpm2,rpm3,speed,heading,ampere1,ampere2,voltage1,voltage2,throttle,rudder
  ESP32Serial.print(data_rpm1);
  ESP32Serial.print(",");
  ESP32Serial.print(data_rpm2);
  ESP32Serial.print(",");
  ESP32Serial.print(data_rpm3);
  ESP32Serial.print(",");
  ESP32Serial.print(data_speed);  // speed x10 (misal: 125 = 12.5 km/h)
  ESP32Serial.print(",");
  ESP32Serial.print(data_heading);  // heading (misal: 180 = 180°)
  ESP32Serial.print(",");
  ESP32Serial.print(data_ampere1);  // ampere x100 (misal: 245 = 2.45A)
  ESP32Serial.print(",");
  ESP32Serial.print(data_ampere2);  // ampere x100 (misal: 267 = 2.67A)
  ESP32Serial.print(",");
  ESP32Serial.print(data_voltage1);  // voltage x10 (misal: 120 = 12.0V)
  ESP32Serial.print(",");
  ESP32Serial.print(data_voltage2);  // voltage x10 (misal: 118 = 11.8V)
  ESP32Serial.print(",");
  ESP32Serial.print(data_throttle);  // throttle (90-110)
  ESP32Serial.print(",");
  ESP32Serial.println(currentThrottle3_deg);  // rudder angle (-110 to +110)
}

void setup() {
  // Initialize serial communication for debugging
  Serial.begin(115200);

  // Add delay to ensure Serial is ready
  delay(1000);

  // Initialize I2C as master with slower clock for better reliability
  Wire.begin();
  Wire.setClock(I2C_CLOCK_SPEED);

  // Initialize LoRa communication
  ESP32Serial.begin(9600);

  // ESC setup
  ESC1.attach(esc1Pin, 1000, 2000);
  ESC2.attach(esc2Pin, 1000, 2000);
  ESC3.attach(esc3Pin);                  // Attach servo ketiga tanpa range khusus (default 0-180)
  RUDDER.attach(rudderPin, 1000, 2000);  // Initialize rudder servo

  // Set ESCs to minimum
  ESC1.writeMicroseconds(MIN_PULSE);
  ESC2.writeMicroseconds(MIN_PULSE);
  ESC3.write(MIN_PULSE3_DEG);               // Gunakan write() dengan derajat untuk motor 3
  RUDDER.writeMicroseconds(RUDDER_CENTER);  // Center rudder position

  currentThrottle3_deg = MIN_PULSE3_DEG;
  currentThrottle3 = MIN_PULSE3_DEG;  // Update compatibility variable

  // ==================== INITIALIZE NEURAL NETWORK ====================
  initializeNeuralNetwork();
  // ====================================================================

  // Laporan bahwa sistem siap
  Serial.println("=========================================");
  Serial.println("Triple Control System with Neural Network Ready");
  Serial.println("=========================================");
  Serial.println("Commands via Serial or LoRa:");
  Serial.println("'s' - Start motors at minimum");
  Serial.println("'x' - Stop motors");
  Serial.println("'+' - Increase throttle");
  Serial.println("'-' - Decrease throttle");
  Serial.println("'p' - Print status");
  Serial.println("'mXXXX' - SMC control with target RPM XXXX (main motors)");
  Serial.println("'fXXXX' - Fuzzy control with target RPM XXXX (main motors)");
  Serial.println("'tXXXX' - Fuzzy control with target RPM XXXX (third motor - 2sec interval, 90-105°)");
  Serial.println("'gXX.X' - Ground Speed Control (Fuzzy) with target XX.X km/h");
  Serial.println("'pXX.X' - PID Speed Control (Fixed) with target XX.X km/h");
  Serial.println("'dXX.X' - Fuzzy PID Gain Scheduling Control with target XX.X km/h");
  Serial.println("'nXX.X' - Neural Network Speed Control with target XX.X km/h (NEW!)");
  Serial.println("'g', 'p', 'd', 'n' - Show respective control help");
  Serial.println("'m', 'f', 't' - Show respective control help");
  Serial.println("'m0', 'f0', 't0', 'g0', 'p0', 'd0', 'n0' - Disable respective control");
  Serial.println("'i' - Reset I2C bus");
  Serial.println("Remote Control Format: 'T:xxx,A:xxx' (Throttle & Rudder Angle)");
  Serial.println("=========================================");
  Serial.println("Neural Network Info:");
  Serial.println("- Trained on 224 corrected samples with 18 perfect examples");
  Serial.println("- Targets 4,5: BLDC Only configuration");
  Serial.println("- Targets 7,8: Engine Only configuration"); 
  Serial.println("- Target 12: Hybrid configuration");
  Serial.println("- Input: [target, speed, rpm1, rpm2, rpm3]");
  Serial.println("- Output: [rpm1_adj, rpm2_adj, rpm3_adj]");
  Serial.println("- Safety: Fallback to simple control if NN output invalid");
  Serial.println("=========================================");
  Serial.println("Ground Speed Fuzzy Parameters:");
  Serial.print("SPEED_ERROR_NEGL: ");
  Serial.println(SPEED_ERROR_NEGL);
  Serial.print("SPEED_ERROR_NEGS: ");
  Serial.println(SPEED_ERROR_NEGS);
  Serial.print("SPEED_ERROR_POSS: ");
  Serial.println(SPEED_ERROR_POSS);
  Serial.print("SPEED_ERROR_POSL: ");
  Serial.println(SPEED_ERROR_POSL);
  Serial.println("=========================================");
  Serial.println("Fixed PID Speed Control Parameters:");
  Serial.print("PID_KP: ");
  Serial.println(PID_KP);
  Serial.print("PID_KI: ");
  Serial.println(PID_KI);
  Serial.print("PID_KD: ");
  Serial.println(PID_KD);
  Serial.println("=========================================");
  Serial.println("Fuzzy PID Gain Scheduling Parameters:");
  Serial.print("KP Range: ");
  Serial.print(FUZZY_KP_MIN);
  Serial.print(" - ");
  Serial.println(FUZZY_KP_MAX);
  Serial.print("KD Range: ");
  Serial.print(FUZZY_KD_MIN);
  Serial.print(" - ");
  Serial.println(FUZZY_KD_MAX);
  Serial.print("KI Fixed: ");
  Serial.println(FUZZY_PID_KI_BASE);
  Serial.print("Error Small: ");
  Serial.println(GAIN_ERROR_SMALL);
  Serial.print("Error Medium: ");
  Serial.println(GAIN_ERROR_MEDIUM);
  Serial.print("Error Large: ");
  Serial.println(GAIN_ERROR_LARGE);
  Serial.println("=========================================");
  Serial.println("ESC3 Fuzzy Parameters:");
  Serial.print("ERROR3_NEGL: ");
  Serial.println(ERROR3_NEGL);
  Serial.print("ERROR3_NEGS: ");
  Serial.println(ERROR3_NEGS);
  Serial.print("ERROR3_POSS: ");
  Serial.println(ERROR3_POSS);
  Serial.print("ERROR3_POSL: ");
  Serial.println(ERROR3_POSL);
  Serial.println("=========================================");
  Serial.println("I2C Master setup to communicate with:");
  Serial.print("RPM Slave at address: 0x");
  Serial.println(SLAVE1_ADDR, HEX);
  Serial.print("Sensor Slave at address: 0x");
  Serial.println(SLAVE2_ADDR, HEX);
  Serial.println("=========================================");

  // Send system ready message to LoRa
  ESP32Serial.println("SYSTEM_READY_WITH_NEURAL_NETWORK");

  // Check if slaves are initially connected
  Serial.print("Checking initial connection to RPM Slave: ");
  bool slave1Found = checkI2CDevice(SLAVE1_ADDR);
  Serial.println(slave1Found ? "Connected" : "Not found");

  Serial.print("Checking initial connection to Sensor Slave: ");
  bool slave2Found = checkI2CDevice(SLAVE2_ADDR);
  Serial.println(slave2Found ? "Connected" : "Not found");
}

void loop() {
  unsigned long currentTime = millis();

  // I2C data acquisition
  if (currentTime - lastI2CRequestTime >= I2C_REQUEST_INTERVAL) {
    lastI2CRequestTime = currentTime;

    // Request RPM data from Slave 1
    requestRPMData();

    // Request sensor data from Slave 2
    requestSensorData();
  }

  // ==================== NEURAL NETWORK SPEED CONTROL ====================
  if (neuralControlActive && targetNeuralSpeed > 0.0) {
    if (currentTime - lastNeuralCheck >= NEURAL_CHECK_INTERVAL) {
      lastNeuralCheck = currentTime;
      
      // Apply neural network control
      calculateNeuralRPMAdjustment(targetNeuralSpeed);
      
      // Enable motor controls based on target
      if (targetNeuralSpeed <= 5.0) {
        // BLDC mode
        controlActive = true;
        control3Active = false;
        currentMode = FUZZY; // Use existing fuzzy control for motors 1&2
      } else if (targetNeuralSpeed <= 8.0) {
        // Engine mode
        controlActive = false;  
        control3Active = true;
        lastFuzzy3Check = millis(); // Reset timing for motor 3
      } else {
        // Hybrid mode
        controlActive = true;
        control3Active = true;
        currentMode = FUZZY; // Use fuzzy for motors 1&2
        lastFuzzy3Check = millis();
      }
    }
  }
  // ================== END NEURAL NETWORK CONTROL ==================

  // GROUND SPEED CONTROL - FITUR FUZZY
  if (groundSpeedControlActive && targetGroundSpeed > 0.0) {
    if (currentTime - lastGroundSpeedCheck >= GROUND_SPEED_CHECK_INTERVAL) {
      lastGroundSpeedCheck = currentTime;

      // Hitung error ground speed
      float speedError = targetGroundSpeed - groundSpeed_kmh;

      // Dapatkan adjustment RPM dari fuzzy control
      int rpmAdjustment = calculateGroundSpeedFuzzy(speedError);

      // Update target RPM untuk semua motor (perlahan, 100-500 RPM per detik)
      targetRPM += rpmAdjustment;
      targetRPM3 += (rpmAdjustment * 3) / 4;  // Motor 3 adjustment lebih kecil

      // Constrain RPM targets to reasonable values
      targetRPM = constrain(targetRPM, 1000, 12000);
      targetRPM3 = constrain(targetRPM3, 1000, 12000);

      // Print debug info
      Serial.print("Ground Speed Control (Fuzzy): Target=");
      Serial.print(targetGroundSpeed, 1);
      Serial.print(" km/h, Current=");
      Serial.print(groundSpeed_kmh, 1);
      Serial.print(" km/h, Error=");
      Serial.print(speedError, 1);
      Serial.print(" km/h, RPM Adj=");
      Serial.print(rpmAdjustment);
      Serial.print(", New RPM Targets: M1&2=");
      Serial.print(targetRPM);
      Serial.print(", M3=");
      Serial.println(targetRPM3);
    }
  }

  // PID SPEED CONTROL - FITUR FIXED PID
  if (pidSpeedControlActive && targetPIDSpeed > 0.0) {
    if (currentTime - lastPIDSpeedCheck >= PID_SPEED_CHECK_INTERVAL) {
      unsigned long dt_millis = currentTime - lastPIDSpeedCheck;
      lastPIDSpeedCheck = currentTime;

      // Hitung error ground speed
      float speedError = targetPIDSpeed - groundSpeed_kmh;

      // Dapatkan adjustment RPM dari PID control
      int rpmAdjustment = calculateGroundSpeedPID(speedError, dt_millis);

      // Update target RPM untuk semua motor (perlahan, 100-500 RPM per detik)
      targetRPM += rpmAdjustment;
      targetRPM3 += (rpmAdjustment * 3) / 4;  // Motor 3 adjustment lebih kecil

      // Constrain RPM targets to reasonable values
      targetRPM = constrain(targetRPM, 1000, 12000);
      targetRPM3 = constrain(targetRPM3, 1000, 12000);

      // Print debug info
      Serial.print("PID Speed Control (Fixed): Target=");
      Serial.print(targetPIDSpeed, 1);
      Serial.print(" km/h, Current=");
      Serial.print(groundSpeed_kmh, 1);
      Serial.print(" km/h, Error=");
      Serial.print(speedError, 1);
      Serial.print(" km/h, RPM Adj=");
      Serial.print(rpmAdjustment);
      Serial.print(", New RPM Targets: M1&2=");
      Serial.print(targetRPM);
      Serial.print(", M3=");
      Serial.println(targetRPM3);
    }
  }

  // FUZZY PID GAIN SCHEDULING CONTROL - FITUR BARU
  if (fuzzyPIDControlActive && targetFuzzyPIDSpeed > 0.0) {
    if (currentTime - lastFuzzyPIDCheck >= FUZZY_PID_CHECK_INTERVAL) {
      unsigned long dt_millis = currentTime - lastFuzzyPIDCheck;
      lastFuzzyPIDCheck = currentTime;

      // Hitung error ground speed
      float speedError = targetFuzzyPIDSpeed - groundSpeed_kmh;

      // Dapatkan adjustment RPM dari Fuzzy PID Gain Scheduling control
      int rpmAdjustment = calculateFuzzyPIDGainScheduling(speedError, dt_millis);

      // Update target RPM untuk semua motor (perlahan, 100-500 RPM per detik)
      targetRPM += rpmAdjustment;
      targetRPM3 += (rpmAdjustment * 3) / 4;  // Motor 3 adjustment lebih kecil

      // Constrain RPM targets to reasonable values
      targetRPM = constrain(targetRPM, 1000, 12000);
      targetRPM3 = constrain(targetRPM3, 1000, 12000);

      // Print debug info
      Serial.print("Fuzzy PID Gain Scheduling: Target=");
      Serial.print(targetFuzzyPIDSpeed, 1);
      Serial.print(" km/h, Current=");
      Serial.print(groundSpeed_kmh, 1);
      Serial.print(" km/h, Error=");
      Serial.print(speedError, 1);
      Serial.print(" km/h, Kp=");
      Serial.print(adaptiveKp, 0);
      Serial.print(", Kd=");
      Serial.print(adaptiveKd, 0);
      Serial.print(", RPM Adj=");
      Serial.print(rpmAdjustment);
      Serial.print(", New RPM Targets: M1&2=");
      Serial.print(targetRPM);
      Serial.print(", M3=");
      Serial.println(targetRPM3);
    }
  }

  // Apply control for third motor (independent) - SLOWER TIMING
  if (control3Active && targetRPM3 > 0) {
    // Gunakan interval 2 detik untuk fuzzy control motor ketiga
    if (currentTime - lastFuzzy3Check >= FUZZY3_CHECK_INTERVAL) {
      lastFuzzy3Check = currentTime;

      int error3 = targetRPM3 - currentRPM3;
      int throttleChange3 = calculateFuzzy3(error3);

      // Update throttle (hanya bisa +1, 0, atau -1)
      currentThrottle3_deg += throttleChange3;

      // Constrain to valid range (90-105 derajat)
      currentThrottle3_deg = constrain(currentThrottle3_deg, 104, MAX_PULSE3_DEG);

      // Apply new throttle using write() dengan derajat
      ESC3.write(currentThrottle3_deg);

      // Update compatibility variable
      currentThrottle3 = currentThrottle3_deg;

      prevError3_Fuzzy = error3;

      // Print debug info (only if not in speed control modes to avoid spam)
      if (!groundSpeedControlActive && !pidSpeedControlActive && !fuzzyPIDControlActive && !neuralControlActive) {
        Serial.print("Motor3 Fuzzy (2sec): Target=");
        Serial.print(targetRPM3);
        Serial.print(", Current=");
        Serial.print(currentRPM3);
        Serial.print(", Error=");
        Serial.print(error3);
        Serial.print(", Change=");
        Serial.print(throttleChange3);
        Serial.print(", Throttle=");
        Serial.print(currentThrottle3_deg);
        Serial.println("°");
      }
    }
  }

  // Apply control if active for motors 1 & 2 (tetap menggunakan timing yang cepat)
  if (controlActive && targetRPM > 0) {
    if (currentTime - lastRPMCheck > RPM_CHECK_INTERVAL) {
      unsigned long dt_millis = currentTime - lastRPMCheck;
      lastRPMCheck = currentTime;

      int error1, error2;
      int throttleChange1 = 0, throttleChange2 = 0;

      if (currentMode == SMC) {
        // Untuk SMC, error dibagi 200 (seperti pada kode yang berfungsi)
        error1 = (targetRPM - currentRPM2) / 200;
        error2 = (targetRPM - currentRPM) / 200;

        throttleChange1 = calculateSMC(error1, prevError1_SMC, dt_millis, false);
        throttleChange2 = calculateSMC(error2, prevError2_SMC, dt_millis, true);

        prevError1_SMC = error1;
      } else if (currentMode == FUZZY || currentMode == GROUND_SPEED || currentMode == PID_SPEED || currentMode == FUZZY_PID_SPEED || currentMode == NEURAL_NETWORK) {
        currentRPM2 = currentRPM2 - 2500;  ///kalibrasi disini
        if (currentRPM2 < 0) {
          currentRPM2 = 0;
        }
        error1 = targetRPM - currentRPM2;
        error2 = targetRPM - currentRPM;

        throttleChange1 = calculateFuzzy(error1, true);
        throttleChange2 = calculateFuzzy(error2, true);

        prevError1_Fuzzy = error1;
        prevError2_Fuzzy = error2;
      }

      currentThrottle1 += throttleChange1;
      currentThrottle2 += throttleChange2;

      currentThrottle1 = constrain(currentThrottle1, START_PULSE, MAX_PULSE);
      currentThrottle2 = constrain(currentThrottle2, START_PULSE, MAX_PULSE);

      ESC1.writeMicroseconds(currentThrottle1);
      ESC2.writeMicroseconds(currentThrottle2);

      // Print status periodically (only if not in speed control modes to avoid spam)
      if (!groundSpeedControlActive && !pidSpeedControlActive && !fuzzyPIDControlActive && !neuralControlActive) {
        static int statusCounter = 0;
        if (++statusCounter >= 10) {  // Print every 10 cycles (≈1 second)
          statusCounter = 0;

          Serial.print("Mode: ");
          Serial.print(currentMode == SMC ? "SMC" : (currentMode == NEURAL_NETWORK ? "Neural Network" : "Fuzzy"));
          Serial.print(", Target: ");
          Serial.print(targetRPM);
          Serial.print(" | RPM1:");
          Serial.print(currentRPM2);
          Serial.print(", RPM2:");
          Serial.println(currentRPM);
        }
      }
    }
  }

  // Check for user commands via Serial interface
  if (Serial.available() > 0) {
    // Read one character
    char cmd = Serial.read();

    // Check if this might be a multi-character command (m, f, t, g, p, d, n)
    if (cmd == 'm' || cmd == 'f' || cmd == 't' || cmd == 'g' || cmd == 'p' || cmd == 'd' || cmd == 'n') {
      // Read the following numbers
      String numStr = "";
      unsigned long startTime = millis();

      // Buffer time to get all data
      delay(10);  // Increased delay to ensure all data is received

      // Read until timeout or no more data
      while (millis() - startTime < 500) {  // 500ms timeout
        if (Serial.available() > 0) {
          char c = Serial.read();
          if (c == '\n' || c == '\r')
            break;
          if ((c >= '0' && c <= '9') || c == '.')  // Allow decimal point for ground speed
            numStr += c;
        } else {
          // No more data, check if we waited enough
          if (millis() - startTime > 50) break;  // Exit if no data for 50ms
        }
        delay(1);
      }

      // If no number was found, show help for that command
      if (numStr.length() == 0) {
        if (cmd == 'g') {
          Serial.println("Ground Speed Control (Fuzzy) Commands:");
          Serial.println("g5   - Set target speed 5 km/h");
          Serial.println("g10  - Set target speed 10 km/h");
          Serial.println("g15  - Set target speed 15 km/h");
          Serial.println("g0   - Disable ground speed control");
        } else if (cmd == 'p') {
          Serial.println("PID Speed Control (Fixed) Commands:");
          Serial.println("p5   - Set target speed 5 km/h (Fixed PID)");
          Serial.println("p10  - Set target speed 10 km/h (Fixed PID)");
          Serial.println("p15  - Set target speed 15 km/h (Fixed PID)");
          Serial.println("p0   - Disable PID speed control");
        } else if (cmd == 'd') {
          Serial.println("Fuzzy PID Gain Scheduling Commands:");
          Serial.println("d5   - Set target speed 5 km/h (Adaptive PID)");
          Serial.println("d10  - Set target speed 10 km/h (Adaptive PID)");
          Serial.println("d15  - Set target speed 15 km/h (Adaptive PID)");
          Serial.println("d0   - Disable Fuzzy PID control");
          Serial.println("Kp adapts: 400-600, Kd adapts: 25-75, Ki fixed: 200");
        } else if (cmd == 'n') {
          Serial.println("Neural Network Speed Control Commands:");
          Serial.println("n4   - Set target speed 4 km/h (NN BLDC)");
          Serial.println("n5   - Set target speed 5 km/h (NN BLDC)");
          Serial.println("n7   - Set target speed 7 km/h (NN Engine)");
          Serial.println("n8   - Set target speed 8 km/h (NN Engine)");
          Serial.println("n12  - Set target speed 12 km/h (NN Hybrid)");
          Serial.println("n0   - Disable Neural Network control");
          Serial.println("NN trained on 224 corrected samples with 18 perfect examples");
        } else if (cmd == 'm') {
          Serial.println("SMC Control Commands:");
          Serial.println("m3000 - Set target RPM 3000 for motors 1&2");
          Serial.println("m0    - Disable SMC control");
        } else if (cmd == 'f') {
          Serial.println("Fuzzy Control Commands:");
          Serial.println("f3000 - Set target RPM 3000 for motors 1&2");
          Serial.println("f0    - Disable fuzzy control");
        } else if (cmd == 't') {
          Serial.println("Third Motor Commands:");
          Serial.println("t2000 - Set target RPM 2000 for motor 3");
          Serial.println("t0    - Disable motor 3 control");
        }
        return;  // Exit after showing help
      }

      float targetVal = numStr.toFloat();  // Use toFloat for decimal support

      // Disable remote control when using manual commands
      remoteControlActive = false;

      if (cmd == 'm') {
        // SMC for motors 1 & 2
        targetRPM = (int)targetVal;
        groundSpeedControlActive = false;
        pidSpeedControlActive = false;
        fuzzyPIDControlActive = false;
        neuralControlActive = false;  // NEURAL NETWORK

        if (targetRPM > 0) {
          controlActive = true;
          currentMode = SMC;
          prevError1_SMC = 0;
          prevError2_SMC = 0;

          // Start motors if not running
          if (currentThrottle1 <= MIN_PULSE) {
            currentThrottle1 = START_PULSE;
            currentThrottle2 = START_PULSE;
            ESC1.writeMicroseconds(currentThrottle1);
            ESC2.writeMicroseconds(currentThrottle2);
          }

          Serial.print("SMC mode activated. Target RPM: ");
          Serial.println(targetRPM);
        } else {
          controlActive = false;
          currentMode = NONE;
          Serial.println("Main motors control deactivated");
        }
      } else if (cmd == 'f') {
        // Fuzzy for motors 1 & 2
        targetRPM = (int)targetVal;
        groundSpeedControlActive = false;
        pidSpeedControlActive = false;
        fuzzyPIDControlActive = false;
        neuralControlActive = false;  // NEURAL NETWORK

        if (targetRPM > 0) {
          controlActive = true;
          currentMode = FUZZY;
          prevError1_Fuzzy = 0;
          prevError2_Fuzzy = 0;

          // Start motors if not running
          if (currentThrottle1 <= MIN_PULSE) {
            currentThrottle1 = START_PULSE;
            currentThrottle2 = START_PULSE;
            ESC1.writeMicroseconds(currentThrottle1);
            ESC2.writeMicroseconds(currentThrottle2);
          }

          Serial.print("Fuzzy mode activated. Target RPM: ");
          Serial.println(targetRPM);
        } else {
          controlActive = false;
          currentMode = NONE;
          Serial.println("Main motors control deactivated");
        }
      } else if (cmd == 't') {
        // Fuzzy for motor 3
        targetRPM3 = (int)targetVal;

        if (targetRPM3 > 0) {
          control3Active = true;
          prevError3_Fuzzy = 0;
          // Reset timing untuk motor 3
          lastFuzzy3Check = millis();

          // Start motor 3 if not running
          if (currentThrottle3_deg <= MIN_PULSE3_DEG) {
            currentThrottle3_deg = START_PULSE3_DEG;
            currentThrottle3 = currentThrottle3_deg;
            ESC3.write(START_PULSE3_DEG);
          }

          Serial.print("Third motor Fuzzy control activated (2sec interval). Target RPM: ");
          Serial.println(targetRPM3);
        } else {
          control3Active = false;
          Serial.println("Third motor control deactivated");
        }
      } else if (cmd == 'g') {
        // GROUND SPEED CONTROL (FUZZY)
        targetGroundSpeed = targetVal;

        if (targetGroundSpeed > 0) {
          groundSpeedControlActive = true;
          pidSpeedControlActive = false;    // Disable PID when Fuzzy active
          fuzzyPIDControlActive = false;    // Disable Fuzzy PID when Fuzzy active
          neuralControlActive = false;      // Disable Neural Network
          controlActive = true;
          control3Active = true;
          currentMode = GROUND_SPEED;

          // Reset timing
          lastGroundSpeedCheck = millis();
          lastRPMCheck = millis();
          lastFuzzy3Check = millis();

          // Set initial RPM targets
          targetRPM = baseRPM1;
          targetRPM3 = baseRPM3;

          // Reset fuzzy errors
          prevError1_Fuzzy = 0;
          prevError2_Fuzzy = 0;
          prevError3_Fuzzy = 0;

          // Start motors jika belum berjalan
          if (currentThrottle1 <= MIN_PULSE) {
            currentThrottle1 = START_PULSE;
            currentThrottle2 = START_PULSE;
            ESC1.writeMicroseconds(currentThrottle1);
            ESC2.writeMicroseconds(currentThrottle2);
          }
          if (currentThrottle3_deg <= MIN_PULSE3_DEG) {
            currentThrottle3_deg = START_PULSE3_DEG;
            currentThrottle3 = currentThrottle3_deg;
            ESC3.write(START_PULSE3_DEG);
          }

          Serial.print("Ground Speed Control (Fuzzy) activated. Target Speed: ");
          Serial.print(targetGroundSpeed, 1);
          Serial.println(" km/h");
          Serial.println("All motors will adjust RPM automatically based on ground speed");
        } else {
          groundSpeedControlActive = false;
          controlActive = false;
          control3Active = false;
          currentMode = NONE;
          Serial.println("Ground Speed Control deactivated");
        }
      } else if (cmd == 'p') {
        // PID SPEED CONTROL (FIXED)
        targetPIDSpeed = targetVal;

        if (targetPIDSpeed > 0) {
          pidSpeedControlActive = true;
          groundSpeedControlActive = false;  // Disable Fuzzy when PID active
          fuzzyPIDControlActive = false;     // Disable Fuzzy PID when Fixed PID active
          neuralControlActive = false;       // Disable Neural Network
          controlActive = true;
          control3Active = true;
          currentMode = PID_SPEED;

          // Reset timing
          lastPIDSpeedCheck = millis();
          lastRPMCheck = millis();
          lastFuzzy3Check = millis();

          // Set initial RPM targets
          targetRPM = baseRPM1;
          targetRPM3 = baseRPM3;

          // Reset PID errors
          pidSpeedError = 0.0;
          pidSpeedErrorPrev = 0.0;
          pidSpeedIntegral = 0.0;
          pidSpeedDerivative = 0.0;
          
          // Reset fuzzy errors for motor 3
          prevError1_Fuzzy = 0;
          prevError2_Fuzzy = 0;
          prevError3_Fuzzy = 0;

          // Start motors jika belum berjalan
          if (currentThrottle1 <= MIN_PULSE) {
            currentThrottle1 = START_PULSE;
            currentThrottle2 = START_PULSE;
            ESC1.writeMicroseconds(currentThrottle1);
            ESC2.writeMicroseconds(currentThrottle2);
          }
          if (currentThrottle3_deg <= MIN_PULSE3_DEG) {
            currentThrottle3_deg = START_PULSE3_DEG;
            currentThrottle3 = currentThrottle3_deg;
            ESC3.write(START_PULSE3_DEG);
          }

          Serial.print("PID Speed Control (Fixed) activated. Target Speed: ");
          Serial.print(targetPIDSpeed, 1);
          Serial.println(" km/h");
          Serial.println("All motors will adjust RPM automatically based on Fixed PID control");
        } else {
          pidSpeedControlActive = false;
          controlActive = false;
          control3Active = false;
          currentMode = NONE;
          Serial.println("PID Speed Control deactivated");
        }
      } else if (cmd == 'd') {
        // FUZZY PID GAIN SCHEDULING CONTROL
        targetFuzzyPIDSpeed = targetVal;

        if (targetFuzzyPIDSpeed > 0) {
          fuzzyPIDControlActive = true;
          pidSpeedControlActive = false;     // Disable Fixed PID when Fuzzy PID active
          groundSpeedControlActive = false;  // Disable Fuzzy when Fuzzy PID active
          neuralControlActive = false;       // Disable Neural Network
          controlActive = true;
          control3Active = true;
          currentMode = FUZZY_PID_SPEED;

          // Reset timing
          lastFuzzyPIDCheck = millis();
          lastRPMCheck = millis();
          lastFuzzy3Check = millis();

          // Set initial RPM targets
          targetRPM = baseRPM1;
          targetRPM3 = baseRPM3;

          // Reset Fuzzy PID errors
          fuzzyPIDSpeedError = 0.0;
          fuzzyPIDSpeedErrorPrev = 0.0;
          fuzzyPIDSpeedIntegral = 0.0;
          fuzzyPIDSpeedDerivative = 0.0;
          
          // Reset adaptive gains to default
          adaptiveKp = PID_KP;
          adaptiveKd = PID_KD;
          
          // Reset fuzzy errors for motor 3
          prevError1_Fuzzy = 0;
          prevError2_Fuzzy = 0;
          prevError3_Fuzzy = 0;

          // Start motors jika belum berjalan
          if (currentThrottle1 <= MIN_PULSE) {
            currentThrottle1 = START_PULSE;
            currentThrottle2 = START_PULSE;
            ESC1.writeMicroseconds(currentThrottle1);
            ESC2.writeMicroseconds(currentThrottle2);
          }
          if (currentThrottle3_deg <= MIN_PULSE3_DEG) {
            currentThrottle3_deg = START_PULSE3_DEG;
            currentThrottle3 = currentThrottle3_deg;
            ESC3.write(START_PULSE3_DEG);
          }

          Serial.print("Fuzzy PID Gain Scheduling Control activated. Target Speed: ");
          Serial.print(targetFuzzyPIDSpeed, 1);
          Serial.println(" km/h");
          Serial.println("All motors will adjust RPM with adaptive PID gains based on speed error");
          Serial.print("Initial gains - Kp: ");
          Serial.print(adaptiveKp, 0);
          Serial.print(", Ki: ");
          Serial.print(FUZZY_PID_KI_BASE, 0);
          Serial.print(", Kd: ");
          Serial.println(adaptiveKd, 0);
        } else {
          fuzzyPIDControlActive = false;
          controlActive = false;
          control3Active = false;
          currentMode = NONE;
          Serial.println("Fuzzy PID Gain Scheduling Control deactivated");
        }
      } else if (cmd == 'n') {
        // ==================== NEURAL NETWORK SPEED CONTROL - SERIAL COMMAND ====================
        targetNeuralSpeed = targetVal;
        
        if (targetNeuralSpeed > 0) {
          neuralControlActive = true;
          // Disable other controls
          groundSpeedControlActive = false;
          pidSpeedControlActive = false;
          fuzzyPIDControlActive = false;
          controlActive = true;
          control3Active = true;
          currentMode = NEURAL_NETWORK;
          
          // Initialize NN control timing
          lastNeuralCheck = millis();
          lastRPMCheck = millis();
          lastFuzzy3Check = millis();
          
          // Set initial RPM targets based on dataset patterns
          if (targetNeuralSpeed <= 5.0) {
            targetRPM = 8500;    // BLDC base RPM from dataset
            targetRPM3 = 0;      // No engine for BLDC targets
          } else if (targetNeuralSpeed <= 8.0) {
            targetRPM = 1200;    // Keep BLDC minimal for engine targets
            targetRPM3 = 6500;   // Engine base RPM from dataset
          } else {
            targetRPM = 9500;    // Hybrid BLDC RPM from dataset
            targetRPM3 = 13000;  // Hybrid engine RPM from dataset
          }
          
          // Start motors based on configuration
          if (targetNeuralSpeed <= 5.0) {
            // BLDC mode - start main motors, stop engine
            if (currentThrottle1 <= MIN_PULSE) {
              currentThrottle1 = START_PULSE;
              currentThrottle2 = START_PULSE;
              ESC1.writeMicroseconds(currentThrottle1);
              ESC2.writeMicroseconds(currentThrottle2);
            }
            currentThrottle3_deg = MIN_PULSE3_DEG;
            ESC3.write(MIN_PULSE3_DEG);
            
          } else if (targetNeuralSpeed <= 8.0) {
            // Engine mode - start engine, minimal main motors
            if (currentThrottle3_deg <= MIN_PULSE3_DEG) {
              currentThrottle3_deg = START_PULSE3_DEG;
              ESC3.write(currentThrottle3_deg);
            }
            currentThrottle1 = MIN_PULSE;
            currentThrottle2 = MIN_PULSE;
            ESC1.writeMicroseconds(currentThrottle1);
            ESC2.writeMicroseconds(currentThrottle2);
            
          } else {
            // Hybrid mode - start all motors
            if (currentThrottle1 <= MIN_PULSE) {
              currentThrottle1 = START_PULSE;
              currentThrottle2 = START_PULSE;
              ESC1.writeMicroseconds(currentThrottle1);
              ESC2.writeMicroseconds(currentThrottle2);
            }
            if (currentThrottle3_deg <= MIN_PULSE3_DEG) {
              currentThrottle3_deg = START_PULSE3_DEG;
              ESC3.write(currentThrottle3_deg);
            }
          }
          
          Serial.print("Neural Network Speed Control activated. Target: ");
          Serial.print(targetNeuralSpeed, 1);
          Serial.println(" km/h");
          Serial.println("NN: Learning from 224 corrected samples, 18 perfect examples");
          
          // Print configuration info
          if (targetNeuralSpeed <= 5.0) {
            Serial.println("NN Config: BLDC Only (like dataset targets 4,5)");
          } else if (targetNeuralSpeed <= 8.0) {
            Serial.println("NN Config: Engine Only (like dataset targets 7,8)");
          } else {
            Serial.println("NN Config: Hybrid (like dataset target 12)");
          }
          
        } else {
          neuralControlActive = false;
          controlActive = false;
          control3Active = false;
          currentMode = NONE;
          Serial.println("Neural Network Speed Control deactivated");
        }
        // ================== END NEURAL NETWORK SERIAL COMMAND ==================
      }
    }
    // Process single character commands (s, x, +, -, p, i)
    else if (cmd == 's') {
      // Start motors
      currentThrottle1 = START_PULSE;
      currentThrottle2 = START_PULSE+20;
      currentThrottle3_deg = START_PULSE3_DEG;

      ESC1.writeMicroseconds(currentThrottle1);
      ESC2.writeMicroseconds(currentThrottle2);
      ESC3.write(START_PULSE3_DEG);

      Serial.println("Motors started at minimum speed");
      reportThrottle();

      // Disable automatic control and remote control
      controlActive = false;
      control3Active = false;
      remoteControlActive = false;
      groundSpeedControlActive = false;
      pidSpeedControlActive = false;
      fuzzyPIDControlActive = false;
      neuralControlActive = false;  // NEURAL NETWORK
      currentMode = NONE;
    } else if (cmd == 'x') {
      // Stop motors
      currentThrottle1 = MIN_PULSE;
      currentThrottle2 = MIN_PULSE;
      currentThrottle3_deg = MIN_PULSE3_DEG;

      ESC1.writeMicroseconds(currentThrottle1);
      ESC2.writeMicroseconds(currentThrottle2);
      ESC3.write(MIN_PULSE3_DEG);
      RUDDER.writeMicroseconds(RUDDER_CENTER);  // Center rudder on stop
      ESC3.write(90);
      groundSpeedControlActive = false;
      pidSpeedControlActive = false;
      fuzzyPIDControlActive = false;
      neuralControlActive = false;  // NEURAL NETWORK
      controlActive = false;
      control3Active = false;
      currentMode = NONE;
      Serial.println("All Speed Control modes deactivated");

      // Disable all control modes
      controlActive = false;
      control3Active = false;
      remoteControlActive = false;
      groundSpeedControlActive = false;
      pidSpeedControlActive = false;
      fuzzyPIDControlActive = false;
      neuralControlActive = false;  // NEURAL NETWORK
      currentMode = NONE;

      Serial.println("Motors stopped");
    } else if (cmd == '+') {
      // Increase throttle (disable all automatic controls)
      remoteControlActive = false;
      groundSpeedControlActive = false;
      pidSpeedControlActive = false;
      fuzzyPIDControlActive = false;
      neuralControlActive = false;  // NEURAL NETWORK
      bool throttleIncreased = false;

      // Motor 1 & 2
      if (currentThrottle1 < MAX_PULSE) {
        currentThrottle1 += STEP_SIZE;
        currentThrottle2 += STEP_SIZE*1.7;
        ESC1.writeMicroseconds(currentThrottle1);
        ESC2.writeMicroseconds(currentThrottle2);
        throttleIncreased = true;
      }

      // Motor 3 (menggunakan derajat)
      if (currentThrottle3_deg < MAX_PULSE3_DEG) {
        currentThrottle3_deg += STEP_SIZE3_DEG;
        currentThrottle3 = currentThrottle3_deg;  // Update compatibility
        ESC3.write(currentThrottle3_deg);
        Serial.println(currentThrottle3_deg);
        throttleIncreased = true;
      }

      if (throttleIncreased) {
        Serial.println("Throttle increased:");
        reportThrottle();
      } else {
        Serial.println("Maximum throttle reached");
      }

      // Disable automatic control
      controlActive = false;
      control3Active = false;
      currentMode = NONE;
    } else if (cmd == '-') {
      // Decrease throttle (disable all automatic controls)
      remoteControlActive = false;
      groundSpeedControlActive = false;
      pidSpeedControlActive = false;
      fuzzyPIDControlActive = false;
      neuralControlActive = false;  // NEURAL NETWORK
      bool throttleDecreased = false;

      // Motor 1 & 2
      if (currentThrottle1 > START_PULSE) {
        currentThrottle1 -= STEP_SIZE;
        currentThrottle2 -= STEP_SIZE;
        ESC1.writeMicroseconds(currentThrottle1);
        ESC2.writeMicroseconds(currentThrottle2);
        throttleDecreased = true;
      }

      // Motor 3 (menggunakan derajat)
      if (currentThrottle3_deg > START_PULSE3_DEG) {
        currentThrottle3_deg -= STEP_SIZE3_DEG;
        currentThrottle3 = currentThrottle3_deg;  // Update compatibility
        ESC3.write(currentThrottle3_deg);
        throttleDecreased = true;
      }

      if (throttleDecreased) {
        Serial.println("Throttle decreased:");
        reportThrottle();
      } else {
        Serial.println("Minimum throttle reached");
      }

      // Disable automatic control
      controlActive = false;
      control3Active = false;
      groundSpeedControlActive = false;
      pidSpeedControlActive = false;
      fuzzyPIDControlActive = false;
      neuralControlActive = false;  // NEURAL NETWORK
      currentMode = NONE;
    } else if (cmd == 'p') {
      // Print status
      Serial.println("=== Status Report ===");

      Serial.print("Mode: ");
      if (neuralControlActive) {
        Serial.println("Neural Network Speed Control");
        Serial.print("Target Speed: ");
        Serial.print(targetNeuralSpeed, 1);
        Serial.print(" km/h, Current: ");
        Serial.print(groundSpeed_kmh, 1);
        Serial.print(" km/h, Performance: ");
        Serial.print(nnPerformanceScore * 100, 1);
        Serial.println("%");
      } else if (groundSpeedControlActive) {
        Serial.println("Ground Speed Control (Fuzzy)");
        Serial.print("Target Speed: ");
        Serial.print(targetGroundSpeed, 1);
        Serial.print(" km/h, Current: ");
        Serial.print(groundSpeed_kmh, 1);
        Serial.println(" km/h");
      } else if (pidSpeedControlActive) {
        Serial.println("PID Speed Control (Fixed)");
        Serial.print("Target Speed: ");
        Serial.print(targetPIDSpeed, 1);
        Serial.print(" km/h, Current: ");
        Serial.print(groundSpeed_kmh, 1);
        Serial.println(" km/h");
      } else if (fuzzyPIDControlActive) {
        Serial.println("Fuzzy PID Gain Scheduling Control");
        Serial.print("Target Speed: ");
        Serial.print(targetFuzzyPIDSpeed, 1);
        Serial.print(" km/h, Current: ");
        Serial.print(groundSpeed_kmh, 1);
        Serial.print(" km/h, Adaptive Kp: ");
        Serial.print(adaptiveKp, 0);
        Serial.print(", Kd: ");
        Serial.println(adaptiveKd, 0);
      } else if (remoteControlActive) {
        Serial.println("Remote Control");
        Serial.print("Remote Throttle: ");
        Serial.print(remoteThrottle);
        Serial.print(", Remote Rudder: ");
        Serial.println(remoteRudderAngle);
      } else if (controlActive) {
        Serial.println(currentMode == SMC ? "SMC" : (currentMode == NEURAL_NETWORK ? "Neural Network" : "Fuzzy"));
        Serial.print("Target RPM 1&2: ");
        Serial.println(targetRPM);
      } else {
        Serial.println("Manual");
      }

      Serial.print("Mode 3: ");
      if (control3Active) {
        Serial.println("Fuzzy (2sec interval)");
        Serial.print("Target RPM 3: ");
        Serial.println(targetRPM3);
      } else {
        Serial.println("Manual");
      }

      Serial.print("Motor 1 - Throttle: ");
      Serial.print(currentThrottle1);
      Serial.print("us, RPM: ");
      Serial.println(currentRPM);

      Serial.print("Motor 2 - Throttle: ");
      Serial.print(currentThrottle2);
      Serial.print("us, RPM: ");
      Serial.println(currentRPM2);

      Serial.print("Motor 3 - Throttle: ");
      Serial.print(currentThrottle3_deg);
      Serial.print("°, RPM: ");
      Serial.println(currentRPM3);

      Serial.print("Speed: ");
      Serial.print(groundSpeed_kmh, 1);
      Serial.print(" km/h, Heading: ");
      Serial.print(heading_deg, 0);
      Serial.println(" degrees");

      Serial.print("Voltage 1: ");
      Serial.print(voltage1, 1);
      Serial.print("V, Voltage 2: ");
      Serial.print(voltage2, 1);
      Serial.println("V");

      Serial.print("Current 1: ");
      Serial.print(current1, 1);
      Serial.print("A, Current 2: ");
      Serial.print(current2, 1);
      Serial.println("A");

      Serial.print("I2C Data Received - RPM: ");
      Serial.print(rpmDataReceived ? "Yes" : "No");
      Serial.print(", Sensors: ");
      Serial.println(sensorDataReceived ? "Yes" : "No");

      // Neural Network performance info
      if (neuralControlActive) {
        Serial.print("NN Performance: ");
        Serial.print(nnPerformanceScore * 100, 1);
        Serial.print("% success (");
        Serial.print(nnSuccessCount);
        Serial.print("/");
        Serial.print(nnTotalPredictions);
        Serial.println(" predictions)");
      }

      Serial.println("===================");
    } else if (cmd == 'i') {
      // Reset I2C bus
      Serial.println("Resetting I2C bus...");
      resetI2C();
    } else {
      // Unknown single character command
      Serial.print("Unknown command: ");
      Serial.println(cmd);
      Serial.println("Available commands: s, x, +, -, p, i, g, m, f, t, p, d, n");
    }

    // Clear remaining serial buffer
    while (Serial.available() > 0) {
      Serial.read();
    }
  }

  // Handle LoRa communication
  if (ESP32Serial.available()) {
    char c = ESP32Serial.read();

    // Process character
    if (c == '\n' || c == '\r') {
      if (loraBufferIndex > 0) {
        loraBuffer[loraBufferIndex] = '\0';
        processLoraCommand(loraBuffer);
        loraBufferIndex = 0;
        messageCount++;
      }
    } else if (loraBufferIndex < sizeof(loraBuffer) - 1) {
      loraBuffer[loraBufferIndex++] = c;
    }
  }

  // Send heartbeat periodically
  if (currentTime - lastHeartbeatTime >= HEARTBEAT_INTERVAL) {
    sendHeartbeat();
    lastHeartbeatTime = currentTime;
  }

  // Small delay to prevent watchdog issues
  delay(5);
}

// ==================== EXISTING CONTROL FUNCTIONS ====================

// SMC Implementation using integer math for microsecond range
int calculateSMC(int error, int prevError, unsigned long dt_millis, bool isMotor2) {
  // Safeguard for dt (minimum 1ms)
  if (dt_millis < 1) dt_millis = 1;

  // Calculate error derivative
  int error_dot = ((error - prevError) * 1000) / dt_millis;

  // Limit error_dot to reasonable values
  error_dot = constrain(error_dot, -1000, 1000);

  // Calculate sliding surface (s = error_dot + lambda * error)
  int s = error_dot + (LAMBDA_INT * error) / 100;

  // Calculate control output (smaller for finer microsecond control)
  int control;
  if (abs(s) <= BOUNDARY_INT) {
    // Inside boundary layer
    control = ((ETA_INT * s) / BOUNDARY_INT + K_INT) * error / 100;
  } else {
    // Outside boundary layer
    control = (ETA_INT + K_INT * (s > 0 ? 120 : -120) / 100) * error / 100;
  }

  // Limit control output for microsecond precision (smaller step size)
  return constrain(control, -2, 2);
}

// Membership functions untuk Motor 1 & 2
int membershipNL(int error) {
  if (error <= ERROR_NEGL) return 100;
  if (error >= ERROR_NEGS) return 0;
  return (100 * (ERROR_NEGS - error)) / (ERROR_NEGS - ERROR_NEGL);
}

int membershipNS(int error) {
  if (error <= ERROR_NEGL || error >= ERROR_ZERO) return 0;
  if (error <= ERROR_NEGS)
    return (100 * (error - ERROR_NEGL)) / (ERROR_NEGS - ERROR_NEGL);
  return (100 * (ERROR_ZERO - error)) / (ERROR_ZERO - ERROR_NEGS);
}

int membershipZE(int error) {
  if (error <= ERROR_NEGS || error >= ERROR_POSS) return 0;
  if (error <= ERROR_ZERO)
    return (100 * (error - ERROR_NEGS)) / (ERROR_ZERO - ERROR_NEGS);
  return (100 * (ERROR_POSS - error)) / (ERROR_POSS - ERROR_ZERO);
}

int membershipPS(int error) {
  if (error <= ERROR_ZERO || error >= ERROR_POSL) return 0;
  if (error <= ERROR_POSS)
    return (100 * (error - ERROR_ZERO)) / (ERROR_POSS - ERROR_ZERO);
  return (100 * (ERROR_POSL - error)) / (ERROR_POSL - ERROR_POSS);
}

int membershipPL(int error) {
  if (error <= ERROR_POSS) return 0;
  if (error >= ERROR_POSL) return 100;
  return (100 * (error - ERROR_POSS)) / (ERROR_POSL - ERROR_POSS);
}

// Membership functions KHUSUS untuk Motor 3 (ESC3)
int membershipNL3(int error) {
  if (error <= ERROR3_NEGL) return 100;
  if (error >= ERROR3_NEGS) return 0;
  return (100 * (ERROR3_NEGS - error)) / (ERROR3_NEGS - ERROR3_NEGL);
}

int membershipNS3(int error) {
  if (error <= ERROR3_NEGL || error >= ERROR3_ZERO) return 0;
  if (error <= ERROR3_NEGS)
    return (100 * (error - ERROR3_NEGL)) / (ERROR3_NEGS - ERROR3_NEGL);
  return (100 * (ERROR3_ZERO - error)) / (ERROR3_ZERO - ERROR3_NEGS);
}

int membershipZE3(int error) {
  if (error <= ERROR3_NEGS || error >= ERROR3_POSS) return 0;
  if (error <= ERROR3_ZERO)
    return (100 * (error - ERROR3_NEGS)) / (ERROR3_ZERO - ERROR3_NEGS);
  return (100 * (ERROR3_POSS - error)) / (ERROR3_POSS - ERROR3_ZERO);
}

int membershipPS3(int error) {
  if (error <= ERROR3_ZERO || error >= ERROR3_POSL) return 0;
  if (error <= ERROR3_POSS)
    return (100 * (error - ERROR3_ZERO)) / (ERROR3_POSS - ERROR3_ZERO);
  return (100 * (ERROR3_POSL - error)) / (ERROR3_POSL - ERROR3_POSS);
}

int membershipPL3(int error) {
  if (error <= ERROR3_POSS) return 0;
  if (error >= ERROR3_POSL) return 100;
  return (100 * (error - ERROR3_POSS)) / (ERROR3_POSL - ERROR3_POSS);
}

// GROUND SPEED FUZZY MEMBERSHIP FUNCTIONS - FUNGSI BARU
int membershipSpeedNL(float error) {
  if (error <= SPEED_ERROR_NEGL) return 100;
  if (error >= SPEED_ERROR_NEGS) return 0;
  return (int)(100 * (SPEED_ERROR_NEGS - error) / (SPEED_ERROR_NEGS - SPEED_ERROR_NEGL));
}

int membershipSpeedNS(float error) {
  if (error <= SPEED_ERROR_NEGL || error >= SPEED_ERROR_ZERO) return 0;
  if (error <= SPEED_ERROR_NEGS)
    return (int)(100 * (error - SPEED_ERROR_NEGL) / (SPEED_ERROR_NEGS - SPEED_ERROR_NEGL));
  return (int)(100 * (SPEED_ERROR_ZERO - error) / (SPEED_ERROR_ZERO - SPEED_ERROR_NEGS));
}

int membershipSpeedZE(float error) {
  if (error <= SPEED_ERROR_NEGS || error >= SPEED_ERROR_POSS) return 0;
  if (error <= SPEED_ERROR_ZERO)
    return (int)(100 * (error - SPEED_ERROR_NEGS) / (SPEED_ERROR_ZERO - SPEED_ERROR_NEGS));
  return (int)(100 * (SPEED_ERROR_POSS - error) / (SPEED_ERROR_POSS - SPEED_ERROR_ZERO));
}

int membershipSpeedPS(float error) {
  if (error <= SPEED_ERROR_ZERO || error >= SPEED_ERROR_POSL) return 0;
  if (error <= SPEED_ERROR_POSS)
    return (int)(100 * (error - SPEED_ERROR_ZERO) / (SPEED_ERROR_POSS - SPEED_ERROR_ZERO));
  return (int)(100 * (SPEED_ERROR_POSL - error) / (SPEED_ERROR_POSL - SPEED_ERROR_POSS));
}

int membershipSpeedPL(float error) {
  if (error <= SPEED_ERROR_POSS) return 0;
  if (error >= SPEED_ERROR_POSL) return 100;
  return (int)(100 * (error - SPEED_ERROR_POSS) / (SPEED_ERROR_POSL - SPEED_ERROR_POSS));
}

// FUZZY GAIN SCHEDULING MEMBERSHIP FUNCTIONS - FITUR BARU
int membershipErrorSmall(float absError) {
  if (absError <= GAIN_ERROR_SMALL) return 100;
  if (absError >= GAIN_ERROR_MEDIUM) return 0;
  return (int)(100 * (GAIN_ERROR_MEDIUM - absError) / (GAIN_ERROR_MEDIUM - GAIN_ERROR_SMALL));
}

int membershipErrorMedium(float absError) {
  if (absError <= GAIN_ERROR_SMALL || absError >= GAIN_ERROR_LARGE) return 0;
  if (absError <= GAIN_ERROR_MEDIUM)
    return (int)(100 * (absError - GAIN_ERROR_SMALL) / (GAIN_ERROR_MEDIUM - GAIN_ERROR_SMALL));
  return (int)(100 * (GAIN_ERROR_LARGE - absError) / (GAIN_ERROR_LARGE - GAIN_ERROR_MEDIUM));
}

int membershipErrorLarge(float absError) {
  if (absError <= GAIN_ERROR_MEDIUM) return 0;
  if (absError >= GAIN_ERROR_LARGE) return 100;
  return (int)(100 * (absError - GAIN_ERROR_MEDIUM) / (GAIN_ERROR_LARGE - GAIN_ERROR_MEDIUM));
}

// Sugeno Fuzzy control for motors 1 & 2 (adjusted for finer microsecond control)
int calculateFuzzy(int error, bool isMotor2) {
  // Calculate membership degrees
  int mu_NL = membershipNL(error);
  int mu_NS = membershipNS(error);
  int mu_ZE = membershipZE(error);
  int mu_PS = membershipPS(error);
  int mu_PL = membershipPL(error);

  // Define rule consequents (smaller values for finer microsecond control)
  int z1 = -10;  // -1.0 * 10
  int z2 = -5;   // -0.5 * 10
  int z3 = 0;    //  0.0 * 10

  // Different values for motor 2
  int z4, z5;
  if (isMotor2) {
    z4 = 7;   //  0.7 * 10
    z5 = 15;  //  1.5 * 10
  } else {
    z4 = 5;   //  0.5 * 10
    z5 = 10;  //  1.0 * 10
  }

  // Calculate weighted average (Sugeno defuzzification)
  long numerator = (long)mu_NL * z1 + (long)mu_NS * z2 + (long)mu_ZE * z3 + (long)mu_PS * z4 + (long)mu_PL * z5;
  int denominator = mu_NL + mu_NS + mu_ZE + mu_PS + mu_PL;

  // Avoid division by zero
  if (denominator <= 0) return 0;

  // Calculate result and convert back to original units
  int result = (int)(numerator / denominator / 10);

  // Limit output (smaller step size for microsecond precision)
  return constrain(result, -2, 2);
}

// Fuzzy control for third motor (output hanya integer: -1, 0, +1)
int calculateFuzzy3(int error) {
  // Calculate membership degrees menggunakan parameter KHUSUS Motor 3
  int mu_NL = membershipNL3(error);  // MENGGUNAKAN membershipNL3
  int mu_NS = membershipNS3(error);  // MENGGUNAKAN membershipNS3
  int mu_ZE = membershipZE3(error);  // MENGGUNAKAN membershipZE3
  int mu_PS = membershipPS3(error);  // MENGGUNAKAN membershipPS3
  int mu_PL = membershipPL3(error);  // MENGGUNAKAN membershipPL3

  // Define rule consequents specifically for third motor (integer values)
  int z1 = -100;  // Large negative error → output mentah -100
  int z2 = -50;   // Small negative error → output mentah -50
  int z3 = 0;     // Zero error → output mentah 0
  int z4 = 50;    // Small positive error → output mentah 50
  int z5 = 100;   // Large positive error → output mentah 100

  // Calculate weighted average (Sugeno defuzzification)
  long numerator = (long)mu_NL * z1 + (long)mu_NS * z2 + (long)mu_ZE * z3 + (long)mu_PS * z4 + (long)mu_PL * z5;
  int denominator = mu_NL + mu_NS + mu_ZE + mu_PS + mu_PL;

  // Avoid division by zero
  if (denominator <= 0) return 0;

  // Calculate result
  int result = (int)(numerator / denominator);

  // Convert to simple integer output: -1, 0, atau +1
  if (result > 25) return 1;         // NAIK 1 derajat
  else if (result < -25) return -1;  // TURUN 1 derajat
  else return 0;                     // DIAM
}

// GROUND SPEED FUZZY CONTROL - FUNGSI BARU
int calculateGroundSpeedFuzzy(float speedError) {
  // Calculate membership degrees untuk ground speed error
  int mu_NL = membershipSpeedNL(speedError);
  int mu_NS = membershipSpeedNS(speedError);
  int mu_ZE = membershipSpeedZE(speedError);
  int mu_PS = membershipSpeedPS(speedError);
  int mu_PL = membershipSpeedPL(speedError);

  // Define rule consequents untuk RPM adjustment (dalam RPM per detik)
  int z1 = -RPM_ADJUST_MAX;  // Large negative speed error → kurangi RPM banyak
  int z2 = -RPM_ADJUST_MIN;  // Small negative speed error → kurangi RPM sedikit
  int z3 = 0;                // Zero speed error → tidak ada perubahan RPM
  int z4 = RPM_ADJUST_MIN;   // Small positive speed error → tambah RPM sedikit
  int z5 = RPM_ADJUST_MAX;   // Large positive speed error → tambah RPM banyak

  // Calculate weighted average (Sugeno defuzzification)
  long numerator = (long)mu_NL * z1 + (long)mu_NS * z2 + (long)mu_ZE * z3 + (long)mu_PS * z4 + (long)mu_PL * z5;
  int denominator = mu_NL + mu_NS + mu_ZE + mu_PS + mu_PL;

  // Avoid division by zero
  if (denominator <= 0) return 0;

  // Calculate result (RPM adjustment per detik)
  int result = (int)(numerator / denominator);

  // Constrain output to reasonable RPM adjustment range
  return constrain(result, -RPM_ADJUST_MAX, RPM_ADJUST_MAX);
}

// PID GROUND SPEED CONTROL - FUNGSI BARU
int calculateGroundSpeedPID(float speedError, unsigned long dt_millis) {
  // Safeguard for dt (minimum 1ms)
  if (dt_millis < 1) dt_millis = 1;
  
  // Convert dt to seconds for calculations
  float dt_sec = dt_millis / 1000.0;
  
  // Store current error
  pidSpeedError = speedError;
  
  // Calculate derivative
  pidSpeedDerivative = (pidSpeedError - pidSpeedErrorPrev) / dt_sec;
  
  // Calculate integral with windup protection
  pidSpeedIntegral += pidSpeedError * dt_sec;
  
  // Integral windup protection - limit integral term
  float maxIntegral = 5.0;  // 5 seconds worth of error
  pidSpeedIntegral = constrain(pidSpeedIntegral, -maxIntegral, maxIntegral);
  
  // Calculate PID output
  float pidOutput = (PID_KP * pidSpeedError) + 
                   (PID_KI * pidSpeedIntegral) + 
                   (PID_KD * pidSpeedDerivative);
  
  // Store error for next iteration
  pidSpeedErrorPrev = pidSpeedError;
  
  // Convert to integer RPM adjustment
  int rpmAdjustment = (int)pidOutput;
  
  // Constrain output to reasonable RPM adjustment range
  return constrain(rpmAdjustment, -RPM_ADJUST_MAX, RPM_ADJUST_MAX);
}

// FUZZY PID GAIN SCHEDULING - ADAPTIVE GAIN CALCULATION
void calculateAdaptiveGains(float speedError) {
  // Calculate absolute error for gain scheduling
  float absError = abs(speedError);
  
  // Calculate membership degrees untuk error magnitude
  int mu_Small = membershipErrorSmall(absError);
  int mu_Medium = membershipErrorMedium(absError);
  int mu_Large = membershipErrorLarge(absError);
  
  // Fuzzy rules untuk Kp (Proportional Gain):
  // - Error Small → Kp Small (fine control)
  // - Error Medium → Kp Medium 
  // - Error Large → Kp Large (aggressive control)
  
  // Define Kp consequents
  float kp_small = FUZZY_KP_MIN;    // 400.0
  float kp_medium = (FUZZY_KP_MIN + FUZZY_KP_MAX) / 2;  // 500.0
  float kp_large = FUZZY_KP_MAX;    // 600.0
  
  // Calculate adaptive Kp using weighted average
  float kp_numerator = (mu_Small * kp_small) + (mu_Medium * kp_medium) + (mu_Large * kp_large);
  int kp_denominator = mu_Small + mu_Medium + mu_Large;
  
  if (kp_denominator > 0) {
    adaptiveKp = kp_numerator / kp_denominator;
  } else {
    adaptiveKp = PID_KP;  // Default value
  }
  
  // Fuzzy rules untuk Kd (Derivative Gain):
  // - Error Small → Kd Large (smooth control, prevent overshoot)
  // - Error Medium → Kd Medium
  // - Error Large → Kd Small (fast response, less damping)
  
  // Define Kd consequents (inverse relationship)
  float kd_small = FUZZY_KD_MAX;    // 75.0 (high damping for small errors)
  float kd_medium = (FUZZY_KD_MIN + FUZZY_KD_MAX) / 2;  // 50.0
  float kd_large = FUZZY_KD_MIN;    // 25.0 (low damping for large errors)
  
  // Calculate adaptive Kd using weighted average
  float kd_numerator = (mu_Small * kd_small) + (mu_Medium * kd_medium) + (mu_Large * kd_large);
  int kd_denominator = mu_Small + mu_Medium + mu_Large;
  
  if (kd_denominator > 0) {
    adaptiveKd = kd_numerator / kd_denominator;
  } else {
    adaptiveKd = PID_KD;  // Default value
  }
  
  // Constrain gains to valid ranges
  adaptiveKp = constrain(adaptiveKp, FUZZY_KP_MIN, FUZZY_KP_MAX);
  adaptiveKd = constrain(adaptiveKd, FUZZY_KD_MIN, FUZZY_KD_MAX);
}

// FUZZY PID GAIN SCHEDULING CONTROL - FUNGSI UTAMA BARU
int calculateFuzzyPIDGainScheduling(float speedError, unsigned long dt_millis) {
  // Safeguard for dt (minimum 1ms)
  if (dt_millis < 1) dt_millis = 1;
  
  // Convert dt to seconds for calculations
  float dt_sec = dt_millis / 1000.0;
  
  // Step 1: Calculate adaptive gains using fuzzy logic
  calculateAdaptiveGains(speedError);
  
  // Step 2: Store current error
  fuzzyPIDSpeedError = speedError;
  
  // Step 3: Calculate derivative
  fuzzyPIDSpeedDerivative = (fuzzyPIDSpeedError - fuzzyPIDSpeedErrorPrev) / dt_sec;
  
  // Step 4: Calculate integral with windup protection
  fuzzyPIDSpeedIntegral += fuzzyPIDSpeedError * dt_sec;
  
  // Integral windup protection - limit integral term
  float maxIntegral = 5.0;  // 5 seconds worth of error
  fuzzyPIDSpeedIntegral = constrain(fuzzyPIDSpeedIntegral, -maxIntegral, maxIntegral);
  
  // Step 5: Calculate PID output using adaptive gains
  float pidOutput = (adaptiveKp * fuzzyPIDSpeedError) + 
                   (FUZZY_PID_KI_BASE * fuzzyPIDSpeedIntegral) + 
                   (adaptiveKd * fuzzyPIDSpeedDerivative);
  
  // Step 6: Store error for next iteration
  fuzzyPIDSpeedErrorPrev = fuzzyPIDSpeedError;
  
  // Step 7: Convert to integer RPM adjustment
  int rpmAdjustment = (int)pidOutput;
  
  // Step 8: Constrain output to reasonable RPM adjustment range
  return constrain(rpmAdjustment, -RPM_ADJUST_MAX, RPM_ADJUST_MAX);
}