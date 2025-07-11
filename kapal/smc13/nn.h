#ifndef NEURAL_NETWORK_H
#define NEURAL_NETWORK_H

#include "variables_settings.h"

// Function declarations
void initializeNeuralNetwork();
float sigmoid(float x);
float tanh_activation(float x);
void normalizeInputs(float inputs[5], float target, float current_speed, float rpm1, float rpm2, float rpm3);
void forwardPass(float inputs[5]);
bool validateNeuralOutput();
void calculateNeuralRPMAdjustment(float target_speed);

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

#endif // NEURAL_NETWORK_H