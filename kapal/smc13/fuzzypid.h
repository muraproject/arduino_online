#ifndef FUZZY_PID_GROUNDSPEED_H
#define FUZZY_PID_GROUNDSPEED_H

#include "variables_settings.h"

// Function declarations
int membershipErrorSmall(float absError);
int membershipErrorMedium(float absError);
int membershipErrorLarge(float absError);
void calculateAdaptiveGains(float speedError);
int calculateFuzzyPIDGainScheduling(float speedError, unsigned long dt_millis);

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

#endif // FUZZY_PID_GROUNDSPEED_H