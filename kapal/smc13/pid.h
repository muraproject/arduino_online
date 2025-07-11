#ifndef PID_GROUNDSPEED_H
#define PID_GROUNDSPEED_H

#include "variables_settings.h"

// Function declaration
int calculateGroundSpeedPID(float speedError, unsigned long dt_millis);

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

#endif // PID_GROUNDSPEED_H