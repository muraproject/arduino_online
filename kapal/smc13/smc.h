#ifndef SMC_BLDC_H
#define SMC_BLDC_H

#include "variables_settings.h"

// Function declaration
int calculateSMC(int error, int prevError, unsigned long dt_millis, bool isMotor2);

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

#endif // SMC_BLDC_H