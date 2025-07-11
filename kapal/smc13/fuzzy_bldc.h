#ifndef FUZZY_BLDC_H
#define FUZZY_BLDC_H

#include "variables_settings.h"

// Function declarations
int membershipNL(int error);
int membershipNS(int error);
int membershipZE(int error);
int membershipPS(int error);
int membershipPL(int error);
int calculateFuzzy(int error, bool isMotor2);

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

#endif // FUZZY_BLDC_H