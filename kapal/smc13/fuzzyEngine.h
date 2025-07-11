#ifndef FUZZY_HYBRID_ESC3_H
#define FUZZY_HYBRID_ESC3_H

#include "variables_settings.h"

// Function declarations
int membershipNL3(int error);
int membershipNS3(int error);
int membershipZE3(int error);
int membershipPS3(int error);
int membershipPL3(int error);
int calculateFuzzy3(int error);

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

#endif // FUZZY_HYBRID_ESC3_H