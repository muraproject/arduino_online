#ifndef FUZZY_GROUNDSPEED_H
#define FUZZY_GROUNDSPEED_H

#include "variables_settings.h"

// Function declarations
int membershipSpeedNL(float error);
int membershipSpeedNS(float error);
int membershipSpeedZE(float error);
int membershipSpeedPS(float error);
int membershipSpeedPL(float error);
int calculateGroundSpeedFuzzy(float speedError);

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

#endif // FUZZY_GROUNDSPEED_H