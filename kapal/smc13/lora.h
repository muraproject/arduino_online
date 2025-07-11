#ifndef COMMUNICATION_H
#define COMMUNICATION_H

#include "variables_settings.h"

// Function declarations
bool checkI2CDevice(uint8_t address);
void resetI2C();
void handleRemoteControl();
void setRemoteThrottle(int throttle);
void setRemoteRudder(int angle);
void processLoraCommand(const char* command);
void reportStatusToLora();
void sendHeartbeat();
uint8_t calculateChecksum(const char* data, int length);
void sendSimpleData();
void reportThrottle();

// Function implementations - using global variables from variable_definitions.h
bool checkI2CDevice(uint8_t address) {
  Wire.beginTransmission(address);
  byte error = Wire.endTransmission();
  return (error == 0);
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

// Function declaration for reportThrottle (used in serial_commands.h)
void reportThrottle() {
  Serial.print("Throttle positions: Main1=");
  Serial.print(currentThrottle1);
  Serial.print("us, Main2=");
  Serial.print(currentThrottle2);
  Serial.print("us, Third=");
  Serial.print(currentThrottle3_deg);
  Serial.println("°");
}

#endif // COMMUNICATION_H