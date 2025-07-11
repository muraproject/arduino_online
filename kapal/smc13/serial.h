#ifndef SERIAL_COMMANDS_H
#define SERIAL_COMMANDS_H

#include "variables_settings.h"

// Function declarations
void processSerialCommands();

void processSerialCommands() {
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
}

#endif // SERIAL_COMMANDS_H