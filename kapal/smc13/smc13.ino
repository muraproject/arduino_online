/*
  Triple Control System with Neural Network
  
  File structure:
  - main.ino (this file) - Main Arduino sketch
  - variable_definitions.h - All variable definitions and instances
  - variables_settings.h - Variable declarations (extern) and constants
  - communication.h - LoRa and I2C communication functions
  - sensor_access.h - I2C sensor data access functions
  - fuzzy_bldc.h - Fuzzy control for BLDC motors 1&2
  - smc_bldc.h - SMC control for BLDC motors 1&2
  - fuzzy_hybrid_esc3.h - Fuzzy control for hybrid motor 3 (ESC3)
  - fuzzy_groundspeed.h - Fuzzy ground speed control
  - pid_groundspeed.h - PID ground speed control
  - fuzzy_pid_groundspeed.h - Fuzzy PID gain scheduling control
  - neural_network.h - Neural Network speed control
  - serial_commands.h - Serial command processing
  
  Control Systems Available:
  1. SMC (Sliding Mode Control) for BLDC motors
  2. Fuzzy Logic Control for BLDC motors
  3. Fuzzy Logic Control for ESC3 (hybrid motor)
  4. Ground Speed Fuzzy Control
  5. Ground Speed PID Control (Fixed gains)
  6. Ground Speed Fuzzy PID Control (Adaptive gains)
  7. Neural Network Speed Control (NEW!)
  
  Commands:
  - Serial: 's', 'x', '+', '-', 'p', 'i'
  - Control: 'mXXXX', 'fXXXX', 'tXXXX', 'gXX.X', 'pXX.X', 'dXX.X', 'nXX.X'
  - LoRa: T:xxx,A:xxx (throttle and rudder)
*/

#include <Servo.h>
#include <Wire.h>

// Include files in correct order
#include "variable_definitions.h"    // FIRST - defines all variables and instances
#include "variables_settings.h"      // SECOND - extern declarations for all variables

// Include all function implementations (order doesn't matter now)
#include "lora.h"           
#include "sensor.h"           
#include "fuzzy_bldc.h"             
#include "smc.h"               
#include "fuzzyEngine.h"      
#include "fuzzyHybrid.h"      
#include "pid.h"        
#include "fuzzypid.h"  
#include "nn.h"         
#include "serial.h"

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

  // Process serial commands
  processSerialCommands();

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