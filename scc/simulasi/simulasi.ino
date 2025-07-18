/*
 * PURE FUZZY LOGIC IMPLEMENTATION
 * Sugeno Fuzzy Inference System untuk PWM Control
 * Input: Current Error | Output: Delta PWM
 */

// Fungsi fuzzy logic pure - hanya current error input
int fuzzyPWMControl(float currentError) {
  // Fungsi keanggotaan fuzzy untuk current error
  float NB = 0.0, NS = 0.0, ZE = 0.0, PS_val = 0.0, PB_val = 0.0;
  
  // Fuzzifikasi error dengan rentang yang sensitif
  if (currentError <= -0.5) {
    NB = 1.0;  // Error negatif besar - kurangi PWM
  } else if (currentError > -0.5 && currentError < -0.1) {
    NB = (-0.1 - currentError) / 0.4;
    NS = (currentError + 0.5) / 0.4;
  } else if (currentError >= -0.1 && currentError < 0) {
    NS = (-currentError) / 0.1;
    ZE = (currentError + 0.1) / 0.1;
  } else if (currentError >= 0 && currentError < 0.1) {
    ZE = (0.1 - currentError) / 0.1;
    PS_val = currentError / 0.1;
  } else if (currentError >= 0.1 && currentError < 0.5) {
    PS_val = (0.5 - currentError) / 0.4;
    PB_val = (currentError - 0.1) / 0.4;
  } else {  // currentError >= 0.5
    PB_val = 1.0;  // Error positif besar - naikkan PWM agresif
  }
  
  // Output singleton untuk Sugeno
  int NBOutput = -10;   // Decrease PWM by 10
  int NSOutput = -1;    // Decrease PWM by 1
  int ZEOutput = 0;     // No change
  int PSOutput = 1;     // Increase PWM by 1
  int PBOutput = 10;    // Increase PWM by 10
  
  // Defuzzifikasi (weighted average - Sugeno)
  float totalWeight = NB + NS + ZE + PS_val + PB_val;
  
  if (totalWeight > 0) {
    int deltaOutput = (int)((NB * NBOutput + NS * NSOutput + ZE * ZEOutput + PS_val * PSOutput + PB_val * PBOutput) / totalWeight);
    
    // Special rule: Jika masih di bawah target dan error positif, minimal naik 1
    if (currentError > 0.05 && deltaOutput == 0) {
      deltaOutput = 1;  // Pastikan selalu naik jika target belum tercapai
    }
    
    return deltaOutput;
  }
  
  return 0; // Jika tidak ada aturan yang terpicu
}

// Fungsi untuk menampilkan detail membership values
void showMembershipValues(float currentError) {
  float NB = 0.0, NS = 0.0, ZE = 0.0, PS_val = 0.0, PB_val = 0.0;
  
  // Sama dengan fuzzifikasi di fuzzyPWMControl
  if (currentError <= -0.5) {
    NB = 1.0;
  } else if (currentError > -0.5 && currentError < -0.1) {
    NB = (-0.1 - currentError) / 0.4;
    NS = (currentError + 0.5) / 0.4;
  } else if (currentError >= -0.1 && currentError < 0) {
    NS = (-currentError) / 0.1;
    ZE = (currentError + 0.1) / 0.1;
  } else if (currentError >= 0 && currentError < 0.1) {
    ZE = (0.1 - currentError) / 0.1;
    PS_val = currentError / 0.1;
  } else if (currentError >= 0.1 && currentError < 0.5) {
    PS_val = (0.5 - currentError) / 0.4;
    PB_val = (currentError - 0.1) / 0.4;
  } else {
    PB_val = 1.0;
  }
  
  // Serial.println("=== MEMBERSHIP VALUES ===");
  // Serial.println("NB (Negative Big): " + String(NB, 3));
  // Serial.println("NS (Negative Small): " + String(NS, 3));
  // Serial.println("ZE (Zero): " + String(ZE, 3));
  // Serial.println("PS (Positive Small): " + String(PS_val, 3));
  // Serial.println("PB (Positive Big): " + String(PB_val, 3));
  // Serial.println("Total Weight: " + String(NB + NS + ZE + PS_val + PB_val, 3));
  
  // Tampilkan weighted calculation
  int NBOutput = -10, NSOutput = -1, ZEOutput = 0, PSOutput = 1, PBOutput = 10;
  float weightedSum = NB * NBOutput + NS * NSOutput + ZE * ZEOutput + PS_val * PSOutput + PB_val * PBOutput;
  float totalWeight = NB + NS + ZE + PS_val + PB_val;
  
  Serial.println("\n=== DEFUZZIFICATION ===");
  Serial.println("Weighted Sum: " + String(weightedSum, 3));
  Serial.println("Total Weight: " + String(totalWeight, 3));
  if (totalWeight > 0) {
    Serial.println("Delta Output: " + String(weightedSum / totalWeight, 3));
    // Serial.println("Rounded Output: " + String((int)(weightedSum / totalWeight)));
  }
}

// Fungsi test dengan detail output
void testFuzzyLogic(float currentError) {
  Serial.println("\n" + String("=").substring(0, 50));
  Serial.println("FUZZY LOGIC TEST");
  Serial.println("Current Error: " + String(currentError, 3) + " A");
  Serial.println(String("-").substring(0, 50));
  
  // Tampilkan membership values
  showMembershipValues(currentError);
  
  // Jalankan fuzzy logic
  int deltaOutput = fuzzyPWMControl(currentError);
  
  // Serial.println("\n=== HASIL AKHIR ===");
  // Serial.println("Delta PWM: " + String(deltaOutput));
  
  // Analisis hasil
  if (deltaOutput > 0) {
    Serial.println("Action: INCREASE PWM");
  } else if (deltaOutput < 0) {
    Serial.println("Action: DECREASE PWM");
  } else {
    Serial.println("Action: NO CHANGE");
  }
  
  // Check special rule
  if (currentError > 0.05 && deltaOutput == 1) {
    Serial.println("Note: Special rule applied (force +1)");
  }
  
  Serial.println(String("=").substring(0, 50));
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("===============================================");
  Serial.println("       PURE FUZZY LOGIC SIMULATOR");
  Serial.println("    Sugeno Fuzzy Inference System");
  Serial.println("===============================================");
  Serial.println("Input: Current Error (Ampere)");
  Serial.println("Output: Delta PWM");
  Serial.println("");
  Serial.println("MEMBERSHIP FUNCTIONS:");
  Serial.println("NB: currentError <= -0.5");
  Serial.println("NS: -0.5 < currentError < -0.1");
  Serial.println("ZE: -0.1 <= currentError < 0.1");
  Serial.println("PS: 0.1 <= currentError < 0.5");
  Serial.println("PB: currentError >= 0.5");
  Serial.println("");
  Serial.println("SINGLETON OUTPUTS:");
  Serial.println("NB → -10 | NS → -1 | ZE → 0 | PS → +1 | PB → +10");
  Serial.println("");
  Serial.println("SPECIAL RULE:");
  Serial.println("If currentError > 0.05 & output = 0 → force +1");
  Serial.println("===============================================");
  
  // Test cases predefined
  Serial.println("\n>>> RUNNING PREDEFINED TESTS <<<");
  
  testFuzzyLogic(4.0);    // PB case
  testFuzzyLogic(-2.0);   // NB case
  testFuzzyLogic(0.3);    // PS case
  testFuzzyLogic(-0.3);   // NS case
  testFuzzyLogic(0.0);    // ZE case
  testFuzzyLogic(0.07);   // Special rule case
  testFuzzyLogic(0.25);   // Mixed PS/PB case
  testFuzzyLogic(-0.25);  // Mixed NS/ZE case
  
  Serial.println("\n===============================================");
  Serial.println("Masukkan Current Error di Serial Monitor:");
  Serial.println("Format: angka (contoh: 4.0, -2.5, 0.3)");
  Serial.println("===============================================");
}

void loop() {
  // Interactive testing via Serial Monitor
  if (Serial.available() > 0) {
    float inputError = Serial.parseFloat();
    
    // Clear buffer
    while (Serial.available() > 0) {
      Serial.read();
    }
    
    // Test the input
    testFuzzyLogic(inputError);
    
    Serial.println("\nMasukkan Current Error berikutnya:");
  }
  
  delay(100);
}

/*
 * EXPECTED RESULTS:
 * 
 * Input: 4.0    → PB=1.0 → Delta=+10
 * Input: -2.0   → NB=1.0 → Delta=-10
 * Input: 0.3    → PS=0.5, PB=0.5 → Delta=+5.5≈+6
 * Input: -0.3   → NS=1.0 → Delta=-1
 * Input: 0.0    → ZE=1.0 → Delta=0
 * Input: 0.07   → ZE=0.3, PS=0.7 → Delta=+0.7≈+1 (or special rule)
 * Input: 0.25   → PS=0.625, PB=0.375 → Delta=+4.375≈+4
 * Input: -0.25  → NS=0.25, ZE=0.75 → Delta=-0.25≈0
 */