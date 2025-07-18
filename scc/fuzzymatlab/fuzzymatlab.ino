/*
 * Simple Fuzzy Logic Function for Solar Charge Controller
 * Input: fuzzy(currentError, voltageError)
 * Output: deltaPWM value
 * 
 * Serial input format: currentError,voltageError
 * Example: 0.1,0.2
 */

// Struktur untuk membership function triangular
struct TriangleMF {
  float a, b, c;  // Parameter segitiga (a=kiri, b=puncak, c=kanan)
};

// Membership functions untuk Current Error
TriangleMF currentErrorMF[5] = {
  {-2.0, -2.0, -0.5},  // NB (Negative Big)
  {-0.5, -0.1, 0.0},   // NS (Negative Small)
  {-0.1, 0.0, 0.1},    // ZE (Zero)
  {0.0, 0.1, 0.5},     // PS (Positive Small)
  {0.5, 2.0, 2.0}      // PB (Positive Big)
};

// Membership functions untuk Voltage Error
TriangleMF voltageErrorMF[3] = {
  {-0.5, -0.5, 0.05},  // OverLimit (tegangan melebihi/sangat dekat batas)
  {0.0, 0.1, 0.4},     // NearLimit (tegangan mendekati batas)
  {0.3, 1.5, 1.5}      // SafeRange (tegangan masih aman dari batas)
};

// Output singletons untuk Sugeno
float outputSingletons[5] = {-10, -2, 0, 2, 8};

// Fungsi untuk menghitung derajat keanggotaan triangular
float triangularMF(float x, TriangleMF mf) {
  if (x <= mf.a || x >= mf.c) {
    return 0.0;
  } else if (x >= mf.a && x <= mf.b) {
    if (mf.b == mf.a) return 1.0;
    return (x - mf.a) / (mf.b - mf.a);
  } else {
    if (mf.c == mf.b) return 1.0;
    return (mf.c - x) / (mf.c - mf.b);
  }
}

// Fungsi fuzzy logic utama
float fuzzy(float currentError, float voltageError) {
  // Fuzzifikasi Current Error
  float currentMu[5];
  for (int i = 0; i < 5; i++) {
    currentMu[i] = triangularMF(currentError, currentErrorMF[i]);
  }
  
  // Fuzzifikasi Voltage Error
  float voltageMu[3];
  for (int i = 0; i < 3; i++) {
    voltageMu[i] = triangularMF(voltageError, voltageErrorMF[i]);
  }
  
  // Rule base (15 rules)
  // Format: [current_error_index][voltage_error_index] = output_index
  // Output: 0=DecreaseBig(-10), 1=DecreaseSmall(-2), 2=Hold(0), 3=IncreaseSmall(2), 4=IncreaseBig(8)
  int ruleBase[5][3] = {
    {0, 0, 0},  // NB current + any voltage: Selalu decrease big (arus terlalu tinggi)
    {0, 1, 1},  // NS current: decrease big jika over limit, decrease small jika near/safe
    {1, 2, 3},  // ZE current: tergantung voltage (decrease small, hold, increase small)
    {1, 3, 3},  // PS current: konservatif, hanya increase small
    {2, 3, 4}   // PB current: hold jika over limit, increase small/big jika aman
  };
  
  // Evaluasi rules dan agregasi
  float totalWeight = 0.0;
  float weightedSum = 0.0;
  
  for (int i = 0; i < 5; i++) {
    for (int j = 0; j < 3; j++) {
      // Strength rule (minimum)
      float ruleStrength = min(currentMu[i], voltageMu[j]);
      
      if (ruleStrength > 0) {
        int outputIndex = ruleBase[i][j];
        float outputValue = outputSingletons[outputIndex];
        
        weightedSum += ruleStrength * outputValue;
        totalWeight += ruleStrength;
      }
    }
  }
  
  // Defuzzifikasi (weighted average untuk Sugeno)
  float deltaOutput = 0.0;
  if (totalWeight > 0) {
    deltaOutput = weightedSum / totalWeight;
  }
  
  return deltaOutput;
}

void setup() {
  Serial.begin(115200);
  Serial.println("=== Fuzzy Logic Test Cases ===");
  
  // 10 uji coba acak
  float testCases[10][2] = {
    {0.1, 0.2},    // Test case 1
    {-0.4, 0.05},  // Test case 2
    {0.6, 0.1},    // Test case 3
    {-0.8, -0.1},  // Test case 4
    {1.2, 0.8},    // Test case 5
    {0.0, 0.0},    // Test case 6
    {-0.2, 0.3},   // Test case 7
    {0.8, -0.2},   // Test case 8
    {0.05, 0.15},  // Test case 9
    {1.5, 1.0}     // Test case 10
  };
  
  for (int i = 0; i < 10; i++) {
    float currentError = testCases[i][0];
    float voltageError = testCases[i][1];
    float deltaPWM = fuzzy(currentError, voltageError);
    
    Serial.print("Test ");
    Serial.print(i + 1);
    Serial.print(" - Error Arus: ");
    Serial.print(currentError, 2);
    Serial.print(", Error Tegangan: ");
    Serial.print(voltageError, 2);
    Serial.print(", Delta PWM: ");
    Serial.println(deltaPWM, 2);
  }
  
  Serial.println("=================================");
  Serial.println("Manual Input Ready");
  Serial.println("Format: currentError,voltageError");
  Serial.println("Example: 0.1,0.2");
}

void loop() {
  if (Serial.available()) {
    String input = Serial.readString();
    input.trim();
    
    // Parse input (currentError,voltageError)
    int commaIndex = input.indexOf(',');
    if (commaIndex > 0) {
      float currentError = input.substring(0, commaIndex).toFloat();
      float voltageError = input.substring(commaIndex + 1).toFloat();
      
      // Hitung hasil fuzzy
      float result = fuzzy(currentError, voltageError);
      
      // Print hasil sesuai format MATLAB
      Serial.println(result);
    } else {
      Serial.println("Error: Use format currentError,voltageError");
    }
  }
}