// Sistem Fuzzy untuk Kontrol Kecepatan Boat tanpa library eksternal
// Menggunakan metode Sugeno Orde 1 (output berupa fungsi linear)

// Variabel input dan output
float targetSpeed = 0.0;  // Target kecepatan boat
float actualSpeed = 0.0;  // Kecepatan aktual boat
float error = 0.0;        // Error = target - actual
float engineRPM = 0.0;    // Output RPM engine (kurang agresif)
float bldcRPM = 0.0;      // Output RPM BLDC (lebih agresif)

// Variabel untuk membaca input serial
String inputString = "";
bool stringComplete = false;

// Variabel untuk debug
bool debugMode = true;    // Set true untuk melihat nilai keanggotaan

void setup() {
  // Inisialisasi komunikasi serial
  Serial.begin(9600);
  inputString.reserve(200);

  // Pesan selamat datang
  Serial.println("Sistem Kontrol Kecepatan Boat dengan Fuzzy Logic Sugeno Orde 1");
  Serial.println("Format input: target,actual");
  Serial.println("Contoh: 50,45");
  Serial.println("--------------------------------------------------");
}

void loop() {
  // Jika ada input lengkap dari serial
  if (stringComplete) {
    parseInput();
    
    // Hitung error
    error = targetSpeed - actualSpeed;
    
    // Proses fuzzy
    calculateFuzzyOutputs();
    
    // Output hasil
    Serial.print("Target: ");
    Serial.print(targetSpeed);
    Serial.print(" km/h | Aktual: ");
    Serial.print(actualSpeed);
    Serial.print(" km/h | Error: ");
    Serial.print(error);
    Serial.println(" km/h");
    
    Serial.print("Output - Engine RPM: ");
    Serial.print(engineRPM);
    Serial.print(" | BLDC RPM: ");
    Serial.println(bldcRPM);
    Serial.println("--------------------------------------------------");
    
    // Reset untuk input berikutnya
    inputString = "";
    stringComplete = false;
  }
}

// Fungsi keanggotaan trapezoidal (bentuk trapesium) untuk error
float trapezoidMF(float x, float a, float b, float c, float d) {
  float result = 0;
  
  if (x <= a || x >= d) {
    result = 0;
  } else if (x >= b && x <= c) {
    result = 1;
  } else if (x > a && x < b) {
    result = (x - a) / (b - a);
  } else if (x > c && x < d) {
    result = (d - x) / (d - c);
  }
  
  return constrain(result, 0, 1);
}

// Fungsi keanggotaan untuk nilai error negatif besar
float negLargeMF(float error) {
  // Bentuk trapesium: nilai penuh di bawah -20, menurun hingga -10
  return trapezoidMF(error, -10, -100, -100, -5);
}

// Fungsi keanggotaan untuk nilai error negatif kecil
float negSmallMF(float error) {
  return trapezoidMF(error, -15, -10, -5, 0);
}

// Fungsi keanggotaan untuk nilai error sekitar nol
float zeroMF(float error) {
  return trapezoidMF(error, -5, -2, 2, 5);
}

// Fungsi keanggotaan untuk nilai error positif kecil
float posSmallMF(float error) {
  return trapezoidMF(error, 0, 5, 10, 15);
}

// Fungsi keanggotaan untuk nilai error positif besar
float posLargeMF(float error) {
  // Bentuk trapesium: nilai penuh di atas 20, menurun hingga 10
  return trapezoidMF(error, 5, 10, 100, 100);
}

// Fungsi output untuk Sugeno Orde 1
// Format: f(x) = a*x + b
float linearOutput(float x, float a, float b) {
  return a * x + b;
}

// Proses fuzzy dengan metode Sugeno Orde 1
void calculateFuzzyOutputs() {
  // Menghitung derajat keanggotaan untuk input error
  float negLargeDegree = negLargeMF(error);
  float negSmallDegree = negSmallMF(error);
  float zeroDegree = zeroMF(error);
  float posSmallDegree = posSmallMF(error);
  float posLargeDegree = posLargeMF(error);
  
  // Tampilkan nilai keanggotaan jika dalam debug mode
  if (debugMode) {
    Serial.println("Fuzzy Membership Values:");
    Serial.print("NegLarge: "); Serial.print(negLargeDegree);
    Serial.print(" | NegSmall: "); Serial.print(negSmallDegree);
    Serial.print(" | Zero: "); Serial.print(zeroDegree);
    Serial.print(" | PosSmall: "); Serial.print(posSmallDegree);
    Serial.print(" | PosLarge: "); Serial.println(posLargeDegree);
  }
  
  // Rules untuk Engine RPM (Kurang agresif) dengan output linear:
  // Rule 1: IF error is negativeLarge THEN engineRPM = -5*error + 800
  // Rule 2: IF error is negativeSmall THEN engineRPM = -10*error + 950
  // Rule 3: IF error is zero THEN engineRPM = 0*error + 1500
  // Rule 4: IF error is positiveSmall THEN engineRPM = 15*error + 1650
  // Rule 5: IF error is positiveLarge THEN engineRPM = 10*error + 2100
  
  float engineOut1 = linearOutput(error, -5, 800);   // Negative Large
  float engineOut2 = linearOutput(error, -10, 950);  // Negative Small
  float engineOut3 = linearOutput(error, 0, 1500);   // Zero
  float engineOut4 = linearOutput(error, 15, 1650);  // Positive Small
  float engineOut5 = linearOutput(error, 10, 2100);  // Positive Large
  
  // Rules untuk BLDC RPM (Lebih agresif) dengan output linear:
  // Rule 1: IF error is negativeLarge THEN bldcRPM = -80*error + 500
  // Rule 2: IF error is negativeSmall THEN bldcRPM = -120*error + 1200
  // Rule 3: IF error is zero THEN bldcRPM = 0*error + 2500
  // Rule 4: IF error is positiveSmall THEN bldcRPM = 150*error + 2700
  // Rule 5: IF error is positiveLarge THEN bldcRPM = 100*error + 3200
  
  float bldcOut1 = linearOutput(error, -80, 500);    // Negative Large
  float bldcOut2 = linearOutput(error, -120, 1200);  // Negative Small
  float bldcOut3 = linearOutput(error, 0, 2500);     // Zero
  float bldcOut4 = linearOutput(error, 150, 2700);   // Positive Small
  float bldcOut5 = linearOutput(error, 100, 3200);   // Positive Large
  
  if (debugMode) {
    Serial.println("Output Values Before Weighting:");
    Serial.print("Engine - NL: "); Serial.print(engineOut1);
    Serial.print(" | NS: "); Serial.print(engineOut2);
    Serial.print(" | Z: "); Serial.print(engineOut3);
    Serial.print(" | PS: "); Serial.print(engineOut4);
    Serial.print(" | PL: "); Serial.println(engineOut5);
    
    Serial.print("BLDC - NL: "); Serial.print(bldcOut1);
    Serial.print(" | NS: "); Serial.print(bldcOut2);
    Serial.print(" | Z: "); Serial.print(bldcOut3);
    Serial.print(" | PS: "); Serial.print(bldcOut4);
    Serial.print(" | PL: "); Serial.println(bldcOut5);
  }
  
  // Metode Sugeno Weighted Average
  float totalWeight = negLargeDegree + negSmallDegree + zeroDegree + posSmallDegree + posLargeDegree;
  
  if (debugMode) {
    Serial.print("Total Weight: "); Serial.println(totalWeight);
  }
  
  // Hindari pembagian dengan nol
  if (totalWeight > 0) {
    // Weighted average untuk engine RPM
    engineRPM = (negLargeDegree * engineOut1 +
                 negSmallDegree * engineOut2 +
                 zeroDegree * engineOut3 +
                 posSmallDegree * engineOut4 +
                 posLargeDegree * engineOut5) / totalWeight;
    
    // Weighted average untuk BLDC RPM
    bldcRPM = (negLargeDegree * bldcOut1 +
               negSmallDegree * bldcOut2 +
               zeroDegree * bldcOut3 +
               posSmallDegree * bldcOut4 +
               posLargeDegree * bldcOut5) / totalWeight;
  } else {
    // Default values jika tidak ada rule yang aktif
    engineRPM = 1500;
    bldcRPM = 2500;
    
    if (debugMode) {
      Serial.println("WARNING: No active rules! Using default values.");
    }
  }
  
  // Pastikan output berada dalam rentang yang wajar
  engineRPM = constrain(engineRPM, 800, 2500);
  bldcRPM = constrain(bldcRPM, 800, 5000);  // Memperluas batas atas BLDC untuk respons lebih agresif
}

// Fungsi untuk memproses input serial
void serialEvent() {
  while (Serial.available()) {
    char inChar = (char)Serial.read();
    
    // Jika karakter new line, tandai string sudah lengkap
    if (inChar == '\n') {
      stringComplete = true;
    } else {
      // Tambahkan ke inputString
      inputString += inChar;
    }
  }
}

// Fungsi untuk memproses input string
void parseInput() {
  int commaIndex = inputString.indexOf(',');
  
  if (commaIndex != -1) {
    // Ekstrak target dan actual speed
    String targetStr = inputString.substring(0, commaIndex);
    String actualStr = inputString.substring(commaIndex + 1);
    
    // Konversi ke float
    targetSpeed = targetStr.toFloat();
    actualSpeed = actualStr.toFloat();
    
    // Jika nilai kedua berisi 'debug', beralih mode debug
    if (actualStr.indexOf("debug") >= 0) {
      debugMode = !debugMode;
      Serial.print("Debug mode: ");
      Serial.println(debugMode ? "ON" : "OFF");
    }
  } else {
    Serial.println("Format input salah! Gunakan format: target,actual");
  }
}