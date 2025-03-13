// Sistem Fuzzy PID untuk Kontrol Kecepatan Boat tanpa library eksternal
// Menggunakan metode Fuzzy untuk tuning parameter PID

// Variabel input dan output
float targetSpeed = 0.0;  // Target kecepatan boat
float actualSpeed = 0.0;  // Kecepatan aktual boat
float error = 0.0;        // Error = target - actual
float prevError = 0.0;    // Error sebelumnya untuk perhitungan derivative
float sumError = 0.0;     // Akumulasi error untuk perhitungan integral
float engineRPM = 0.0;    // Output RPM engine (kurang agresif)
float bldcRPM = 0.0;      // Output RPM BLDC (lebih agresif)

// Parameter PID dasar
float Kp_base = 10.0;     // Konstanta Proportional
float Ki_base = 0.5;      // Konstanta Integral
float Kd_base = 5.0;      // Konstanta Derivative

// Parameter PID yang akan di-tuning oleh fuzzy
float Kp = 0.0;
float Ki = 0.0;
float Kd = 0.0;

// Variabel untuk membaca input serial
String inputString = "";
bool stringComplete = false;

// Variabel waktu untuk perhitungan PID
unsigned long currentTime = 0;
unsigned long previousTime = 0;
float deltaTime = 0.0;

// Variabel untuk debug
bool debugMode = true;    // Set true untuk melihat nilai keanggotaan

// Konstanta untuk output base
const float ENGINE_BASE_RPM = 1500.0;
const float BLDC_BASE_RPM = 2500.0;

void setup() {
  // Inisialisasi komunikasi serial
  Serial.begin(9600);
  inputString.reserve(200);

  // Pesan selamat datang
  Serial.println("Sistem Kontrol Kecepatan Boat dengan Fuzzy PID");
  Serial.println("Format input: target,actual");
  Serial.println("Contoh: 50,45");
  Serial.println("--------------------------------------------------");
}

void loop() {
  // Jika ada input lengkap dari serial
  if (stringComplete) {
    parseInput();
    
    // Hitung waktu delta untuk perhitungan PID
    currentTime = millis();
    deltaTime = (currentTime - previousTime) / 1000.0; // Konversi ke detik
    
    // Jika ini adalah pengukuran pertama atau delta waktu sangat kecil, set nilai simulasi
    if (previousTime == 0 || deltaTime < 0.001) {
      // Dalam mode simulasi/testing, gunakan nilai default 0.1 detik
      deltaTime = 0.1;
    }
    previousTime = currentTime;

    // Hitung error
    prevError = error;
    error = targetSpeed - actualSpeed;
    
    // Batasi akumulasi sumError untuk menghindari integral windup
    sumError += error * deltaTime;
    sumError = constrain(sumError, -50, 50);
    
    // Proses fuzzy untuk menentukan parameter PID
    tuningFuzzyPID();
    
    // Hitung output PID
    calculatePIDOutputs();
    
    // Output hasil
    Serial.print("Target: ");
    Serial.print(targetSpeed);
    Serial.print(" km/h | Aktual: ");
    Serial.print(actualSpeed);
    Serial.print(" km/h | Error: ");
    Serial.print(error);
    Serial.println(" km/h");
    
    Serial.print("Fuzzy PID Parameters - Kp: ");
    Serial.print(Kp);
    Serial.print(" | Ki: ");
    Serial.print(Ki);
    Serial.print(" | Kd: ");
    Serial.println(Kd);
    
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

// Fungsi keanggotaan trapezoidal (bentuk trapesium)
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
  return trapezoidMF(error, 5, 10, 100, 100);
}

// Fungsi keanggotaan untuk perubahan error (untuk Derivative)
float errorChangeRate() {
  // Dalam mode testing, gunakan nilai error langsung sebagai rate jika deltaTime sangat kecil
  if (deltaTime < 0.001) {
    // Simulasi error rate berdasarkan besarnya error saat ini
    // Semakin besar error, semakin cepat perubahan yang disimulasikan
    return error * 0.5; // 50% dari error sebagai simulasi rate
  }
  return (error - prevError) / deltaTime;
}

// Proses fuzzy untuk tuning parameter PID
void tuningFuzzyPID() {
  // Menghitung derajat keanggotaan untuk input error
  float negLargeDegree = negLargeMF(error);
  float negSmallDegree = negSmallMF(error);
  float zeroDegree = zeroMF(error);
  float posSmallDegree = posSmallMF(error);
  float posLargeDegree = posLargeMF(error);
  
  // Hitung rate of change dari error untuk tuning Kd
  float errorRate = errorChangeRate();
  
  // Tampilkan nilai keanggotaan jika dalam debug mode
  if (debugMode) {
    // Serial.println("Fuzzy Membership Values:");
    // Serial.print("NegLarge: "); Serial.print(negLargeDegree);
    // Serial.print(" | NegSmall: "); Serial.print(negSmallDegree);
    // Serial.print(" | Zero: "); Serial.print(zeroDegree);
    // Serial.print(" | PosSmall: "); Serial.print(posSmallDegree);
    // Serial.print(" | PosLarge: "); Serial.println(posLargeDegree);
    // Serial.print("Error Rate: "); Serial.println(errorRate);
  }
  
  // Rules untuk tuning Kp:
  // - Error besar (positif atau negatif): Kp besar
  // - Error kecil: Kp kecil
  float Kp_neg_large = 1.5;  // Multiplikator untuk Kp saat error negatif besar
  float Kp_neg_small = 1.2;  // Multiplikator untuk Kp saat error negatif kecil
  float Kp_zero = 1.0;       // Multiplikator untuk Kp saat error sekitar nol
  float Kp_pos_small = 1.2;  // Multiplikator untuk Kp saat error positif kecil
  float Kp_pos_large = 1.5;  // Multiplikator untuk Kp saat error positif besar
  
  // Rules untuk tuning Ki:
  // - Error kecil: Ki besar (untuk mempercepat menuju setpoint)
  // - Error besar: Ki kecil (untuk menghindari overshoot)
  float Ki_neg_large = 0.7;  // Multiplikator untuk Ki saat error negatif besar
  float Ki_neg_small = 1.2;  // Multiplikator untuk Ki saat error negatif kecil
  float Ki_zero = 1.5;       // Multiplikator untuk Ki saat error sekitar nol
  float Ki_pos_small = 1.2;  // Multiplikator untuk Ki saat error positif kecil
  float Ki_pos_large = 0.7;  // Multiplikator untuk Ki saat error positif besar
  
  // Rules untuk tuning Kd:
  // - Error berubah cepat: Kd besar (untuk mengurangi overshoot)
  // - Error berubah lambat: Kd kecil
  float Kd_multiplier = 1.0;
  
  // Tambahkan penambahan Kd berdasarkan error rate dan besarnya error
  if (abs(errorRate) > 10 || abs(error) > 15) {
    Kd_multiplier = 1.8;  // Perubahan cepat atau error besar, perbesar Kd signifikan
  } else if (abs(errorRate) > 5 || abs(error) > 8) {
    Kd_multiplier = 1.4;  // Perubahan sedang atau error menengah
  } else if (abs(error) > 3) {
    Kd_multiplier = 1.2;  // Error kecil tapi signifikan
  }
  
  // Metode Weighted Average untuk tuning parameter
  float totalWeight = negLargeDegree + negSmallDegree + zeroDegree + posSmallDegree + posLargeDegree;
  
  // Hindari pembagian dengan nol
  if (totalWeight > 0) {
    // Weighted average untuk Kp
    Kp = Kp_base * (negLargeDegree * Kp_neg_large +
                    negSmallDegree * Kp_neg_small +
                    zeroDegree * Kp_zero +
                    posSmallDegree * Kp_pos_small +
                    posLargeDegree * Kp_pos_large) / totalWeight;
    
    // Weighted average untuk Ki
    Ki = Ki_base * (negLargeDegree * Ki_neg_large +
                    negSmallDegree * Ki_neg_small +
                    zeroDegree * Ki_zero +
                    posSmallDegree * Ki_pos_small +
                    posLargeDegree * Ki_pos_large) / totalWeight;
    
    // Kd berdasarkan rate of change dari error
    Kd = Kd_base * Kd_multiplier;
  } else {
    // Default values jika tidak ada rule yang aktif
    Kp = Kp_base;
    Ki = Ki_base;
    Kd = Kd_base;
    
    if (debugMode) {
      Serial.println("WARNING: No active rules! Using default PID values.");
    }
  }
  
  if (debugMode) {
    // Serial.println("PID Tuning Results:");
    // Serial.print("Kp Base: "); Serial.print(Kp_base);
    // Serial.print(" | Tuned Kp: "); Serial.println(Kp);
    // Serial.print("Ki Base: "); Serial.print(Ki_base);
    // Serial.print(" | Tuned Ki: "); Serial.println(Ki);
    // Serial.print("Kd Base: "); Serial.print(Kd_base);
    // Serial.print(" | Tuned Kd: "); Serial.println(Kd);
  }
}

// Hitung output PID berdasarkan parameter yang di-tuning oleh fuzzy
void calculatePIDOutputs() {
  // Hindari deltaTime terlalu kecil atau nol
  if (deltaTime < 0.001) {
    deltaTime = 0.001;
  }
  
  // Hitung komponen PID
  float P = Kp * error;
  float I = Ki * sumError;
  
  // Untuk komponen D, gunakan simulasi derivative saat testing
  float errorRate;
  if (deltaTime < 0.01) { // Kondisi testing/simulasi
    // Simulasi derivative berdasarkan error
    errorRate = error * 0.5; // 50% dari error sebagai simulasi rate
  } else {
    errorRate = (error - prevError) / deltaTime;
  }
  float D = Kd * errorRate;
  
  if (debugMode) {
    // Serial.println("PID Components:");
    // Serial.print("P: "); Serial.print(P);
    // Serial.print(" | I: "); Serial.print(I);
    // Serial.print(" | D: "); Serial.println(D);
  }
  
  // Hitung output PID untuk engine (kurang agresif)
  float pidOutput = P + I + D;
  
  if (debugMode) {
    Serial.print("Final PID Output: "); Serial.println(pidOutput);
  }
  
  // Mapping output PID ke RPM dengan skala yang lebih sesuai
  // Skala untuk engine (kurang agresif)
  float engineScale = 5.0;
  // Skala untuk BLDC (lebih agresif)
  float bldcScale = 10.0;
  
  // Engine (kurang agresif) - base ENGINE_BASE_RPM
  engineRPM = ENGINE_BASE_RPM + pidOutput * engineScale;
  
  // BLDC (lebih agresif) - base BLDC_BASE_RPM, dengan respons lebih tajam
  bldcRPM = BLDC_BASE_RPM + pidOutput * bldcScale;
  
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
    
    // Reset sumError jika target berubah untuk menghindari integral buildup
    if (targetSpeed != targetSpeed) {
      sumError = 0;
    }
  } else {
    Serial.println("Format input salah! Gunakan format: target,actual");
  }
}