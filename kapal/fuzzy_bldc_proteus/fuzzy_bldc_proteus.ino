// Program kontrol motor DC dengan fuzzy logic
// untuk Arduino Nano dengan PWM 0-255

const int motorPin = 9;        // Pin PWM untuk motor DC pada Arduino Nano
const int sensorPin = A0;      // Pin sensor pada A0

// Konstanta untuk PWM motor DC
const int MIN_PWM = 0;         // PWM minimum (motor mati)
const int START_PWM = 50;      // PWM minimum untuk mulai berputar
const int MAX_PWM = 255;       // PWM maksimum
const int STEP_SIZE = 5;       // Kenaikan kecil untuk kontrol manual

// Variabel untuk PWM
int currentPWM = MIN_PWM;

// Variabel untuk pembacaan RPM
int rpm = 0;
unsigned long last_millis = 0;
const int interval = 50;      // Interval pembacaan RPM (ms)
bool lastSensorState = false; // Untuk mencatat state terakhir sensor

// Variabel untuk kontrol RPM
int targetRPM = 0;
int currentRPM = 0;
int lastRPM = 0;
int rpmError = 0;
int lastError = 0;
int rpmErrorChange = 0;
unsigned long lastRPMCheck = 0;
const unsigned long RPM_CHECK_INTERVAL = 100; // Check RPM every 100ms
bool rpmControlActive = false;

// Konstanta untuk fuzzy logic
const int ERROR_SMALL = 100;
const int ERROR_MEDIUM = 300;
const int ERROR_BIG = 700;
const int DELTA_SMALL = 50;
const int DELTA_MEDIUM = 150;
const int DELTA_BIG = 300;

void setup() {
  // Mulai komunikasi serial
  Serial.begin(9600);
  
  Serial.println("DC Motor Control System dengan Fuzzy Logic");
  
  // Setup pin sensor
  pinMode(sensorPin, INPUT);
  
  // Setup pin motor dan LED built-in
  pinMode(motorPin, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);

  // Set motor ke kondisi awal
  analogWrite(motorPin, MIN_PWM);
  
  // Konfirmasi siap
  digitalWrite(LED_BUILTIN, HIGH);
  delay(200);
  digitalWrite(LED_BUILTIN, LOW);
  delay(200);
  digitalWrite(LED_BUILTIN, HIGH);
  delay(200);
  digitalWrite(LED_BUILTIN, LOW);

  Serial.println("Sistem siap. Gunakan perintah berikut melalui Serial:");
  Serial.println("'s' - Mulai dengan kecepatan terendah");
  Serial.println("'+' - Tingkatkan kecepatan");
  Serial.println("'-' - Kurangi kecepatan");
  Serial.println("'x' - Stop motor");
  Serial.println("'p' - Tampilkan nilai PWM dan RPM saat ini");
  Serial.println("'rXXXX' - Set target RPM (contoh: r1000 untuk 1000 RPM)");
  Serial.println("'r0' - Matikan kontrol RPM");
}

void loop() {
  // Baca sensor RPM - deteksi setiap putaran (perubahan dari <500 ke >500)
  bool currentSensorState = analogRead(sensorPin) > 500;
  
  // Jika terdeteksi perubahan dari LOW ke HIGH (1 putaran)
  if (currentSensorState && !lastSensorState) {
    rpm++;
  }
  
  // Simpan status sensor saat ini untuk putaran berikutnya
  lastSensorState = currentSensorState;

  // Update RPM setiap interval
  if (millis() - last_millis > interval) {
    last_millis = millis();
    
    // Simpan RPM terakhir
    lastRPM = currentRPM;
    
    // Hitung RPM aktual (konversi dari frekuensi pembacaan)
    // Faktor 60000/interval adalah untuk konversi ke RPM (60 detik * 1000 ms / interval ms)
    currentRPM = rpm * (60000 / interval);
    
    // Reset counter RPM
    rpm = 0;
    
    // Print info RPM
    Serial.print("50 ");
    Serial.print(currentRPM);
    Serial.println(" 1");
  }
  
  // Kontrol RPM dengan Fuzzy Logic jika aktif
  if (rpmControlActive && targetRPM > 0) {
    if (millis() - lastRPMCheck > RPM_CHECK_INTERVAL) {
      lastRPMCheck = millis();
      
      // Hitung error dan perubahan error
      lastError = rpmError;
      rpmError = targetRPM - currentRPM;
      rpmErrorChange = rpmError - lastError;
      
      // Terapkan kontrol fuzzy
      int pwmChange = applyFuzzyControl(rpmError, rpmErrorChange);
      
      // Terapkan perubahan PWM dengan batas
      if (pwmChange != 0) {
        int newPWM = currentPWM + pwmChange;
        newPWM = constrain(newPWM, START_PWM, MAX_PWM);
        
        if (newPWM != currentPWM) {
          currentPWM = newPWM;
          analogWrite(motorPin, currentPWM);
        }
      }
      
      Serial.print("Target: ");
      Serial.print(targetRPM);
      Serial.print(" RPM, Current: ");
      Serial.print(currentRPM);
      Serial.print(" RPM, Error: ");
      Serial.print(rpmError);
      Serial.print(", Change: ");
      Serial.print(pwmChange);
      Serial.print(", PWM: ");
      Serial.println(currentPWM);
    }
  }

  // Cek input serial untuk kontrol manual
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    
    // Periksa jika ini adalah perintah set RPM
    if (input.startsWith("r")) {
      // Parse nilai RPM target
      targetRPM = input.substring(1).toInt();
      
      if (targetRPM > 0) {
        rpmControlActive = true;
        // Reset error tracking
        rpmError = 0;
        lastError = 0;
        rpmErrorChange = 0;
        
        Serial.print("Mode kontrol RPM aktif. Target: ");
        Serial.println(targetRPM);
        
        // Jika motor masih mati, hidupkan dengan START_PWM
        if (currentPWM <= MIN_PWM) {
          currentPWM = START_PWM;
          analogWrite(motorPin, currentPWM);
        }
      } else {
        rpmControlActive = false;
        Serial.println("Mode kontrol RPM dimatikan.");
      }
    }
    else if (input.length() == 1) {
      char command = input.charAt(0);
      
      // Matikan mode RPM control jika ada perintah manual
      // kecuali perintah 'p'
      if (command != 'p') {
        rpmControlActive = false;
      }

      // Eksekusi perintah
      switch (command) {
        case 's':  // Start
          currentPWM = START_PWM;
          Serial.print("Mulai dengan PWM: ");
          Serial.println(currentPWM);
          break;

        case '+':  // Increase speed
          if (currentPWM < MAX_PWM) {
            currentPWM += STEP_SIZE;
            Serial.print("PWM +: ");
            Serial.println(currentPWM);
          } else {
            Serial.println("Sudah pada PWM maksimum!");
          }
          break;

        case '-':  // Decrease speed
          if (currentPWM > START_PWM) {
            currentPWM -= STEP_SIZE;
            Serial.print("PWM -: ");
            Serial.println(currentPWM);
          } else {
            Serial.println("Sudah pada PWM minimum!");
          }
          break;

        case 'x':  // Stop
          currentPWM = MIN_PWM;
          Serial.println("STOP - Motor dimatikan");
          break;

        case 'p':  // Print current values
          Serial.print("Nilai PWM saat ini: ");
          Serial.println(currentPWM);
          Serial.print("RPM saat ini: ");
          Serial.println(currentRPM);
          if (rpmControlActive) {
            Serial.print("Target RPM: ");
            Serial.print(targetRPM);
            Serial.print(", Error: ");
            Serial.println(rpmError);
          }
          break;
      }

      // Terapkan PWM baru
      analogWrite(motorPin, currentPWM);
    }
  }

  // Flash LED untuk indikasi program berjalan
  static unsigned long lastBlink = 0;
  if (millis() - lastBlink > 1000) {
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    lastBlink = millis();
  }
}

// Fungsi fuzzy logic untuk kontrol RPM
int applyFuzzyControl(int error, int errorChange) {
  // Fuzzy logic untuk mengontrol motor DC
  // Berdasarkan error (selisih RPM) dan errorChange (perubahan error)
  
  // 1. Absolut error dan errorChange
  int absError = abs(error);
  int absErrorChange = abs(errorChange);
  
  // 2. Fuzzy membership untuk error
  float smallError = 0.0, mediumError = 0.0, bigError = 0.0;
  
  // Small error membership
  if (absError <= ERROR_SMALL) {
    smallError = 1.0;
  } else if (absError <= ERROR_MEDIUM) {
    smallError = (float)(ERROR_MEDIUM - absError) / (ERROR_MEDIUM - ERROR_SMALL);
  } else {
    smallError = 0.0;
  }
  
  // Medium error membership
  if (absError <= ERROR_SMALL) {
    mediumError = 0.0;
  } else if (absError <= ERROR_MEDIUM) {
    mediumError = (float)(absError - ERROR_SMALL) / (ERROR_MEDIUM - ERROR_SMALL);
  } else if (absError <= ERROR_BIG) {
    mediumError = (float)(ERROR_BIG - absError) / (ERROR_BIG - ERROR_MEDIUM);
  } else {
    mediumError = 0.0;
  }
  
  // Big error membership
  if (absError <= ERROR_MEDIUM) {
    bigError = 0.0;
  } else if (absError <= ERROR_BIG) {
    bigError = (float)(absError - ERROR_MEDIUM) / (ERROR_BIG - ERROR_MEDIUM);
  } else {
    bigError = 1.0;
  }
  
  // 3. Fuzzy membership untuk errorChange
  float smallDelta = 0.0, mediumDelta = 0.0, bigDelta = 0.0;
  
  // Small delta membership
  if (absErrorChange <= DELTA_SMALL) {
    smallDelta = 1.0;
  } else if (absErrorChange <= DELTA_MEDIUM) {
    smallDelta = (float)(DELTA_MEDIUM - absErrorChange) / (DELTA_MEDIUM - DELTA_SMALL);
  } else {
    smallDelta = 0.0;
  }
  
  // Medium delta membership
  if (absErrorChange <= DELTA_SMALL) {
    mediumDelta = 0.0;
  } else if (absErrorChange <= DELTA_MEDIUM) {
    mediumDelta = (float)(absErrorChange - DELTA_SMALL) / (DELTA_MEDIUM - DELTA_SMALL);
  } else if (absErrorChange <= DELTA_BIG) {
    mediumDelta = (float)(DELTA_BIG - absErrorChange) / (DELTA_BIG - DELTA_MEDIUM);
  } else {
    mediumDelta = 0.0;
  }
  
  // Big delta membership
  if (absErrorChange <= DELTA_MEDIUM) {
    bigDelta = 0.0;
  } else if (absErrorChange <= DELTA_BIG) {
    bigDelta = (float)(absErrorChange - DELTA_MEDIUM) / (DELTA_BIG - DELTA_MEDIUM);
  } else {
    bigDelta = 1.0;
  }
  
  // 4. Fuzzy rules dan inferensi
  // Rule 1: Jika error kecil dan delta kecil, maka perubahan kecil
  float rule1 = min(smallError, smallDelta);
  
  // Rule 2: Jika error sedang dan delta kecil, maka perubahan sedang
  float rule2 = min(mediumError, smallDelta);
  
  // Rule 3: Jika error besar, maka perubahan besar
  float rule3 = bigError;
  
  // Rule 4: Jika error meningkat cepat (delta besar), kurangi perubahan
  float rule4 = bigDelta;
  
  // 5. Defuzzification (weighted average)
  float smallChange = 1.0;
  float mediumChange = 3.0;
  float bigChange = 5.0;
  float reduceFactor = 0.5;
  
  float pwmChangeValue = 
    (rule1 * smallChange + 
     rule2 * mediumChange + 
     rule3 * bigChange) / 
    (rule1 + rule2 + rule3 + 0.001);  // Hindari divisi dengan 0
    
  // Kurangi perubahan jika errorChange besar
  pwmChangeValue = pwmChangeValue * (1.0 - rule4 * reduceFactor);
  
  // 6. Hasil akhir (bulat ke integer)
  int pwmChange = round(pwmChangeValue);
  
  // Arah perubahan
  if (error < 0) {
    // RPM terlalu tinggi, kurangi PWM
    pwmChange = -pwmChange;
  }
  
  return pwmChange;
}