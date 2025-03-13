// Sistem Neural Network untuk Kontrol Kecepatan Boat dengan Training Data
// Neural network ditraining dari sample data yang disimpan di RAM

// Variabel input dan output
float targetSpeed = 0.0;  // Target kecepatan boat
float actualSpeed = 0.0;  // Kecepatan aktual boat
float prevActualSpeed = 0.0; // Kecepatan sebelumnya untuk menghitung perubahan
float engineRPM = 0.0;    // Output RPM engine (kurang agresif)
float bldcRPM = 0.0;      // Output RPM BLDC (lebih agresif)

// Variabel untuk membaca input serial
String inputString = "";
bool stringComplete = false;

// Variabel untuk debug dan training
bool debugMode = true;
bool trainingFinished = false;

// Variabel waktu untuk perhitungan delta
unsigned long currentTime = 0;
unsigned long previousTime = 0;
float deltaTime = 0.0;

// Konstanta untuk output base
const float ENGINE_BASE_RPM = 1500.0;
const float BLDC_BASE_RPM = 2500.0;

// Struktur Neural Network
const int INPUT_NEURONS = 3;     // [error, delta error, error integral]
const int HIDDEN_NEURONS = 5;    // Jumlah neuron di hidden layer
const int OUTPUT_NEURONS = 1;    // Output tunggal: respons kontrol

// Weights dan biases network (akan ditraining)
float weightsInputHidden[INPUT_NEURONS][HIDDEN_NEURONS];
float biasesHidden[HIDDEN_NEURONS];
float weightsHiddenOutput[HIDDEN_NEURONS][OUTPUT_NEURONS];
float biasesOutput[OUTPUT_NEURONS];

// Nilai untuk normalisasi input
const float ERROR_MAX = 30.0;         // Maximum error yang diharapkan (untuk normalisasi)
const float DELTA_ERROR_MAX = 10.0;   // Maximum delta error yang diharapkan
const float INTEGRAL_MAX = 50.0;      // Maximum nilai integral

// Variabel state network
float hiddenOutputs[HIDDEN_NEURONS];
float networkOutput = 0.0;

// Variable untuk integral error
float errorIntegral = 0.0;

// Learning rate untuk training
const float LEARNING_RATE = 0.1;
const int MAX_EPOCHS = 500;

// Data training yang disimpan di RAM
// Format: [error, deltaError, errorIntegral, expectedOutput]
const int TRAINING_SAMPLES = 15;
float trainingData[TRAINING_SAMPLES][4] = {
  // Error, DeltaError, ErrorIntegral, ExpectedOutput (dalam range -1 sampai 1)
  {-20.0, -5.0, -30.0, -0.9},  // Error besar negatif -> output minimal
  {-15.0, -3.0, -20.0, -0.8},  // Error sedang negatif, turun
  {-10.0, -2.0, -15.0, -0.6},  // Error kecil negatif, turun
  {-10.0, 2.0, -10.0, -0.4},   // Error kecil negatif, naik
  {-5.0, -1.0, -5.0, -0.3},    // Error sangat kecil negatif, turun
  {-5.0, 1.0, -3.0, -0.1},     // Error sangat kecil negatif, naik
  {0.0, 0.0, 0.0, 0.0},        // Tidak ada error -> maintain
  {0.0, 0.5, 2.0, 0.1},        // Tidak ada error tapi integral positif -> sedikit naikkan
  {0.0, -0.5, -2.0, -0.1},     // Tidak ada error tapi integral negatif -> sedikit turunkan
  {5.0, 1.0, 3.0, 0.3},        // Error kecil positif, naik
  {5.0, -1.0, 5.0, 0.2},       // Error kecil positif, turun
  {10.0, 2.0, 10.0, 0.6},      // Error sedang positif, naik
  {10.0, -2.0, 15.0, 0.4},     // Error sedang positif, turun
  {15.0, 3.0, 20.0, 0.8},      // Error besar positif, naik
  {20.0, 5.0, 30.0, 0.9}       // Error sangat besar positif -> output maksimal
};

void setup() {
  // Inisialisasi komunikasi serial
  Serial.begin(9600);
  inputString.reserve(200);

  // Pesan selamat datang
  Serial.println("Sistem Kontrol Kecepatan Boat dengan Neural Network + Training");
  Serial.println("Melakukan training dari sample data...");
  
  // Inisialisasi weights dan biases dengan nilai random kecil
  randomSeed(analogRead(0));
  initializeNetworkWeights();
  
  // Training network dari data sampel
  trainNetworkFromSamples();
  
  Serial.println("Training selesai!");
  Serial.println("Format input: target,actual");
  Serial.println("Contoh: 50,45");
  Serial.println("--------------------------------------------------");
  
  trainingFinished = true;
}

void loop() {
  // Jika ada input lengkap dari serial
  if (stringComplete) {
    parseInput();
    
    // Hitung waktu delta
    currentTime = millis();
    deltaTime = (currentTime - previousTime) / 1000.0; // Konversi ke detik
    
    // Jika ini adalah pengukuran pertama atau delta waktu sangat kecil, set nilai simulasi
    if (previousTime == 0 || deltaTime < 0.001) {
      deltaTime = 0.1; // Nilai default untuk testing
    }
    previousTime = currentTime;
    
    // Hitung error dan delta error
    float error = targetSpeed - actualSpeed;
    float deltaError = (error - (targetSpeed - prevActualSpeed)) / deltaTime;
    prevActualSpeed = actualSpeed;
    
    // Update integral error (dengan anti-windup)
    errorIntegral += error * deltaTime;
    errorIntegral = constrain(errorIntegral, -INTEGRAL_MAX, INTEGRAL_MAX);
    
    // Proses neural network
    processNeuralNetwork(error, deltaError, errorIntegral);
    
    // Mapping output network ke RPM
    mapNetworkOutputToRPM();
    
    // Output hasil
    Serial.print("Target: ");
    Serial.print(targetSpeed);
    Serial.print(" km/h | Aktual: ");
    Serial.print(actualSpeed);
    Serial.print(" km/h | Error: ");
    Serial.print(error);
    Serial.println(" km/h");
    
    Serial.print("Delta Error: ");
    Serial.print(deltaError);
    Serial.print(" | Error Integral: ");
    Serial.println(errorIntegral);
    
    Serial.print("Network Output: ");
    Serial.println(networkOutput);
    
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

// Inisialisasi weights dan biases dengan nilai random kecil
void initializeNetworkWeights() {
  // Weights dari input ke hidden
  for (int i = 0; i < INPUT_NEURONS; i++) {
    for (int j = 0; j < HIDDEN_NEURONS; j++) {
      weightsInputHidden[i][j] = (random(200) - 100) / 200.0; // Range -0.5 to 0.5
    }
  }
  
  // Biases hidden layer
  for (int i = 0; i < HIDDEN_NEURONS; i++) {
    biasesHidden[i] = (random(200) - 100) / 200.0; // Range -0.5 to 0.5
  }
  
  // Weights dari hidden ke output
  for (int i = 0; i < HIDDEN_NEURONS; i++) {
    for (int j = 0; j < OUTPUT_NEURONS; j++) {
      weightsHiddenOutput[i][j] = (random(200) - 100) / 200.0; // Range -0.5 to 0.5
    }
  }
  
  // Biases output layer
  for (int i = 0; i < OUTPUT_NEURONS; i++) {
    biasesOutput[i] = (random(200) - 100) / 200.0; // Range -0.5 to 0.5
  }
}

// Fungsi aktivasi sigmoid
float sigmoid(float x) {
  return 1.0 / (1.0 + exp(-x));
}

// Derivative sigmoid untuk backpropagation
float sigmoidDerivative(float x) {
  float sigValue = sigmoid(x);
  return sigValue * (1 - sigValue);
}

// Fungsi ReLU (Rectified Linear Unit)
float relu(float x) {
  return (x > 0) ? x : 0;
}

// Derivative ReLU untuk backpropagation
float reluDerivative(float x) {
  return (x > 0) ? 1 : 0;
}

// Fungsi tanh (Hyperbolic Tangent)
float tanh_activation(float x) {
  return tanh(x);
}

// Derivative tanh untuk backpropagation
float tanhDerivative(float x) {
  float tanhValue = tanh(x);
  return 1 - (tanhValue * tanhValue);
}

// Normalisasi input ke range [-1, 1]
float normalize(float value, float max_value) {
  return constrain(value / max_value, -1.0, 1.0);
}

// Denormalisasi dari [-1, 1] ke nilai asli
float denormalize(float normalized, float max_value) {
  return normalized * max_value;
}

// Proses neural network
void processNeuralNetwork(float error, float deltaError, float integral) {
  // Normalisasi input
  float normalizedError = normalize(error, ERROR_MAX);
  float normalizedDeltaError = normalize(deltaError, DELTA_ERROR_MAX);
  float normalizedIntegral = normalize(integral, INTEGRAL_MAX);
  
  // Log nilai input yang ternormalisasi jika dalam mode debug
  if (debugMode && trainingFinished) {
    Serial.println("Neural Network Inputs (Normalized):");
    Serial.print("Error: "); Serial.print(normalizedError);
    Serial.print(" | Delta Error: "); Serial.print(normalizedDeltaError);
    Serial.print(" | Integral: "); Serial.println(normalizedIntegral);
  }
  
  // Forward pass - Hidden layer
  for (int i = 0; i < HIDDEN_NEURONS; i++) {
    float sum = biasesHidden[i];
    sum += normalizedError * weightsInputHidden[0][i];
    sum += normalizedDeltaError * weightsInputHidden[1][i];
    sum += normalizedIntegral * weightsInputHidden[2][i];
    
    // Gunakan ReLU sebagai fungsi aktivasi untuk hidden layer
    hiddenOutputs[i] = relu(sum);
  }
  
  // Debug output hidden neurons
  if (debugMode && trainingFinished) {
    Serial.println("Hidden Layer Outputs:");
    for (int i = 0; i < HIDDEN_NEURONS; i++) {
      Serial.print("Neuron "); Serial.print(i); 
      Serial.print(": "); Serial.println(hiddenOutputs[i]);
    }
  }
  
  // Forward pass - Output layer
  float sum = biasesOutput[0];
  for (int i = 0; i < HIDDEN_NEURONS; i++) {
    sum += hiddenOutputs[i] * weightsHiddenOutput[i][0];
  }
  
  // Gunakan tanh sebagai fungsi aktivasi output untuk mendapatkan range [-1, 1]
  networkOutput = tanh_activation(sum);
  
  // Debug output nilai network
  if (debugMode && trainingFinished) {
    Serial.print("Raw Network Output: "); Serial.println(networkOutput);
  }
}

// Mapping output network ke RPM engine dan BLDC
void mapNetworkOutputToRPM() {
  // Network output dalam range [-1, 1]
  // Konversi ke range RPM yang sesuai
  
  // Untuk engine (kurang agresif)
  // Mapped dari [-1, 1] ke [800, 2500]
  float engineRange = 1700; // 2500 - 800
  float engineMin = 800;
  // Transformasi dari [-1, 1] ke [0, 1] lalu ke [800, 2500]
  float engineScale = (networkOutput + 1) / 2.0;
  engineRPM = engineMin + engineScale * engineRange;
  
  // Untuk BLDC (lebih agresif)
  // Mapped dari [-1, 1] ke [800, 5000]
  float bldcRange = 4200; // 5000 - 800
  float bldcMin = 800;
  // Transformasi dari [-1, 1] ke [0, 1] lalu ke [800, 5000]
  float bldcScale = (networkOutput + 1) / 2.0;
  bldcRPM = bldcMin + bldcScale * bldcRange;
  
  // Tambahan untuk membuat BLDC lebih responsif
  if (networkOutput > 0) {
    // Respons lebih agresif untuk akselerasi
    bldcRPM = bldcMin + bldcScale * bldcScale * bldcRange;
  }
  
  // Konstrain untuk memastikan dalam range yang aman
  engineRPM = constrain(engineRPM, 800, 2500);
  bldcRPM = constrain(bldcRPM, 800, 5000);
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
    
    // Reset integral jika target berubah signifikan
    if (abs(targetSpeed - prevActualSpeed) > 5) {
      errorIntegral = 0;
    }
  } else {
    Serial.println("Format input salah! Gunakan format: target,actual");
  }
}

// Fungsi untuk training network dari data sampel
void trainNetworkFromSamples() {
  float totalError = 0;
  float averageError = 1.0; // Set awal di atas threshold
  float errorThreshold = 0.001;
  int epoch = 0;
  
  Serial.println("Mulai training neural network...");
  
  // Train sampai error kecil atau mencapai max epochs
  while (averageError > errorThreshold && epoch < MAX_EPOCHS) {
    totalError = 0;
    
    // Iterasi melalui semua training samples
    for (int sample = 0; sample < TRAINING_SAMPLES; sample++) {
      float error = trainingData[sample][0];
      float deltaError = trainingData[sample][1];
      float integral = trainingData[sample][2];
      float expectedOutput = trainingData[sample][3];
      
      // Normalisasi input
      float normalizedError = normalize(error, ERROR_MAX);
      float normalizedDeltaError = normalize(deltaError, DELTA_ERROR_MAX);
      float normalizedIntegral = normalize(integral, INTEGRAL_MAX);
      
      // ---------- Forward Pass ----------
      
      // Hidden layer
      float hiddenInputs[HIDDEN_NEURONS];
      float hiddenOutputsForTraining[HIDDEN_NEURONS];
      
      for (int i = 0; i < HIDDEN_NEURONS; i++) {
        hiddenInputs[i] = biasesHidden[i];
        hiddenInputs[i] += normalizedError * weightsInputHidden[0][i];
        hiddenInputs[i] += normalizedDeltaError * weightsInputHidden[1][i];
        hiddenInputs[i] += normalizedIntegral * weightsInputHidden[2][i];
        
        // ReLU activation
        hiddenOutputsForTraining[i] = relu(hiddenInputs[i]);
      }
      
      // Output layer
      float outputInput = biasesOutput[0];
      for (int i = 0; i < HIDDEN_NEURONS; i++) {
        outputInput += hiddenOutputsForTraining[i] * weightsHiddenOutput[i][0];
      }
      
      // Tanh activation for output
      float outputValue = tanh_activation(outputInput);
      
      // Calculate output error
      float outputError = expectedOutput - outputValue;
      totalError += abs(outputError);
      
      // ---------- Backward Pass (Backpropagation) ----------
      
      // Output layer gradients
      float outputDelta = outputError * tanhDerivative(outputInput);
      
      // Update output layer weights and bias
      for (int i = 0; i < HIDDEN_NEURONS; i++) {
        weightsHiddenOutput[i][0] += LEARNING_RATE * outputDelta * hiddenOutputsForTraining[i];
      }
      biasesOutput[0] += LEARNING_RATE * outputDelta;
      
      // Hidden layer gradients
      float hiddenDeltas[HIDDEN_NEURONS];
      
      for (int i = 0; i < HIDDEN_NEURONS; i++) {
        hiddenDeltas[i] = outputDelta * weightsHiddenOutput[i][0] * reluDerivative(hiddenInputs[i]);
        
        // Update input->hidden weights
        weightsInputHidden[0][i] += LEARNING_RATE * hiddenDeltas[i] * normalizedError;
        weightsInputHidden[1][i] += LEARNING_RATE * hiddenDeltas[i] * normalizedDeltaError;
        weightsInputHidden[2][i] += LEARNING_RATE * hiddenDeltas[i] * normalizedIntegral;
        
        // Update hidden biases
        biasesHidden[i] += LEARNING_RATE * hiddenDeltas[i];
      }
    }
    
    // Calculate average error for this epoch
    averageError = totalError / TRAINING_SAMPLES;
    
    // Print progress setiap 50 epochs
    if (epoch % 50 == 0 || epoch == MAX_EPOCHS - 1) {
      Serial.print("Epoch ");
      Serial.print(epoch);
      Serial.print(": Error rata-rata = ");
      Serial.println(averageError);
    }
    
    epoch++;
  }
  
  Serial.print("Training selesai setelah ");
  Serial.print(epoch);
  Serial.println(" epochs.");
  
  // Tampilkan weights akhir jika dalam mode debug
  if (debugMode) {
    Serial.println("Weights Input->Hidden akhir:");
    for (int i = 0; i < INPUT_NEURONS; i++) {
      for (int j = 0; j < HIDDEN_NEURONS; j++) {
        Serial.print(weightsInputHidden[i][j]);
        Serial.print("\t");
      }
      Serial.println();
    }
    
    Serial.println("Weights Hidden->Output akhir:");
    for (int i = 0; i < HIDDEN_NEURONS; i++) {
      Serial.println(weightsHiddenOutput[i][0]);
    }
  }
  
  // Test network dengan beberapa sampel
  testNetwork();
}

// Test network dengan data sampel setelah training
void testNetwork() {
  Serial.println("Test network setelah training:");
  
  // Test dengan beberapa sampel representatif dari data training
  int testSamples[] = {0, 6, 14}; // Sampel dengan error negatif besar, nol, dan positif besar
  
  for (int i = 0; i < 3; i++) {
    int sampleIdx = testSamples[i];
    float error = trainingData[sampleIdx][0];
    float deltaError = trainingData[sampleIdx][1];
    float integral = trainingData[sampleIdx][2];
    float expectedOutput = trainingData[sampleIdx][3];
    
    // Proses neural network
    processNeuralNetwork(error, deltaError, integral);
    
    Serial.print("Sample ");
    Serial.print(sampleIdx);
    Serial.print(" - Error: ");
    Serial.print(error);
    Serial.print(", Delta: ");
    Serial.print(deltaError);
    Serial.print(", Integral: ");
    Serial.print(integral);
    Serial.print(" | Expected: ");
    Serial.print(expectedOutput);
    Serial.print(", Actual: ");
    Serial.println(networkOutput);
  }
  
  Serial.println("--------------------------------------------------");
}