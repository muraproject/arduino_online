#include "AudioTools.h"
#include "AudioLibs/AudioBoardStream.h"
#include <SPI.h>
#include <SD.h>

const char *raw_file_name = "/rec.wav";
const char *compressed_file_name = "/compressed.txt";
const char *decompressed_file_name = "/decompressed.wav";
AudioInfo info(16000, 1, 16);  // Mono, 16kHz, 16-bit
AudioBoardStream kit(AudioKitEs8388V2);
File file;
StreamCopy copier;
bool recording = false;

// Fungsi kompresi sederhana
void compressWav(const char* inputFile, const char* outputFile) {
  File input = SD.open(inputFile, FILE_READ);
  File output = SD.open(outputFile, FILE_WRITE);
  
  if (!input || !output) {
    Serial.println("Error opening files for compression");
    return;
  }
  
  // Lewati header WAV
  input.seek(44);
  
  // Baca dan kompres data
  const int bufferSize = 1024;
  int16_t buffer[bufferSize];
  while (input.available()) {
    int bytesRead = input.read((uint8_t*)buffer, bufferSize * 2);
    int samplesRead = bytesRead / 2;
    
    for (int i = 0; i < samplesRead; i++) {
      // Kompresi sederhana: kurangi resolusi menjadi 8-bit
      uint8_t compressedSample = map(buffer[i], -32768, 32767, 0, 255);
      output.write(compressedSample);
    }
  }
  
  input.close();
  output.close();
  Serial.println("Compression completed");
}

// Fungsi dekompresi sederhana
void decompressWav(const char* inputFile, const char* outputFile) {
  File input = SD.open(inputFile, FILE_READ);
  File output = SD.open(outputFile, FILE_WRITE);
  
  if (!input || !output) {
    Serial.println("Error opening files for decompression");
    return;
  }
  
  // Tulis header WAV
  writeWavHeader(output, info);
  
  // Baca dan dekompres data
  uint8_t compressedSample;
  while (input.available()) {
    input.read(&compressedSample, 1);
    int16_t decompressedSample = map(compressedSample, 0, 255, -32768, 32767);
    output.write((uint8_t*)&decompressedSample, 2);
  }
  
  // Update header WAV
  updateWavHeader(output);
  
  input.close();
  output.close();
  Serial.println("Decompression completed");
}

void record_start(bool pinStatus, int pin, void* ref) {
  Serial.println("Recording...");
  file = SD.open(raw_file_name, FILE_WRITE);
  writeWavHeader(file, info);
  copier.begin(file, kit);  
  recording = true;
}

void record_end(bool pinStatus, int pin, void* ref) {
  if (recording) {
    Serial.println("Processing...");
    updateWavHeader(file);
    file.close();
    recording = false;
    
    // Kompres file WAV
    compressWav(raw_file_name, compressed_file_name);
    
    // Dekompres file
    decompressWav(compressed_file_name, decompressed_file_name);
    
    // Putar file yang sudah didekompres
    file = SD.open(decompressed_file_name);
    copier.begin(kit, file);
    Serial.println("Playing decompressed audio...");
  }
}

void setup() {
  Serial.begin(115200);
  AudioLogger::instance().begin(Serial, AudioLogger::Warning);

  auto cfg = kit.defaultConfig(RXTX_MODE);
  cfg.sd_active = true;
  cfg.copyFrom(info);
  cfg.input_device = ADC_INPUT_LINE2;
  kit.begin(cfg);
  kit.setVolume(1.0);

  if (!SD.begin(PIN_AUDIO_KIT_SD_CARD_CS)) {
    Serial.println("SD card initialization failed!");
    while (1);
  }
  Serial.println("SD card initialized.");

  kit.audioActions().add(kit.getKey(1), record_start, record_end);
  Serial.println("Press Key 1 to record");
}

void loop() {
  copier.copy();

  if (!recording && file.size() > 0 && file.available() == 0) {
    file.seek(44);
    Serial.println("Replay...");
  }

  kit.processActions();
}

// Fungsi untuk menulis header WAV
void writeWavHeader(File &file, AudioInfo &info) {
  file.write((const uint8_t*)"RIFF", 4);
  uint32_t fileSize = 0;  // Placeholder for file size
  file.write((const uint8_t*)&fileSize, 4);
  file.write((const uint8_t*)"WAVE", 4);
  file.write((const uint8_t*)"fmt ", 4);
  uint32_t subchunk1Size = 16;
  file.write((const uint8_t*)&subchunk1Size, 4);
  uint16_t audioFormat = 1;  // PCM
  file.write((const uint8_t*)&audioFormat, 2);
  file.write((const uint8_t*)&info.channels, 2);
  file.write((const uint8_t*)&info.sample_rate, 4);
  uint32_t byteRate = info.sample_rate * info.channels * (info.bits_per_sample / 8);
  file.write((const uint8_t*)&byteRate, 4);
  uint16_t blockAlign = info.channels * (info.bits_per_sample / 8);
  file.write((const uint8_t*)&blockAlign, 2);
  file.write((const uint8_t*)&info.bits_per_sample, 2);
  file.write((const uint8_t*)"data", 4);
  uint32_t dataSize = 0;  // Placeholder for data size
  file.write((const uint8_t*)&dataSize, 4);
}

// Fungsi untuk memperbarui header WAV dengan ukuran file akhir
void updateWavHeader(File &file) {
  uint32_t fileSize = file.size() - 8;
  uint32_t dataSize = fileSize - 44;
  
  file.seek(4);
  file.write((const uint8_t*)&fileSize, 4);
  
  file.seek(40);
  file.write((const uint8_t*)&dataSize, 4);
}