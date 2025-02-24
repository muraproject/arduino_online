/**
 * @file streams-audiokit-sd-audiokit.ino
 * @author Phil Schatzmann
 * @brief We record the input from the microphone to a file and constantly repeat to play the file
 * The input is triggered by pressing key 1. Recording stops when key 1 is released!
 * @version 0.1
 * @date 2022-09-01
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#include "AudioTools.h"
#include "AudioLibs/AudioBoardStream.h"
#include <SPI.h>
#include <SD.h>

const char *file_name = "/rec.wav";
AudioInfo info(20000, 1, 16);
AudioBoardStream kit(AudioKitEs8388V2);
File file;   // final output stream
StreamCopy copier; // copies data
bool recording = false;  // flag to make sure that close is only executed one
uint64_t end_time; // trigger to call endRecord
 
void record_start(bool pinStatus, int pin, void* ref){
  Serial.println("Recording...");
  // open the output file
  file = SD.open(file_name, FILE_WRITE);
  
  // Write WAV header
  writeWavHeader(file, info);
  
  copier.begin(file, kit);  
  recording = true; 
}

void record_end(bool pinStatus, int pin, void* ref){
  if (recording == true){
    Serial.println("Playing...");
    
    // Update WAV header with final file size
    updateWavHeader(file);
    
    file.close();
    recording = false;
    file = SD.open(file_name); // reopen in read mode
    copier.begin(kit, file);  // start playback
  }
}

void setup(){
  Serial.begin(115200);
  while(!Serial); // wait for serial to be ready
  AudioLogger::instance().begin(Serial, AudioLogger::Warning);  

  // setup input and output: setup audiokit before SD!
  auto cfg = kit.defaultConfig(RXTX_MODE);
  cfg.sd_active = true;
  cfg.copyFrom(info);
  cfg.input_device = ADC_INPUT_LINE2;
  kit.begin(cfg);
  kit.setVolume(1.0);

  // Open SD drive
  if (!SD.begin(PIN_AUDIO_KIT_SD_CARD_CS)) {
    Serial.println("Initialization failed!");
    while (1); // stop
  }
  Serial.println("Initialization done.");

  // record when key 1 is pressed
  kit.audioActions().add(kit.getKey(1), record_start, record_end);
  Serial.println("Press Key 1 to record");
}

void loop(){
  // record or play file
  copier.copy();  

  // while playing: at end of file -> reposition to beginning 
  if (!recording && file.size()>0 && file.available()==0){
      file.seek(44); // Skip WAV header
      Serial.println("Replay...");
  }

  // Process keys
  kit.processActions();
}

// Function to write WAV header
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

// Function to update WAV header with final file size
void updateWavHeader(File &file) {
  uint32_t fileSize = file.size() - 8;
  uint32_t dataSize = fileSize - 44;
  
  file.seek(4);
  file.write((const uint8_t*)&fileSize, 4);
  
  file.seek(40);
  file.write((const uint8_t*)&dataSize, 4);
}