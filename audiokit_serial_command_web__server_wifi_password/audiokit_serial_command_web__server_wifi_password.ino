/**
 * @file esp-recorder.ino
 * @brief ESP Recorder program with folder management, file deletion, and memory capacity check.
 */

#include "AudioTools.h"
#include "AudioLibs/AudioBoardStream.h"
#include <SPI.h>
#include <SD.h>
#include <vector>
#include <algorithm>
#include "web.h"

AudioInfo info(24000, 1, 16);
AudioBoardStream kit(AudioKitEs8388V2);
File file;

StreamCopy copier;  // copies data
bool isRecording = false;
bool isPlaying = false;
bool isBackup = false;

void setup() {
  Serial.begin(115200);
  AudioLogger::instance().begin(Serial, AudioLogger::Warning);

  // Setup input and output: setup audiokit before SD!
  auto cfg = kit.defaultConfig(RXTX_MODE);
  cfg.sd_active = true;
  cfg.copyFrom(info);
  cfg.input_device = ADC_INPUT_LINE2;
  kit.begin(cfg);
  kit.setVolume(1.0);

  // Open SD drive
  if (!SD.begin(PIN_AUDIO_KIT_SD_CARD_CS)) {
    Serial.println("SD_FAIL");
    while (1)
      ;
  }
  Serial.println("SD_OK");
  setupWebServer();

  Serial.println("ESP Recorder ready. Waiting for commands...");
}

void loop() {
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    handleCommand(command);
  }

  if (isRecording || isPlaying) {
    copier.copy();
  }

  if (isPlaying && file.available() == 0) {
    stopPlayback();
    Serial.println("FINISHED");
  }
  if (isBackup) {
    loopWebServer();
  }
}

void handleCommand(String command) {
  if (command.startsWith("RECORD_")) {
    startRecording(command.substring(7));
  } else if (command == "STOP_RECORD") {
    stopRecording();
  } else if (command.startsWith("PLAY_")) {
    startPlayback(command.substring(5));
  } else if (command == "STOP_PLAY") {
    stopPlayback();
  } else if (command.startsWith("LIST_")) {
    listFiles(command.substring(5));
  } else if (command.startsWith("DELETE_")) {
    deleteFile(command.substring(7));
  } else if (command == "CHECK_MEMORY") {
    checkMemoryCapacity();
  } else if (command == "BACKUP") {
    // checkMemoryCapacity();
    isBackup=true;
    Serial.println("BACKUP START");
  } else if (command == "STOP_BACKUP") {
    // checkMemoryCapacity();
    isBackup=false;
    Serial.println("BACKUP STOP");
  }
}

void startRecording(String filename) {
  if (isRecording || isPlaying) {
    Serial.println("BUSY");
    return;
  }

  // Extract folder name from filename (assuming format: /YYYYMMDD/X.wav)
  int slashIndex = filename.indexOf('/', 1);
  if (slashIndex == -1) {
    Serial.println("INVALID_FILENAME");
    return;
  }
  String folderName = filename.substring(0, slashIndex);

  // Create folder if it doesn't exist
  if (!SD.exists(folderName)) {
    if (!SD.mkdir(folderName)) {
      Serial.println("FOLDER_CREATE_FAIL");
      return;
    }
  }

  file = SD.open(filename, FILE_WRITE);
  if (!file) {
    Serial.println("FAIL_OPEN");
    return;
  }

  writeWavHeader(file, info);
  copier.begin(file, kit);
  isRecording = true;
  Serial.println("REC_START");
}

void stopRecording() {
  if (!isRecording) {
    Serial.println("NOT_RECORDING");
    return;
  }

  copier.end();
  updateWavHeader(file);
  file.close();
  isRecording = false;
  Serial.println("REC_STOP");
}

void startPlayback(String filename) {
  if (isRecording || isPlaying) {
    Serial.println("BUSY");
    return;
  }

  file = SD.open(filename);
  if (!file) {
    Serial.println("FAIL_OPEN");
    return;
  }

  file.seek(44);  // Skip WAV header
  copier.begin(kit, file);
  isPlaying = true;
  Serial.println("PLAY_START");
}

void stopPlayback() {
  if (!isPlaying) {
    Serial.println("NOT_PLAYING");
    return;
  }

  copier.end();
  file.close();
  playHening();
  isPlaying = false;
  Serial.println("PLAY_STOP");
}

void playHening() {
  File heningFile = SD.open("/hening.wav");
  if (!heningFile) {
    Serial.println("HENING_NOT_FOUND");
    return;
  }

  heningFile.seek(44);  // Skip WAV header
  StreamCopy heningCopier;
  heningCopier.begin(kit, heningFile);

  unsigned long startTime = millis();
  while (millis() - startTime < 1000 && heningFile.available()) {
    heningCopier.copy();
  }

  heningCopier.end();
  heningFile.close();
}

struct FileInfo {
  String name;
  bool isDirectory;
  time_t lastModified;
};

bool shouldHide(const String& fileName) {
  return fileName == "hening.wav" || fileName == "System Volume Information";
}

void listFiles(String path) {
  File dir = SD.open(path);
  if (!dir) {
    Serial.println("FAIL_DIR");
    return;
  }
  if (!dir.isDirectory()) {
    Serial.println("NOT_DIR");
    return;
  }

  if (path != "/") {
    int lastSlash = path.lastIndexOf('/');
    String parentDir = path.substring(0, lastSlash);
    if (parentDir.isEmpty()) parentDir = "/";
    Serial.println("../");
  }

  std::vector<String> entries;
  File file = dir.openNextFile();
  while (file) {
    String fileName = String(file.name());
    if (fileName != "hening.wav" && fileName != "System Volume Information") {
      entries.push_back(fileName);
    }
    file = dir.openNextFile();
  }

  std::sort(entries.begin(), entries.end(), std::greater<String>());

  for (const auto& fileName : entries) {
    String fullPath = path + (path.endsWith("/") ? "" : "/") + fileName;
    File file = SD.open(fullPath);
    if (file.isDirectory()) {
      Serial.print(fileName);
      Serial.println("/");
    } else if (fileName.endsWith(".wav")) {
      String fileSize = String(file.size());
      Serial.println(fileName);
    }
    file.close();
  }

  Serial.println("END_OF_LIST");
  dir.close();
}


void deleteFile(String filename) {
  if (SD.remove(filename)) {
    Serial.println("FILE_DELETED");
  } else {
    Serial.println("DELETE_FAIL");
  }
}

// ... (kode sebelumnya tetap sama)

String findOldestFile(const char* dirPath) {
  File dir = SD.open(dirPath);
  if (!dir || !dir.isDirectory()) {
    return "";
  }

  String oldestFileName = "";
  uint32_t oldestTime = 0xFFFFFFFF;  // Initialize with maximum value

  File entry;
  while (entry = dir.openNextFile()) {
    if (entry.isDirectory()) {
      // Recursive call for subdirectories
      String subDirOldest = findOldestFile(entry.name());
      if (subDirOldest != "") {
        File subFile = SD.open(subDirOldest, FILE_READ);
        if (subFile) {
          time_t lastWrite = subFile.getLastWrite();
          if (lastWrite < oldestTime) {
            oldestFileName = subDirOldest;
            oldestTime = lastWrite;
          }
          subFile.close();
        }
      }
    } else if (String(entry.name()) != "hening.wav") {
      time_t lastWrite = entry.getLastWrite();
      if (lastWrite < oldestTime) {
        oldestFileName = String(dirPath) + "/" + String(entry.name());
        oldestTime = lastWrite;
      }
    }
    entry.close();
  }
  dir.close();

  return oldestFileName;
}

void checkMemoryCapacity() {
  uint64_t totalBytes = SD.totalBytes();
  uint64_t usedBytes = SD.usedBytes();
  uint64_t freeBytes = totalBytes - usedBytes;
  float usagePercentage = (float)usedBytes / totalBytes * 100;

  Serial.print("TOTAL_BYTES:");
  Serial.println(totalBytes);
  Serial.print("USED_BYTES:");
  Serial.println(usedBytes);
  Serial.print("FREE_BYTES:");
  Serial.println(freeBytes);
  Serial.print("USAGE_PERCENTAGE:");
  Serial.print(usagePercentage, 2);  // Menampilkan dengan 2 angka desimal
  Serial.println("%");

  String oldestFile = findOldestFile("/");
  if (oldestFile != "") {
    Serial.print("OLDEST_FILE:");
    Serial.println(oldestFile);
  } else {
    Serial.println("NO_FILES_FOUND");
  }
}

// ... (sisa kode tetap sama)

void writeWavHeader(File& file, AudioInfo& info) {
  file.write((const uint8_t*)"RIFF", 4);
  uint32_t fileSize = 0;
  file.write((const uint8_t*)&fileSize, 4);
  file.write((const uint8_t*)"WAVE", 4);
  file.write((const uint8_t*)"fmt ", 4);
  uint32_t subchunk1Size = 16;
  file.write((const uint8_t*)&subchunk1Size, 4);
  uint16_t audioFormat = 1;
  file.write((const uint8_t*)&audioFormat, 2);
  file.write((const uint8_t*)&info.channels, 2);
  file.write((const uint8_t*)&info.sample_rate, 4);
  uint32_t byteRate = info.sample_rate * info.channels * (info.bits_per_sample / 8);
  file.write((const uint8_t*)&byteRate, 4);
  uint16_t blockAlign = info.channels * (info.bits_per_sample / 8);
  file.write((const uint8_t*)&blockAlign, 2);
  file.write((const uint8_t*)&info.bits_per_sample, 2);
  file.write((const uint8_t*)"data", 4);
  uint32_t dataSize = 0;
  file.write((const uint8_t*)&dataSize, 4);
}

void updateWavHeader(File& file) {
  uint32_t fileSize = file.size() - 8;
  uint32_t dataSize = fileSize - 44;

  file.seek(4);
  file.write((const uint8_t*)&fileSize, 4);

  file.seek(40);
  file.write((const uint8_t*)&dataSize, 4);
}