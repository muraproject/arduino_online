#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <SPIFFS.h>
#include <U8g2lib.h>
#include <EEPROM.h> // Include EEPROM library

// OLED Display setup
U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/U8X8_PIN_NONE);

// WiFi Configuration
const char* ssid = "ESP32-Piano";
const char* password = "12345678";

// Pin GPIO untuk klakson/horn (adjusted to avoid conflicts with I2C pins)
const int hornPins[6] = { 32, 33, 25, 26, 27, 12 };                 // Changed from {5, 18, 19, 21, 22, 23} || { 13, 12, 14, 27, 26, 25 }
bool noteStates[6] = { false, false, false, false, false, false };  // Track note states

// Tombol kontrol
const int BUTTON_PREV = 5;
const int BUTTON_PLAY = 17;
const int BUTTON_STOP = 16;
const int BUTTON_NEXT = 4;

// EEPROM address for storing tempo
const int TEMPO_EEPROM_ADDR = 0;

// File management
int currentFileIndex = 0;
String fileList[20];  // Maximum 20 files
int totalFiles = 0;

// MIDI timing variables
unsigned long midiTick = 0;
unsigned long lastEventTime = 0;
static uint32_t userTempo = 900000;
uint32_t tempo = userTempo;
uint16_t ticksPerBeat = 480;

// Tempo modification variables
bool isTempoMode = false;
unsigned long lastTempoAdjustTime = 0;
const unsigned long tempoAdjustDelay = 200; // Delay between tempo adjustments

WebServer server(80);
File uploadFile;
File currentMidiFile;
bool isPlaying = false;
String currentFileName = "";

// Task handle untuk MIDI player
TaskHandle_t MidiPlayerTask;

void updateFileList() {
  totalFiles = 0;
  File root = SPIFFS.open("/");
  File file = root.openNextFile();

  while (file && totalFiles < 20) {
    String fileName = file.name();
    if (fileName.endsWith(".mid") || fileName.endsWith(".midi")) {
      if (fileName.startsWith("/")) {
        fileName = fileName.substring(1);
      }
      fileList[totalFiles] = fileName;
      totalFiles++;
    }
    file = root.openNextFile();
  }
  root.close();
}

void updateOLEDDisplay() {
  u8g2.clearBuffer();

  // Header kecil
  u8g2.setFont(u8g2_font_6x10_tf);
  u8g2.drawStr(2, 8, "ESP32 Piano");

  // Garis pemisah
  u8g2.drawHLine(0, 10, 128);

  // Area notasi yang lebih kecil
  const char* smallNotes[] = { "D", "R", "M", "F", "S", "L" };
  int noteX = 2;
  for (int i = 0; i < 6; i++) {
    if (noteStates[i]) {
      u8g2.drawBox(noteX, 13, 12, 12);
      u8g2.setDrawColor(0);
    } else {
      u8g2.drawFrame(noteX, 13, 12, 12);
      u8g2.setDrawColor(1);
    }
    u8g2.setCursor(noteX + 3, 22);
    u8g2.print(smallNotes[i]);
    u8g2.setDrawColor(1);
    noteX += 14;
  }

  // Kontrol buttons
  u8g2.drawFrame(2, 27, 20, 12);   // PREV
  u8g2.drawFrame(24, 27, 20, 12);  // PLAY
  u8g2.drawFrame(46, 27, 20, 12);  // STOP
  u8g2.drawFrame(68, 27, 20, 12);  // NEXT

  u8g2.setFont(u8g2_font_5x7_tf);
  u8g2.drawStr(4, 36, "<");    // PREV
  u8g2.drawStr(29, 36, ">");   // PLAY
  u8g2.drawStr(51, 36, "[]");  // STOP
  u8g2.drawStr(73, 36, ">");   // NEXT

  // Status dan nama file
  u8g2.setFont(u8g2_font_6x10_tf);
  u8g2.setCursor(0, 50);
  if (isPlaying && currentFileName != "") {
    String displayName = currentFileName;
    if (displayName.startsWith("/")) {
      displayName = displayName.substring(1);
    }
    if (displayName.length() > 16) {
      displayName = displayName.substring(0, 13) + "...";
    }
    u8g2.print("Now: ");
    u8g2.print(displayName);
  } else if (isTempoMode) {
    u8g2.print("Tempo: ");
    u8g2.print(userTempo);
  } else if (totalFiles > 0) {
    String displayName = fileList[currentFileIndex];
    if (displayName.length() > 16) {
      displayName = displayName.substring(0, 13) + "...";
    }
    u8g2.print("File: ");
    u8g2.print(displayName);
  } else {
    u8g2.print("No files");
  }

  // Status bar / Tempo indicator
  u8g2.drawFrame(0, 54, 128, 10);
  if (isPlaying) {
    int progress = 126;
    u8g2.drawBox(1, 55, progress, 8);
  } else {
    // Display current tempo value in status bar when not playing
    String tempoStr = "T:" + String(userTempo);
    u8g2.setFont(u8g2_font_5x7_tf);
    u8g2.drawStr(3, 61, tempoStr.c_str());
  }

  u8g2.sendBuffer();
}

void saveTempo() {
  // Save tempo to EEPROM
  EEPROM.put(TEMPO_EEPROM_ADDR, userTempo);
  EEPROM.commit();
}

void loadTempo() {
  // Load tempo from EEPROM
  EEPROM.get(TEMPO_EEPROM_ADDR, userTempo);
  
  // Check if loaded value is in valid range
  if (userTempo < 100000 || userTempo > 1500000) {
    userTempo = 900000; // Default if invalid value
  }
  
  tempo = userTempo;
}

void nextFile() {
  if (totalFiles > 0) {
    currentFileIndex = (currentFileIndex + 1) % totalFiles;
    updateOLEDDisplay();
  }
}

void prevFile() {
  if (totalFiles > 0) {
    currentFileIndex = (currentFileIndex - 1 + totalFiles) % totalFiles;
    updateOLEDDisplay();
  }
}



void handlePlayMidi() {
  // Handle manual note control
  if (server.hasArg("note") && server.hasArg("action")) {
    int noteIndex = server.arg("note").toInt();
    String action = server.arg("action");

    if (noteIndex >= 0 && noteIndex < 6) {
      if (action == "on") {
        digitalWrite(hornPins[noteIndex], HIGH);
        noteStates[noteIndex] = true;
        const int midiNotes[] = { 60, 62, 64, 65, 67, 69 };  // C, D, E, F, G, A
        Serial.printf("Horn %d ON (Note: %d)\n", noteIndex + 1, midiNotes[noteIndex]);
      } else if (action == "off") {
        digitalWrite(hornPins[noteIndex], LOW);
        noteStates[noteIndex] = false;
        const int midiNotes[] = { 60, 62, 64, 65, 67, 69 };  // C, D, E, F, G, A
        Serial.printf("Horn %d OFF (Note: %d)\n", noteIndex + 1, midiNotes[noteIndex]);
      }
      updateOLEDDisplay();
    }
    server.send(200, "text/plain", "OK");
    return;
  }

  // Handle MIDI file playback
  if (server.hasArg("file")) {
    currentFileName = server.arg("file");
    if (!currentFileName.startsWith("/")) {
      currentFileName = "/" + currentFileName;
    }
    if (server.hasArg("tempo")) {
      userTempo = server.arg("tempo").toInt();
      tempo = userTempo;
      saveTempo(); // Save tempo to EEPROM
    }
    isPlaying = true;
    updateOLEDDisplay();
    server.send(200, "text/plain", "Playing " + currentFileName);
  } else if (server.hasArg("tempo")) {
    userTempo = server.arg("tempo").toInt();
    tempo = userTempo;
    saveTempo(); // Save tempo to EEPROM
    server.send(200, "text/plain", "Tempo updated");
  } else {
    server.send(400, "text/plain", "No file specified");
  }
}

void handleStopMidi() {
  isPlaying = false;
  currentFileName = "";
  for (int i = 0; i < 6; i++) {
    noteStates[i] = false;
    digitalWrite(hornPins[i], LOW);
  }
  updateOLEDDisplay();
  server.send(200, "text/plain", "Playback stopped");
}

uint32_t readVarLen(File* file) {
  uint32_t value = 0;
  uint8_t byte;

  do {
    byte = file->read();
    value = (value << 7) | (byte & 0x7F);
  } while (byte & 0x80);

  return value;
}

void midiPlayerTask(void* parameter) {
  while (true) {
    if (isPlaying && currentFileName != "") {
      File midiFile = SPIFFS.open(currentFileName, "r");
      if (!midiFile) {
        Serial.println("Failed to open file for reading");
        isPlaying = false;
        updateOLEDDisplay();
        delay(10);
        continue;
      }

      // Skip MIDI header (MThd and length - total 14 bytes)
      for (int i = 0; i < 14; i++) {
        if (midiFile.available()) {
          midiFile.read();
        } else {
          Serial.println("Invalid MIDI file - header too short");
          break;
        }
      }

      lastEventTime = micros();
      tempo = userTempo;  // Use user-defined tempo if set

      while (midiFile.available() && isPlaying) {
        uint32_t deltaTime = readVarLen(&midiFile);

        // Handle timing
        if (deltaTime > 0) {
          unsigned long waitTime = (deltaTime * tempo) / ticksPerBeat;
          unsigned long targetTime = lastEventTime + waitTime;
          while (micros() < targetTime && isPlaying) {
            delay(1);
          }
          lastEventTime = targetTime;
        }

        // Read status byte
        uint8_t status = midiFile.read();

        if (status == 0xFF) {  // Meta event
          uint8_t type = midiFile.read();
          uint32_t len = readVarLen(&midiFile);

          if (type == 0x51 && len == 3) {  // Tempo change
            uint32_t newTempo = 0;
            for (int i = 0; i < 3; i++) {
              newTempo = (newTempo << 8) | midiFile.read();
            }
            // Only change tempo if user hasn't set a custom tempo
            if (tempo == 500000) {  // Default MIDI tempo
              tempo = newTempo;
              Serial.printf("MIDI Tempo changed to: %d\n", tempo);
            }
          } else {
            // Skip other meta events
            for (int i = 0; i < len; i++) {
              midiFile.read();
            }
          }
        } else if ((status & 0xF0) == 0x90 || (status & 0xF0) == 0x80) {  // Note events
          uint8_t note = midiFile.read();
          uint8_t velocity = midiFile.read();

          // Map MIDI note to horn index (0-5)
          int noteBase = note % 12;
          int hornIndex = -1;

          // Map notes to horns using Do-Re-Mi scale
          switch (noteBase) {
            case 0:  // C (Do)
              hornIndex = 0;
              break;
            case 2:  // D (Re)
              hornIndex = 1;
              break;
            case 4:  // E (Mi)
              hornIndex = 2;
              break;
            case 5:  // F (Fa)
              hornIndex = 3;
              break;
            case 7:  // G (Sol)
              hornIndex = 4;
              break;
            case 9:  // A (La)
              hornIndex = 5;
              break;
            default:
              hornIndex = -1;
              break;
          }

          if (hornIndex >= 0 && hornIndex < 6) {
            bool isNoteOn = (status & 0xF0) == 0x90 && velocity > 0;
            digitalWrite(hornPins[hornIndex], isNoteOn ? HIGH : LOW);
            noteStates[hornIndex] = isNoteOn;
            Serial.printf("Horn %d %s (Note: %d)\n",
                          hornIndex + 1,
                          isNoteOn ? "ON" : "OFF",
                          note);
            updateOLEDDisplay();
          }
        } else if ((status & 0xF0) == 0xB0) {  // Control Change
          // Skip two bytes (controller number and value)
          midiFile.read();
          midiFile.read();
        } else if ((status & 0xF0) == 0xC0 || (status & 0xF0) == 0xD0) {  // Program Change or Channel Pressure
          // Skip one byte
          midiFile.read();
        } else if ((status & 0xF0) == 0xE0) {  // Pitch Bend
          // Skip two bytes
          midiFile.read();
          midiFile.read();
        }
      }

      // Clean up after playback
      midiFile.close();

      // Reset all horns
      for (int i = 0; i < 6; i++) {
        digitalWrite(hornPins[i], LOW);
        noteStates[i] = false;
      }

      isPlaying = false;
      currentFileName = "";
      updateOLEDDisplay();
    }
    delay(10);  // Small delay to prevent watchdog timer issues
  }
}





void handleFileUpload() {
  HTTPUpload& upload = server.upload();

  if (upload.status == UPLOAD_FILE_START) {
    String filename = upload.filename;
    if (!filename.startsWith("/")) filename = "/" + filename;
    Serial.printf("handleFileUpload Name: %s\n", filename.c_str());
    uploadFile = SPIFFS.open(filename, "w");
  } else if (upload.status == UPLOAD_FILE_WRITE && uploadFile) {
    uploadFile.write(upload.buf, upload.currentSize);
  } else if (upload.status == UPLOAD_FILE_END) {
    uploadFile.close();
    updateFileList();     // Update file list after new upload
    updateOLEDDisplay();  // Update display to show new file
    server.sendHeader("Location", "/");
    server.send(303);
  }
}

void handleDelete() {
  if (server.hasArg("file")) {
    String fileName = server.arg("file");
    if (!fileName.startsWith("/")) fileName = "/" + fileName;
    if (SPIFFS.remove(fileName)) {
      updateFileList();     // Update file list after deletion
      updateOLEDDisplay();  // Update display
      server.sendHeader("Location", "/");
      server.send(303);
    } else {
      server.send(500, "text/plain", "Delete failed");
    }
  }
}

void handleFile() {
  if (server.hasArg("name")) {
    String fileName = server.arg("name");
    if (!fileName.startsWith("/")) fileName = "/" + fileName;
    if (SPIFFS.exists(fileName)) {
      File file = SPIFFS.open(fileName, "r");
      if (file) {
        server.streamFile(file, "application/octet-stream");
        file.close();
        return;
      }
    }
    server.send(404, "text/plain", "File not found");
  }
}

void handleRoot() {
  String html = "<!DOCTYPE html><html>";
  html += "<head><title>ESP32 Piano & MIDI Player</title>";
  html += "<meta name='viewport' content='width=device-width, initial-scale=1'>";
  html += "<style>";
  html += "body{font-family:Arial,sans-serif;margin:20px;background:#222;color:#eee}";
  html += ".container{max-width:800px;margin:0 auto;background:#333;padding:20px;border-radius:8px;box-shadow:0 2px 4px rgba(0,0,0,0.3);padding-top:0px}";
  html += "table{width:100%;border-collapse:collapse;margin-top:20px}";
  html += "th,td{padding:12px;text-align:left;border-bottom:1px solid #444}";
  html += "th{background-color:#4CAF50;color:white}";
  html += "tr:hover{background-color:#444}";
  html += "h3{color:#eee}";
  html += ".upload-form{margin:20px 0;padding:20px;background:#2a2a2a;border-radius:5px}";
  html += "button{background-color:#4CAF50;color:white;padding:10px 15px;border:none;border-radius:4px;cursor:pointer;margin-right:5px}";
  html += "button.delete{background-color:#f44336}";
  html += "button.play{background-color:#2196F3}";
  html += "button.stop{background-color:#ff9800}";
  html += "input[type='file']{background:#444;color:#fff;padding:8px;border-radius:4px;border:1px solid #555}";
  html += "input[type='number']{background:#444;color:#fff;border:1px solid #555}";
  html += "label{color:#eee}";

  // Piano styles
  html += ".piano-container{width:100%;max-width:600px;margin:0 auto;overflow-x:auto}";
  html += ".piano{position:relative;height:200px;background:#1a1a1a;padding:20px;border-radius:10px;box-shadow:0 5px 15px rgba(0,0,0,0.3)}";
  html += ".white-keys{display:flex;justify-content:space-between;height:100%;gap:4px}";
  html += ".white-key{flex:1;background:white;border:1px solid #ccc;border-radius:0 0 5px 5px;cursor:pointer;position:relative;transition:background 0.1s}";
  html += ".white-key:active,.white-key.active{background:#e6e6e6;transform:translateY(2px)}";
  html += ".white-key span{position:absolute;bottom:10px;left:50%;transform:translateX(-50%);font-size:14px;color:#666}";

  // Rainbow text and marquee styles
  html += ".rainbow-text {";
  html += "  font-size: 22px;";
  html += "  font-weight: bold;";
  html += "  padding: 10px 0;";
  html += "  text-align: center;";
  html += "  background: linear-gradient(to right, #ff0000, #ff7f00, #ffff00, #00ff00, #0000ff, #4b0082, #8b00ff);";
  html += "  background-size: 200% 100%;";
  html += "  -webkit-background-clip: text;";
  html += "  background-clip: text;";
  html += "  color: transparent;";
  html += "  animation: rainbow 6s linear infinite;";
  html += "}";
  html += "@keyframes rainbow {";
  html += "  0% { background-position: 0% 50%; }";
  html += "  100% { background-position: 100% 50%; }";
  html += "}";

  html += ".marquee-container {";
  html += "  overflow: hidden;";
  html += "  margin: 0;";
  html += "  padding: 0;";
  html += "  position: sticky;";
  html += "  top: 0;";
  html += "  z-index: 100;";
  html += "  background: #1a1a1a;";
  html += "  width: 100%;";
  html += "}";

  html += ".marquee {";
  html += "  white-space: nowrap;";
  html += "  animation: bounce-text 8s ease-in-out infinite alternate;";
  html += "  padding: 5px 0;";
  html += "  padding-top: 0px;";
  html += "  display: inline-block;";
  html += "  position: relative;";
  html += "  width: 100%;";
  html += "  text-align: center;";
  html += "}";

  html += "@keyframes bounce-text {";
  html += "  0%, 100% { transform: translateX(0); }";
  html += "  50% { transform: translateX(calc(100% - 300px)); }";
  html += "}";
  
  html += ".piano-border {";
  html += "  padding: 4px;";
  html += "  border-radius: 14px;";
  html += "  background: linear-gradient(-45deg, #ff0000, #ff7f00, #ffff00, #00ff00, #0000ff, #4b0082, #8b00ff);";
  html += "  background-size: 400% 400%;";
  html += "  animation: gradient 15s ease infinite;";
  html += "  margin-bottom: 15px;";
  html += "}";
  
  html += "@keyframes gradient {";
  html += "  0% { background-position: 0% 50%; }";
  html += "  50% { background-position: 100% 50%; }";
  html += "  100% { background-position: 0% 50%; }";
  html += "}";

  html += "@media (max-width: 600px) {";
  html += "  .piano{height:150px}";
  html += "  .white-key span{font-size:12px}";
  html += "}";
  
  html += "@media (orientation: landscape) {";
  html += "  .header-container {";
  html += "    position: fixed;";
  html += "    top: 0;";
  html += "    left: 0;";
  html += "    right: 0;";
  html += "    width: 100%;";
  html += "    z-index: 1000;";
  html += "  }";
  html += "  body {";
  html += "    padding-top: 40px;";
  html += "  }";
  html += "}";

  html += "</style></head><body><div class='container' style='padding-top: 0px;'>";

  // Rainbow text header
  html += "<div class='header-container'>";
  html += "<div class='rainbow-text'>TELOLET BASURI V1</div>";
  html += "</div>";

  // Piano interface
  html += "<div class='control-panel' style='margin: 20px 0; padding: 20px; background: #2a2a2a; border-radius: 5px; margin-top: 0px;'>";
  
  // Piano with animated colorful border
  html += "<div class='piano-container'>";
  html += "<div class='piano-border'>";
  html += "<div class='piano'>";
  html += "<div class='white-keys'>";
  const char* noteNames[] = { "Do", "Re", "Mi", "Fa", "Sol", "La" };
  for (int i = 0; i < 6; i++) {
    html += "<button class='white-key' onmousedown='playNote(" + String(i) + ")' onmouseup='stopNote(" + String(i) + ")' "
                                                                                                                     "ontouchstart='playNote("
            + String(i) + ")' ontouchend='stopNote(" + String(i) + ")'>"
                                                                   "<span>"
            + String(noteNames[i]) + "</span></button>";
  }
  html += "</div>";
  html += "</div>";
  html += "</div>";
  html += "</div>";

  // Upload form
  html += "<div class='upload-form'>";
  html += "<h3>MIDI File Upload</h3>";
  html += "<form action='/upload' method='post' enctype='multipart/form-data'>";
  html += "<input type='file' name='file' accept='.mid,.midi' required>";
  html += "<button type='submit'>Upload File</button>";
  html += "</form></div>";

  // File table
  html += "<h3>MIDI Files</h3>";
  html += "<table><tr><th>Filename</th><th>Size</th><th>Actions</th></tr>";

  for (int i = 0; i < totalFiles; i++) {
    String fileName = fileList[i];
    File file = SPIFFS.open("/" + fileName, "r");
    if (file) {
      html += "<tr>";
      html += "<td>" + fileName + "</td>";
      html += "<td>" + String(file.size()) + " bytes</td>";
      html += "<td>";
      html += "<button class='play' onclick='playMidi(\"" + fileName + "\")'>Play</button>";
      html += "<button class='stop' onclick='stopMidi()'>Stop</button>";
      html += "<button onclick='window.location.href=\"/file?name=" + fileName + "\"'>Download</button>";
      html += "<form action='/delete' method='post' style='display:inline'>";
      html += "<input type='hidden' name='file' value='" + fileName + "'>";
      html += "<button type='submit' class='delete'>Delete</button>";
      html += "</form></td></tr>";
      file.close();
    }
  }
  html += "</table>";

  // Tempo control
  html += "<div class='control-panel' style='margin: 20px 0; padding: 20px; background: #2a2a2a; border-radius: 5px;'>";
  html += "<h3>Tempo Control</h3>";
  html += "<div style='margin: 10px 0;'>";
  html += "<label>Tempo (microseconds per beat): </label>";
  html += "<input type='number' id='tempoInput' value='" + String(userTempo) + "' min='100000' max='1500000' step='1000' style='width: 150px; padding: 5px;'>";
  html += "</div>";
  html += "<div style='margin: 10px 0;'>";
  html += "<button onclick='updateTempo()'>Update Tempo</button>";
  html += "<span id='currentTempo' style='margin-left: 10px;'>Current: " + String(userTempo) + " μs/beat</span>";
  html += "</div>";
  html += "</div>";

  // JavaScript
  html += "<script>";

  // Touch event handling
  html += "document.addEventListener('touchstart', function(e) {";
  html += "  if(e.target.classList.contains('white-key')) {";
  html += "    e.preventDefault();";
  html += "  }";
  html += "}, {passive: false});";

  // Keyboard controls
  html += "document.addEventListener('keydown', function(e) {";
  html += "  if(e.repeat) return;";
  html += "  const key = e.key.toLowerCase();";
  html += "  switch(key) {";
  html += "    case 'a': playNote(0); break;";
  html += "    case 's': playNote(1); break;";
  html += "    case 'd': playNote(2); break;";
  html += "    case 'f': playNote(3); break;";
  html += "    case 'g': playNote(4); break;";
  html += "    case 'h': playNote(5); break;";
  html += "  }";
  html += "});";

  html += "document.addEventListener('keyup', function(e) {";
  html += "  const key = e.key.toLowerCase();";
  html += "  switch(key) {";
  html += "    case 'a': stopNote(0); break;";
  html += "    case 's': stopNote(1); break;";
  html += "    case 'd': stopNote(2); break;";
  html += "    case 'f': stopNote(3); break;";
  html += "    case 'g': stopNote(4); break;";
  html += "    case 'h': stopNote(5); break;";
  html += "  }";
  html += "});";

  // Note control functions
  html += "function playNote(index) {";
  html += "  fetch('/play', {";
  html += "    method: 'POST',";
  html += "    headers: {'Content-Type': 'application/x-www-form-urlencoded'},";
  html += "    body: 'note=' + index + '&action=on'";
  html += "  });";
  html += "}";

  html += "function stopNote(index) {";
  html += "  fetch('/play', {";
  html += "    method: 'POST',";
  html += "    headers: {'Content-Type': 'application/x-www-form-urlencoded'},";
  html += "    body: 'note=' + index + '&action=off'";
  html += "  });";
  html += "}";

  // MIDI control functions
  html += "function playMidi(filename) {";
  html += "  const tempo = document.getElementById('tempoInput').value;";
  html += "  fetch('/play', {";
  html += "    method: 'POST',";
  html += "    headers: {'Content-Type': 'application/x-www-form-urlencoded'},";
  html += "    body: 'file=' + filename + '&tempo=' + tempo";
  html += "  });";
  html += "}";

  html += "function stopMidi() {";
  html += "  fetch('/stop', {method: 'POST'});";
  html += "}";

  html += "function updateTempo() {";
  html += "  const tempo = document.getElementById('tempoInput').value;";
  html += "  if (tempo >= 100000 && tempo <= 1500000) {";
  html += "    fetch('/play', {";
  html += "      method: 'POST',";
  html += "      headers: {'Content-Type': 'application/x-www-form-urlencoded'},";
  html += "      body: 'tempo=' + tempo";
  html += "    }).then(() => {";
  html += "      document.getElementById('currentTempo').textContent = 'Current: ' + tempo + ' μs/beat';";
  html += "    });";
  html += "  } else {";
  html += "    alert('Tempo must be between 100,000 and 1,500,000 microseconds per beat');";
  html += "  }";
  html += "}";
  html += "</script>";

  html += "</div></body></html>";
  server.send(200, "text/html", html);
}


void setup() {
  Serial.begin(115200);

  // Initialize EEPROM
  EEPROM.begin(512);
  
  // Load tempo from EEPROM
  loadTempo();

  // Initialize OLED
  u8g2.begin();
  u8g2.enableUTF8Print();
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_ncenB08_tr);
  u8g2.drawStr(0, 24, "Initializing...");
  u8g2.sendBuffer();
  Serial.println("lcd ok");
  // Initialize SPIFFS
  if (!SPIFFS.begin(true)) {
    Serial.println("SPIFFS Mount Failed");
    return;
  }

  // Setup pin modes
  // Horn pins
  for (int i = 0; i < 6; i++) {
    pinMode(hornPins[i], OUTPUT);
    digitalWrite(hornPins[i], LOW);
    noteStates[i] = false;
  }

  // Button pins
  pinMode(BUTTON_PREV, INPUT_PULLUP);
  pinMode(BUTTON_PLAY, INPUT_PULLUP);
  pinMode(BUTTON_STOP, INPUT_PULLUP);
  pinMode(BUTTON_NEXT, INPUT_PULLUP);

  // Configure Access Point
  WiFi.softAP(ssid, password);
  IPAddress IP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(IP);

  // Setup web routes
  server.on("/", HTTP_GET, handleRoot);
  server.on(
    "/upload", HTTP_POST, []() {
      server.send(200);
    },
    handleFileUpload);
  server.on("/delete", HTTP_POST, handleDelete);
  server.on("/file", HTTP_GET, handleFile);
  server.on("/play", HTTP_POST, handlePlayMidi);
  server.on("/stop", HTTP_POST, handleStopMidi);

  server.begin();

  // Create MIDI player task on core 0
  xTaskCreatePinnedToCore(
    midiPlayerTask,
    "MidiPlayer",
    10000,
    NULL,
    1,
    &MidiPlayerTask,
    0);

  updateFileList();
  updateOLEDDisplay();
}

void loop() {
  server.handleClient();

  // Check physical buttons
  static bool prevState = true;
  static bool playState = true;
  static bool stopState = true;
  static bool nextState = true;

  bool currentPrev = digitalRead(BUTTON_PREV);
  bool currentPlay = digitalRead(BUTTON_PLAY);
  bool currentStop = digitalRead(BUTTON_STOP);
  bool currentNext = digitalRead(BUTTON_NEXT);

  // Button handling with debounce
  static unsigned long lastDebounceTime = 0;
  unsigned long debounceDelay = 50;
  unsigned long currentMillis = millis();

  if (currentMillis - lastDebounceTime > debounceDelay) {
    // Deteksi tombol STOP yang ditekan sendiri
    if (!currentStop && currentPrev && currentNext) {
      // Tombol STOP ditekan sendiri
      if (isPlaying) {
        isPlaying = false;
        currentFileName = "";
        for (int i = 0; i < 6; i++) {
          noteStates[i] = false;
          digitalWrite(hornPins[i], LOW);
        }
        updateOLEDDisplay();
        lastDebounceTime = currentMillis;
      }
    }
    
    // Deteksi kombinasi untuk pengaturan tempo - STOP + NEXT atau STOP + PREV
    else if (!currentStop && (!currentNext || !currentPrev)) {
      // Masuk mode tempo jika belum dalam mode tempo
      if (!isTempoMode) {
        isTempoMode = true;
        updateOLEDDisplay();
      }
      
      // Penanganan pengaturan tempo
      if (currentMillis - lastTempoAdjustTime > tempoAdjustDelay) {
        if (!currentNext && currentPrev) { // Naikkan tempo (kurangi mikrodetik)
          if (userTempo > 110000) {  // Prevent going below 100000
            userTempo = userTempo - 10000;
          } else {
            userTempo = 100000;  // Minimum value
          }
          tempo = userTempo;
          saveTempo();
          lastTempoAdjustTime = currentMillis;
          updateOLEDDisplay();
        } 
        else if (!currentPrev && currentNext) { // Turunkan tempo (tambah mikrodetik)
          if (userTempo < 1490000) {  // Prevent going above 1500000
            userTempo = userTempo + 10000;
          } else {
            userTempo = 1500000;  // Maximum value
          }
          tempo = userTempo;
          saveTempo();
          lastTempoAdjustTime = currentMillis;
          updateOLEDDisplay();
        }
      }
      
      lastDebounceTime = currentMillis;
    } 
    // Penanganan tombol normal ketika tidak dalam kombinasi khusus
    else {
      // Keluar dari mode tempo jika kita dalam mode tempo dan tombol dilepas
      if (isTempoMode && currentStop) {
        isTempoMode = false;
        updateOLEDDisplay();
      }
      
      // Penanganan tombol normal
      if (prevState && !currentPrev) {
        prevFile();
        lastDebounceTime = currentMillis;
      }
      if (playState && !currentPlay && !isPlaying && totalFiles > 0) {
        currentFileName = "/" + fileList[currentFileIndex];
        isPlaying = true;
        updateOLEDDisplay();
        lastDebounceTime = currentMillis;
      }
      if (nextState && !currentNext) {
        nextFile();
        lastDebounceTime = currentMillis;
      }
    }
  }

  prevState = currentPrev;
  playState = currentPlay;
  stopState = currentStop;
  nextState = currentNext;

  delay(2);
}