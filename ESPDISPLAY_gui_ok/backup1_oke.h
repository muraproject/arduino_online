// // ESP 32 yang mengelola Display dan Input User.

// // Library yang digunakan dalam project.
// #include <uRTCLib.h>
// #include <uEEPROMLib.h>
// #include <Wire.h>
// #include <Bounce2.h>
// #include <TFT_eSPI.h>
// #include <Adafruit_GFX.h>
// #include "LOGOFULL.h"
// #include "LOGO20.h"
// #include "Arial10.h"
// #include "Arial20.h"

// // Definisi konstanta
// #define MAX_FILES 200
// #define MAX_VISIBLE_FILES 9
// #define TEXT_HEIGHT 20
// #define WHITE 0xFFFF
// #define RED 0xF800
// #define YELLOW 0xFFE0
// #define TEXT YELLOW
// #define SELECT WHITE
// #define ButtonChoose 14
// #define ButtonBack 27
// #define ButtonUp 13
// #define ButtonDown 12
// #define TriggerPin 26

// // Variabel global
// String filenames[MAX_FILES];
// String menu[4] = {"SET DATE", "STORAGE CARD", "BACK UP", "DELAY"};
// String currentDir = "/";
// int level = 0;
// int currentFileIndex = 0, scrollOffset = 0;
// int fileCount = 0;
// int cursor = 0;
// int screen = 0;
// int delayTime = 10;
// int timeSet[6] = {0,0,0,0,0,0};
// int minute, second;
// bool edit = false;
// bool isRecording = false, isPlaying = false;
// unsigned long lastUpdate = 0;

// // Objek
// TFT_eSPI tft = TFT_eSPI();
// uRTCLib rtc(0x68);
// uEEPROMLib eeprom(0x57);
// Bounce Trigger = Bounce();
// Bounce Choose = Bounce();
// Bounce Back = Bounce();
// Bounce Up = Bounce();
// Bounce Down = Bounce();

// uint16_t BLUE = tft.color565(1,0,128);

// void setup() {
//   Serial.begin(115200);

//   tft.begin();
//   tft.setRotation(1);

//   URTCLIB_WIRE.begin();
//   Wire.begin();

//   eeprom.eeprom_read(0, &delayTime);

//   Trigger.attach(TriggerPin, INPUT_PULLUP);
//   Trigger.interval(25);

//   Choose.attach(ButtonChoose, INPUT_PULLUP);
//   Choose.interval(25);

//   Back.attach(ButtonBack, INPUT_PULLUP);
//   Back.interval(25);

//   Up.attach(ButtonUp, INPUT_PULLUP);
//   Up.interval(25);

//   Down.attach(ButtonDown, INPUT_PULLUP);
//   Down.interval(25);

//   startScreen();
//   mainScreen();
// }

// void loop() {
//   unsigned long currentMillis = millis();
  
//   if(currentMillis - lastUpdate > 1000) {
//     updateScreen();
//     lastUpdate = currentMillis;
//   }

//   handleSerialInput();
//   handleButtonInputs();
//   handleTrigger();
// }

// void updateScreen() {
//   if(screen == 0) {
//     mainScreen();
//   } else if (isPlaying && screen == 3) {
//     updatePlaybackTimer();
//   }
// }

// void handleSerialInput() {
//   if(Serial.available() > 0) {
//     String request = Serial.readStringUntil('\n');
//     request.trim();
//     processSerialRequest(request);
//   }
// }

// void processSerialRequest(String request) {
//   if(isPlaying && request == "FINISHED") {
//     stopPlayback();
//   } else if(screen == 4 && request == "PERMISSIONREQUEST") {
//     permissionScreen();
//     screen = 6;
//   } else if(screen == 4 && request == "CLOSED") {
//     menuScreen();
//     screen = 1;
//   } else if(screen == 6 && request == "ANSWERED") {
//     uploadScreen();
//     screen = 4;
//   }
// }

// void handleButtonInputs() {
//   Choose.update();
//   if(Choose.fell()) {
//     Serial.println("Choose button pressed");
//     chooseFunc();
//   }

//   Back.update();
//   if(Back.fell()) {
//     backFunc();
//   }

//   Up.update();
//   if(Up.fell()) {
//     upFunc();
//   }

//   Down.update();
//   if(Down.fell()) {
//     downFunc();
//   }
// }

// void handleTrigger() {
//   Trigger.update();
//   if(Trigger.fell()) {
//     if(!isRecording) {
//       startRecording();
//     }
//   } else if(Trigger.read() == HIGH && Trigger.currentDuration() > delayTime*1000) {
//     if(isRecording) {
//       stopRecording();
//     }
//   }
// }

// // ... (lanjut ke bagian berikutnya)

// // ... (lanjutan dari bagian sebelumnya)

// void startRecording() {
//   isRecording = true;
//   Serial.printf("RECORD_/20%02d-%02d-%02d/%02d-%02d-%02d\n", rtc.year(), rtc.month(), rtc.day(), rtc.hour(), rtc.minute(), rtc.second());
// }

// void stopRecording() {
//   isRecording = false;
//   Serial.println("STOP_RECORD");
// }

// void stopPlayback() {
//   isPlaying = false;
//   level = 1;
//   tft.fillRect(250, 0, 70, 40, BLUE);
// }

// void updatePlaybackTimer() {
//   second++;
//   if(second > 59) {
//     if (minute < 99) {
//       minute++;
//       second = 0;
//     } else {
//       second = 59;
//     }
//   }
//   tft.loadFont(Arial10);
//   tft.setCursor(250, 0);
//   tft.printf("%02d:%02d", minute, second);
// }

// void startScreen() {
//   tft.setSwapBytes(true);
//   tft.pushImage(0, 0, LOGOFULLWidth, LOGOFULLHeight, LOGOFULL);
//   delay(5000);
//   tft.pushImage(0, 0, LOGO20Width, LOGO20Height, LOGO20);

//   if(Serial.available() > 0) {
//     String report = Serial.readStringUntil('\n');
//     report.trim();
//     if (report == "SD_FAIL") {
//       tft.fillScreen(BLUE);
//       tft.unloadFont();
//       tft.loadFont(Arial20);
//       tft.setTextColor(RED);
//       tft.setCursor(120, 100);
//       tft.print("SD CARD NOT FOUND");
//     }
//   }
// }

// void mainScreen() {
//   rtc.refresh();

//   tft.unloadFont();
//   tft.loadFont(Arial20);
//   tft.setTextColor(TEXT, BLUE, true);
//   tft.setCursor(5, 5);
//   tft.print("MENU");

//   tft.unloadFont();
//   tft.loadFont(Arial10);
//   tft.setCursor(100, 85);
//   tft.printf("%02d/%02d/20%02d", rtc.day(), rtc.month(), rtc.year());
//   tft.setCursor(112, 130);
//   tft.printf("%02d:%02d:%02d", rtc.hour(), rtc.minute(), rtc.second());

//   tft.setTextColor(isRecording ? RED : BLUE, BLUE, true);
//   tft.unloadFont();
//   tft.loadFont(Arial20);
//   tft.setCursor(133, 175);
//   tft.print("REC");
// }

// void menuScreen() {
//   tft.fillScreen(BLUE);
//   tft.unloadFont();
//   tft.loadFont(Arial20);
//   tft.setTextColor(TEXT, BLUE, true);
//   tft.setCursor(112, 15);
//   tft.print("MENU");

//   tft.unloadFont();
//   tft.loadFont(Arial10);
//   for(int i = 0; i < 4; i++) {
//     int y = 72 + 36*i;
//     tft.setCursor(40, y);
//     tft.setTextColor(i == cursor ? SELECT : TEXT, BLUE, true);
//     tft.print(menu[i]);
//   }
// }

// void permissionScreen() {
//   tft.fillScreen(BLUE);
//   tft.unloadFont();
//   tft.loadFont(Arial20);
//   tft.setCursor(34, 15);
//   tft.setTextColor(TEXT, BLUE, true);
//   tft.print("UPLOAD REQUEST");
//   tft.setCursor(34, 120);
//   tft.print("ALLOW UPLOAD ?");
// }

// // ... (lanjut ke bagian berikutnya)

// // ... (lanjutan dari bagian sebelumnya)

// void fileScreen() {
//   listFiles(currentDir);
//   drawFileList();
// }

// void timeScreen() {
//   tft.fillScreen(BLUE);
//   tft.unloadFont();
//   tft.loadFont(Arial20);
//   tft.setTextColor(TEXT, BLUE, true);
//   tft.setCursor(43, 15);
//   tft.print("SET DATE/TIME");
//   tft.unloadFont();
//   tft.loadFont(Arial10);
//   for (int i = 0; i < 6; i++) {
//     tft.setTextColor(i == cursor ? SELECT : TEXT, BLUE, true);
//     if(i < 2) {
//       tft.setCursor((100 + i*36), 96);
//       tft.printf("%02d/", timeSet[i]);
//     } else if (i == 2) {
//       tft.setCursor((100 + i*36), 96);
//       tft.printf("20%02d", timeSet[i]);
//     } else if (i < 5) {
//       tft.setCursor((112 + (i-3)*36), 140);
//       tft.printf("%02d:", timeSet[i]);
//     } else if (i == 5) {
//       tft.setCursor((112 + (i-3)*36), 140);
//       tft.printf("%02d", timeSet[i]);
//     }
//   }
// }

// void delayScreen() {
//   tft.fillScreen(BLUE);
//   tft.unloadFont();
//   tft.loadFont(Arial20);
//   tft.setTextColor(TEXT, BLUE, true);
//   tft.setCursor(115, 15);
//   tft.print("DELAY");

//   tft.unloadFont();
//   tft.loadFont(Arial10);
//   tft.setCursor(118, 112);
//   tft.printf("%03d SEC", delayTime);
// }

// void uploadScreen() {
//   tft.fillScreen(BLUE);
//   tft.unloadFont();
//   tft.loadFont(Arial20);
//   tft.setTextColor(TEXT, BLUE, true);
//   tft.setCursor(97, 15);
//   tft.print("BACK UP");
//   tft.unloadFont();
//   tft.loadFont(Arial10);
//   tft.setCursor(60, 100);
//   tft.print("IP : 192.168.50.1");
// }

// void chooseFunc() {
//   if (screen == 0) {
//     tft.fillScreen(BLUE);
//     delay(1000);
//     menuScreen();
//     screen = 1;
//   } else if(screen == 1) {
//     switch (cursor) {
//       case 0:
//         rtc.refresh();
//         timeSet[5] = rtc.second();
//         timeSet[4] = rtc.minute();
//         timeSet[3] = rtc.hour();
//         timeSet[0] = rtc.day();
//         timeSet[1] = rtc.month();
//         timeSet[2] = rtc.year();
//         cursor = 0;
//         tft.fillScreen(BLUE);
//         timeScreen();
//         screen = 2;
//         break;
//       case 1:
//         tft.fillScreen(BLUE);
//         fileScreen();
//         screen = 3;
//         level = 0;
//         cursor = 0;
//         scrollOffset = 0;
//         break;
//       case 2:
//         uploadScreen();
//         screen = 4;
//         Serial.println("TRANSFER");
//         break;
//       case 3:
//         tft.fillScreen(BLUE);
//         delayScreen();
//         screen = 5;
//         break;
//     }
//   } else if(screen == 2) {
//     if(cursor != 6) {
//       edit = !edit;
//     }
//   } else if(screen == 3) {
//     if(level == 0) {
//       level = 1;
//       tft.fillScreen(BLUE);
//       String name = filenames[currentFileIndex + scrollOffset];
//       currentDir += name;
//       currentFileIndex = 0;
//       cursor = 0;
//       scrollOffset = 0;
//       listFiles(currentDir);
//       drawFileList();
//     } else if(level == 1) {
//       level = 2;
//       String name = filenames[currentFileIndex + scrollOffset];
//       String tempDir = currentDir +"/"+ name;
//       Serial.println(tempDir);
//       Serial.printf("PLAY_%s\n", tempDir.c_str());
//       isPlaying = true;
//       minute = 0;
//       second = 0;
//     }
//   } else if(screen == 6) {
//     Serial.println("YES");
//   }
// }

// // ... (lanjut ke bagian berikutnya)

// // ... (lanjutan dari bagian sebelumnya)

// void backFunc() {
//   if (screen == 0 && Choose.read() == LOW) {
//     tft.fillScreen(BLUE);
//     menuScreen();
//     screen = 1;
//   } else if(screen == 1) {
//     screen = 0;
//     tft.fillScreen(BLUE);
//     tft.pushImage(0, 0, LOGO20Width, LOGO20Height, LOGO20);
//     mainScreen();
//   } else if(screen == 2) {
//     rtc.set(timeSet[5], timeSet[4], timeSet[3], 0, timeSet[0], timeSet[1], timeSet[2]);
//     tft.fillScreen(BLUE);
//     screen = 1;
//     cursor = 0;
//     menuScreen();
//   } else if(screen == 3) {
//     tft.fillScreen(BLUE);
//     if(level == 2) {
//       Serial.println("STOP_PLAY");
//       isPlaying = false;
//       level = 1;
//       drawFileList();
//     } else if (level == 1) {
//       currentDir = "/";
//       level = 0;
//       cursor = 0;
//       currentFileIndex = 0;
//       scrollOffset = 0;
//       fileScreen();
//     } else if (level == 0) {
//       screen = 1;
//       cursor = 0;
//       menuScreen();
//     }
//   } else if(screen == 4) {
//     screen = 1;
//     cursor = 0;
//     Serial.println("STOP_TRANSFER");
//     tft.fillScreen(BLUE);
//     menuScreen();
//   } else if(screen == 5) {
//     eeprom.eeprom_write(0, delayTime);
//     tft.fillScreen(BLUE);
//     cursor = 0;
//     screen = 1;
//     menuScreen();
//   } else if(screen == 6) {
//     Serial.println("NO");
//   }
// }

// void downFunc() {
//   if(screen == 1) {
//     if(cursor < 3) {
//       cursor++;
//       menuScreen();
//     }
//   } else if (screen == 2) {
//     if (edit) {
//       switch(cursor) {
//         case 0:
//           timeSet[cursor] = (timeSet[cursor] > 1) ? timeSet[cursor] - 1 : 31;
//           break;
//         case 1:
//           timeSet[cursor] = (timeSet[cursor] > 1) ? timeSet[cursor] - 1 : 12;
//           break;
//         case 2:
//           timeSet[cursor] = (timeSet[cursor] > 1) ? timeSet[cursor] - 1 : 99;
//           break;
//         case 3:
//           timeSet[cursor] = (timeSet[cursor] > 0) ? timeSet[cursor] - 1 : 23;
//           break;
//         case 4:
//         case 5:
//           timeSet[cursor] = (timeSet[cursor] > 0) ? timeSet[cursor] - 1 : 59;
//           break;
//       }
//     } else {
//       if(cursor < 5) {
//         cursor++;
//       }
//     }
//     timeScreen();
//   } else if (screen == 3) {
//     fileIndexDown();
//   } else if (screen == 5) {
//     if(delayTime > 0) {
//       delayTime--;
//       delayScreen();
//     }
//   }
// }

// void upFunc() {
//   if(screen == 1) {
//     if(cursor > 0) {
//       cursor--;
//       menuScreen();
//     }
//   } else if (screen == 2) {
//     if (edit) {
//       switch(cursor) {
//         case 0:
//           timeSet[cursor] = (timeSet[cursor] < 31) ? timeSet[cursor] + 1 : 1;
//           break;
//         case 1:
//           timeSet[cursor] = (timeSet[cursor] < 12) ? timeSet[cursor] + 1 : 1;
//           break;
//         case 2:
//           timeSet[cursor] = (timeSet[cursor] < 99) ? timeSet[cursor] + 1 : 0;
//           break;
//         case 3:
//           timeSet[cursor] = (timeSet[cursor] < 23) ? timeSet[cursor] + 1 : 0;
//           break;
//         case 4:
//         case 5:
//           timeSet[cursor] = (timeSet[cursor] < 59) ? timeSet[cursor] + 1 : 0;
//           break;
//       }
//     } else {
//       if(cursor > 0) {
//         cursor--;
//       }
//     }
//     timeScreen();
//   } else if (screen == 3) {
//     fileIndexUp();
//   } else if(screen == 5) {
//     if(delayTime < 250) {
//       delayTime++;
//       delayScreen();
//     }
//   }
// }

// void fileIndexUp() {
//   if(currentFileIndex > 0) {
//     currentFileIndex--;
//   } else if (scrollOffset > 0) {
//     scrollOffset--;
//   }
//   drawFileList();
// }

// void fileIndexDown() {
//   if(currentFileIndex < min(MAX_VISIBLE_FILES - 1, fileCount - 1)) {
//     currentFileIndex++;
//   } else if (scrollOffset < fileCount - MAX_VISIBLE_FILES) {
//     scrollOffset++;
//   }
//   drawFileList();
// }

// void listFiles(String path) {
//   for(int i = 0; i < MAX_FILES; i++) {
//     filenames[i] = "";
//   }
//   fileCount = 0;

//   Serial.printf("LIST_%s\n", path.c_str());
  
//   tft.fillScreen(BLUE);
//   tft.setTextColor(TEXT, BLUE, true);
//   tft.unloadFont();
//   tft.loadFont(Arial20);
//   tft.setCursor(40, 100);
//   tft.print("OPENING FOLDER...");

//   delay(5000);

//   tft.fillScreen(BLUE);

//   while (!Serial.available()) {
//     // wait for data
//   }
  
//   while (Serial.available() > 0) {
//     String name = Serial.readStringUntil('\n');
//     name.trim();
//     if (name.length() > 0) {
//       filenames[fileCount++] = name;
//       if(fileCount >= MAX_FILES) {
//         while (Serial.available() > 0) {
//           Serial.read();
//         }
//         break;
//       }
//     }
//   }

//   drawFileList();
// }

// void drawFileList() {
//   tft.fillScreen(BLUE);
//   tft.setTextColor(TEXT, BLUE, true);
//   tft.unloadFont();
//   tft.loadFont(Arial20);
//   tft.setCursor(currentDir != "/" ? 115 : 97, 15);
//   tft.print(currentDir != "/" ? "FILES" : "FOLDERS");
  
//   tft.unloadFont();
//   tft.loadFont(Arial10);
//   for (int i = 0; i < min(MAX_VISIBLE_FILES, fileCount); i++) {
//     int index = i + scrollOffset;
//     int y = i * TEXT_HEIGHT + 50;
//     tft.setCursor(40, y);
//     tft.setTextColor(index == (currentFileIndex + scrollOffset) ? SELECT : TEXT, BLUE, true);
//     tft.print(filenames[index]);
//   }
// }

// // End of the program