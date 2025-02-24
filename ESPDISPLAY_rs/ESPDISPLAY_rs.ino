// ESP 32 yang mengelola Display dan Input User.

// Library yang digunakan dalam project.
#include <uRTCLib.h>        // Library yang digunakan untuk mengakses RTC.
#include <uEEPROMLib.h>     // Library yang digunakan untuk mengakses EEPROM dari RTC.
#include <Wire.h>           // Library yang digunakan untuk komunikasi I2C.
#include <Bounce2.h>        // Library yang digunakan untuk melakukan debouncing button.
#include <TFT_eSPI.h>       // Library yang digunakan untuk mengakses TFT Display.
#include <Adafruit_GFX.h>   // Library yang digunakan untuk keperluan display font maupun gambar.
#include "LOGOFULL.h"       // Array berisi data gambar untuk Logo Start Screen.
#include "LOGO20.h"         // Array berisi data gambar untuk Logo serta tulisan pada halaman Menu Screen.
#include "Arial10.h"        // Array yang berisi data untuk font Arial yang kecil.
#include "Arial20.h"        // Array yang berisi data untuk font Arial yang besar.

// Deklarasi variabel dan pin.
#define MAX_FILES 200       // Menentukan jumlah maksimal file yang dapat ditampilkan adalah 200, dengan asumsi 6 bulan, maka 6*30 kurang lebih ada 180 file. 
#define MAX_VISIBLE_FILES 9 // Menentukan jumlah maksimal file yang ditampilan pada TFT pada satu waktu.
#define TEXT_HEIGHT 20      // Mendefinisikan bahwa tinggi dari text serta jarak antar text pada tampilan folder.
// Mendefinisikan warna yang digunakan dalam TFT Display dalam format rgb565.
#define WHITE          0xFFFF
#define RED            0xF800
#define YELLOW         0xFFE0
#define TEXT           YELLOW
#define SELECT         WHITE
// Mendefinisikan pin yang akan digunakan untuk tombol.
#define ButtonChoose 14 // 14
#define ButtonBack 27   // 27
#define ButtonUp 13     // 13
#define ButtonDown 12   // 12
#define TriggerPin 26
// Mendeklarasikan variabel yang digunakan dalam project.
String filenames[MAX_FILES];    // Buffer untuk menampung nama file/folder yang akan ditampilkan
String menu[4] = {"SET DATE", "STORAGE CARD", "BACK UP", "DELAY"};  // String untuk menampung pilihan pada tampilan menu.
String currentDir = "/";  // String yang menampung directory yang diakses saat ini.
int level = 0; // Fungsi yang menampung level directory saat ini.
int currentFileIndex = 0, scrollOffset = 0; // Variable yang bertanggung jawab untuk fungsi scrolling pada akses SD Card.
int fileCount = 0; // Variabel yang digunakan untuk menghitung jumlah file yang ditampung pada buffer.
int cursor = 0; // Variable yang menunjukkan posisi cursor pada menu.
int screen = 0; // Variabel yang menampung posisi screen saat ini.
int delayTime = 10; // Variabel yang menampung durasi timeOut untuk trigger
int timeSet[6] = {0,0,0,0,0,0}; // Variable yang menampung setting untuk rtc pada mode pengaturan tanggal dan waktu
int minute, second; // Variabel untuk timer pada playback audio.
bool edit = false; // Variabel yang menampung status mengedit nilai setting rtc atau tidak
bool isRecording = false, isPlaying = false; // Variabel untuk menampung status apakah sedang melakukan record atau playback audio.
unsigned long lastUpdate = 0; // Variabel untuk mencatat waktu terakhir update display.

// Deklarasi Objek
TFT_eSPI tft = TFT_eSPI();  // Objek untuk TFT Display.
uRTCLib rtc(0x68);          // Objek untuk RTC.
uEEPROMLib eeprom(0x57);    // Objek untuk EEPROM.
Bounce Trigger = Bounce();  // Objek untuk debouncing Trigger.
Bounce Choose = Bounce();   // Objek untuk debouncing Button Choose.
Bounce Back = Bounce();     // Objek untuk debouncing Button Back.
Bounce Up = Bounce();       // Objek untuk debouncing Button Up.
Bounce Down = Bounce();     // Objek untuk debouncing Button Down.

uint16_t BLUE = tft.color565(1,0,128);  // Karena saya tidak tau kode warna 565 untuk background biru sesuai dengan logo, maka saya convert berdasarkan code warna rgb.

void setup() {
  Serial.begin(115200); // Serial komunikasi dengan komputer untuk debugging.

  Serial1.begin(115200, SERIAL_8N1, 16, 17);  // Serial komunikasi dengan ESP Recorder untuk mengirimkan perintah.

  tft.begin();  // Memulai setup TFT.
  tft.setRotation(1); // Mengatur TFT dalam mode Landscape;

  URTCLIB_WIRE.begin(); // Memulai setup RTC.
  Wire.begin();         // Memulai I2C.

  eeprom.eeprom_read(0, &delayTime);

  Trigger.attach(TriggerPin, INPUT_PULLUP);   // Setup Trigger dan debouncing selama 25 ms.
  Trigger.interval(25);

  Choose.attach(ButtonChoose, INPUT_PULLUP);  // Setup Button dan debouncing selama 25 ms.
  Choose.interval(25);

  Back.attach(ButtonBack, INPUT_PULLUP);      // Setup Button dan debouncing selama 25 ms.
  Back.interval(25);

  Up.attach(ButtonUp, INPUT_PULLUP);          // Setup Button dan debouncing selama 25 ms.
  Up.interval(25);

  Down.attach(ButtonDown, INPUT_PULLUP);      // Setup Button dan debouncing selama 25 ms.
  Down.interval(25);

  startScreen();    // Menampilkan Start Screen

  mainScreen();     // Menampilkan Main Screen
}

void loop() {
  if(millis() - lastUpdate > 1000){ // Melakukan update display setiap detik, karena tampilan menampilkan jam hingga detik
    if(screen == 0){
      mainScreen();   // Memanggil fungsi untuk menampilkan Main Screen guna memperbaharui waktu.
    } else if (isPlaying && screen == 3){ // Apabila screen sedang menampilkan file, serta sedang melakukan playback audio,
      second++;                           // maka mulai menampilkan timer dan mengupdate timer setiap detik.
      if(second > 59){
        if (minute < 99){                 // Waktu playback maksimum yang terhitung 99:59
          minute++;
          second = 0;
        }
        else {
          second = 59;
        }
      }
      tft.loadFont(Arial10);
      tft.setCursor(250, 0);
      tft.printf("%02d:%02d", minute, second);
    }
    lastUpdate+=1000; // Memperbaharui waktu update terakhir.
  }
  if(isPlaying && Serial.available() > 0){   // Apabila sedang melakukan audio playback, menunggu informasi dari ESP Recorder mengenai status playback.
    String request = Serial.readStringUntil('\n');   // Membaca hingga newLine.
    request.trim(); // Menghilangkan karakter yang tak terbaca seperti '\r' dan '\n'
    if(request == "FINISHED"){  // Apabila ESP Recorder mengirimkan FINISHED, menandakan playback telah selesai dan menghentikan timer playback.
      isPlaying = false;
      level = 1;
      tft.fillRect(250, 0, 70, 40, BLUE); // Digunakan untuk menghilangkan timer pada pojok kiri atas.
    }
  }
  if(screen == 4 && Serial.available() > 0){ // Apabila sedang melakukan transfer dan berada pada halaman Back Up, akan mengecek status server.
    String request = Serial.readStringUntil('\n');
    request.trim();   // Menghilangkan karakter yang tak terbaca seperti '\r' dan '\n'
    if (request == "PERMISSIONREQUEST") {   // Apabila ESP Recorder mengirimkan PERMISSIONREQUEST maka akan menampilkan Screen Request dan menunggu input untuk akses.
      permissionScreen();
      screen = 6;
    } else if (request == "CLOSED") {   // Apabila ESP Recorder mengirimkan CLOSED maka akan mengembalikan screen ke Main Screen.
      menuScreen();
      screen = 1;
    }
  }
  if(screen == 6 && Serial.available() > 0){   // Apabila telah mengirimkan response akses maka akan menunggu hingga ESP Recorder menerima sebelum kembali ke halaman Back Up
    String request = Serial.readStringUntil('\n');
    request.trim(); // Menghilangkan karakter yang tak terbaca seperti '\r' dan '\n'
    Serial.println(request);
    if (request == "ANSWERED") {  // Apabila telah mendapatkan konfirmasi dari ESP Recorder maka kembali ke halaman Back Up.
      uploadScreen();
      screen = 4;
    }
  }
  Choose.update(); // Mengupdate status Button Choose.
  if(Choose.fell()){  // Apabila Button ditekan, maka akan memanggil fungsi dari button Choose.
    Serial.println("ditekan");
    chooseFunc();
  }
  Back.update();  // Mengupdate status dari Button Back.
  if(Back.fell()){  // Apabila Button ditekan, maka akan memanggil fungsi dari button Back.
    backFunc();
  }
  Up.update();    // Mengupdate status dari Button Up.
  if(Up.fell()){  // Apabila Button ditekan, maka akan memanggil fungsi dari button Up.
    upFunc();
  }
  Down.update();  // Mengupdate status dari Button Down.
  if(Down.fell()){  // Apabila Button ditekan, maka akan memanggil fungsi dari button Down.
    downFunc();
  }
  Trigger.update(); // Mengupdate status dari Trigger.
  if(Trigger.fell()){ // Apabila trigger menjadi LOW, 
    if(!isRecording){ // dan status belum memulai record
      isRecording = true; // maka akan memulai record dan mengirimkan perintah untuk record.
                          // Perintah yang dikirim dalam format RECORD_/yyyy-mm-dd/hh-mm-ss
                          // Contoh : RECORD_/2024-09-05/16-07-20
      Serial.printf("RECORD_/20%02d-%02d-%02d/%02d-%02d-%02d\n", rtc.year(), rtc.month(), rtc.day(), rtc.hour(), rtc.minute(), rtc.second());
    }
  } else if(Trigger.read() == HIGH && Trigger.currentDuration() > delayTime*1000){ // Apabila trigger menjadi high, akan mengecek apakah durasi High telah lebih lama dari durasi timeOut
    if(isRecording){  // Apabila iya, akan mengecek apakah status sedang recording, 
      isRecording = false;    // Bila iya, akan menghentikan recording dan mengirim perintah pada ESP Recorder untuk menghentikan recording.
      Serial.println("STOP_RECORD");
    }
  }
}

void startScreen(){ // Fungsi Start Screen.
  tft.setSwapBytes(true); // Harus di swap bytes karena byte hasil convert punya nilai terbalik. agar logo yang ditampilkan memiliki warna sesuai.

  tft.pushImage(0, 0, LOGOFULLWidth, LOGOFULLHeight, LOGOFULL); // Menampilkan gambar Logo berukuran FULLSCREEN
                                                                // fungsi TFT untuk menampilkan gambar dalam bentuk array.
                                                                // Parameter berupa (Coord X, Coord Y, Width Gambar, Height Gambar, Array Gambar)

  delay(5000);    // Menahan gambar selama 5 detik.

  tft.pushImage(0, 0, LOGO20Width, LOGO20Height, LOGO20);       // Menampilkan gambar Logo dan tulisan untuk indonesia raya pada bagian bawah screen.

  if(Serial.available() > 0){  // Mengecek apakah ada report masuk dari ESP Recorder, apakah inisialisasi SD Card sukses atau tidak.
    String report = Serial.readStringUntil('\n');
    report.trim();
    if (report == "SD_FAIL"){
      tft.fillScreen(BLUE);
      tft.unloadFont();
      tft.loadFont(Arial20);
      tft.setTextColor(RED);
      tft.setCursor(120, 100);
      tft.print("SD CARD NOT FOUND");
    }
  }
}

void mainScreen(){  // Fungsi Main Screen
  rtc.refresh();    // Mengupdate data dari rtc.

  tft.unloadFont(); // Mengganti font sebelumnya
  tft.loadFont(Arial20);  // Menjadi font yang dipilih, antara Arial10, dan Arial20
  tft.setTextColor(TEXT, BLUE, true); // Mengubah warna text menjadi kuning dengan background biru dan mengaktifkan smooth font.
  tft.setCursor(5, 5);  // Mengubah kursor ke posisi (Coord x, Coord y)
  tft.print("MENU");  // Menampilkan tulisan MENU pada posisi kursor.

  tft.unloadFont();
  tft.loadFont(Arial10);
  tft.setCursor(100, 85);
  tft.printf("%02d/%02d/20%02d", rtc.day(), rtc.month(), rtc.year()); // Menampilkan tanggal dengan printf atau print format.
                                                                      // %d berarti mengganti "%d" dengan variabel angka, %s mengganti "%s" menjadi variabel string pada print format.
                                                                      // "%02d" berarti memastikan angka yang dimuat dalam format 2 angka, dan mengisi kekosongan dengan angka 0
                                                                      // Contoh : printf("%02d", 9) akan menghasilkan angka 09.
                                                                      // Penggantian variabel dilakukan mulai dari % paling awal dengan variabel yang dimasukkan paling awal secara berurutan.
  tft.setCursor(112, 130);
  tft.printf("%02d:%02d:%02d", rtc.hour(), rtc.minute(), rtc.second()); // Menampilkan waktu dengan printf
  if(isRecording){  // Apabila sedang melakukan proses recording, maka menampilkan tulisan recording dengan warna merah.
    tft.setTextColor(RED, BLUE, true);
  } else{
    tft.setTextColor(BLUE); // Apabila tidak melakukan proses recording, ditulis dengan warna background agar tidak terlihat.
  }
  tft.unloadFont();
  tft.loadFont(Arial20);
  tft.setCursor(133, 175);
  tft.print("REC");
}

void menuScreen(){  // Fungsi untuk menampilkan Menu Screen

  tft.unloadFont();
  tft.loadFont(Arial20);
  tft.setTextColor(TEXT, BLUE, true);
  tft.setCursor(112, 15);
  tft.print("MENU");

  tft.unloadFont();
  tft.loadFont(Arial10);
  for(int i = 0; i < 4; i++){ // Menampilkan menu yang disiapkan pada variabel string menu diatas.
    int y = 72 + 36*i;        // Posisi cursor ditentukan berdasarkan persamaan berikut dimana i merupakan posisi dari menu pada string variabel.
    tft.setCursor(40, y);
    if(i == cursor){          // Apabila posisi kursor sesuai dengan posisi text, maka text akan dicetak dengan warna putih. 
      tft.setTextColor(SELECT, BLUE);
    }
    else{                     // Apabila tidak akan dicetak dengan warna kuning.
      tft.setTextColor(TEXT, BLUE, true);
    }
    tft.print(menu[i]);
  }
}

void permissionScreen(){  // Fungsi untuk menampilkan permission Screen
  tft.fillScreen(BLUE);
  tft.unloadFont();
  tft.loadFont(Arial20);
  tft.setCursor(34, 15);
  tft.setTextColor(TEXT, BLUE, true);
  tft.print("UPLOAD REQUEST");
  tft.setCursor(34, 120);
  tft.print("ALLOW UPLOAD ?");
}

void fileScreen(){  // Fungsi untuk menampilkan file screen
  listFiles(currentDir);  // Dilakukan dengan memanggil fungsi List File dengan directory saat ini.
  drawFileList();         // Kemudian mengambarkan list dari file yang didapat.
}

void timeScreen(){  // Fungsi untuk menampilkan tampilan untuk melakukan setting pada waktu dan tanggal.
  tft.unloadFont();
  tft.loadFont(Arial20);
  tft.setTextColor(TEXT, BLUE, true);
  tft.setCursor(43, 15);
  tft.print("SET DATE/TIME");
  tft.unloadFont();
  tft.loadFont(Arial10);
  for (int i = 0; i < 6; i++){  // seperti pada menu, disini menu tampilan waktu juga ditulis satu persatu guna untuk bisa memberikan warna putih pada bagian yang ditunjuk oleh kursor
    if(i == cursor){
      tft.setTextColor(SELECT, BLUE, true);
    } else{
      tft.setTextColor(TEXT, BLUE, true);
    }
    if(i < 2){  // Penulisan dilakukan seperti ini karena menyesuaikan tanda baca. Pada bagian tahun dan menit, merupakan bagian terujung sehingga tidak perlu ditambahkan tanda ":" maupun "/"
      tft.setCursor((100 + i*36), 96);
      tft.printf("%02d/", timeSet[i]);
    } else if (i == 2){
      tft.setCursor((100 + i*36), 96);
      tft.printf("20%02d", timeSet[i]);
    } else if (i < 5){
      tft.setCursor((112 + (i-3)*36), 140);
      tft.printf("%02d:", timeSet[i]);
    } else if (i == 5){
      tft.setCursor((112 + (i-3)*36), 140);
      tft.printf("%02d", timeSet[i]);
    }
  }
}

void delayScreen(){ // Fungsi untuk menampilkan tampilan untuk melakukan setting dari waktu timeOut
  tft.unloadFont();
  tft.loadFont(Arial20);
  tft.setTextColor(TEXT, BLUE, true);
  tft.setCursor(115, 15);
  tft.print("DELAY");

  tft.unloadFont();
  tft.loadFont(Arial10);
  tft.setCursor(118, 112);
  tft.printf("%03d SEC", delayTime);
}

void uploadScreen(){  // Fungsi untuk menampilkan tampilan Back Up, berupa tampilan berisi IP dari device.
  tft.fillScreen(BLUE);
  tft.unloadFont();
  tft.loadFont(Arial20);
  tft.setTextColor(TEXT, BLUE, true);
  tft.setCursor(97, 15);
  tft.print("BACK UP");
  tft.unloadFont();
  tft.loadFont(Arial10);
  tft.setCursor(60, 100);
  tft.print("IP : 192.168.50.1");
}

void chooseFunc(){  // Fungsi dari Button Choose.
  if (screen == 0 ){ // Ketika pada main screen, dan tombol choose telah ditekan, maka ESP akan berpindah ke Menu Screen.
    tft.fillScreen(BLUE);
    delay(1000);
    menuScreen();
    screen = 1;
  }
  if(screen == 1){  // Berpindah layar berdasarkan posisi kursor saat ini
    switch (cursor){
      case 0:       // Pada kursor 0, maka akan masuk ke mode untuk pengaturan tanggal dan waktu.
        rtc.refresh();              // ESP akan mengupdate rtc terlebih dahulu dan menyimpan waktu saat ini kedalam timeSet.
                                    // Kemudian user akan dipindahkan ke Time Screen dan dapat mengubah waktu pada Time Screen.
        timeSet[5] = rtc.second();
        timeSet[4] = rtc.minute();
        timeSet[3] = rtc.hour();
        timeSet[0] = rtc.day();
        timeSet[1] = rtc.month();
        timeSet[2] = rtc.year();
        cursor = 0;
        tft.fillScreen(BLUE);
        timeScreen();
        screen = 2;
        break;
      case 1:       // Pada kursor 1, maka akan masuk ke mode pembacaan SD Card.
        tft.fillScreen(BLUE);   // ESP Display akan melakukan request kepada ESP Recorder untuk melihat daftar folder/file pada SD Card.
        fileScreen();
        screen = 3;
        level = 0;
        cursor = 0;
        scrollOffset = 0;
        break;
      case 2:       // Pada kursor 2, maka akan masuk ke mode Back Up, dimana ESP Display akan mengirim perintah untuk
                    // Mengaktifkan mode transfer pada ESP Recorder dan berpindah ke layar Back Up.
        uploadScreen();
        screen = 4;
        Serial.println("TRANSFER");
        break;
      case 3:       // Pada kursor 3, maka akan masuk ke mode untuk pengaturan timeout dari trigger dan berpindah ke layar setting.
        tft.fillScreen(BLUE);
        delayScreen();
        screen = 5;
        break;
    }
  } else if(screen == 2){   // Ketika pada layar Time Screen, button choose akan mengaktifkan mode edit.
    if(cursor != 6){        // Ini memungkinkan user hanya mengganti parameter satu per satu.
      edit = !edit;         // Parameter yang diganti tergantu pada selection kursor yang ditandai dengan teks diwarnai putih.
    }
  } else if(screen == 3){   // Ketika pada layar file Screen.
    if(level == 0){         // Pada level 0, ketika dipilih artinya ESP berpindah dari melihat folder utama ke folder hari.
      level = 1;            // ESP Display akan melakukan request ulang dengan folder baru yang dipilih kemudian menampilkan folder tersebut.
      tft.fillScreen(BLUE);
      String name = filenames[currentFileIndex + scrollOffset];
      currentDir += name;
      currentFileIndex = 0;
      cursor = 0;
      scrollOffset = 0;
      listFiles(currentDir);
      drawFileList();
    } else if(level == 1) { // Pada Level 1, ketika dipilih artinya ESP berpindah dari folder harian, memilih satu file wav dan memulai playback.
      level = 2;            // ESP Display akan melakukan request untuk playback audio kepada ESP Recorder.
      String name = filenames[currentFileIndex + scrollOffset];
      String tempDir = currentDir +"/"+ name;
      Serial.println(tempDir);
      Serial.printf("PLAY_%s\n", tempDir.c_str());
      isPlaying = true;
      minute = 0;
      second = 0;
    }
  } else if(screen == 6){   // Ketika pada screen 6, artinya berada pada Permission Screen. Menekan tombol Choose berarti memberikan akses backup pada komputer.
    Serial.println("YES");
  }
}

void backFunc(){    // Fungsi dari Button Back.
  if (screen == 0 && Choose.read() == LOW){ // Ketika pada main screen, dan tombol choose telah ditekan, maka ESP akan berpindah ke Menu Screen.
    tft.fillScreen(BLUE);
    menuScreen();
    screen = 1;
  } else if(screen == 1){   // Pada screen 1, artinya menu screen, menekan tombol back akan mengembalikan ESP Display ke Main Screen.
    screen = 0;

    tft.fillScreen(BLUE);

    tft.pushImage(0, 0, LOGO20Width, LOGO20Height, LOGO20);

    mainScreen();
  } else if(screen == 2){   // Pada screen 2, artinya time Screen, ketika menekan tombol back, ESP Display akan menyimpan waktu yang telah diatur dan kembali ke Menu Screen.
    rtc.set(timeSet[5], timeSet[4], timeSet[3], 0, timeSet[0], timeSet[1], timeSet[2]);
    tft.fillScreen(BLUE);
    screen = 1;
    cursor = 0;
    menuScreen();
  } else if(screen == 3){   // Pada screen 3, artinya pada file Screen, menekan tombol back bergantung pada level saat ini.
    tft.fillScreen(BLUE);
    if(level == 2){         // Pada level 2, artinya sedang melakukan playback, menekan tombol back akan menghentikan playback
                            // serta mengirimkan perintah pada ESP Recorder untuk menghentikan playback audio.
      Serial.println("STOP_PLAY");
      isPlaying = false;
      level = 1;
      drawFileList();
    } else if (level == 1){ // Pada Level 1, artinya sedang dalam folder harian, menekan tombol back akan kembali ke folder utama.
      currentDir = "/";
      level = 0;
      cursor = 0;
      currentFileIndex = 0;
      scrollOffset = 0;
      fileScreen();
    } else if (level == 0) {// Pada level 0, artinya sedang dalam folder utama, menekan tombol back akan mengembalikan ke menu Screen.
      screen = 1;
      cursor = 0;
      menuScreen();
    }
  } else if(screen == 4){   // Pada screen 4, artinya pada Back Up screen, menekan tombol back akan menghentikan proses upload.
                            // ESP Display akan mengirimkan perintah untuk menghentikan server dan Access Point kemudian kembali ke Menu Screen.
    screen = 1;
    cursor = 0;
    Serial.println("STOP_TRANSFER");
    tft.fillScreen(BLUE);
    menuScreen();
  } else if(screen == 5){   // Pada screen 5, artinya pada delay Screen, menekan tombol akan menyimpan timeout yang di atur dan kembali ke Menu Screen.
    eeprom.eeprom_write(0, delayTime);
    tft.fillScreen(BLUE);
    cursor = 0;
    screen = 1;
    menuScreen();
  } else if(screen == 6){   // Pada screen 6, artinya pada permission screen, menekan tombol back akan menolak akses untuk backup.
    Serial.println("NO");
  }
}

void downFunc(){    // Fungsi dari Button Down.
  if(screen == 1){    // Pada menu screen menekan tombol down akan menggeser kursor kebawah.
    if(cursor < 4){
      cursor++;
      menuScreen();
    }
  } else if (screen == 2){  // Pada time screen, menekan tombol down akan menggeser kursor ke parameter selanjutnya.
                            // dan pada mode edit akan menurunkan nilai parameter.
    if (edit){
      switch(cursor){
        case 0:
          if(timeSet[cursor] > 1){
            timeSet[cursor] = timeSet[cursor] - 1;
          }
          else{
            timeSet[cursor] = 31;
          }
          break;
        case 1:
          if(timeSet[cursor] > 1){
            timeSet[cursor] = timeSet[cursor] - 1;
          }
          else{
            timeSet[cursor] = 12;
          }
          break;
        case 2:
          if(timeSet[cursor] > 1){
            timeSet[cursor] = timeSet[cursor] - 1;
          }
          else{
            timeSet[cursor] = 99;
          }
          break;
        case 3:
          if(timeSet[cursor] > 0){
            timeSet[cursor] = timeSet[cursor] - 1;
          }
          else{
            timeSet[cursor] = 23;
          }
          break;
        case 4:
          if(timeSet[cursor] > 0){
            timeSet[cursor] = timeSet[cursor] - 1;
          }
          else{
            timeSet[cursor] = 59;
          }
          break;
        case 5:
          if(timeSet[cursor] > 0){
            timeSet[cursor] = timeSet[cursor] - 1;
          }
          else{
            timeSet[cursor] = 59;
          }
          break;
      }
    } else{
      if(cursor < 6){
        cursor++;
      }
    }
    timeScreen();
  } else if (screen == 3){  // Pada mode file screen, menekan tombol akan menggeser kursor, dan apabila kursor telah mencapai limit, akan memulai fungsi scrolling,
    fileIndexDown();
  } else if (screen == 5){  // Pada mode delay Screen, menekan tombol akan mengurangi nilai dari durasi timout.
    if(delayTime > 0){
      delayTime--;
      delayScreen();
    }
  }
}

void upFunc(){    // Fungsi dari Button Up.
  if(screen == 1){  // pada Menu Screen, menekan tombol akan menaikkan kursor.
    if(cursor > 0){
      cursor--;
      menuScreen();
    }
  } else if (screen == 2){  // Pada Time Screen, menekan tombol akan menggeser kursor ke parameter sebelumnya, 
                            // dan pada mode edit, akan menambah nilai parameter.
    if (edit){
      switch(cursor){
        case 0:
          if(timeSet[cursor] < 31){
            timeSet[cursor] = timeSet[cursor] + 1;
          }
          else{
            timeSet[cursor] = 1;
          }
          break;
        case 1:
          if(timeSet[cursor] < 11){
            timeSet[cursor] = timeSet[cursor] + 1;
          }
          else{
            timeSet[cursor] = 1;
          }
          break;
        case 2:
          if(timeSet[cursor] < 99){
            timeSet[cursor] = timeSet[cursor] + 1;
          }
          else{
            timeSet[cursor] = 0;
          }
          break;
        case 3:
          if(timeSet[cursor] < 23){
            timeSet[cursor] = timeSet[cursor] + 1;
          }
          else{
            timeSet[cursor] = 0;
          }
          break;
        case 4:
          if(timeSet[cursor] < 59){
            timeSet[cursor] = timeSet[cursor] + 1;
          }
          else{
            timeSet[cursor] = 0;
          }
          break;
        case 5:
          if(timeSet[cursor] < 59){
            timeSet[cursor] = timeSet[cursor] + 1;
          }
          else{
            timeSet[cursor] = 0;
          }
          break;
      }
    } else{
      if(cursor > 0){
        cursor--;
      }
    }
    timeScreen();
  } else if (screen == 3){  // Pada filescreen, menekan tombol akan menaikkan kursor, dan apabila kursor sudah berada diatas, akan memulai fungsi scrolling.
    fileIndexUp();
  } else if(screen == 5){   // Pada delayScreen, menekan tombol akan menaikkan nilai timeout
    if(delayTime < 250){
      delayTime++;
      delayScreen();
    }
  }
}

void fileIndexUp() {  // Fungsi untuk mengaplikasikan scrolling pada TFT.
  if(currentFileIndex >0){  // Scrolling dilakukan dengan menggeser posisi index terlebih dahulu
    currentFileIndex--;
  } else if (scrollOffset > 0){ // Kemudian ketika beradda di posisi paling atas akan mulai mengurangi posisi scrollOffset.
    scrollOffset--;
  }
  drawFileList();
}

void fileIndexDown() {  // Fungsi untuk mengaplikasikan scrolling pada TFT.
                        // Scrolling dilakukan kearah bawah.
  if(currentFileIndex < 8){ // Dilakukan dengan menggeser index file kebawah terlebih dahulu
    currentFileIndex++;
  } else if (scrollOffset < fileCount - MAX_VISIBLE_FILES){ // dan apabila file index sudah ada dibagian paling bawah akan mulai menggeser scroll kebawah.
    scrollOffset++;
  }
  drawFileList();
}

void listFiles(String path) { // Fungsi untuk melakukan request untuk mendapatkan list file dalam folder pada ESP Recorder.
  for(int i = 0; i < MAX_FILES; i++){ // Buffer file akan di reset terlebih dahulu sebelum melakukan request.
    filenames[i] = "";
  }
  fileCount = 0;  // Index file juga akan direset terlebih dahulu.

  Serial.printf("LIST_%s\n", path.c_str());  // Mengirimkan request pada ESP Recorder fengan format LIST_/dir.
                                              // Contoh perintah : LIST_/2024-09-05 atau LIST_/
  tft.fillScreen(BLUE);
  tft.setTextColor(TEXT, BLUE, true);
  tft.unloadFont();
  tft.loadFont(Arial20);
  tft.setCursor(40, 100);
  tft.print("OPENING FOLDER...");

  delay(5000);                                // Delay dilakukan supaya ESP dapat menerima seluruh file dalam buffer terlebih dahulu sebelum diterima, mencegah adanya file yang terlambat di list

  tft.fillScreen(BLUE);

  while (!Serial.available()){
    //halt;
  }
  
  while (Serial.available() > 0){
    String name = Serial.readStringUntil('\n'); // Membaca data serial sampai menemukan newLine '\n'.
    filenames[fileCount++] = name;               // Nama File/Folder akan disimpan kedalam buffer.
    if(fileCount >= MAX_FILES){                  // Apabila jumlah file telah melebihi batas buffer, maka akan membuang data serial.
      while (Serial.available() > 0){
        Serial.read();
      }
      break;
    }
  }

  drawFileList();                                 // Setelah menerima data nama File, menggambarkan File pada TFT.
}

void drawFileList() {   // Fungsi untuk menggambar daftar File/Folder pada TFT.
    tft.setTextColor(TEXT, BLUE, true);
    tft.unloadFont();
    tft.loadFont(Arial20);
    if(currentDir != "/"){    // Apabila directory utama, maka menampilkan folder, apabila bukan maka menampilkan file.
      tft.setCursor(115, 15);
      tft.print("FILES");
    } else {
      tft.setCursor(97, 15);
      tft.print("FOLDERS");
    }
    tft.unloadFont();
    tft.loadFont(Arial10);
    for (int i = scrollOffset; i < min(scrollOffset + MAX_VISIBLE_FILES, fileCount); i++) {
        int y = (i - scrollOffset) * TEXT_HEIGHT + 50;  // Penggambaran dimulai dari scrollOffset hingga jumlah maksimum file.
                                                        // Hal ini memungkinkan pembuatan fitur scrolling pada tft sehingga dapat menampilkan banyak data sekaligus.
        tft.setCursor(40, y);
        if (i == (currentFileIndex + scrollOffset)){
          tft.setTextColor(SELECT, BLUE, true);
        } else{
          tft.setTextColor(TEXT, BLUE, true);
        }
        tft.print(filenames[i]);
    }
}