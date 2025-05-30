#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <EEPROM.h>

// Inisialisasi LCD (alamat I2C, kolom, baris)
LiquidCrystal_I2C lcd(0x27, 20, 4);

// Pin GPIO pada Wemos D1 Mini
int pushPin = D6;
int LEDPin = D5;

// Nada-nada naik 2 oktaf
int c = 1044;
int d = 1172;
int e = 1316;
int f = 1396;
int g = 1564;
int a = 1760;
int Bb = 1864;
int b = 1972;
int highc = 2092;

// Durasi not
int quarter = 500;
int half = 1000;

// Pengaturan WiFi
const char* ssid = "BirthdaySong_Config";  // Nama SSID AP/Hotspot
const char* password = "12345678";         // Password AP/Hotspot

// String untuk menyimpan teks LCD
// Teks awal saat startup
String welcome_line1 = "  SELAMAT ULANG";
String welcome_line2 = "      TAHUN!";

// Teks selama lagu (8 baris total - 4 baris untuk loop pertama, 4 baris untuk loop kedua)
String song1_line1 = "Selamat ulang";
String song1_line2 = "tahun kepadamu";
String song1_line3 = "Selamat ulang";
String song1_line4 = "tahun wahai kamu";

String song2_line1 = "Semoga panjang";
String song2_line2 = "umur dan sehat";
String song2_line3 = "Bahagia selalu";
String song2_line4 = "sepanjang masa";

// Alamat EEPROM untuk menyimpan teks
#define EEPROM_SIZE 250
#define WELCOME_LINE1_ADDR 0
#define WELCOME_LINE2_ADDR 25
#define SONG1_LINE1_ADDR 50
#define SONG1_LINE2_ADDR 75
#define SONG1_LINE3_ADDR 100
#define SONG1_LINE4_ADDR 125
#define SONG2_LINE1_ADDR 150
#define SONG2_LINE2_ADDR 175
#define SONG2_LINE3_ADDR 200
#define SONG2_LINE4_ADDR 225

ESP8266WebServer server(80);

void setup() {
  Serial.begin(115200);
  
  // Setup pin
  pinMode(pushPin, INPUT);
  pinMode(LEDPin, OUTPUT);
  digitalWrite(LEDPin, LOW);
  
  // Inisialisasi LCD
  lcd.init();
  lcd.backlight();
  lcd.clear();
  
  // Inisialisasi EEPROM
  EEPROM.begin(EEPROM_SIZE);
  
  // Baca teks tersimpan dari EEPROM
  readFromEEPROM();
  
  // Setup WiFi AP
  WiFi.softAP(ssid, password);
  Serial.println("Access Point started");
  Serial.print("IP Address: ");
  Serial.println(WiFi.softAPIP());
  
  // Setup server routes
  server.on("/", handleRoot);
  server.on("/save", handleSave);
  server.begin();
  Serial.println("HTTP server started");
  
  // Tampilkan pesan awal sesuai dengan kode asli
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(centerText(welcome_line1));
  lcd.setCursor(0, 1);
  lcd.print(centerText(welcome_line2));
  // lcd.setCursor(0, 2);
  // lcd.print("Config: ");
  // lcd.setCursor(0, 3);
  // lcd.print(WiFi.softAPIP().toString());
  delay(2000);
  lcd.clear();
}

void loop() {
  server.handleClient();
  
  digitalWrite(LEDPin, HIGH);
  
  // Menampilkan teks sesuai dengan urutan kode asli
  lcd.setCursor(0, 0);
  lcd.print(centerText(song1_line1));
  tone(LEDPin, c, 300); delay(400);
  tone(LEDPin, c, 100); delay(100);
  tone(LEDPin, d, quarter); delay(quarter);
  tone(LEDPin, c, quarter); delay(quarter);
  tone(LEDPin, f, quarter); delay(quarter);
  tone(LEDPin, e, half); delay(half);
  
  lcd.setCursor(0, 1);
  lcd.print(centerText(song1_line2));
  tone(LEDPin, c, 300); delay(400);
  tone(LEDPin, c, 100); delay(100);
  tone(LEDPin, d, quarter); delay(quarter);
  tone(LEDPin, c, quarter); delay(quarter);
  tone(LEDPin, g, quarter); delay(quarter);
  tone(LEDPin, f, half); delay(half);
  
  lcd.setCursor(0, 2);
  lcd.print(centerText(song1_line3));
  tone(LEDPin, c, 300); delay(400);
  tone(LEDPin, c, 100); delay(100);
  tone(LEDPin, highc, quarter); delay(quarter);
  tone(LEDPin, a, quarter); delay(quarter);
  tone(LEDPin, f, quarter); delay(quarter);
  tone(LEDPin, e, quarter); delay(quarter);
  tone(LEDPin, d, half); delay(half);
  
  lcd.setCursor(0, 3);
  lcd.print(centerText(song1_line4));
  tone(LEDPin, Bb, 300); delay(400);
  tone(LEDPin, Bb, 100); delay(100);
  tone(LEDPin, a, quarter); delay(quarter);
  tone(LEDPin, f, quarter); delay(quarter);
  tone(LEDPin, g, quarter); delay(quarter);
  tone(LEDPin, f, half); delay(half);
  
  digitalWrite(LEDPin, LOW);
  delay(1000);
  lcd.clear();
  
  // Loop kedua dengan 4 baris kedua
  lcd.setCursor(0, 0);
  lcd.print(centerText(song2_line1));
  tone(LEDPin, c, 300); delay(400);
  tone(LEDPin, c, 100); delay(100);
  tone(LEDPin, d, quarter); delay(quarter);
  tone(LEDPin, c, quarter); delay(quarter);
  tone(LEDPin, f, quarter); delay(quarter);
  tone(LEDPin, e, half); delay(half);
  
  lcd.setCursor(0, 1);
  lcd.print(centerText(song2_line2));
  tone(LEDPin, c, 300); delay(400);
  tone(LEDPin, c, 100); delay(100);
  tone(LEDPin, d, quarter); delay(quarter);
  tone(LEDPin, c, quarter); delay(quarter);
  tone(LEDPin, g, quarter); delay(quarter);
  tone(LEDPin, f, half); delay(half);
  
  lcd.setCursor(0, 2);
  lcd.print(centerText(song2_line3));
  tone(LEDPin, c, 300); delay(400);
  tone(LEDPin, c, 100); delay(100);
  tone(LEDPin, highc, quarter); delay(quarter);
  tone(LEDPin, a, quarter); delay(quarter);
  tone(LEDPin, f, quarter); delay(quarter);
  tone(LEDPin, e, quarter); delay(quarter);
  tone(LEDPin, d, half); delay(half);
  
  lcd.setCursor(0, 3);
  lcd.print(centerText(song2_line4));
  tone(LEDPin, Bb, 300); delay(400);
  tone(LEDPin, Bb, 100); delay(100);
  tone(LEDPin, a, quarter); delay(quarter);
  tone(LEDPin, f, quarter); delay(quarter);
  tone(LEDPin, g, quarter); delay(quarter);
  tone(LEDPin, f, half); delay(half);
  
  digitalWrite(LEDPin, LOW);
  delay(1000);
  lcd.clear();
}

// Fungsi untuk membuat teks rata tengah
String centerText(String text) {
  String result = "";
  int spaces = (20 - text.length()) / 2;
  
  if (spaces > 0) {
    for (int i = 0; i < spaces; i++) {
      result += " ";
    }
  }
  
  result += text;
  return result;
}

void handleRoot() {
  String html = "<!DOCTYPE html>"
                "<html lang='id'>"
                "<head>"
                "<meta charset='UTF-8'>"
                "<meta name='viewport' content='width=device-width, initial-scale=1.0'>"
                "<title>Konfigurasi Teks Ulang Tahun</title>"
                "<style>"
                "body { font-family: Arial, sans-serif; margin: 0; padding: 20px; color: #333; }"
                ".container { max-width: 500px; margin: 0 auto; background: #f9f9f9; padding: 20px; border-radius: 8px; box-shadow: 0 2px 4px rgba(0,0,0,0.1); }"
                "h1 { color: #FF4081; text-align: center; margin-bottom: 20px; }"
                "h2 { color: #FF4081; margin-top: 30px; font-size: 18px; }"
                ".form-group { margin-bottom: 15px; }"
                "label { display: block; margin-bottom: 5px; font-weight: bold; }"
                "input[type='text'] { width: 100%; padding: 10px; border: 1px solid #ddd; border-radius: 4px; box-sizing: border-box; font-size: 16px; }"
                ".error { color: red; font-size: 14px; height: 20px; margin-top: 5px; }"
                "button { background: #FF4081; color: white; border: none; padding: 12px 20px; border-radius: 4px; cursor: pointer; width: 100%; font-size: 16px; margin-top: 20px; }"
                "button:hover { background: #E91E63; }"
                ".char-count { text-align: right; font-size: 14px; color: #777; margin-top: 5px; }"
                ".description { font-size: 14px; color: #666; margin-bottom: 20px; }"
                "</style>"
                "</head>"
                "<body>"
                "<div class='container'>"
                "<h1>Konfigurasi Teks Ulang Tahun</h1>"
                "<p class='description'>Semua teks akan ditampilkan rata tengah pada LCD. Maksimal 20 karakter per baris.</p>"
                
                "<form id='configForm' onsubmit='return validateForm()' action='/save' method='GET'>"
                
                "<h2>Pesan Awal</h2>"
                "<div class='form-group'>"
                "<label for='welcome_line1'>Baris 1 (Pesan Awal):</label>"
                "<input type='text' id='welcome_line1' name='welcome_line1' value='" + welcome_line1 + "' maxlength='20' oninput='countChars(this, \"count_w1\")'>"
                "<div class='char-count'><span id='count_w1'>" + welcome_line1.length() + "</span>/20</div>"
                "<div id='error_w1' class='error'></div>"
                "</div>"
                
                "<div class='form-group'>"
                "<label for='welcome_line2'>Baris 2 (Pesan Awal):</label>"
                "<input type='text' id='welcome_line2' name='welcome_line2' value='" + welcome_line2 + "' maxlength='20' oninput='countChars(this, \"count_w2\")'>"
                "<div class='char-count'><span id='count_w2'>" + welcome_line2.length() + "</span>/20</div>"
                "<div id='error_w2' class='error'></div>"
                "</div>"
                
                "<h2>Teks Lagu - Loop Pertama</h2>"
                "<div class='form-group'>"
                "<label for='song1_line1'>Baris 1:</label>"
                "<input type='text' id='song1_line1' name='song1_line1' value='" + song1_line1 + "' maxlength='20' oninput='countChars(this, \"count1\")'>"
                "<div class='char-count'><span id='count1'>" + song1_line1.length() + "</span>/20</div>"
                "<div id='error1' class='error'></div>"
                "</div>"
                
                "<div class='form-group'>"
                "<label for='song1_line2'>Baris 2:</label>"
                "<input type='text' id='song1_line2' name='song1_line2' value='" + song1_line2 + "' maxlength='20' oninput='countChars(this, \"count2\")'>"
                "<div class='char-count'><span id='count2'>" + song1_line2.length() + "</span>/20</div>"
                "<div id='error2' class='error'></div>"
                "</div>"
                
                "<div class='form-group'>"
                "<label for='song1_line3'>Baris 3:</label>"
                "<input type='text' id='song1_line3' name='song1_line3' value='" + song1_line3 + "' maxlength='20' oninput='countChars(this, \"count3\")'>"
                "<div class='char-count'><span id='count3'>" + song1_line3.length() + "</span>/20</div>"
                "<div id='error3' class='error'></div>"
                "</div>"
                
                "<div class='form-group'>"
                "<label for='song1_line4'>Baris 4:</label>"
                "<input type='text' id='song1_line4' name='song1_line4' value='" + song1_line4 + "' maxlength='20' oninput='countChars(this, \"count4\")'>"
                "<div class='char-count'><span id='count4'>" + song1_line4.length() + "</span>/20</div>"
                "<div id='error4' class='error'></div>"
                "</div>"
                
                "<h2>Teks Lagu - Loop Kedua</h2>"
                "<div class='form-group'>"
                "<label for='song2_line1'>Baris 1:</label>"
                "<input type='text' id='song2_line1' name='song2_line1' value='" + song2_line1 + "' maxlength='20' oninput='countChars(this, \"count5\")'>"
                "<div class='char-count'><span id='count5'>" + song2_line1.length() + "</span>/20</div>"
                "<div id='error5' class='error'></div>"
                "</div>"
                
                "<div class='form-group'>"
                "<label for='song2_line2'>Baris 2:</label>"
                "<input type='text' id='song2_line2' name='song2_line2' value='" + song2_line2 + "' maxlength='20' oninput='countChars(this, \"count6\")'>"
                "<div class='char-count'><span id='count6'>" + song2_line2.length() + "</span>/20</div>"
                "<div id='error6' class='error'></div>"
                "</div>"
                
                "<div class='form-group'>"
                "<label for='song2_line3'>Baris 3:</label>"
                "<input type='text' id='song2_line3' name='song2_line3' value='" + song2_line3 + "' maxlength='20' oninput='countChars(this, \"count7\")'>"
                "<div class='char-count'><span id='count7'>" + song2_line3.length() + "</span>/20</div>"
                "<div id='error7' class='error'></div>"
                "</div>"
                
                "<div class='form-group'>"
                "<label for='song2_line4'>Baris 4:</label>"
                "<input type='text' id='song2_line4' name='song2_line4' value='" + song2_line4 + "' maxlength='20' oninput='countChars(this, \"count8\")'>"
                "<div class='char-count'><span id='count8'>" + song2_line4.length() + "</span>/20</div>"
                "<div id='error8' class='error'></div>"
                "</div>"
                
                "<button type='submit'>Simpan</button>"
                "</form>"
                "</div>"
                
                "<script>"
                "function countChars(input, countId) {"
                "  document.getElementById(countId).innerHTML = input.value.length;"
                "}"
                
                "function validateForm() {"
                "  let valid = true;"
                "  const inputs = ["
                "    {id: 'welcome_line1', errorId: 'error_w1'},"
                "    {id: 'welcome_line2', errorId: 'error_w2'},"
                "    {id: 'song1_line1', errorId: 'error1'},"
                "    {id: 'song1_line2', errorId: 'error2'},"
                "    {id: 'song1_line3', errorId: 'error3'},"
                "    {id: 'song1_line4', errorId: 'error4'},"
                "    {id: 'song2_line1', errorId: 'error5'},"
                "    {id: 'song2_line2', errorId: 'error6'},"
                "    {id: 'song2_line3', errorId: 'error7'},"
                "    {id: 'song2_line4', errorId: 'error8'}"
                "  ];"
                
                "  inputs.forEach((item) => {"
                "    const input = document.getElementById(item.id);"
                "    const error = document.getElementById(item.errorId);"
                "    if (input.value.length > 20) {"
                "      error.textContent = 'Maksimal 20 karakter!';"
                "      valid = false;"
                "    } else if (input.value.length === 0) {"
                "      error.textContent = 'Baris tidak boleh kosong!';"
                "      valid = false;"
                "    } else {"
                "      error.textContent = '';"
                "    }"
                "  });"
                
                "  return valid;"
                "}"
                
                "// Pre-count on page load"
                "window.onload = function() {"
                "  countChars(document.getElementById('welcome_line1'), 'count_w1');"
                "  countChars(document.getElementById('welcome_line2'), 'count_w2');"
                "  countChars(document.getElementById('song1_line1'), 'count1');"
                "  countChars(document.getElementById('song1_line2'), 'count2');"
                "  countChars(document.getElementById('song1_line3'), 'count3');"
                "  countChars(document.getElementById('song1_line4'), 'count4');"
                "  countChars(document.getElementById('song2_line1'), 'count5');"
                "  countChars(document.getElementById('song2_line2'), 'count6');"
                "  countChars(document.getElementById('song2_line3'), 'count7');"
                "  countChars(document.getElementById('song2_line4'), 'count8');"
                "};"
                "</script>"
                "</body>"
                "</html>";
  
  server.send(200, "text/html", html);
}

void handleSave() {
  if (server.hasArg("welcome_line1") && server.hasArg("welcome_line2") && 
      server.hasArg("song1_line1") && server.hasArg("song1_line2") && 
      server.hasArg("song1_line3") && server.hasArg("song1_line4") && 
      server.hasArg("song2_line1") && server.hasArg("song2_line2") && 
      server.hasArg("song2_line3") && server.hasArg("song2_line4")) {
    
    // Ambil nilai-nilai baru
    String new_welcome_line1 = server.arg("welcome_line1");
    String new_welcome_line2 = server.arg("welcome_line2");
    String new_song1_line1 = server.arg("song1_line1");
    String new_song1_line2 = server.arg("song1_line2");
    String new_song1_line3 = server.arg("song1_line3");
    String new_song1_line4 = server.arg("song1_line4");
    String new_song2_line1 = server.arg("song2_line1");
    String new_song2_line2 = server.arg("song2_line2");
    String new_song2_line3 = server.arg("song2_line3");
    String new_song2_line4 = server.arg("song2_line4");
    
    // Validasi panjang untuk semua teks
    bool valid = true;
    if (new_welcome_line1.length() > 20 || new_welcome_line1.length() == 0) valid = false;
    if (new_welcome_line2.length() > 20 || new_welcome_line2.length() == 0) valid = false;
    if (new_song1_line1.length() > 20 || new_song1_line1.length() == 0) valid = false;
    if (new_song1_line2.length() > 20 || new_song1_line2.length() == 0) valid = false;
    if (new_song1_line3.length() > 20 || new_song1_line3.length() == 0) valid = false;
    if (new_song1_line4.length() > 20 || new_song1_line4.length() == 0) valid = false;
    if (new_song2_line1.length() > 20 || new_song2_line1.length() == 0) valid = false;
    if (new_song2_line2.length() > 20 || new_song2_line2.length() == 0) valid = false;
    if (new_song2_line3.length() > 20 || new_song2_line3.length() == 0) valid = false;
    if (new_song2_line4.length() > 20 || new_song2_line4.length() == 0) valid = false;
    
    if (valid) {
      // Update nilai global
      welcome_line1 = new_welcome_line1;
      welcome_line2 = new_welcome_line2;
      song1_line1 = new_song1_line1;
      song1_line2 = new_song1_line2;
      song1_line3 = new_song1_line3;
      song1_line4 = new_song1_line4;
      song2_line1 = new_song2_line1;
      song2_line2 = new_song2_line2;
      song2_line3 = new_song2_line3;
      song2_line4 = new_song2_line4;
      
      // Simpan ke EEPROM
      saveToEEPROM();
      
      // Redirect kembali ke halaman utama
      server.sendHeader("Location", "/", true);
      server.send(302, "text/plain", "");
    } else {
      server.send(400, "text/plain", "Teks tidak valid. Pastikan setiap baris maksimal 20 karakter dan tidak kosong.");
    }
  } else {
    server.send(400, "text/plain", "Parameter tidak lengkap");
  }
}

void saveToEEPROM() {
  // Hapus EEPROM sebelum menulis
  for (int i = 0; i < EEPROM_SIZE; i++) {
    EEPROM.write(i, 0);
  }
  
  // Tulis welcome_line1
  for (int i = 0; i < welcome_line1.length(); i++) {
    EEPROM.write(WELCOME_LINE1_ADDR + i, welcome_line1[i]);
  }
  
  // Tulis welcome_line2
  for (int i = 0; i < welcome_line2.length(); i++) {
    EEPROM.write(WELCOME_LINE2_ADDR + i, welcome_line2[i]);
  }
  
  // Tulis song1_line1
  for (int i = 0; i < song1_line1.length(); i++) {
    EEPROM.write(SONG1_LINE1_ADDR + i, song1_line1[i]);
  }
  
  // Tulis song1_line2
  for (int i = 0; i < song1_line2.length(); i++) {
    EEPROM.write(SONG1_LINE2_ADDR + i, song1_line2[i]);
  }
  
  // Tulis song1_line3
  for (int i = 0; i < song1_line3.length(); i++) {
    EEPROM.write(SONG1_LINE3_ADDR + i, song1_line3[i]);
  }
  
  // Tulis song1_line4
  for (int i = 0; i < song1_line4.length(); i++) {
    EEPROM.write(SONG1_LINE4_ADDR + i, song1_line4[i]);
  }
  
  // Tulis song2_line1
  for (int i = 0; i < song2_line1.length(); i++) {
    EEPROM.write(SONG2_LINE1_ADDR + i, song2_line1[i]);
  }
  
  // Tulis song2_line2
  for (int i = 0; i < song2_line2.length(); i++) {
    EEPROM.write(SONG2_LINE2_ADDR + i, song2_line2[i]);
  }
  
  // Tulis song2_line3
  for (int i = 0; i < song2_line3.length(); i++) {
    EEPROM.write(SONG2_LINE3_ADDR + i, song2_line3[i]);
  }
  
  // Tulis song2_line4
  for (int i = 0; i < song2_line4.length(); i++) {
    EEPROM.write(SONG2_LINE4_ADDR + i, song2_line4[i]);
  }
  
  // Commit perubahan
  EEPROM.commit();
  Serial.println("Teks tersimpan ke EEPROM");
}

void readFromEEPROM() {
  // Baca welcome_line1
  welcome_line1 = "";
  for (int i = 0; i < 20; i++) {
    char c = EEPROM.read(WELCOME_LINE1_ADDR + i);
    if (c != 0) welcome_line1 += c;
    else break;
  }
  
  // Baca welcome_line2
  welcome_line2 = "";
  for (int i = 0; i < 20; i++) {
    char c = EEPROM.read(WELCOME_LINE2_ADDR + i);
    if (c != 0) welcome_line2 += c;
    else break;
  }
  
  // Baca song1_line1
  song1_line1 = "";
  for (int i = 0; i < 20; i++) {
    char c = EEPROM.read(SONG1_LINE1_ADDR + i);
    if (c != 0) song1_line1 += c;
    else break;
  }
  
  // Baca song1_line2
  song1_line2 = "";
  for (int i = 0; i < 20; i++) {
    char c = EEPROM.read(SONG1_LINE2_ADDR + i);
    if (c != 0) song1_line2 += c;
    else break;
  }
  
  // Baca song1_line3
  song1_line3 = "";
  for (int i = 0; i < 20; i++) {
    char c = EEPROM.read(SONG1_LINE3_ADDR + i);
    if (c != 0) song1_line3 += c;
    else break;
  }
  
  // Baca song1_line4
  song1_line4 = "";
  for (int i = 0; i < 20; i++) {
    char c = EEPROM.read(SONG1_LINE4_ADDR + i);
    if (c != 0) song1_line4 += c;
    else break;
  }
  
  // Baca song2_line1
  song2_line1 = "";
  for (int i = 0; i < 20; i++) {
    char c = EEPROM.read(SONG2_LINE1_ADDR + i);
    if (c != 0) song2_line1 += c;
    else break;
  }
  
  // Baca song2_line2
  song2_line2 = "";
  for (int i = 0; i < 20; i++) {
    char c = EEPROM.read(SONG2_LINE2_ADDR + i);
    if (c != 0) song2_line2 += c;
    else break;
  }
  
  // Baca song2_line3
  song2_line3 = "";
  for (int i = 0; i < 20; i++) {
    char c = EEPROM.read(SONG2_LINE3_ADDR + i);
    if (c != 0) song2_line3 += c;
    else break;
  }
  
  // Baca song2_line4
  song2_line4 = "";
  for (int i = 0; i < 20; i++) {
    char c = EEPROM.read(SONG2_LINE4_ADDR + i);
    if (c != 0) song2_line4 += c;
    else break;
  }
  
  // Jika baris kosong, isi dengan nilai default
  if (welcome_line1.length() == 0) welcome_line1 = "  SELAMAT ULANG";
  if (welcome_line2.length() == 0) welcome_line2 = "      TAHUN!";
  if (song1_line1.length() == 0) song1_line1 = "Selamat ulang";
  if (song1_line2.length() == 0) song1_line2 = "tahun kepadamu";
  if (song1_line3.length() == 0) song1_line3 = "Selamat ulang";
  if (song1_line4.length() == 0) song1_line4 = "tahun wahai kamu";
  if (song2_line1.length() == 0) song2_line1 = "Semoga panjang";
  if (song2_line2.length() == 0) song2_line2 = "umur dan sehat";
  if (song2_line3.length() == 0) song2_line3 = "Bahagia selalu";
  if (song2_line4.length() == 0) song2_line4 = "sepanjang masa";
  
  Serial.println("Teks berhasil dibaca dari EEPROM");

}