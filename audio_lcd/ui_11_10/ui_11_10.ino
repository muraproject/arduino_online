#include <Arduino.h>
#include <ArduinoJson.h>
#include <TFT_eSPI.h>

TFT_eSPI tft = TFT_eSPI();

enum KeadaanUI {
    LAYAR_UTAMA,
    MENU,
    DAFTAR_FOLDER,
    DAFTAR_FILE
};

KeadaanUI keadaanSaatIni = LAYAR_UTAMA;
int indeksMenu = 0;
int indeksFolder = 0;
int indeksFile = 0;
int offsetScrollFolder = 0;
int offsetScrollFile = 0;

const int ITEM_PER_LAYAR = 7;

const int TOMBOL_ATAS = 25;
const int TOMBOL_BAWAH = 26;
const int TOMBOL_PILIH = 32;
const int TOMBOL_KEMBALI = 33;

unsigned long waktuTerakhirTekan = 0;
const unsigned long DELAY_DEBOUNCE = 200;

StaticJsonDocument<4096> doc;

void setup() {
    Serial.begin(115200);
    Serial.println("Memulai program...");
    
    tft.init();
    tft.setRotation(1);
    
    pinMode(TOMBOL_ATAS, INPUT_PULLUP);
    pinMode(TOMBOL_BAWAH, INPUT_PULLUP);
    pinMode(TOMBOL_PILIH, INPUT_PULLUP);
    pinMode(TOMBOL_KEMBALI, INPUT_PULLUP);
    
    // Inisialisasi data awal
    doc["folders"][0]["nama"] = "20240817";
    doc["folders"][0]["files"][0] = "162030.mp3";
    doc["folders"][0]["files"][1] = "162050.mp3";
    doc["waktuSaatIni"] = "2024-08-17 16:30:45";
    doc["sedangMerekam"] = false;

    tampilkanLayarUtama();
    Serial.println("Setup selesai. Memulai loop utama.");
}

void loop() {
    bacaTombol();
    delay(10);
}

void bacaTombol() {
    static bool tombolSebelumnya[4] = {HIGH, HIGH, HIGH, HIGH};
    bool tombolSekarang[4];
    
    tombolSekarang[0] = digitalRead(TOMBOL_ATAS);
    tombolSekarang[1] = digitalRead(TOMBOL_BAWAH);
    tombolSekarang[2] = digitalRead(TOMBOL_PILIH);
    tombolSekarang[3] = digitalRead(TOMBOL_KEMBALI);
    
    for (int i = 0; i < 4; i++) {
        if (tombolSekarang[i] == LOW && tombolSebelumnya[i] == HIGH) {
            if (millis() - waktuTerakhirTekan > DELAY_DEBOUNCE) {
                waktuTerakhirTekan = millis();
                switch (i) {
                    case 0: navigasiAtas(); break;
                    case 1: navigasiBawah(); break;
                    case 2: pilihItem(); break;
                    case 3: kembali(); break;
                }
                perbaruiTampilan();
            }
        }
        tombolSebelumnya[i] = tombolSekarang[i];
    }
}

void navigasiAtas() {
    Serial.println("Tombol ATAS ditekan");
    switch (keadaanSaatIni) {
        case MENU:
            if (indeksMenu > 0) indeksMenu--;
            Serial.print("Navigasi menu: "); Serial.println(indeksMenu);
            break;
        case DAFTAR_FOLDER:
            if (indeksFolder > 0) {
                indeksFolder--;
                if (indeksFolder < offsetScrollFolder) {
                    offsetScrollFolder = indeksFolder;
                }
            }
            Serial.print("Navigasi folder: "); Serial.println(indeksFolder);
            break;
        case DAFTAR_FILE:
            if (indeksFile > 0) {
                indeksFile--;
                if (indeksFile < offsetScrollFile) {
                    offsetScrollFile = indeksFile;
                }
            }
            Serial.print("Navigasi file: "); Serial.println(indeksFile);
            break;
        default:
            break;
    }
}

void navigasiBawah() {
    Serial.println("Tombol BAWAH ditekan");
    switch (keadaanSaatIni) {
        case MENU:
            if (indeksMenu < 2) indeksMenu++;
            Serial.print("Navigasi menu: "); Serial.println(indeksMenu);
            break;
        case DAFTAR_FOLDER:
            if (indeksFolder < doc["folders"].size() - 1) {
                indeksFolder++;
                if (indeksFolder >= offsetScrollFolder + ITEM_PER_LAYAR) {
                    offsetScrollFolder = indeksFolder - ITEM_PER_LAYAR + 1;
                }
            }
            Serial.print("Navigasi folder: "); Serial.println(indeksFolder);
            break;
        case DAFTAR_FILE:
            if (indeksFile < doc["folders"][indeksFolder]["files"].size() - 1) {
                indeksFile++;
                if (indeksFile >= offsetScrollFile + ITEM_PER_LAYAR) {
                    offsetScrollFile = indeksFile - ITEM_PER_LAYAR + 1;
                }
            }
            Serial.print("Navigasi file: "); Serial.println(indeksFile);
            break;
        default:
            break;
    }
}

void pilihItem() {
    Serial.println("Tombol PILIH ditekan");
    switch (keadaanSaatIni) {
        case LAYAR_UTAMA:
            keadaanSaatIni = MENU;
            Serial.println("Pindah ke MENU");
            break;
        case MENU:
            if (indeksMenu == 0) {
                keadaanSaatIni = DAFTAR_FOLDER;
                indeksFolder = 0;
                offsetScrollFolder = 0;
                Serial.println("Pindah ke DAFTAR_FOLDER");
            }
            break;
        case DAFTAR_FOLDER:
            keadaanSaatIni = DAFTAR_FILE;
            indeksFile = 0;
            offsetScrollFile = 0;
            Serial.println("Pindah ke DAFTAR_FILE");
            break;
        case DAFTAR_FILE:
            Serial.print("PLAY: ");
            Serial.println(doc["folders"][indeksFolder]["files"][indeksFile].as<const char*>());
            break;
    }
}

void kembali() {
    Serial.println("Tombol KEMBALI ditekan");
    switch (keadaanSaatIni) {
        case MENU:
            keadaanSaatIni = LAYAR_UTAMA;
            Serial.println("Kembali ke LAYAR_UTAMA");
            break;
        case DAFTAR_FOLDER:
            keadaanSaatIni = MENU;
            Serial.println("Kembali ke MENU");
            break;
        case DAFTAR_FILE:
            keadaanSaatIni = DAFTAR_FOLDER;
            indeksFile = 0;
            offsetScrollFile = 0;
            Serial.println("Kembali ke DAFTAR_FOLDER");
            break;
        default:
            break;
    }
}

void perbaruiTampilan() {
    Serial.print("Memperbarui tampilan: ");
    switch (keadaanSaatIni) {
        case LAYAR_UTAMA:
            Serial.println("LAYAR_UTAMA");
            tampilkanLayarUtama();
            break;
        case MENU:
            Serial.println("MENU");
            tampilkanMenu();
            break;
        case DAFTAR_FOLDER:
            Serial.println("DAFTAR_FOLDER");
            tampilkanDaftarFolder();
            break;
        case DAFTAR_FILE:
            Serial.println("DAFTAR_FILE");
            tampilkanDaftarFile();
            break;
    }
}

void tampilkanLayarUtama() {
    tft.fillScreen(TFT_NAVY);
    tft.setTextColor(TFT_YELLOW);
    tft.setTextSize(2);
    
    tft.setCursor(10, 10);
    tft.print(doc["waktuSaatIni"].as<const char*>());
    
    tft.setCursor(10, tft.height() - 30);
    tft.print("Untuk Indonesia Raya");
    
    if (doc["sedangMerekam"]) {
        tft.setTextColor(TFT_RED);
        tft.setCursor(tft.width() - 50, 10);
        tft.print("REC");
    }

    tft.setTextColor(TFT_WHITE);
    tft.setCursor(10, tft.height() / 2);
    tft.print("Tekan PILIH untuk Menu");
}

void tampilkanMenu() {
    tft.fillScreen(TFT_NAVY);
    tft.setTextColor(TFT_YELLOW);
    tft.setTextSize(2);
    
    const char* itemMenu[] = {"Kartu Penyimpanan", "Kualitas Audio", "Tunda"};
    
    for (int i = 0; i < 3; i++) {
        tft.setCursor(10, 30 * i + 10);
        if (i == indeksMenu) {
            tft.setTextColor(TFT_WHITE);
        } else {
            tft.setTextColor(TFT_YELLOW);
        }
        tft.print(itemMenu[i]);
    }
}

void tampilkanDaftarFolder() {
    tft.fillScreen(TFT_NAVY);
    tft.setTextColor(TFT_YELLOW);
    tft.setTextSize(2);
    
    tft.setCursor(10, 10);
    tft.print("Folder:");
    
    int indeksAkhir = min((int)doc["folders"].size(), offsetScrollFolder + ITEM_PER_LAYAR);
    for (int i = offsetScrollFolder; i < indeksAkhir; i++) {
        tft.setCursor(10, 40 + (i - offsetScrollFolder) * 30);
        if (i == indeksFolder) {
            tft.setTextColor(TFT_WHITE);
        } else {
            tft.setTextColor(TFT_YELLOW);
        }
        tft.print(doc["folders"][i]["nama"].as<const char*>());
    }
    
    if (offsetScrollFolder > 0) {
        tft.fillTriangle(tft.width() - 20, 20, tft.width() - 10, 10, tft.width(), 20, TFT_WHITE);
    }
    if (indeksAkhir < (int)doc["folders"].size()) {
        tft.fillTriangle(tft.width() - 20, tft.height() - 20, tft.width() - 10, tft.height() - 10, tft.width(), tft.height() - 20, TFT_WHITE);
    }
}

void tampilkanDaftarFile() {
    tft.fillScreen(TFT_NAVY);
    tft.setTextColor(TFT_YELLOW);
    tft.setTextSize(2);
    
    tft.setCursor(10, 10);
    tft.print("File dalam ");
    tft.print(doc["folders"][indeksFolder]["nama"].as<const char*>());
    
    JsonArray files = doc["folders"][indeksFolder]["files"];
    int indeksAkhir = min((int)files.size(), offsetScrollFile + ITEM_PER_LAYAR);
    for (int i = offsetScrollFile; i < indeksAkhir; i++) {
        tft.setCursor(10, 40 + (i - offsetScrollFile) * 30);
        if (i == indeksFile) {
            tft.setTextColor(TFT_WHITE);
        } else {
            tft.setTextColor(TFT_YELLOW);
        }
        tft.print(files[i].as<const char*>());
    }
    
    if (offsetScrollFile > 0) {
        tft.fillTriangle(tft.width() - 20, 20, tft.width() - 10, 10, tft.width(), 20, TFT_WHITE);
    }
    if (indeksAkhir < (int)files.size()) {
        tft.fillTriangle(tft.width() - 20, tft.height() - 20, tft.width() - 10, tft.height() - 10, tft.width(), tft.height() - 20, TFT_WHITE);
    }
}