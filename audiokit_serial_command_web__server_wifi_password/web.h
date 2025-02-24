#ifndef WEB_H
#define WEB_H

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <SD.h>
#include <vector>
#include <algorithm>
#include <EEPROM.h>

#define EEPROM_SIZE 512
#define SSID_ADDR 100
#define PASS_ADDR 200

String ssid = "ESP32_Hotspot";
String password = "password123";

WebServer server(80);

// Function declarations
String getContentType(String filename);
bool handleFileRead(String path);
bool shouldHide(String fileName);
void saveWiFiCredentials();
void loadWiFiCredentials();
String htmlHeader(String title);
void listDirectory(String path);
void handleFileList();
void handleWiFiPage();
void handleUpdateWiFi();
void setupWebServer();
void loopWebServer();

// Function implementations
String getContentType(String filename) {
  if (filename.endsWith(".wav")) return "audio/wav";
  else if (filename.endsWith(".html")) return "text/html";
  else if (filename.endsWith(".css")) return "text/css";
  else if (filename.endsWith(".js")) return "application/javascript";
  return "text/plain";
}

bool handleFileRead(String path) {
  Serial.println("handleFileRead: " + path);
  if (path.endsWith("/")) path += "index.html";
  String contentType = getContentType(path);
  if (SD.exists(path)) {
    File file = SD.open(path, FILE_READ);
    size_t sent = server.streamFile(file, contentType);
    file.close();
    return true;
  }
  Serial.println("\tFile Not Found");
  return false;
}

bool shouldHide(String fileName) {
  return fileName == "hening.wav" || fileName == "System Volume Information";
}

void saveWiFiCredentials() {
  EEPROM.writeString(SSID_ADDR, ssid);
  EEPROM.writeString(PASS_ADDR, password);
  EEPROM.commit();
}

void loadWiFiCredentials() {
  ssid = EEPROM.readString(SSID_ADDR);
  password = EEPROM.readString(PASS_ADDR);
  
  if (ssid.length() == 0) {
    ssid = "ESP32_Hotspot";
  }
  if (password.length() == 0) {
    password = "password123";
  }
}

String htmlHeader(String title) {
  String header = "<html><head><title>" + title + "</title>";
  header += "<style>";
  header += "body { font-family: Arial, sans-serif; margin: 0; padding: 20px; background-color: #f0f0f0; }";
  header += "h1, h2 { color: #333; }";
  header += "ul { list-style-type: none; padding: 0; }";
  header += "li { margin-bottom: 10px; }";
  header += "a { color: #1a73e8; text-decoration: none; }";
  header += "a:hover { text-decoration: underline; }";
  header += ".folder { font-weight: bold; }";
  header += ".file { margin-left: 20px; }";
  header += ".settings { margin-top: 20px; padding: 10px; background-color: #e0e0e0; border-radius: 5px; }";
  header += "input[type='text'], input[type='password'] { margin: 5px 0; padding: 5px; width: 200px; }";
  header += "input[type='submit'] { margin-top: 10px; padding: 5px 10px; background-color: #4CAF50; color: white; border: none; cursor: pointer; }";
  header += ".error { color: red; margin-top: 10px; }";
  header += "</style>";
  header += "</head><body>";
  return header;
}

void listDirectory(String path) {
  String output = htmlHeader("WAV File Browser");
  output += "<h1>WAV File Browser</h1>";
  output += "<div class='settings'>";
  output += "<a href='/wifi'>WiFi Settings</a>";
  output += "</div>";
  output += "<h2>Current Directory: " + path + "</h2>";
  output += "<ul>";

  if (path != "/") {
    int lastSlash = path.lastIndexOf('/');
    String parentDir = path.substring(0, lastSlash);
    if (parentDir.isEmpty()) parentDir = "/";
    output += "<li class='folder'><a href='?dir=" + parentDir + "'>..</a> (Parent Directory)</li>";
  }
  
  File root = SD.open(path);
  if (!root || !root.isDirectory()) {
    output += "<li>Failed to open directory</li>";
  } else {
    std::vector<String> entries;
    File file = root.openNextFile();
    while (file) {
      String fileName = String(file.name());
      if (!shouldHide(fileName)) {
        entries.push_back(fileName);
      }
      file = root.openNextFile();
    }
    
    std::sort(entries.begin(), entries.end(), std::greater<String>());
    
    for (const auto& fileName : entries) {
      String fullPath = path + (path.endsWith("/") ? "" : "/") + fileName;
      File file = SD.open(fullPath);
      if (file.isDirectory()) {
        output += "<li class='folder'><a href='?dir=" + fullPath + "'>" + fileName + "/</a></li>";
      } else if (fileName.endsWith(".wav")) {
        String fileSize = String(file.size());
        output += "<li class='file'><a href='javascript:void(0);' onclick='playAudio(\"" + fullPath + "\")'>" + fileName + " (" + fileSize + " bytes)</a></li>";
      }
      file.close();
    }
  }
  root.close();
  
  output += "</ul>";
  
  output += "<audio id='audioPlayer' controls style='display:none;'></audio>";
  output += "<script>";
  output += "function playAudio(url) {";
  output += "  let audio = document.getElementById('audioPlayer');";
  output += "  audio.src = url;";
  output += "  audio.style.display = 'block';";
  output += "  audio.play();";
  output += "}";
  output += "</script>";
  output += "</body></html>";
  
  server.send(200, "text/html", output);
}

void handleFileList() {
  String path = server.hasArg("dir") ? server.arg("dir") : "/";
  listDirectory(path);
}

void handleWiFiPage() {
  String output = htmlHeader("WiFi Settings");
  output += "<h1>WiFi Settings</h1>";
  output += "<form action='/update-wifi' method='post'>";
  output += "SSID: <input type='text' name='ssid' value='" + ssid + "'><br>";
  output += "Password: <input type='password' name='password' value='" + password + "'><br>";
  output += "<input type='submit' value='Update WiFi Settings'>";
  output += "</form>";
  output += "<p><a href='/'>Back to File Browser</a></p>";
  output += "</body></html>";
  server.send(200, "text/html", output);
}

void handleUpdateWiFi() {
  String newSsid = server.arg("ssid");
  String newPassword = server.arg("password");
  String errorMessage = "";

  if (newSsid.length() == 0) {
    errorMessage += "SSID cannot be empty. ";
  }
  if (newPassword.length() < 8) {
    errorMessage += "Password must be at least 8 characters long. ";
  }

  if (errorMessage.length() > 0) {
    String output = htmlHeader("Error Updating WiFi Settings");
    output += "<h1>Error Updating WiFi Settings</h1>";
    output += "<p class='error'>" + errorMessage + "</p>";
    output += "<p><a href='/wifi'>Go back</a></p>";
    output += "</body></html>";
    server.send(400, "text/html", output);
  } else {
    ssid = newSsid;
    password = newPassword;
    saveWiFiCredentials();
    
    String output = htmlHeader("WiFi Settings Updated");
    output += "<h1>WiFi Settings Updated</h1>";
    output += "<p>SSID: " + ssid + "</p>";
    output += "<p>The device will restart in 5 seconds.</p>";
    output += "</body></html>";
    server.send(200, "text/html", output);
    delay(5000);
    Serial.println("esp_goto_restart");
    delay(200);
    ESP.restart();
  }
}

void setupWebServer() {
  EEPROM.begin(EEPROM_SIZE);
  loadWiFiCredentials();

  WiFi.softAP(ssid.c_str(), password.c_str());
  IPAddress myIP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(myIP);

  server.on("/", HTTP_GET, handleFileList);
  server.on("/wifi", HTTP_GET, handleWiFiPage);
  server.on("/update-wifi", HTTP_POST, handleUpdateWiFi);

  server.onNotFound([]() {
    if (!handleFileRead(server.uri()))
      server.send(404, "text/plain", "FileNotFound");
  });

  server.begin();
  Serial.println("HTTP server started");
}

void loopWebServer() {
  server.handleClient();
}

#endif // WEB_H