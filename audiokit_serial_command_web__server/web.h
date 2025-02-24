#ifndef WEB_H
#define WEB_H

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <SD.h>
#include <vector>
#include <algorithm>

const char* ssid = "ESP32_Hotspot";
const char* password = "password123";

WebServer server(80);

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

void listDirectory(String path) {
  String output = "<html><head><title>WAV File Browser</title>";
  output += "<style>";
  output += "body { font-family: Arial, sans-serif; margin: 0; padding: 20px; background-color: #f0f0f0; }";
  output += "h1 { color: #333; }";
  output += "ul { list-style-type: none; padding: 0; }";
  output += "li { margin-bottom: 10px; }";
  output += "a { color: #1a73e8; text-decoration: none; }";
  output += "a:hover { text-decoration: underline; }";
  output += ".folder { font-weight: bold; }";
  output += ".file { margin-left: 20px; }";
  output += "</style>";
  output += "</head><body>";
  output += "<h1>WAV File Browser</h1>";
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

void setupWebServer() {
  WiFi.softAP(ssid, password);
  IPAddress myIP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(myIP);

  server.on("/", HTTP_GET, handleFileList);

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