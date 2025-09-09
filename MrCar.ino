#include "esp_wifi.h"
#include "esp_camera.h"
#include <WiFi.h>
#include <Arduino.h>

const WiFiClient client;
const IPAddress ipServer();   // Troca isso para o ip do servidor
const int port = 0;           // Troque isso também a porta do TCP
const char *ssid = "rede";    // Troque isso para o nome do Access Point
const char *psswd = "";       // 

inline void setupWifi(){
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); // previne brownouts
  Serial.begin(115200);
  Serial.println();
  
  WiFi.mode(WIFI_STA);
  
  WiFi.begin(ssid, psswd);

  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
  }
}

void setup() {
  setupWifi();
  
}

void loop() {
  
}
