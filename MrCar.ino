#include "esp_wifi.h"
#include "esp_camera.h"
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include <esp32-hal-ledc.h>
#include <WiFi.h>
#include <WiFiUdp.h>

#define PORT 6060

#define LEFT_M0     13
#define LEFT_M1     12
#define RIGHT_M0    14
#define RIGHT_M1    15

#define UP 1
#define DOWN 2
#define RIGHT 3
#define LEFT 4

static WiFiClient client;
static WiFiUDP udpClient;
const IPAddress ipServer(192, 168, 137, 1);   // Troca isso para o ip do servidor
const char *ssid = "PEDRO";    // Troque isso para o nome do Access Point
const char *psswd = "cavalos123";       // Mesma coisa para senha 
const char idJSON[] = "{\"type\":\"register_car\",\"carId\":\"Mr.Car\"}";
unsigned long previous_time;

// Connection timeout constants
const unsigned long WIFI_TIMEOUT = 20000;  // 20 seconds
const unsigned long CLIENT_TIMEOUT = 10000; // 10 seconds

inline void setupWifi(){  
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, psswd);

  unsigned long startTime = millis();
  while (WiFi.status() != WL_CONNECTED) {
    if (millis() - startTime > WIFI_TIMEOUT) {
      Serial.println("WiFi connection timeout! Restarting...");
      ESP.restart();
    }
    delay(500);  // Reduced delay for faster response
    Serial.println("Tentando se conectar");
  }
  Serial.println("WiFi conectado!");
  Serial.println(WiFi.localIP());
}

void setup() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); // previne brownouts
  Serial.begin(115200);
  Serial.println("Iniciando sistema...");
  
  // Setup robot hardware first
  robotSetup();
  
  setupWifi();
  
  // Attempt TCP connection with timeout
  unsigned long startTime = millis();
  while(!client.connected()) {
    if (millis() - startTime > CLIENT_TIMEOUT) {
      Serial.println("TCP connection timeout!");
      break;
    }
    Serial.println("Tentando conectar ao servidor TCP...");
    if (client.connect(ipServer, PORT)) {
      Serial.println("Conectado ao servidor TCP!");
      client.write(idJSON, sizeof(idJSON) - 1);
      break;
    }
    delay(1000);
  }

  // Setup UDP (this should not block indefinitely)
  if (udpClient.begin(PORT)) {
    Serial.println("UDP client iniciado com sucesso");
  } else {
    Serial.println("Falha ao iniciar UDP client");
  }
  
  Serial.println("Setup completo!");
}

void loop() {
  // Check WiFi connection
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi desconectado, reiniciando...");
    ESP.restart();
  }
  
  // Check TCP connection and reconnect if needed
  if (!client.connected()) {
    Serial.println("Reconectando ao servidor...");
    if (client.connect(ipServer, PORT)) {
      client.write(idJSON, sizeof(idJSON) - 1);
    }
  }
  
  // Process incoming commands with non-blocking read
  if (client.connected() && client.available()) {
    int command = client.read();
    Serial.printf("Comando recebido: %d\n", command);
    
    switch (command) {
      case UP:
        Serial.println("#UP");
        robotFwd();
        break;
      case DOWN:
        Serial.println("#DOWN");
        robotStop();
        break;
      case RIGHT:
        Serial.println("#RIGHT");
        robotRight();
        break;
      case LEFT:
        Serial.println("#LEFT");
        robotLeft();
        break;
      default:
        Serial.printf("Comando desconhecido: %d\n", command);
        break;
    }
  }
  
  // Add a small delay to prevent watchdog issues
  delay(10);
}

inline void robotSetup()
{
    Serial.println("Configurando robot...");
    
    // Pins for Motor Controller
    pinMode(LEFT_M0, OUTPUT);
    pinMode(LEFT_M1, OUTPUT);
    pinMode(RIGHT_M0, OUTPUT);
    pinMode(RIGHT_M1, OUTPUT);
    pinMode(33, OUTPUT);
    
    // Setup PWM channels with proper pin assignments
    ledcSetup(3, 2000, 8); /* 2000 hz PWM, 8-bit resolution */
    ledcSetup(4, 2000, 8); 
    ledcSetup(5, 2000, 8); 
    ledcSetup(6, 2000, 8); 
    
    // Attach pins to PWM channels
    ledcAttachPin(RIGHT_M0, 3);  // Pin 14 to channel 3
    ledcAttachPin(RIGHT_M1, 4);  // Pin 15 to channel 4
    ledcAttachPin(LEFT_M0, 5);   // Pin 13 to channel 5
    ledcAttachPin(LEFT_M1, 6);   // Pin 12 to channel 6
    
    // Make sure we are stopped
    robotStop();
    
    Serial.println("Robot configurado!");
}

void robotStop()
{
  ledcWrite(3, 0);  // RIGHT_M0
  ledcWrite(4, 0);  // RIGHT_M1
  ledcWrite(5, 0);  // LEFT_M0
  ledcWrite(6, 0);  // LEFT_M1
}

void robotFwd()
{
  // Move forward: RIGHT_M1 and LEFT_M1 active
  ledcWrite(3, 0);    // RIGHT_M0 off
  ledcWrite(4, 150);  // RIGHT_M1 on (reduced speed for safety)
  ledcWrite(5, 0);    // LEFT_M0 off  
  ledcWrite(6, 150);  // LEFT_M1 on
  
  previous_time = millis();
}

void robotBack()
{
  // Move backward: RIGHT_M0 and LEFT_M0 active
  ledcWrite(3, 150);  // RIGHT_M0 on
  ledcWrite(4, 0);    // RIGHT_M1 off
  ledcWrite(5, 150);  // LEFT_M0 on
  ledcWrite(6, 0);    // LEFT_M1 off
  
  previous_time = millis();
}

void robotRight()
{
  // Turn right: LEFT_M1 active, RIGHT_M0 active
  ledcWrite(3, 150);  // RIGHT_M0 on (reverse right wheel)
  ledcWrite(4, 0);    // RIGHT_M1 off
  ledcWrite(5, 0);    // LEFT_M0 off
  ledcWrite(6, 150);  // LEFT_M1 on (forward left wheel)
  
  previous_time = millis();
}

void robotLeft()
{
  // Turn left: RIGHT_M1 active, LEFT_M0 active
  ledcWrite(3, 0);    // RIGHT_M0 off
  ledcWrite(4, 150);  // RIGHT_M1 on (forward right wheel)
  ledcWrite(5, 150);  // LEFT_M0 on (reverse left wheel)
  ledcWrite(6, 0);    // LEFT_M1 off

  previous_time = millis();
}