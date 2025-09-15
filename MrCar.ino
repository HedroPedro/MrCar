#include "esp_wifi.h"
#include "esp_camera.h"
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include <esp32-hal-ledc.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <Arduino.h>

#define PORT 3000
#define LEFT_M0 13
#define LEFT_M1 12
#define RIGHT_M0 14
#define RIGHT_M1 15

#define UP 1
#define DOWN 2
#define RIGHT 3
#define LEFT 4
#define STOP 5

#define CAMERA_MODEL_AI_THINKER
#define PWDN_GPIO_NUM 32
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM 0
#define SIOD_GPIO_NUM 26
#define SIOC_GPIO_NUM 27

#define Y9_GPIO_NUM 35
#define Y8_GPIO_NUM 34
#define Y7_GPIO_NUM 39
#define Y6_GPIO_NUM 36
#define Y5_GPIO_NUM 21
#define Y4_GPIO_NUM 19
#define Y3_GPIO_NUM 18
#define Y2_GPIO_NUM 5
#define VSYNC_GPIO_NUM 25
#define HREF_GPIO_NUM 23
#define PCLK_GPIO_NUM 22

static WiFiClient client;
static WiFiUDP udpClient;
const IPAddress ipServer(192, 168, 5, 20);  // Troca isso para o ip do servidor
const char *ssid = "PEDRO";                  // Troque isso para o nome do Access Point
const char *psswd = "cavalos123";            // Mesma coisa para senha
const char idJSON[] = "{\"type\":\"register_car\",\"carId\":\"Mr.Car\"}";

unsigned long previous_time;

// Connection timeout constants
const unsigned long WIFI_TIMEOUT = 20000;    // 20 seconds
const unsigned long CLIENT_TIMEOUT = 10000;  // 10 seconds

inline void cameraSetup() {
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;
  config.frame_size = FRAMESIZE_QVGA;
  config.jpeg_quality = 12;
  config.fb_count = 1;
  esp_camera_init(&config);
}

inline void setupWifi() {
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
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);  // previne brownouts
  Serial.begin(115200);
  Serial.println("Iniciando sistema...");

  // Setup robot hardware first
  robotSetup();

  cameraSetup();

  setupWifi();

  // Attempt TCP connection with timeout
  unsigned long startTime = millis();
  Serial.println("Me conectando");
  while (!client.connected()) {
    if (millis() - startTime > CLIENT_TIMEOUT) {
      Serial.println("TCP connection timeout!");
      break;
    }
    /*Serial.println("Tentando conectar ao servidor TCP...");
    if (client.connect(ipServer, PORT)) {
      Serial.println("Conectado ao servidor TCP!");
      client.write(idJSON, sizeof(idJSON) - 1);
      break;
    }*/
    }
    delay(1000);
    client.flush();

  

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
    Serial.println("WiFi desconectado, reconectando...");
    setupWifi();
  }

  // Check TCP connection and reconnect if needed
  if (!client.connected()) {
    Serial.println("Reconectando ao servidor...");
    if (client.connect(ipServer, PORT)) {
      client.flush();
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
        robotBack();
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
        robotStop();
        break;
    }

    robotStop();
    client.flush();
  }

  sendCameraToServer();

  // Add a small delay to prevent watchdog issues
  delay(1000);
}

inline void robotSetup() {
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

inline void sendCameraToServer() {
  camera_fb_t *fb = esp_camera_fb_get();

  udpClient.beginPacket(ipServer, PORT);
  udpClient.write(fb->buf, fb->len);
  udpClient.endPacket();
  esp_camera_fb_return(fb);
}

void robotStop() {
  ledcWrite(3, 0);  // RIGHT_M0
  ledcWrite(4, 0);  // RIGHT_M1
  ledcWrite(5, 0);  // LEFT_M0
  ledcWrite(6, 0);  // LEFT_M1
}

void robotFwd() {
  // Move forward: RIGHT_M1 and LEFT_M1 active
  ledcWrite(3, 0);    // RIGHT_M0 off
  ledcWrite(4, 150);  // RIGHT_M1 on (reduced speed for safety)
  ledcWrite(5, 0);    // LEFT_M0 off
  ledcWrite(6, 150);  // LEFT_M1 on

  previous_time = millis();
}

void robotBack() {
  // Move backward: RIGHT_M0 and LEFT_M0 active
  ledcWrite(3, 150);  // RIGHT_M0 on
  ledcWrite(4, 0);    // RIGHT_M1 off
  ledcWrite(5, 150);  // LEFT_M0 on
  ledcWrite(6, 0);    // LEFT_M1 off

  previous_time = millis();
}

void robotRight() {
  // Turn right: LEFT_M1 active, RIGHT_M0 active
  ledcWrite(3, 150);  // RIGHT_M0 on (reverse right wheel)
  ledcWrite(4, 0);    // RIGHT_M1 off
  ledcWrite(5, 0);    // LEFT_M0 off
  ledcWrite(6, 150);  // LEFT_M1 on (forward left wheel)

  previous_time = millis();
}

void robotLeft() {
  // Turn left: RIGHT_M1 active, LEFT_M0 active
  ledcWrite(3, 0);    // RIGHT_M0 off
  ledcWrite(4, 150);  // RIGHT_M1 on (forward right wheel)
  ledcWrite(5, 150);  // LEFT_M0 on (reverse left wheel)
  ledcWrite(6, 0);    // LEFT_M1 off

  previous_time = millis();
}