#include "esp_wifi.h"
#include "esp_camera.h"
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include <esp32-hal-ledc.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <Arduino.h>

#define PORT 6060
#define FREQ 2000
#define LEFT_M0 13
#define LEFT_M1 12
#define RIGHT_M0 14
#define RIGHT_M1 15

#define MAXPAYLOADSIZE 1400

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

#define SPEED 180

#define WIFI_SSID    "PEDRO"
#define WIFI_PSSWD   "cavalos123"

typedef struct {
  uint32_t totalBytes;     // Total size of the image in bytes
  uint16_t packetNumber;   // The sequence number of the current packet
  uint16_t totalPackets;   // The total number of packets for this image
} __attribute__((packed)) ImageHeader;

static WiFiUDP udpClient;
static WiFiClient client;
const IPAddress ipServer(192, 168, 5, 18);  // Troca isso para o ip do servidor
const char idJSON[] = "{\"type\":\"register_car\",\"carId\":\"Mr.Car\"}";

unsigned long previous_time;

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
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;
  config.frame_size = FRAMESIZE_QVGA;
  config.jpeg_quality = 10;
  config.fb_count = 1;
  esp_camera_init(&config);

  sensor_t *s = esp_camera_sensor_get();
  if (s) {
    s->set_vflip(s, 1);
  }
}

inline void wifiSetup() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PSSWD);

  unsigned long startTime = millis();
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);  // Reduced delay for faster response
    Serial.println("Tentando se conectar");
  }
  Serial.println(WiFi.localIP());

  client.connect(ipServer, PORT);
  client.write(idJSON, sizeof(idJSON) - 1);
}

void setup() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);  // previne brownouts
  Serial.begin(115200);

  // Setup robot hardware first
  robotSetup();

  cameraSetup();

  wifiSetup();

  Serial.println("Setup completo!");
}

void loop() {
  // Check WiFi connection
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi desconectado, reconectando...");
    wifiSetup();
  }

  // Check TCP connection and reconnect if needed
  /*if (!client.connected()) {
    Serial.println("Reconectando ao servidor...");
    if (client.connect(ipServer, PORT)) {
      client.flush();
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
        robotStop();
        break;
    }
    client.flush();
  }*/

  sendCameraToServer();
}

inline void robotSetup() {

  // Pins for Motor Controller
  /*ledcSetup(3, 2000, 8); 2000 hz PWM, 8-bit resolution and range from 0 to 255 
  ledcSetup(4, 2000, 8); /* 2000 hz PWM, 8-bit resolution and range from 0 to 255 
  ledcSetup(5, 2000, 8); /* 2000 hz PWM, 8-bit resolution and range from 0 to 255 
  ledcSetup(6, 2000, 8); /* 2000 hz PWM, 8-bit resolution and range from 0 to 255 */
  ledcAttachChannel(RIGHT_M0, FREQ, 8, 3);
  ledcAttachChannel(RIGHT_M1, FREQ, 8, 4);
  ledcAttachChannel(LEFT_M0, FREQ, 8, 5);
  ledcAttachChannel(LEFT_M1, FREQ, 8, 6);

  pinMode(33, OUTPUT);

  // Make sure we are stopped
  robotStop();

  ledcWrite(8, SPEED);

  Serial.println("Robot configurado!");
}

inline void sendCameraToServer() {
  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) {
    esp_camera_fb_return(fb);
    return;
  }

  uint16_t totalPackets = ceil((double)fb->len / MAXPAYLOADSIZE);

  for (int i = 0; i < totalPackets; i++) {
    ImageHeader header;
    header.totalBytes = fb->len;
    header.packetNumber = i;
    header.totalPackets = totalPackets;

    // Calculate the size and position of the current data chunk
    uint16_t offset = i * MAXPAYLOADSIZE;
    uint16_t currentChunkSize = min(MAXPAYLOADSIZE, (int) fb->len - offset);

    // Create a buffer that combines the header and the image chunk
    uint8_t buffer[sizeof(ImageHeader) + currentChunkSize];
    memcpy(buffer, &header, sizeof(ImageHeader));
    memcpy(buffer + sizeof(ImageHeader), fb->buf + offset, currentChunkSize);

    // Send the combined buffer
    udpClient.beginPacket(ipServer, PORT);
    udpClient.write(buffer, sizeof(ImageHeader) + currentChunkSize);
    udpClient.endPacket();

    delay(5); // Small delay to prevent network congestion
  }
  
  esp_camera_fb_return(fb);
}

void robotStop() {
  ledcWrite(3, LOW);  // RIGHT_M0
  ledcWrite(4, LOW);  // RIGHT_M1
  ledcWrite(5, LOW);  // LEFT_M0
  ledcWrite(6, LOW);  // LEFT_M1
}

void robotFwd() {
  ledcWrite(3, SPEED);
  ledcWrite(4, 0);
  ledcWrite(5, SPEED);
  ledcWrite(6, 0);
}

void robotBack() {
  ledcWrite(3, 0);
  ledcWrite(4, SPEED);
  ledcWrite(5, 0);
  ledcWrite(6, SPEED);
}

void robotRight() {
  ledcWrite(3, SPEED);
  ledcWrite(4, 0);
  ledcWrite(5, 0);
  ledcWrite(6, SPEED);
}

void robotLeft() {
  ledcWrite(3, 0);
  ledcWrite(4, SPEED);
  ledcWrite(5, SPEED);
  ledcWrite(6, 0);
}