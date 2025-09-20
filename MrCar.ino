#include "esp_camera.h"
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include <WiFi.h>
#include <WebSocketsClient.h>
#include <ArduinoJson.h>
#include <cstdint>
#include "camera_pins.h"

#define FREQ 2000
#define LEFT_M0 13
#define LEFT_M1 12
#define RIGHT_M0 14
#define RIGHT_M1 15

/** Esses macros definem a configuração de qual rede, WiFi e porta será executado o WS
*   Use 1 - para sua rede e/ou servidor seu
*   Use 0 - para a rede e/ou servidor do Ricado 
*/
#if 1
#define WIFI_SSID    "PEDRO"
#define WIFI_PSSWD   "cavalos123"
#else
#define WIFI_SSID    "UENP-RACE"
#define WIFI_PSSWD   "competidor"
#endif
#if 1
#define WS_SERVER    "192.168.5.18"
#define PORT         8080
#else
#define WS_SERVER    "uenp-race-de8caa13ef2a.herokuapp.com"
#define PORT         80 //Após pedir ajuda para algumas ias aparentemente é necessário entrar na porta 80 (?). Não sei, se alguem souber compartilhe comigo
#endif
#define WS_PATH      "/ws"

WebSocketsClient webSocket;
String carId = "Mr.Car";
String json;

using ArduinoJson::JsonDocument;

void startCameraServer();

/*
* Função responszável por agrantir que o carrinho mova, altere isso caso use o servo
*/
void moveRobot(int16_t x, int16_t y);

void handleEvent(WStype_t type, uint8_t * payload, size_t length) {
  JsonDocument doc;
  JsonDocument msg;
  switch(type) {
    case WStype_CONNECTED: {
      String streamURL = "http://" + WiFi.localIP().toString() + ":81/stream";
      doc["type"] = "register_car";
      doc["carId"] = carId;
      doc["streamUrl"] = streamURL;
      serializeJson(doc, json);
      webSocket.sendTXT(json.c_str());
      break;
    }
    case WStype_TEXT: {
      DeserializationError error = deserializeJson(msg, payload);
      if (error) {
        Serial.println("Erro ao interpretar JSON");
        return;
      }
      
      String msgType = msg["type"] | "";
      if(msgType == "registered") {
        Serial.println("Carro registrado!");
        return;
      }

      if(msgType == "command") {
        String cmd = msg["command"] | "";
        Serial.println(cmd);
        if(cmd == "forward") {
          moveRobot(0, 180);
          return;
        }

        if(cmd == "backward") {
          moveRobot(0, -180);
          return;
        }

        if(cmd == "left") {
          moveRobot(180, 15);
          return;
        }

        if(cmd == "right") {
          moveRobot(180, 15);
          return;
        }

        moveRobot(0, 0);
        return;
      }

      if(msgType == "status") {
        String status = msg["status"] | "";
        Serial.println(status);
        return;
      }

      if(msgType == "analog_command") {
        int16_t x = (int16_t) (msg["x"] | 0.0)*255;
        int16_t y = (int16_t) (msg["y"] | 0.0)*255;
        moveRobot(x, y);
        return;
      }
      String errMsg = msg["message"] | "Erro desconhecido";
      Serial.println(errMsg);
    }
    default:
    break;
  }
}

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
  config.jpeg_quality = 12;
  config.fb_count = 2;
  esp_camera_init(&config);

  sensor_t *s = esp_camera_sensor_get();
  if (s) {
    s->set_vflip(s, 1);
  }
}

inline void wifiSetup() {
  WiFi.begin(WIFI_SSID, WIFI_PSSWD);
  WiFi.setSleep(false);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);  // Reduced delay for faster response
  }

  startCameraServer();
  Serial.printf("http://%s\n", WiFi.localIP().toString().c_str());
  webSocket.begin(WS_SERVER, PORT, WS_PATH);
  webSocket.onEvent(handleEvent);
  webSocket.setReconnectInterval(5000);
}

void setup() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);  // previne brownouts
  Serial.begin(115200);

  // Setup robot hardware first
  robotSetup();

  cameraSetup();

  wifiSetup();
}

void loop() {
  webSocket.loop();

  delay(1);
}


/** Função responsávle por informar os pinos necessários para o carrinho andar, ajuste essa função também no caso de ter um servo motor
* 
*/
inline void robotSetup() {
  // Pins for Motor Controller
  ledcAttach(RIGHT_M0, FREQ, 8);
  ledcAttach(RIGHT_M1, FREQ, 8);
  ledcAttach(LEFT_M0, FREQ, 8);
  ledcAttach(LEFT_M1, FREQ, 8);

  pinMode(33, OUTPUT);

  // Make sure we are stopped
  moveRobot(0, 0);

  ledcWrite(8, 255);
}

void moveRobot(int16_t speedX, int16_t speedY) {
  int16_t rightSpeed = speedY - speedX;
  uint8_t rightPlus  = static_cast<uint8_t>(max((int16_t) 0, rightSpeed));
  uint8_t rightNeq   = static_cast<uint8_t>(max((int16_t) 0, (int16_t) -rightSpeed));

  int16_t leftSpeed  = speedY + speedX;
  uint8_t leftPlus   = static_cast<uint8_t>(max((int16_t) 0, (int16_t) leftSpeed));
  uint8_t leftNeq    = static_cast<uint8_t>(max((int16_t) 0, (int16_t) -leftSpeed));

  // Aplica PWM nos 2 motores
  ledcWrite(RIGHT_M0, rightPlus);
  ledcWrite(RIGHT_M1, rightNeq);

  ledcWrite(LEFT_M0,  leftPlus);
  ledcWrite(LEFT_M1,  leftNeq);
}