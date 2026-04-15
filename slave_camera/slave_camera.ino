/*
 * ESP32-CAM #2 — Dual Camera WebSocket Client
 * Library: ArduinoWebsockets by Gil Maimon
 *
 * Install via Arduino Library Manager → search "ArduinoWebsockets" by Gil Maimon
 * Board setting: AI-Thinker ESP32-CAM
 *
 * This file is identical to cam1 except CAMERA_ID is "cam2".
 */

#include <WiFi.h>
#include <ArduinoWebsockets.h>
#include "esp_camera.h"
#define CAMERA_MODEL_ESP32S3_EYE
#include "camera_pins.h"
#include "sd_read_write.h"
#include <esp_now.h>
#include <esp_sleep.h>
#include <esp_wifi.h>
#include "ws2812.h"
#include <time.h>
#include <ESPmDNS.h>
#include "FS.h"
#include "SD_MMC.h"
#include "driver/rtc_io.h"

using namespace websockets;

// =================== CONFIGURATION ===================
#define CAMERA_ID "cam2"   // ← This is the only difference from cam1
#define MAX_FRAME_SIZE 228000
#define TRIGGER_WAKEUP_PIN GPIO_NUM_21

const char* ssid     = "Redmi Note 11S";
const char* password = "donatella";

// Point this at the IP running server.js
//const char* ws_url = "ws://10.167.166.163:3000/ws";
const char* server_hostname = "3dom";
const uint16_t server_port = 3000;

String ws_url;
// =================== Globals ===================
WebsocketsClient client;
bool wsConnected             = false;
unsigned long lastReconnect  = 0;
const unsigned long RECONNECT_MS = 5000;


static uint8_t* g_sendBuf = nullptr;

int frameCount =0;
uint8_t masterMAC[] = {0xD0, 0xCF, 0x13, 0x26, 0xE0, 0x6C};

typedef struct struct_message {
  uint64_t timestamp;
} struct_message;

typedef struct ws_message {
  uint8_t* payload;
  size_t size;
} ws_message;

typedef struct Header {
  char camera_id[8];   // fixed size
  uint64_t timestamp;  // milliseconds
  uint64_t data_len;   // image size
} Header;

struct SyncPacket {
    uint8_t type;
    uint64_t timestamp_us;
    uint8_t framesCount;
};


volatile bool shouldCapture = false;
volatile bool shouldSend    = false;
volatile bool shouldConnect = false;
volatile bool shouldSleep   = false;
volatile uint64_t captureTimestamp = 0;
uint64_t* timestampsToSend = nullptr;
volatile uint64_t framesCount = 0;

struct_message out_msg;
struct_message in_msg;
String success;

// =================== Camera ===================
int initCamera(void) {
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
  config.xclk_freq_hz = 10000000;
  config.frame_size = FRAMESIZE_UXGA;
  config.pixel_format = PIXFORMAT_JPEG; // for streaming
  config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
  config.fb_location = CAMERA_FB_IN_PSRAM;
  config.jpeg_quality = 12;
  config.fb_count = 1;
  
  // if PSRAM IC present, init with UXGA resolution and higher JPEG quality
  // for larger pre-allocated frame buffer.
  if(psramFound()){
    config.jpeg_quality = 10;
    config.fb_count = 2;
    config.grab_mode = CAMERA_GRAB_LATEST;
  } else {
    // Limit the frame size when PSRAM is not available
    config.frame_size = FRAMESIZE_SVGA;
    config.fb_location = CAMERA_FB_IN_DRAM;
  }

  //config.frame_size = FRAMESIZE_QVGA;

  // camera init
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return 0;
  }

  sensor_t * s = esp_camera_sensor_get();
  // // initial sensors are flipped vertically and colors are a bit saturated
  s->set_vflip(s, 1); // flip it back
  s->set_brightness(s, 1); // up the brightness just a bit
  s->set_saturation(s, 0); // lower the saturation
  delay(1000);

  Serial.println("Camera configuration complete!");
  return 1;
}

bool resolveServer() {
  IPAddress serverIP = MDNS.queryHost(server_hostname);
  if (serverIP == IPAddress(0, 0, 0, 0)) return false;
  ws_url = "ws://" + serverIP.toString() + ":" + String(server_port) + "/ws";
  Serial.printf("Resolved %s\n", ws_url.c_str());
  return true;
}

uint64_t timems(struct timeval tv_now){
  return (uint64_t)tv_now.tv_sec * 1000000L + (int64_t)tv_now.tv_usec;
}

// =================== WebSocket Callbacks ===================
void onMessage(WebsocketsMessage msg) {
  if (msg.isText()) {
    String data = msg.data();
    Serial.printf("← %s\n", data.c_str());
    if (data == "capture") {
      shouldCapture = true;
    }
  }
}

void onEvent(WebsocketsEvent event, String data) {
  switch (event) {
    case WebsocketsEvent::ConnectionOpened:
      Serial.println("WS: connected");
      wsConnected = true;
      client.send("register:" CAMERA_ID);
      break;

    case WebsocketsEvent::ConnectionClosed:
      Serial.println("WS: disconnected");
      wsConnected = false;
      break;

    case WebsocketsEvent::GotPing:
      break;

    case WebsocketsEvent::GotPong:
      break;
  }
}


void onSend(const uint8_t *mac_addr, esp_now_send_status_t status){
  Serial.print(status == ESP_NOW_SEND_SUCCESS ? "Delivered\n" : "Delivery Fail\n");
}

void onReceive(const esp_now_recv_info_t* info, const uint8_t* data, int len) {
    if (len < (int)sizeof(SyncPacket)) return;

    SyncPacket pkt;
    memcpy(&pkt, data, sizeof(pkt));

    if (pkt.type == 0x01) {
        captureTimestamp = pkt.timestamp_us;
        shouldCapture    = true;
        uint8_t ack      = 0xBB;
        Serial.print("Capture signal: ");
        esp_now_send(masterMAC, &ack, 1);
    } else if (pkt.type == 0x02) {
        shouldConnect = true;
    } else if (pkt.type == 0x03){
        shouldSleep = true;
    } else if (pkt.type == 0x04){
        shouldSend = true;
        framesCount = pkt.framesCount;
        timestampsToSend = (uint64_t*)ps_malloc(framesCount * sizeof(uint64_t));
        if (timestampsToSend == nullptr) return; 
        memcpy(timestampsToSend, data + sizeof(SyncPacket), framesCount * sizeof(uint64_t));
    }
}

void connectToWiFi(){
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  WiFi.setSleep(false);

  Serial.print("WiFi ");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.printf(" OK  IP=%s\n", WiFi.localIP().toString().c_str());

  if (!MDNS.begin(CAMERA_ID)){
    Serial.println("mDNS init failed");
  }else{
    Serial.printf("mDNS started. Device is %s.local\n", CAMERA_ID);
  }
}


// =================== Connect ===================
void connectWS() {
  if (ws_url.isEmpty()) {
    Serial.printf("Resolving %s.local", server_hostname);
    while (!resolveServer()) { delay(100); Serial.print("."); }
  }
  Serial.printf("Connecting to %s …\n", ws_url.c_str());
  client.onMessage(onMessage);
  client.onEvent(onEvent);

  bool ok = client.connect(ws_url.c_str());
  if (!ok) {
    Serial.println("WS connection failed — will retry");
    ws_url = "";
  }
}

// =================== Send from SD (single reused buffer) ===================
// Reads the JPEG directly into g_sendBuf (after the Header) and sends it.
// No extra malloc — g_sendBuf was allocated once at startup.
void sendFromSD(uint64_t timestamp) {
  if (!g_sendBuf) {
    Serial.println("Send buffer not allocated");
    return;
  }

  String path = "/camera/" + String(timestamp) + ".jpg";
  File file = SD_MMC.open(path);
  if (!file) {
    Serial.println("Failed to open: " + path);
    return;
  }

  size_t fileSize = file.size();
  if (fileSize == 0 || fileSize > MAX_FRAME_SIZE) {
    Serial.printf("Invalid file size: %u\n", fileSize);
    file.close();
    return;
  }

  // Read JPEG directly into the send buffer, after where the header will sit
  uint8_t* jpegDst = g_sendBuf + sizeof(Header);
  size_t   bytesRead = file.read(jpegDst, fileSize);
  file.close();

  if (bytesRead != fileSize) {
    Serial.printf("Read mismatch: got %u, expected %u\n", bytesRead, fileSize);
    return;
  }

  // Build header in-place at the start of g_sendBuf
  Header* hdr = (Header*)g_sendBuf;
  memset(hdr, 0, sizeof(Header));
  strncpy(hdr->camera_id, CAMERA_ID, sizeof(hdr->camera_id));
  hdr->timestamp = timestamp;
  hdr->data_len  = fileSize;

  size_t totalLen = sizeof(Header) + fileSize;

  Serial.printf("Sending %s — header: %u B, jpeg: %u B, total: %u B\n",
                path.c_str(), sizeof(Header), fileSize, totalLen);

  if (!client.available()) {
    Serial.println("WS not available, skipping send");
    return;
  }

  bool ret = client.sendBinary((const char*)g_sendBuf, totalLen);
  Serial.println(ret ? "WS send OK" : "WS send Error");
  // delay(100);
}



// void onDataRcv(const uint8_t *mac_addr, const uint8_t *incomingData, int len){
//   memcpy(&in_msg, incomingData, sizeof(in_msg));
//   if(in_msg.timestamp == -1){
//     //connectToWiFi();

//     while (!wsConnected && millis() - lastReconnect > RECONNECT_MS) {
//       lastReconnect = millis();
//       connectWS();
//     }

//     for(int i=0;i<frameCount; i++) 
//       sendFromSD(ts[i]);
//     frameCount = 0;
//     client.close();
//     //WiFi.disconnect();
//     Serial.println("Going to sleep");
//     //esp_light_sleep_start();
//   }

//   Serial.println("Receiving picture: " + in_msg.timestamp);
//   ts[frameCount++] = in_msg.timestamp;
//   uint64_t timestamp = captureAndSave(in_msg.timestamp);
//   out_msg.timestamp = timestamp-in_msg.timestamp;
//   esp_now_send(masterCameraAddress, (uint8_t*) &out_msg, sizeof(out_msg));
// }

int captureToSD(uint64_t timestamp) {
  uint64_t t = esp_timer_get_time();
  Serial.print("Capturing -> ");
  camera_fb_t* fb = esp_camera_fb_get();
  Serial.printf("%llu\n", esp_timer_get_time()-t);
  if (!fb) {
    Serial.println("Capture failed");
    return 0;
  }

  if (fb->len > MAX_FRAME_SIZE) {
    Serial.printf("Frame too large (%u bytes), skipping\n", fb->len);
    esp_camera_fb_return(fb);
    return 0;
  }

  // Write directly from the camera DMA buffer — no copy needed
  String path = "/camera/" + String(timestamp) + ".jpg";
  writejpg(SD_MMC, path.c_str(), fb->buf, fb->len);
  Serial.printf("Saved %u bytes → %s\n", fb->len, path.c_str());

  esp_camera_fb_return(fb);  // Return immediately after write
  return 1;
}

void initEspNow() {
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed, going to sleep");
    goToSleep();
  }

  esp_now_register_send_cb(esp_now_send_cb_t(onSend));
  esp_now_register_recv_cb(esp_now_recv_cb_t(onReceive));

  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, masterMAC, 6);
  peer.channel = 0;
  peer.encrypt = false;
  if (esp_now_add_peer(&peer) != ESP_OK){
    Serial.println("Failed to add peer");
    goToSleep();
  }
}

void goToSleep() {
  Serial.println("Going to sleep");
  Serial.flush();

  esp_wifi_stop();
  WiFi.mode(WIFI_OFF);

  esp_sleep_enable_ext0_wakeup(TRIGGER_WAKEUP_PIN, 1);
  esp_deep_sleep_start();
}

// =================== Arduino Setup ===================
void setup() {
  Serial.begin(115200);
  Serial.printf("\n=== ESP32-CAM [%s] ===\n", CAMERA_ID);

  esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();
  if (cause == ESP_SLEEP_WAKEUP_EXT0) {
      Serial.println("Slave woke from deep sleep via trigger");
  } else {
      Serial.println("Slave cold boot.");
      goToSleep();
  }
  initEspNow();
  sdmmcInit();
  //removeDir(SD_MMC, "/camera");
  createDir(SD_MMC, "/camera");
  initCamera();


}

// =================== Arduino Loop ===================
void loop() {
  if (shouldCapture) {
    shouldCapture = false;
    //Serial.printf("Sync timestamp: %llu us\n", captureTimestamp);
    uint8_t ack = 0xCC;
    Serial.print("Slave ready: ");
    esp_now_send(masterMAC, &ack, 1);
    if(captureToSD(captureTimestamp) == 0)
        Serial.println("Error with capture");
    goToSleep(); //if images are stored and sent later
  } 
  // if (shouldConnect) {
  //   esp_now_deinit();
  //   connectToWiFi();
  //   connectWS();
  //   shouldConnect = false;
  //   // ACK esp_now_send()
  //   //esp_now_deinit();   // free the radio before switching to full WiFi
  //   //startWifiAndSend();
  //   //esp_now_deinit(); // Cam1 wakes cam2 over pin
  //   //goToSleep();
  // }
  if (shouldSend) {
    shouldSend = false;
    esp_now_deinit();
    connectToWiFi();
    connectWS();
    // Allocate the single send buffer from PSRAM once at startup
    g_sendBuf = (uint8_t*)ps_malloc(sizeof(Header) + MAX_FRAME_SIZE);
    if (!g_sendBuf) {
      Serial.println("FATAL: Could not allocate send buffer in PSRAM");
      while (true) delay(1000);  // Halt — nothing will work without this
    }
    Serial.printf("Send buffer allocated: %u bytes\n", sizeof(Header) + MAX_FRAME_SIZE);
    
    for(int i=0;i<framesCount;i++)
        sendFromSD(timestampsToSend[i]);
    free((void*)timestampsToSend);
    goToSleep();
  }
}
