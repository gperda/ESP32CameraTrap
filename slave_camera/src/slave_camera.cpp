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
#include <WebSocketsClient.h>
#include "esp_camera.h"
#define CAMERA_MODEL_ESP32S3_EYE
#include <camera_pins.h>
#include <sd_read_write.h>
#include <ws2812.h>
#include <esp_now.h>
#include <esp_sleep.h>
#include <esp_wifi.h>
#include <time.h>
#include <ESPmDNS.h>
#include "FS.h"
#include "SD_MMC.h"
#include "driver/rtc_io.h"
#include <ota_update.h>

// =================== CONFIGURATION ===================
#define CAMERA_ID "cam2"   // ← This is the only difference from cam1
#define MAX_FRAME_SIZE 1048576
#define FIRMWARE_VERSION  "v2.1.0"
#define FIRMWARE_DEVICE   "slave_camera"
#define GITHUB_REPO       "gperda/ESP32CameraTrap"
#define TRIGGER_WAKEUP_PIN GPIO_NUM_21

#define MAX_WS_WAIT_TIME_MS 4000
#define MAX_WIFI_WAIT_TIME_MS 6000

#ifndef CF_ACCESS_CLIENT_ID
#define CF_ACCESS_CLIENT_ID
#endif

#ifndef CF_ACCESS_CLIENT_SECRET
#define CF_ACCESS_CLIENT_SECRET
#endif

#ifndef MYSSID
#define MYSSID
#endif

#ifndef MYPASSWORD
#define MYPASSWORD
#endif

#ifndef WS_URL
#define WS_URL
#endif

#ifndef REGISTER_TOKEN
#define REGISTER_TOKEN
#endif

#define STRINGIFY_IMPL(x) #x
#define STRINGIFY(x) STRINGIFY_IMPL(x)

String normalizeBuildFlagString(const char* rawValue) {
  String value = String(rawValue);
  if (value.length() >= 2 && value[0] == '"' && value[value.length() - 1] == '"') {
    value = value.substring(1, value.length() - 1);
  }
  return value;
}
// const char* server_hostname = "3dom";
// const uint16_t server_port = 3000;

//String ws_url;
// =================== Globals ===================
WebSocketsClient client;
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

RTC_DATA_ATTR bool otaPendingRTC = false;
volatile bool shouldOTA = false;
volatile bool isUpToDate = false;
// uint64_t* timestampsToSend = nullptr;
// volatile uint64_t framesCount = 0;


extern const char ca_cert_start[] asm("_binary_ca_cert_start");

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
  config.xclk_freq_hz = 16000000;
  config.frame_size = FRAMESIZE_5MP;
  config.pixel_format = PIXFORMAT_JPEG; // for streaming
  config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
  config.fb_location = CAMERA_FB_IN_PSRAM;
  config.jpeg_quality = 12;
  config.fb_count = 1;
  
  // if PSRAM IC present, init with UXGA resolution and higher JPEG quality
  // for larger pre-allocated frame buffer.
  if(psramFound()){
    config.jpeg_quality = 5;
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
  // s->set_vflip(s, 1); // flip it back
  s->set_hmirror(s, 1);
  s->set_brightness(s, 1); // up the brightness just a bit
  s->set_saturation(s, 0); // lower the saturation
  delay(1000);

  Serial.println("Camera configuration complete!");
  return 1;
}

// bool resolveServer() {
//   IPAddress serverIP = MDNS.queryHost(server_hostname);
//   if (serverIP == IPAddress(0, 0, 0, 0)) return false;
//   ws_url = "ws://" + serverIP.toString() + ":" + String(server_port) + "/ws";
//   Serial.printf("Resolved %s\n", ws_url.c_str());
//   return true;
// }

uint64_t timems(struct timeval tv_now){
  return (uint64_t)tv_now.tv_sec * 1000000L + (int64_t)tv_now.tv_usec;
}

// =================== WebSocket Callbacks ===================
void onWsEvent(WStype_t type, uint8_t * payload, size_t length) {
  switch (type) {
    case WStype_CONNECTED:
      Serial.println("WS: connected");
      wsConnected = true;
      client.sendTXT("register:" CAMERA_ID ":" + normalizeBuildFlagString(STRINGIFY(REGISTER_TOKEN)) + ":" + String(FIRMWARE_VERSION));
      break;
    case WStype_DISCONNECTED:
      Serial.println("WS: disconnected");
      wsConnected = false;
      break;
    case WStype_TEXT:
      if (payload && length > 0) {
        String data = String((char*)payload);
        Serial.printf("\u2190 %s\n", data.c_str());
        if (data == "capture") {
          shouldCapture = true;
        }
        if (data == "slave_ota_update"){
          shouldOTA = true;
          otaPendingRTC = true;
        }
      }
      break;
    case WStype_PING:
    case WStype_PONG:
    default:
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
        Serial.println("Received send signal");
        shouldSend = true;
    }
}

void connectToWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(STRINGIFY(MYSSID), STRINGIFY(MYPASSWORD));
  WiFi.setSleep(false);
  Serial.print("WiFi ");
  
  unsigned long t = millis();
  while (WiFi.status() != WL_CONNECTED && millis() -t < MAX_WIFI_WAIT_TIME_MS) { delay(500); Serial.print("."); }
  if(WiFi.status() == WL_CONNECTED){    
    WiFi.config(WiFi.localIP(), WiFi.gatewayIP(), WiFi.subnetMask(),
                IPAddress(1,1,1,1), IPAddress(8,8,8,8));
    Serial.printf(" OK  IP=%s\n", WiFi.localIP().toString().c_str());
  
    if (!MDNS.begin(CAMERA_ID))
      Serial.println("mDNS init failed");
    else
      Serial.printf("mDNS started. Device is %s.local\n", CAMERA_ID);
  }
}

bool waitForWsConnected(uint32_t timeoutMs) {
  unsigned long t0 = millis();
  while (millis() - t0 < timeoutMs) {
    client.loop();
    if (client.isConnected()) {
      return true;
    }
    delay(5);
  }
  return client.isConnected();
}


// =================== Connect to WS ===================
void connectWS() {
  if(WiFi.status() == WL_CONNECTED){
    //bool serverResolved = false;
    bool serverResolved = true;
    // if (ws_url.isEmpty()) {
    //   Serial.printf("Resolving %s.local", server_hostname);
    //   unsigned long t = millis();
    //   serverResolved = resolveServer();
    //   while (!serverResolved && millis() - t < MAX_WS_WAIT_TIME_MS) { delay(500); Serial.print("."); }
    // }
    if(serverResolved){
      Serial.printf("\nConnecting to %s …\n", STRINGIFY(WS_URL));
      String cfClientId = normalizeBuildFlagString(STRINGIFY(CF_ACCESS_CLIENT_ID));
      String cfClientSecret = normalizeBuildFlagString(STRINGIFY(CF_ACCESS_CLIENT_SECRET));
      if (cfClientId.length() > 0 && cfClientSecret.length() > 0) {
        String accessHeaders =
          String("CF-Access-Client-Id: ") + cfClientId + "\r\n" +
          "CF-Access-Client-Secret: " + cfClientSecret;
        client.setExtraHeaders(accessHeaders.c_str());
      }
      client.beginSslWithCA(STRINGIFY(WS_URL), 443, "/ws", ca_cert_start);
      client.onEvent(onWsEvent);
      bool wsReady = waitForWsConnected(MAX_WS_WAIT_TIME_MS);
      if (!wsReady) {
        Serial.println("WS did not reach connected state within timeout");
      }
    }
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

  if (client.isConnected()) {
    client.sendBIN((const uint8_t*)g_sendBuf, totalLen);
    Serial.println("WS send OK");
  } else {
    Serial.println("WS not available, skipping send");
  }
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

  String message = String(timestamp) + "\n";
  appendFile(SD_MMC, "/sendlist.txt", message.c_str());

  esp_camera_fb_return(fb);  // Return immediately after write
  return 1;
}

void goToSleep() {
  Serial.println("Going to sleep");
  Serial.flush();

  esp_wifi_stop();
  WiFi.mode(WIFI_OFF);
  ws2812SetColor(1);

  esp_sleep_enable_ext0_wakeup(TRIGGER_WAKEUP_PIN, 1);
  esp_deep_sleep_start();
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



// =================== OTA Update ===================
bool performOTAIfAvailable() {
  return ::performOTAIfAvailable(FIRMWARE_DEVICE, FIRMWARE_VERSION, GITHUB_REPO, &isUpToDate);
}

// =================== Arduino Setup ===================
void setup() {
  Serial.begin(115200);
  Serial.printf("\n=== ESP32-CAM [%s] ===\n", CAMERA_ID);
  ws2812Init();

  esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();
  if (cause == ESP_SLEEP_WAKEUP_EXT0) {
      Serial.println("Slave woke from deep sleep via trigger");
  } else {
      Serial.println("Slave cold boot.");
      goToSleep();
  }
  initEspNow();
  sdmmcInit();
  createDir(SD_MMC, "/camera");
  ws2812SetColor(2);

  // Resume OTA that was interrupted by WiFi failure on a prior wakeup
  if (otaPendingRTC) { shouldOTA = true; }
}

// =================== Arduino Loop ===================
void loop() {

  if (shouldCapture) {
    Serial.println("ShoulCapture loop");
    initCamera();
    //Serial.printf("Sync timestamp: %llu us\n", captureTimestamp);
    uint8_t ack = 0xCC;
    Serial.print("Slave ready: ");
    esp_now_send(masterMAC, &ack, 1);
    if(captureToSD(captureTimestamp) == 0)
        Serial.println("Error with capture");
  } 
  if (shouldSend) {
    ws2812SetColor(3);
    uint8_t ack = 0xCC;
    Serial.print("Slave ready: ");
    esp_now_send(masterMAC, &ack, 1);
    std::vector<String> flist = getSendList(SD_MMC, "/sendlist.txt");
    esp_now_deinit();
    connectToWiFi();
    connectWS();
    unsigned long pollEnd = millis() + 1000;
    while (millis() < pollEnd) {
      client.loop();
      delay(10);
    }
    if(WiFi.status() == WL_CONNECTED && client.isConnected()){
      // Allocate the single send buffer from PSRAM once at startup
      g_sendBuf = (uint8_t*)ps_malloc(sizeof(Header) + MAX_FRAME_SIZE);
      if (!g_sendBuf) {
        Serial.println("FATAL: Could not allocate send buffer in PSRAM");
        while (true) delay(1000);  // Halt — nothing will work without this
      }
      
      for (const String& line : flist) {
        uint64_t value = strtoull(line.c_str(), nullptr, 10);
        sendFromSD(value);
      }
      free(g_sendBuf);
      deleteFile(SD_MMC, "/sendlist.txt");
      removeDirRecursive(SD_MMC, "/camera");
    } else {
      Serial.println("WiFi currently unavailable, will send later");
    }
    if (shouldOTA) {
      ws2812SetColor(3);
      if (WiFi.status() != WL_CONNECTED) {
        Serial.println("[OTA] WiFi unavailable, will retry on next wakeup");
        goToSleep();
      }
      if (performOTAIfAvailable()) {
        otaPendingRTC = false;
        if(!isUpToDate)
          ESP.restart();
      }
      // on failure: otaPendingRTC stays true, retries next wakeup
    }
  }
  
  // Sleep after doing something
  if (shouldSend || shouldCapture)
    goToSleep();
}
