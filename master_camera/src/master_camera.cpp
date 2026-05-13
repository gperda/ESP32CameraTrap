#include <WiFi.h>
#include <ArduinoWebsockets.h>
#include "esp_camera.h"
#define CAMERA_MODEL_ESP32S3_EYE
#include "camera_pins.h"
#include <sd_read_write.h>
#include <esp_now.h>
#include <esp_sleep.h>
#include <esp_wifi.h>
#include <time.h>
#include <ESPmDNS.h>
#include "FS.h"
#include "SD_MMC.h"
#include <Wire.h>
#include <ws2812.h>
#include <SparkFun_VL53L5CX_Library.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <HTTPUpdate.h>
#include <ArduinoJson.h>

using namespace websockets;

// =================== CONFIGURATION ===================
#define CAMERA_ID       "cam1"   // "cam1" for board 1, "cam2" for board 2
#define MAX_FRAME_SIZE  1048576
#define FIRMWARE_VERSION  "v1.1.0"
#define FIRMWARE_DEVICE   "master_camera"
#define GITHUB_REPO       "gperda/ESP32CameraTrap"
#define MOTIONSENSOR_PIN GPIO_NUM_19
#define TO_SLAVE_PIN    GPIO_NUM_45
#define TOF_SENSOR_PIN GPIO_NUM_47
#define TOF_SENSOR_WAIT_TIME_US 10000000
#define MAX_SESSION_FRAMES 10
#define MAX_WIFI_WAIT_TIME_MS 6000
#define MAX_WS_WAIT_TIME_MS 4000

// ── ToF pin / sensor settings ────────────────────────────────────────────────
#define TOF_SDA_PIN            41
#define TOF_SCL_PIN            42
#define TOF_I2C_SPEED          1000000 // 1 MHz
#define TOF_RANGING_FREQ_HZ    15
#define TOF_ZONES              64      // 8×8 grid
#define PROXIMITY_THRESHOLD_MM 200     // 50 cm
#define PROXIMITY_RATIO        0.5f    // alert when >50 % of valid zones are close
#define VALID_TARGET_STATUS    5       // VL53L5CX status code for a valid measurement

#define uS_TO_S_FACTOR 1000000ULL 

const char* ssid     = "Redmi Note 11S";
const char* password = "donatella";

const char* server_hostname = "3dom";
const uint16_t server_port  = 3000;
String ws_url = "";

//String ws_url = "wss://diana-adventure-belfast-stream.trycloudflare.com/ws";
const char* REGISTER_TOKEN = "reg-token";


// =================== Globals ===================
WebsocketsClient client;
volatile bool shouldCapture = false;
bool wsConnected            = false;

bool motionDetected = false;

static uint8_t* g_sendBuf = nullptr;

// ── ToF globals ──────────────────────────────────────────────────────────────
SparkFun_VL53L5CX tofSensor;
VL53L5CX_ResultsData tofData;
bool tofInitialised = false;

// =================== ESP-NOW COMMUNICATION ================
uint8_t slaveMAC[] = {0xD0, 0xCF, 0x13, 0x26, 0xFB, 0x54};

typedef struct Header {
  char     camera_id[8];   // fixed width
  uint64_t timestamp;      // microseconds
  uint64_t data_len;       // image bytes
} Header;

struct SyncPacket {
  uint8_t  type;           // 0x01 = sync, 0x02 = start sending
  uint64_t timestamp_us;
  uint8_t  framesCount;
};

volatile bool ackReceived = false;
volatile bool slaveReady  = false;

RTC_DATA_ATTR bool otaPendingRTC      = false;
volatile bool otaRequested = false;
volatile bool isUpToDate = false;

// =================== ToF helpers ===================

// ── Initialise the VL53L5CX once per wake cycle ─────────────────────────────
bool initToF() {
  Wire.begin(TOF_SDA_PIN, TOF_SCL_PIN);
  Wire.setClock(TOF_I2C_SPEED);

  if (!tofSensor.begin()) {
    Serial.println("[ToF] Sensor not found — check wiring");
    return false;
  }
  tofSensor.setResolution(TOF_ZONES);
  tofSensor.setRangingFrequency(TOF_RANGING_FREQ_HZ);
  tofSensor.startRanging();

  Serial.printf("[ToF] Ranging started at %d Hz\n", TOF_RANGING_FREQ_HZ);
  return true;
}


bool checkToFProximityAlert() {
  if (!tofInitialised) return false;

  // Wait for a valid measurement (max ~200 ms at 15 Hz)
  unsigned long t0 = millis();
  while (!tofSensor.isDataReady()) {
    if (millis() - t0 > 200) {
      Serial.println("[ToF] Timeout waiting for data");
      return false;
    }
    delay(5);
  }

  if (!tofSensor.getRangingData(&tofData)) {
    Serial.println("[ToF] Failed to retrieve ranging data");
    return false;
  }

  int validZones = 0, closeZones = 0;
  for (int i = 0; i < TOF_ZONES; i++) {
    if (tofData.target_status[i] == VALID_TARGET_STATUS) {
      validZones++;
      if (tofData.distance_mm[i] < PROXIMITY_THRESHOLD_MM) closeZones++;
    }
  }

  if (validZones == 0) return false;

  float ratio = (float)closeZones / (float)validZones;
  bool alert = ratio > PROXIMITY_RATIO;

  // Serial.printf("[ToF] valid=%d  close=%d  ratio=%.2f  alert=%s\n",
  //               validZones, closeZones, ratio, alert ? "YES" : "no");
  //  Serial.print("{\"distances\":[");
  // for (int i = 0; i < 64; i++) {
  //   Serial.print(tofData.distance_mm[i]);
  //   if (i < 63) Serial.print(",");
  // }

  // Serial.print("],\"status\":[");
  // for (int i = 0; i < 64; i++) {
  //   Serial.print(tofData.target_status[i]);
  //   if (i < 63) Serial.print(",");
  // }

  // Serial.print("],\"proximity_alert\":");
  // Serial.print(alert ? "true" : "false");

  // Serial.print(",\"v\":\"");
  // Serial.print("0.2");
  // Serial.println("\"}");
  
  return alert;
}

// =================== Camera ===================
int initCamera(void) {
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer   = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk     = XCLK_GPIO_NUM;
  config.pin_pclk     = PCLK_GPIO_NUM;
  config.pin_vsync    = VSYNC_GPIO_NUM;
  config.pin_href     = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn     = PWDN_GPIO_NUM;
  config.pin_reset    = RESET_GPIO_NUM;
  config.xclk_freq_hz   = 8000000;
  config.frame_size     = FRAMESIZE_5MP;
  config.pixel_format   = PIXFORMAT_JPEG;
  config.grab_mode      = CAMERA_GRAB_WHEN_EMPTY;
  config.fb_location    = CAMERA_FB_IN_PSRAM;
  config.jpeg_quality   = 12;
  config.fb_count       = 1;

  if (psramFound()) {
    config.jpeg_quality = 5;
    config.fb_count     = 2;
    config.grab_mode    = CAMERA_GRAB_LATEST;
  } else {
    config.frame_size  = FRAMESIZE_SVGA;
    config.fb_location = CAMERA_FB_IN_DRAM;
  }

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return 0;
  }

  sensor_t* s = esp_camera_sensor_get();
  //s->set_vflip(s, 1);
  s->set_brightness(s, 1);
  s->set_saturation(s, 0);

  Serial.println("Camera configuration complete!");
  delay(10);
  return 1;
}

bool resolveServer() {
  IPAddress serverIP = MDNS.queryHost(server_hostname);
  if (serverIP == IPAddress(0, 0, 0, 0)) return false;
  ws_url = "ws://" + serverIP.toString() + ":" + String(server_port) + "/ws";
  Serial.printf("Resolved %s\n", ws_url.c_str());
  return true;
}

// =================== WebSocket Callbacks ===================
void onMessage(WebsocketsMessage msg) {
  if (msg.isText()) {
    String data = msg.data();
    Serial.printf("← %s\n", data.c_str());
    if (data == "capture")    shouldCapture = true;
    if (data == "master_ota_update") { Serial.println("Received ota update"); otaRequested = true; otaPendingRTC = true;}
  }
}

void onEvent(WebsocketsEvent event, String data) {
  switch (event) {
    case WebsocketsEvent::ConnectionOpened:
      Serial.println("WS: connected");
      wsConnected = true;
      //client.send("register:" CAMERA_ID);
      client.send("register:" CAMERA_ID ":" + String(REGISTER_TOKEN));
      break;
    case WebsocketsEvent::ConnectionClosed:
      Serial.println("WS: disconnected");
      wsConnected = false;
      break;
    case WebsocketsEvent::GotPing:
    case WebsocketsEvent::GotPong:
      break;
  }
}

uint64_t timems(struct timeval tv_now) {
  return (uint64_t)tv_now.tv_sec * 1000000L + (int64_t)tv_now.tv_usec;
}

void onSend(const uint8_t* mac, esp_now_send_status_t status) {
  Serial.print(status == ESP_NOW_SEND_SUCCESS ? "Delivered\n" : "Delivery Fail\n");
}

void onReceive(const esp_now_recv_info_t* info, const uint8_t* data, int len) {
  if (len == 1 && data[0] == 0xBB){
    Serial.println("Capture ACK");
    ackReceived = true;
  } 
  if (len == 1 && data[0] == 0xCC){
    Serial.println("Slave Ready");
    slaveReady  = true;
  } 
}

void connectToWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  WiFi.setSleep(false);
  Serial.print("WiFi ");
  
  unsigned long t = millis();
  while (WiFi.status() != WL_CONNECTED && millis() -t < MAX_WIFI_WAIT_TIME_MS) { delay(500); Serial.print("."); }
  if(WiFi.status() == WL_CONNECTED){
    Serial.printf(" OK  IP=%s\n", WiFi.localIP().toString().c_str());
    if (!MDNS.begin(CAMERA_ID))
      Serial.println("mDNS init failed");
    else
      Serial.printf("mDNS started. Device is %s.local\n", CAMERA_ID);
  }
}

void connectWS() {
  if(WiFi.status() == WL_CONNECTED){
    bool serverResolved = false;
    //bool serverResolved = true;
    if (ws_url.isEmpty()) {
      Serial.printf("Resolving %s.local", server_hostname);
      unsigned long t = millis();
      serverResolved = resolveServer();
      while (!serverResolved && millis() - t < MAX_WS_WAIT_TIME_MS) { delay(500); Serial.print("."); }
    }
    if(serverResolved){
      Serial.printf("\nConnecting to %s …\n", ws_url.c_str());
      client.onMessage(onMessage);
      client.onEvent(onEvent);
      if (ws_url.startsWith("wss://")) client.setInsecure();
      bool ok = client.connect(ws_url.c_str());
      //bool ok = client.connect("diana-adventure-belfast-stream.trycloudflare.com", 443, "/ws");
      if (!ok) { Serial.println("WS connection failed — will retry"); ws_url = ""; }
    }
  }
}

// =================== Send from SD ===================
void sendFromSD(uint64_t timestamp) {
  String path = "/camera/" + String(timestamp) + ".jpg";
  File file = SD_MMC.open(path);
  if (!file) { Serial.println("Failed to open: " + path); return; }

  size_t fileSize = file.size();
  if (fileSize == 0 || fileSize > MAX_FRAME_SIZE) {
    Serial.printf("Invalid file size: %u\n", fileSize);
    file.close();
    return;
  }

  uint8_t* jpegDst = g_sendBuf + sizeof(Header);
  size_t   bytesRead = file.read(jpegDst, fileSize);
  file.close();

  if (bytesRead != fileSize) {
    Serial.printf("Read mismatch: got %u, expected %u\n", bytesRead, fileSize);
    return;
  }

  Header* hdr = (Header*)g_sendBuf;
  memset(hdr, 0, sizeof(Header));
  strncpy(hdr->camera_id, CAMERA_ID, sizeof(hdr->camera_id));
  hdr->timestamp = timestamp;
  hdr->data_len  = fileSize;

  size_t totalLen = sizeof(Header) + fileSize;
  Serial.printf("Sending %s — %u B total\n", path.c_str(), totalLen);

  if (!client.available()) { Serial.println("WS not available, skipping send"); return; }
  bool ret = client.sendBinary((const char*)g_sendBuf, totalLen);
  Serial.println(ret ? "WS send OK" : "WS send Error");
  delay(100);
}


// =================== Capture → SD ===================
int captureToSD(uint64_t timestamp) {
  uint64_t t = esp_timer_get_time();
  Serial.print("Capturing ->");
  camera_fb_t* fb = esp_camera_fb_get();
  Serial.printf("%llu\n", esp_timer_get_time()-t);
  if (!fb) { Serial.println("Capture failed"); return 0; }
  if (fb->len > MAX_FRAME_SIZE) {
    Serial.printf("Frame too large (%u bytes)\n", fb->len);
    esp_camera_fb_return(fb);
    return 0;
  }
  String path = "/camera/" + String(timestamp) + ".jpg";
  writejpg(SD_MMC, path.c_str(), fb->buf, fb->len);
  Serial.printf("Saved %u bytes → %s\n", fb->len, path.c_str());

  String message = String(timestamp) + "\n";
  appendFile(SD_MMC, "/sendlist.txt", message.c_str());

  esp_camera_fb_return(fb);
  return 1;
}

void goToSleep() {
  Serial.println("Master going to deep sleep, waiting for trigger...");
  Serial.flush();
  esp_wifi_stop();
  WiFi.mode(WIFI_OFF);
  ws2812SetColor(1);
  digitalWrite(TOF_SENSOR_PIN, LOW);
  esp_sleep_enable_ext0_wakeup(MOTIONSENSOR_PIN, 1);
  esp_sleep_enable_timer_wakeup(20 * uS_TO_S_FACTOR);
  esp_deep_sleep_start();
}


// =================== ESP-NOW init ===================
void initEspNow() {
  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) { Serial.println("ESP-NOW init failed"); goToSleep(); }
  esp_now_register_send_cb(esp_now_send_cb_t(onSend));
  esp_now_register_recv_cb(esp_now_recv_cb_t(onReceive));
  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, slaveMAC, 6);
  peer.channel = 0;  // follow the active STA/Wi-Fi channel
  peer.encrypt = false;
  if (esp_now_add_peer(&peer) != ESP_OK) { Serial.println("Failed to add peer"); goToSleep(); }
}


void wakeSlave(){
  // ── Signal slave to send ─────────────────────────────────────────────────

  digitalWrite(TO_SLAVE_PIN, HIGH);
  esp_rom_delay_us(100);
  digitalWrite(TO_SLAVE_PIN, LOW);
  delay(300);
}

// =================== OTA Update ===================
bool performOTAIfAvailable() {
  Serial.printf("[OTA] %s @ %s\n", FIRMWARE_DEVICE, FIRMWARE_VERSION);

  // ── Step 1: Fetch release asset list from GitHub API ────────────────────
  WiFiClientSecure apiClient;
  apiClient.setInsecure();
  HTTPClient http;
  String releaseUrl = "https://api.github.com/repos/" + String(GITHUB_REPO) + "/releases/latest";
  http.begin(apiClient, releaseUrl);
  http.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);
  http.addHeader("User-Agent", "ESP32");
  http.addHeader("Accept", "application/vnd.github+json");
 
  int code = http.GET();
  if (code != 200) {
    Serial.printf("[OTA] GitHub API returned %d\n", code);
    http.end();
    return false;
  }

  String body = http.getString();  // reads entire response before parsing
  http.end();

  JsonDocument filter;
  filter["assets"][0]["name"]                 = true;
  filter["assets"][0]["browser_download_url"] = true;

  JsonDocument doc;
  DeserializationError err = deserializeJson(doc, body,
                                             DeserializationOption::Filter(filter));

  if (err) { Serial.printf("[OTA] JSON parse error: %s\n", err.c_str()); return false; }

  // ── Step 2: Locate versions.json and <device>.bin URLs ──────────────────
  String versionsUrl, binUrl;
  for (JsonObject asset : doc["assets"].as<JsonArray>()) {
    const char* name = asset["name"];
    if (name && strcmp(name, "versions.json") == 0)
      versionsUrl = asset["browser_download_url"].as<String>();
    if (name && strcmp(name, FIRMWARE_DEVICE ".bin") == 0)
      binUrl = asset["browser_download_url"].as<String>();
  }
  if (versionsUrl.isEmpty() || binUrl.isEmpty()) {
    Serial.println("[OTA] Required assets not found in release");
    return false;
  }

  // ── Step 3: Fetch versions.json and compare ──────────────────────────────
  WiFiClientSecure verClient;
  verClient.setInsecure();
  HTTPClient verHttp;
  verHttp.begin(verClient, versionsUrl);
  verHttp.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);
  verHttp.addHeader("User-Agent", "ESP32");
  code = verHttp.GET();
  if (code != 200) {
    Serial.printf("[OTA] versions.json fetch returned %d\n", code);
    verHttp.end();
    return false;
  }

  JsonDocument verDoc;
  err = deserializeJson(verDoc, verHttp.getStream());
  verHttp.end();
  if (err) { Serial.printf("[OTA] versions.json parse error: %s\n", err.c_str()); return false; }

  const char* remoteVersion = verDoc[FIRMWARE_DEVICE].as<const char*>();
  if (!remoteVersion) { Serial.println("[OTA] Device key not found in versions.json"); return false; }
  Serial.printf("[OTA] Remote: %s  Local: %s\n", remoteVersion, FIRMWARE_VERSION);
  if (strcmp(remoteVersion, FIRMWARE_VERSION) == 0) {
    Serial.println("[OTA] Already up to date");
    isUpToDate = true;
    return true;
  }

  // ── Step 4: Flash ────────────────────────────────────────────────────────
  Serial.printf("[OTA] Updating %s → %s\n", FIRMWARE_VERSION, remoteVersion);
  WiFiClientSecure binClient;
  binClient.setInsecure();
  httpUpdate.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);
  httpUpdate.rebootOnUpdate(false);

  t_httpUpdate_return result = httpUpdate.update(binClient, binUrl);
  switch (result) {
    case HTTP_UPDATE_OK:
      Serial.println("[OTA] Flash OK");
      return true;
    case HTTP_UPDATE_FAILED:
      Serial.printf("[OTA] Flash FAILED: (%d) %s\n",
                    httpUpdate.getLastError(),
                    httpUpdate.getLastErrorString().c_str());
      return false;
    case HTTP_UPDATE_NO_UPDATES:
      Serial.println("[OTA] No update reported");
      return false;
  }
  return false;
}

// =================== Arduino Setup ===================
void setup() {
  Serial.begin(115200);
  Serial.printf("\n=== ESP32-CAM [%s] ===\n", CAMERA_ID);

  pinMode(TO_SLAVE_PIN, OUTPUT);
  digitalWrite(TO_SLAVE_PIN, LOW);
  pinMode(TOF_SENSOR_PIN, OUTPUT);
  sdmmcInit();
  initEspNow();
  ws2812Init();

  ws2812SetColor(2);

  // ── Wake-cause guard ────────────────────────────────────────────────────
  esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();
  if (cause == ESP_SLEEP_WAKEUP_TIMER){
    ws2812SetColor(3);

    // // ── OTA retry/slave-signal path (flag persisted from a prior wakeup) ────
    // if (otaPendingRTC) {
    //   ws2812SetColor(3);
    //   connectToWiFi();
    //   if (WiFi.status() != WL_CONNECTED) { 
    //     Serial.println("[OTA] WiFi unavailable, will retry on next wakeup");
    //     goToSleep(); 
    //   }
    //   if (performOTAIfAvailable()) {
    //     otaPendingRTC = false;
    //     if(!isUpToDate) //If updates are necessary
    //       ESP.restart();
    //   }
    //   goToSleep();  // failure: flags stay set, retry next wakeup
    // }

    std::vector<String> flist = getSendList(SD_MMC, "/sendlist.txt");
    wakeSlave();
    slaveReady = false;
    SyncPacket signal;
    signal.type = 0x04;
    Serial.print("Slave send: ");
    esp_now_send(slaveMAC, (uint8_t*)&signal, sizeof(signal));
    unsigned long t = millis();
    while (!(slaveReady) && millis() - t < 2000);
    connectToWiFi();
    // IPAddress resolvedIP;
// bool dnsOk = WiFi.hostByName("diana-adventure-belfast-stream.trycloudflare.com", resolvedIP);
// Serial.printf("DNS: %s → %s\n", dnsOk ? "OK" : "FAIL", resolvedIP.toString().c_str());
  //   Serial.printf("Free heap: %u  Max block: %u  Free PSRAM: %u\n",
  // ESP.getFreeHeap(), ESP.getMaxAllocHeap(), ESP.getFreePsram());
    connectWS();
    // Allow the server's ota_update command to arrive before the send loop
    unsigned long pollEnd = millis() + 1000;
    while (millis() < pollEnd) {
      client.poll();
      delay(10);
    }
    if(WiFi.status() == WL_CONNECTED && ws_url != ""){
      // // Allocate single send buffer from PSRAM
      g_sendBuf = (uint8_t*)ps_malloc(sizeof(Header) + MAX_FRAME_SIZE);
      if (!g_sendBuf) {
        Serial.println("FATAL: Could not allocate send buffer in PSRAM");
        while (true) delay(1000);
      }
      if (!g_sendBuf) { Serial.println("Send buffer not allocated"); return; }
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

    // ── OTA (ota_update received during poll above) ───────────────────────
    if (otaRequested || otaPendingRTC) {
      ws2812SetColor(3);
      if (WiFi.status() != WL_CONNECTED) { 
        Serial.println("[OTA] WiFi unavailable, will retry on next wakeup");
        goToSleep(); 
      }
      isUpToDate = false;
      if (performOTAIfAvailable()) {
        otaPendingRTC = false;
        if (!isUpToDate)
          ESP.restart();
      }
      // on failure: otaPendingRTC stays true, retry next wakeup
    }

    goToSleep();
  } else if (cause != ESP_SLEEP_WAKEUP_EXT0) goToSleep();

  initCamera();
  createDir(SD_MMC, "/camera");
  digitalWrite(TOF_SENSOR_PIN, HIGH);

  tofInitialised = initToF();

  uint64_t startTime   = esp_timer_get_time();
  uint64_t elapsedTime = 0;
  uint64_t ts[MAX_SESSION_FRAMES];
  int i = 0;

  do {
    if (checkToFProximityAlert()) {
      // Wake slave camera via GPIO pulse
      wakeSlave();

      SyncPacket pkt;
      pkt.type         = 0x01;
      pkt.timestamp_us = esp_timer_get_time();

      ackReceived = false;
      slaveReady  = false;
      Serial.print("Slave capture signal: ");
      esp_now_send(slaveMAC, (uint8_t*)&pkt, sizeof(pkt));

      unsigned long t = millis();
      while (!(ackReceived && slaveReady) && millis() - t < 2000);

      if (!ackReceived || !slaveReady) {
        Serial.println("No ACK from slave, aborting");
        goToSleep();
      }

      if (captureToSD(pkt.timestamp_us) == 0)
        Serial.println("Error with capture");
      else
        ts[i++] = pkt.timestamp_us;
    }
    // delay(4000);
    elapsedTime = esp_timer_get_time() - startTime;
  } while (elapsedTime < TOF_SENSOR_WAIT_TIME_US);

  Serial.printf("Elapsed time %llu\n", elapsedTime);
  goToSleep();
}

void loop() {
}
