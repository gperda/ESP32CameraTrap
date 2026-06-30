
#include <WiFi.h>
#include <WebSocketsClient.h>
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
#include <ota_update.h>
#include <Adafruit_VL53L5CX.h>
#include "esp_sntp.h"

// =================== CONFIGURATION ===================
#define CAMERA_ID                 "cam1"
#define MAX_FRAME_SIZE            1048576

#define FIRMWARE_VERSION          "v2.2.2"
#define FIRMWARE_DEVICE           "master_camera"
#define GITHUB_REPO               "gperda/ESP32CameraTrap"
#define uS_TO_S_FACTOR            1000000ULL 

#define MOTIONSENSOR_PIN          GPIO_NUM_14
#define TO_SLAVE_PIN              GPIO_NUM_45

#define TOF_SENSOR_PIN            GPIO_NUM_20
#define TOF_SENSOR_INTERRUPT_PIN  GPIO_NUM_21
#define TOF_SDA_PIN               GPIO_NUM_41
#define TOF_SCL_PIN               GPIO_NUM_42

#define TOF_SENSOR_WAIT_TIME_S    30
#define TOF_SENSOR_WAIT_TIME_US TOF_SENSOR_WAIT_TIME_S * uS_TO_S_FACTOR
#define TOF_I2C_SPEED             1000000 // 1 MHz
#define TOF_RANGING_FREQ_HZ       15
#define TOF_ZONES                 16      

#define THRESHOLD_DISTANCE_MM_LOW  1500
#define THRESHOLD_DISTANCE_MM_HIGH 3000
#define THRESHOLD_DETECTION_MIN_NUMBER_OF_ZONES 6
#define THRESHOLD_MOTION_MAX_ZONES 4
#define THRESHOLD_MOTION_MAX_TOTAL 20 * 16

#define MAX_WIFI_WAIT_TIME_MS     6000
#define MAX_WS_WAIT_TIME_MS       4000
#define WAKEUP_TIMER_SECONDS      10

#define SYNC_TIME_EVERY_N_CONNECTIONS   20

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
//const uint16_t server_port  = 3000;

// =================== Globals ===================
WebSocketsClient client;
volatile bool shouldCapture = false;
bool wsConnected            = false;

extern const char ca_cert_start[] asm("_binary_ca_cert_start");

bool motionDetected = false;

static uint8_t* g_sendBuf = nullptr;

// ── ToF globals ──────────────────────────────────────────────────────────────
// SparkFun_VL53L5CX tofSensor;
Adafruit_VL53L5CX tofSensor;
VL53L5CX_ResultsData tofData;

// =================== ESP-NOW COMMUNICATION ================
uint8_t slaveMAC[] = {0xD0, 0xCF, 0x13, 0x26, 0xE0, 0x6C};

typedef struct Header {
  char     camera_id[8];   // fixed width
  uint64_t timestamp;      // microseconds
  uint64_t data_len;       // image bytes
} Header;

struct SyncPacket {
  uint8_t  type;           // 0x01 = sync, 0x02 = start sending
  uint64_t timestamp_ms;
  uint8_t  framesCount;
};

volatile bool ackReceived = false;
volatile bool slaveReady  = false;

RTC_DATA_ATTR bool otaPendingRTC      = false;
volatile bool otaRequested = false;
volatile bool isUpToDate = false;

RTC_DATA_ATTR int wakeCount = 0;

volatile bool timeSynced = false;

// =================== ToF helpers ===================

// ── Initialise the VL53L5CX once per wake cycle ─────────────────────────────
bool initToF() {
  Wire.begin(TOF_SDA_PIN, TOF_SCL_PIN);
  Wire.setClock(TOF_I2C_SPEED);

  if (!tofSensor.begin(VL53L5CX_DEFAULT_ADDRESS, &Wire, TOF_I2C_SPEED)) {
    Serial.println("[ToF] Sensor not found — check wiring");
    return false;
  }
  //tofSensor.setPowerMode();
  tofSensor.setResolution(TOF_ZONES);
  tofSensor.setRangingFrequency(TOF_RANGING_FREQ_HZ);
  tofSensor.setRangingMode(VL53L5CX_RANGING_MODE_AUTONOMOUS);

  tofSensor.stopRanging();
  if(!tofSensor.initMotionIndicator(TOF_ZONES)) Serial.println("Failed to init motion");
  if(!tofSensor.setMotionDistance(THRESHOLD_DISTANCE_MM_LOW, THRESHOLD_DISTANCE_MM_HIGH)) Serial.println("Failed to set motion distance");

  VL53L5CX_DetectionThresholds thresholds[TOF_ZONES];
  memset(thresholds, 0, sizeof(thresholds));

  Serial.println(F("Configuring detection thresholds..."));

  for (uint8_t zone = 0; zone < TOF_ZONES; zone++) {
    thresholds[zone].zone_num = zone;
    thresholds[zone].measurement = VL53L5CX_DISTANCE_MM;
    thresholds[zone].type = VL53L5CX_IN_WINDOW;
    thresholds[zone].param_low_thresh = THRESHOLD_DISTANCE_MM_LOW;
    thresholds[zone].param_high_thresh = THRESHOLD_DISTANCE_MM_HIGH;
    thresholds[zone].mathematic_operation = VL53L5CX_OPERATION_OR;
  }

  // Mark end of threshold list
  thresholds[TOF_ZONES-1].zone_num = VL53L5CX_LAST_THRESHOLD;

  if(!tofSensor.setDetectionThresholds(thresholds)) Serial.println("Failed to set thresh");
  if(!tofSensor.setDetectionThresholdsEnable(true)) Serial.println("Failed to enable thresh");

  tofSensor.startRanging();
  Serial.printf("[ToF] Ranging started at %d Hz\n", TOF_RANGING_FREQ_HZ);
  return true;
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
  // config.xclk_freq_hz   = 8000000;
  config.xclk_freq_hz   = 16000000;
  config.frame_size     = FRAMESIZE_FHD;
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
  s->set_hmirror(s, 1);
  s->set_brightness(s, 1);
  s->set_saturation(s, 0);

  Serial.println("Camera configuration complete!");
  delay(10);
  return 1;
}

// bool resolveServer() {
//   IPAddress serverIP = MDNS.queryHost(server_hostname);
//   if (serverIP == IPAddress(0, 0, 0, 0)) return false;
//   ws_url = "ws://" + serverIP.toString() + ":" + String(server_port) + "/ws";
//   Serial.printf("Resolved %s\n", ws_url.c_str());
//   return true;
// }

// =================== WebSocket Callbacks ===================
void onWsEvent(WStype_t type, uint8_t * payload, size_t length) {
  switch (type) {
    case WStype_CONNECTED:
      Serial.println("WS: connected");
      wsConnected = true;
      client.sendTXT("register:" CAMERA_ID ":" + normalizeBuildFlagString(STRINGIFY(REGISTER_TOKEN)) + ":" + String(FIRMWARE_VERSION));
      break;
    case WStype_DISCONNECTED:
      if (payload && length > 0) {
        Serial.printf("WS: disconnected (%.*s)\n", (int)length, (const char*)payload);
      } else {
        Serial.println("WS: disconnected");
      }
      wsConnected = false;
      break;
    case WStype_TEXT:
      if (payload && length > 0) {
        String data = String((char*)payload);
        Serial.printf("\u2190 %s\n", data.c_str());
        if (data == "capture")    shouldCapture = true;
        if (data == "master_ota_update") { Serial.println("Received ota update"); otaRequested = true; otaPendingRTC = true;}
      }
      break;
    case WStype_PING:
    case WStype_PONG:
    default:
      break;
  }
}

uint64_t getEpochMillis() {
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return (uint64_t)tv.tv_sec * 1000ULL + (tv.tv_usec / 1000ULL);
}

void onTimeSync(struct timeval *tv) {
    timeSynced = true;
}

void syncTimeFromNTP() {
    sntp_set_time_sync_notification_cb(onTimeSync);
    configTime(0, 0, "pool.ntp.org", "time.nist.gov"); // UTC, no DST offset

    unsigned long start = millis();
    Serial.println();
    Serial.print("Syncing time");
    while (!timeSynced && millis() - start < 10000) {
      Serial.print(".");
        delay(500);
    }

    if (!timeSynced) {
        Serial.println("NTP sync failed/timed out");
    } else {
      Serial.println();
      Serial.print("Time is: ");
      Serial.print(getEpochMillis());
      Serial.println();
    }
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
  WiFi.begin(STRINGIFY(MYSSID), STRINGIFY(MYPASSWORD));
  WiFi.setSleep(false);
  Serial.print("WiFi ");
  
  unsigned long t = millis();
  while (WiFi.status() != WL_CONNECTED && millis() -t < MAX_WIFI_WAIT_TIME_MS) { delay(500); Serial.print("."); }
  if(WiFi.status() == WL_CONNECTED){  
    wakeCount++;
    if (wakeCount == 1 || wakeCount % SYNC_TIME_EVERY_N_CONNECTIONS == 0 ) {
      syncTimeFromNTP();
    }
    WiFi.config(WiFi.localIP(), WiFi.gatewayIP(), WiFi.subnetMask(),
                IPAddress(1,1,1,1), IPAddress(8,8,8,8));
    Serial.printf(" OK  IP=%s\n", WiFi.localIP().toString().c_str());

    // // Debug: verify DNS resolution
    // IPAddress resolvedIP;
    // bool dnsOk = WiFi.hostByName(DEBUG_HOSTNAME, resolvedIP);
    // Serial.printf("DNS: %s → %s\n", dnsOk ? "OK" : "FAIL", resolvedIP.toString().c_str());
  
    if (!MDNS.begin(CAMERA_ID))
      Serial.println("mDNS init failed");
    else
      Serial.printf("mDNS started. Device is %s.local\n", CAMERA_ID);
  }
}

bool waitForWsConnected(uint32_t timeoutMs) {
  unsigned long t0 = millis();
  while (millis() - t0 < timeoutMs + 1000) {
    client.loop();
    if (client.isConnected()) {
      return true;
    }
    delay(5);
  }
  return client.isConnected();
}

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

  if (client.isConnected()) {
    client.sendBIN((const uint8_t*)g_sendBuf, totalLen);
    Serial.println("WS send OK");
  } else {
    Serial.println("WS not available, skipping send");
  }
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
  gpio_deep_sleep_hold_en();
  uint64_t io_mask = (1ULL << MOTIONSENSOR_PIN); 
  esp_sleep_enable_ext1_wakeup_io(io_mask, ESP_EXT1_WAKEUP_ANY_HIGH);
  esp_sleep_enable_timer_wakeup(WAKEUP_TIMER_SECONDS * uS_TO_S_FACTOR);
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
  if(client.isConnected()){
    client.disconnect();
    Serial.println("WS disconnected");
  }
  return ::performOTAIfAvailable(FIRMWARE_DEVICE, FIRMWARE_VERSION, GITHUB_REPO, &isUpToDate);
}

bool checkHighMotion(const VL53L5CX_ResultsData results){
  uint8_t triggerCount = 0;

  for (uint8_t zone = 0; zone < TOF_ZONES; zone++) {
    int16_t distance = results.distance_mm[zone];
    if (distance >= THRESHOLD_DISTANCE_MM_LOW && distance <= THRESHOLD_DISTANCE_MM_HIGH) {
        Serial.println();
        Serial.print("Positive trigger in zones: ");
        Serial.print(triggerCount);
        Serial.println();
        triggerCount++;
    }
  }
  if (results.motion_indicator.nb_of_detected_aggregates >= THRESHOLD_MOTION_MAX_ZONES) {
    return true;
  }

  uint32_t totalMotion = 0;
  for (uint8_t idx = 0; idx < TOF_ZONES; idx++) {
    totalMotion += results.motion_indicator.motion[idx];
    if (totalMotion > THRESHOLD_MOTION_MAX_TOTAL) {
      Serial.print("Too high motion: ");
      Serial.print(totalMotion);
      Serial.println();
      return true; // highMotion
    }
  }

  return false;
}

void onTofInt(){
  if(tofSensor.getRangingData(&tofData)){
    if(!checkHighMotion(tofData)){
      wakeSlave();

      SyncPacket pkt;
      pkt.type         = 0x01;
      // pkt.timestamp_us = esp_timer_get_time();
      pkt.timestamp_ms = getEpochMillis();
      ackReceived = false;
      slaveReady  = false;
      Serial.print("Slave capture signal: ");
      Serial.println(esp_now_send(slaveMAC, (uint8_t*)&pkt, sizeof(pkt)));

      unsigned long t = millis();
      while (!(ackReceived && slaveReady) && millis() - t < 2000);

      if (!ackReceived || !slaveReady) {
        Serial.println("No ACK from slave, aborting");
        goToSleep();
      }

      if (captureToSD(pkt.timestamp_ms) == 0)
        Serial.println("Error with capture");
    }
  }
}

void powerOffToF(){
  digitalWrite(TOF_SENSOR_PIN, HIGH);
  gpio_hold_en(TOF_SENSOR_PIN);
}

void powerOnToF(){
  gpio_hold_dis(TOF_SENSOR_PIN);
  digitalWrite(TOF_SENSOR_PIN, LOW);
}

// =================== Arduino Setup ===================
void setup() {
  Serial.begin(115200);
  Serial.printf("\n=== ESP32-CAM [%s] ===\n", CAMERA_ID);

  pinMode(TO_SLAVE_PIN, OUTPUT);
  digitalWrite(TO_SLAVE_PIN, LOW);
  pinMode(TOF_SENSOR_PIN, OUTPUT);
  powerOffToF();
  pinMode(TOF_SENSOR_INTERRUPT_PIN, INPUT_PULLUP);

  sdmmcInit();
  initEspNow();
  ws2812Init();
  ws2812SetColor(2);

  // ── Wake-cause guard ────────────────────────────────────────────────────
  esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();
  if (cause == ESP_SLEEP_WAKEUP_TIMER){
    ws2812SetColor(3);
    
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
    connectWS();

    // Listen to incoming messages
    unsigned long pollEnd = millis() + 1000;
    while (millis() < pollEnd) {
      client.loop();
      delay(10);
    }

    // If there is something to send
    if(!flist.empty()){
      if(WiFi.status() == WL_CONNECTED && client.isConnected()){
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
      } else Serial.println("WiFi currently unavailable, will send later");
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
  } else if (cause == ESP_SLEEP_WAKEUP_EXT1){
    uint64_t status = esp_sleep_get_ext1_wakeup_status();
    if (status & (1ULL << MOTIONSENSOR_PIN)) {
        Serial.println("Woke up from motion sensor INT");

        powerOnToF();
        if (initToF()){

          initCamera();
          createDir(SD_MMC, "/camera");

          uint64_t startTime   = esp_timer_get_time();
          uint64_t elapsedTime = 0;
          int i = 0;
          while(elapsedTime < TOF_SENSOR_WAIT_TIME_US){
            if(digitalRead(TOF_SENSOR_INTERRUPT_PIN) == LOW)
              onTofInt();
            elapsedTime = esp_timer_get_time() - startTime;
          }
          powerOffToF();
        }
        goToSleep();
    }
  } else Serial.println("Cold boot");
  goToSleep();
}

void loop() {
}
