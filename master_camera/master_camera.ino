/*
 * ESP32-CAM #1 — Dual Camera WebSocket Client
 * Library: ArduinoWebsockets by Gil Maimon
 *
 * Install via Arduino Library Manager → search "ArduinoWebsockets" by Gil Maimon
 * Board setting: AI-Thinker ESP32-CAM
 *
 * Change CAMERA_ID to "cam2" for the second board.
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
#define CAMERA_ID "cam1"   // "cam1" for board 1, "cam2" for board 2
#define MAX_FRAME_SIZE 128000
#define MOTIONSENSOR_PIN GPIO_NUM_14
#define TO_SLAVE_PIN GPIO_NUM_46
#define FROM_SLAVE_PIN 12
#define TOF_SENSOR_WAIT_TIME_US 10000000
#define MAX_SESSION_FRAMES 10

const char* ssid     = "Redmi Note 11S";
const char* password = "donatella";

const char* server_hostname = "3dom";
const uint16_t server_port = 3000;
String ws_url;

// =================== Globals ===================
WebsocketsClient client;
volatile bool shouldCapture  = false;
bool wsConnected             = false;
unsigned long lastReconnect  = 0;
const unsigned long RECONNECT_MS = 5000;
unsigned long startTime;

bool motionDetected = false;

static uint8_t* g_sendBuf = nullptr;
SemaphoreHandle_t receiveSemaphore;

// =================== ESP-NOW COMMUNICATION ================
uint8_t slaveMAC[] = {0xD0, 0xCF, 0x13, 0x26, 0xFB, 0x54};

typedef struct Picture {
  uint64_t timestamp;
  uint8_t* jpeg;
  size_t size;
} Picture;

typedef struct Header {
  char camera_id[8];   // fixed size
  uint64_t timestamp;  // milliseconds
  uint64_t data_len;   // image size
} Header;

typedef struct struct_message {
  uint64_t timestamp;
} struct_message;

struct_message out_msg;
struct_message in_msg;

struct SyncPacket {
    uint8_t type;           // 0x01 = sync, 0x02 = start sending
    uint64_t timestamp_us;
    uint8_t framesCount;
};

volatile bool ackReceived = false;
volatile bool slaveReady = false;
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
  // initial sensors are flipped vertically and colors are a bit saturated
  s->set_vflip(s, 1); // flip it back
  s->set_brightness(s, 1); // up the brightness just a bit
  s->set_saturation(s, 0); // lower the saturation

  Serial.println("Camera configuration complete!");
  delay(10);
  return 1;
}

bool resolveServer(){
  IPAddress serverIP = MDNS.queryHost(server_hostname);
  if (serverIP == IPAddress(0, 0, 0, 0)){
    return false;
  }

  ws_url = "ws://" + serverIP.toString() + ":" + String(server_port) + "/ws";
  Serial.printf("Resolved %s\n", ws_url.c_str());
  return true;
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
      // library auto-responds with pong
      break;

    case WebsocketsEvent::GotPong:
      break;
  }
}



// bool detectMotion(){
//   int sensorStatus = digitalRead(MOTIONSENSOR_PIN);
//   if(sensorStatus == HIGH && !motionDetected){
//     motionDetected = true;
//     esp_now_send(slaveCameraAddress, (uint8_t *) &out_msg, sizeof(out_msg));
//     return true;
//   } else if(sensorStatus == LOW){
//     motionDetected = false;
//   }
//   return false;
// }

uint64_t timems(struct timeval tv_now){
  return (uint64_t)tv_now.tv_sec * 1000000L + (int64_t)tv_now.tv_usec;
}

// void onSend(const uint8_t *mac_addr, esp_now_send_status_t status){
//   Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
//   Serial.printf("Sent timestamp %lu\n", out_msg.timestamp);
// }

// void onReceive(const uint8_t *mac_addr, const uint8_t *incomingData, int len){
//   memcpy(&in_msg, incomingData, sizeof(in_msg));
//   xSemaphoreGiveFromISR(receiveSemaphore, NULL);
// }

void onSend(const uint8_t* mac, esp_now_send_status_t status) {
    // MAC-layer delivery confirmation - not used as app ACK here
    Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

void onReceive(const esp_now_recv_info_t* info, const uint8_t* data, int len) {
    if (len == 1 && data[0] == 0xBB) {
        ackReceived = true;
    }
    if (len == 1 && data[0] == 0xCC) {
      slaveReady = true;
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
  if (ws_url.isEmpty()){
  Serial.printf("Resolving %s.local", server_hostname);
    while(!resolveServer()) {
      delay(100);
      Serial.print(".");
    };
  }
  Serial.printf("\nConnecting to %s …\n", ws_url.c_str());
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
  delay(100);
}

void sendPicture(Picture p){
  Serial.printf("Free heap before send: %u\n", esp_get_free_heap_size());
  Header header = {0};
  strncpy(header.camera_id, CAMERA_ID, sizeof(header.camera_id));
  header.timestamp = p.timestamp;
  header.data_len = p.size;

  size_t totalLen = sizeof(Header) + p.size;
  Serial.printf("Totalen %d\n", totalLen);
  uint8_t* payload = (uint8_t*)ps_malloc(totalLen);

  if (payload) {
    memcpy(payload, &header, sizeof(Header));
    memcpy(payload + sizeof(Header), p.jpeg, p.size);
    Serial.printf("payload ptr: %p\n", payload);  // null = OOM
    Serial.printf("p.jpeg ptr:  %p\n", p.jpeg);

    Serial.printf("Header size: %u\n", sizeof(Header));
    Serial.printf("Image size: %u\n", header.data_len);
    Serial.printf("Timestamp: %llu\n", header.timestamp);

  // ArduinoWebsockets sendBinary(const char* data, size_t len)
    bool ret = client.sendBinary((const char*)payload, totalLen);
    Serial.println(ret ? "WS send OK" : "WS send Error");
    Serial.printf("Sent %u bytes over WS\n", totalLen);
    delay(100);
    free(payload);
  }
}

// =================== Capture → Save (zero extra malloc) ===================
// Captures a frame and writes it directly to SD from the camera frame buffer.
// Returns the timestamp used as the filename key, or 0 on failure.
int captureToSD(uint64_t timestamp) {
  Serial.printf("%llu\n", esp_timer_get_time());
  camera_fb_t* fb = esp_camera_fb_get();
  Serial.printf("%llu\n", esp_timer_get_time());
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
    //WiFi.disconnect();

    if (esp_now_init() != ESP_OK) {
        Serial.println("ESP-NOW init failed");
        goToSleep();
    }

    esp_now_register_send_cb(esp_now_send_cb_t(onSend));
    esp_now_register_recv_cb(esp_now_recv_cb_t(onReceive));

    esp_now_peer_info_t peer = {};
    memcpy(peer.peer_addr, slaveMAC, 6);
    peer.channel = 0;
    peer.encrypt = false;
    if (esp_now_add_peer(&peer) != ESP_OK){
      Serial.println("Failed to add peer");
      goToSleep();
    }
}

void goToSleep() {
    Serial.println("Master going to deep sleep, waiting for trigger...");
    Serial.flush();

    
    esp_wifi_stop();
    WiFi.mode(WIFI_OFF);

    // Wake on rising edge of trigger pin
    esp_sleep_enable_ext0_wakeup(MOTIONSENSOR_PIN, 1);
    // rtc_gpio_init(TO_SLAVE_PIN);
    // rtc_gpio_set_direction(TO_SLAVE_PIN, RTC_GPIO_MODE_OUTPUT_ONLY);
    // rtc_gpio_set_level(TO_SLAVE_PIN, 0);
    // rtc_gpio_hold_en(TO_SLAVE_PIN);
    esp_deep_sleep_start();
    // execution never resumes here - ESP32 reboots on wake
}

// =================== Arduino Setup ===================
void setup() {
  Serial.begin(115200);
  Serial.printf("\n=== ESP32-CAM [%s] ===\n", CAMERA_ID);

  //receiveSemaphore = xSemaphoreCreateBinary();

  // //DECIDERE SE CONNETTERSI PRIMA DI SCATTARE OPPURE SOLO UNA VOLTA AVER TERMINATO GLI SCATTI e spedire tutto insieme
  // connectToWiFi();
  // if (!MDNS.begin(CAMERA_ID)){
  //   Serial.println("mDNS init failed");
  // }else{
  //   Serial.printf("mDNS started. Device is %s.local\n", CAMERA_ID);
  // }
  // WiFi.disconnect();


  esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();
  if (cause != ESP_SLEEP_WAKEUP_EXT0)
    goToSleep();

  pinMode(TO_SLAVE_PIN, OUTPUT);
  digitalWrite(TO_SLAVE_PIN, LOW);
  initCamera();
  // // connectWS();
  sdmmcInit();
  //removeDir(SD_MMC, "/camera");
  createDir(SD_MMC, "/camera");

  
  initEspNow();

  uint64_t startTime = esp_timer_get_time();
  uint64_t elapsedTime = 0;
  bool shouldCapture = true;
  uint64_t ts[MAX_SESSION_FRAMES];
  int i=0;

  do{
    if(shouldCapture){
      // // Cam1 wakes cam2 over PIN
      // rtc_gpio_hold_dis(TO_SLAVE_PIN);
      // rtc_gpio_set_level(TO_SLAVE_PIN, 1);
      // esp_rom_delay_us(1000);
      // rtc_gpio_set_level(TO_SLAVE_PIN,0);
      digitalWrite(TO_SLAVE_PIN, HIGH);
      esp_rom_delay_us(100);
      digitalWrite(TO_SLAVE_PIN, LOW);
      delay(200);

      // --- SYNC PHASE ---
      SyncPacket pkt;
      pkt.type = 0x01;
      pkt.timestamp_us = esp_timer_get_time();

      ackReceived = false;
      slaveReady = false;
      esp_err_t res = esp_now_send(slaveMAC, (uint8_t*)&pkt, sizeof(pkt));

      unsigned long t = millis();
      while (!(ackReceived && slaveReady) && millis() - t < 1000);
      //TODO BLOCKING SEMAPHORE?
      if (!ackReceived) {
        Serial.println("No ACK from slave, aborting");
        goToSleep();
      }

      if(captureToSD(pkt.timestamp_us) == 0)
        Serial.println("Error with capture");
        // do something
      else{
        ts[i++] = pkt.timestamp_us;
      }
    }
    delay(4000);
    elapsedTime = esp_timer_get_time()-startTime;
  }while(elapsedTime < TOF_SENSOR_WAIT_TIME_US);

  Serial.printf("Elapsed time %llu \n", elapsedTime);

  // --- SEND PHASE ---
  SyncPacket signal;
  signal.type = 0x02;
  signal.timestamp_us = 0;
  signal.framesCount = 0;
  digitalWrite(TO_SLAVE_PIN, HIGH);
  esp_rom_delay_us(100);
  digitalWrite(TO_SLAVE_PIN, LOW);

  // Allocate the single send buffer from PSRAM once at startup
  g_sendBuf = (uint8_t*)ps_malloc(sizeof(Header) + MAX_FRAME_SIZE);
  if (!g_sendBuf) {
    Serial.println("FATAL: Could not allocate send buffer in PSRAM");
    while (true) delay(1000);  // Halt — nothing will work without this
  }
  Serial.printf("Send buffer allocated: %u bytes\n", sizeof(Header) + MAX_FRAME_SIZE);
  esp_rom_delay_us(100000);
  esp_now_send(slaveMAC, (uint8_t*)&signal, sizeof(signal));

  //esp_now_deinit();
  if(i>0){
    uint8_t buf[sizeof(SyncPacket) + i * sizeof(uint64_t)];
    size_t ts_bytes = i*sizeof(uint64_t);
    SyncPacket* hdr = (SyncPacket*)buf; 
    hdr->type = 0x04;
    hdr->framesCount = i;
    hdr->timestamp_us = 0;
    memcpy(buf+sizeof(SyncPacket), ts, ts_bytes);
    esp_rom_delay_us(100000);
    esp_now_send(slaveMAC, buf, sizeof(buf));
    Serial.printf("Size of buffer %d", sizeof(buf));

    esp_rom_delay_us(100000);
    esp_now_deinit();

    connectToWiFi();
    connectWS();

    for(int j=0;j<i;j++){
      sendFromSD(ts[j]);
    }
  }

  // If cam1 and cam2 are woken up by PIR
  // SyncPacket signal;
  // signal.type = 0x03;
  // signal.timestamp_us = 0;
  // esp_now_send(slaveMAC, (uint8_t*)&signal, sizeof(signal));

  esp_now_deinit();
  goToSleep();
}

void loop(){

}

// =================== Arduino Loop ===================
// void loop() {
//   unsigned long timeElapsed = 0;
//   unsigned long sendingTime = 0;
//   unsigned long sendingTime_start = 0;
//   uint64_t ts[20];
//   int j = 0;
//   unsigned long startTime = millis();
//   Serial.println("Starting timer");
//   do{
//     //delay(1000);
//     // read ToF data
//     // if data
//     // client.poll();
//     //digitalWrite(SLAVE_TRIGGER_PIN, HIGH);  //Wake up slave camera
//     //digitalWrite(SLAVE_TRIGGER_PIN, LOW);
//     Picture p = capturePicture();
//     ts[j] = savePicture(p);
//     j++;
//     delay(1000);
//     timeElapsed = millis() - startTime;
//     Serial.printf("Elapsed time: %lu\n", timeElapsed);
//   }while(timeElapsed < TOF_SENSOR_WAIT_TIME);
//   unsigned long executionTime = millis() - startTime;
//   Serial.printf("Execution time: %lu\n", executionTime);
//   delay(20);

//   connectToWiFi();
//   Serial.print("WiFi");
//   while(WiFi.status() != WL_CONNECTED){ 
//     Serial.print("."); 
//     delay(100);
//   }
//   while(!wsConnected && millis() - lastReconnect > RECONNECT_MS) {
//     lastReconnect = millis();
//     connectWS();
//   }
//   for(int i=0;i<j;i++){
//     String path = "/camera/" + String(ts[i]) + ".jpg";
//     File file = SD_MMC.open(path);
//     if (!file) {
//         Serial.println("Failed to open " + path);
//         continue;
//     }
//     Picture p = {0, nullptr, 0};
//     p.jpeg = (uint8_t*)ps_malloc(file.size());
//     if (p.jpeg) {
//         file.read(p.jpeg, file.size());
//         Serial.println("Reading file "+ path);
//         p.size = file.size();
//         p.timestamp = ts[i];
//         sendPicture(p);
//         //delay(1000);
//         free(p.jpeg);
//     } else {
//         Serial.println("Memory allocation failed");
//     }

//     file.close();

//     //sendPicture(ts[i])
//   }


//   //sendPicture(p); //on parallel thread

//   WiFi.disconnect();
//   esp_light_sleep_start();

//   // unsigned long timeRemaining =  millis() - startTime;
//   // if(wsConnected timeRemaining < 10000){
//   //   Serial.printf("Time remaining: %d\n", timeRemaining);
//   //   client.poll();           // process incoming frames
//   //   digitalWrite(SLAVE_TRIGGER_PIN, HIGH);  //Wake up slave camera
//   //   digitalWrite(SLAVE_TRIGGER_PIN, LOW);
//   //   ws_message p = captureAndSend();
//   //   sendPicture(p);
//   //   esp_err_t result = esp_now_send(slaveCameraAddress, (uint8_t *) &out_msg, sizeof(out_msg));
//   //   delay(4000);
//   //   esp_light_sleep_start();
//   // }else{
//   //   Serial.println("Going to sleep until next motion detected");
//   //   delay(50);
//   //   WiFi.disconnect();
//   //   esp_light_sleep_start();
//   //   Serial.println("Woke up");
//   //   startTime = millis();
//   // }

//   // if(detectMotion()){
//   //   shouldCapture = true;
//   // }
  

//   // // — Capture when told —
//   // if (shouldCapture) {
//   //   shouldCapture = false;
//   //   if (wsConnected) {
//   //     captureAndSend();
//   //     delay(1000);
//   //   }
//   // }
// }

// void loop() {
//   // — Phase 1: Capture frames to SD for TOF_SENSOR_WAIT_TIME ms —
//   uint64_t ts[20];
//   int      frameCount   = 0;
//   unsigned long startTime = millis();
//   bool doOnce = false;

//   Serial.println("Starting capture phase");

//   do {
//     //if (condition on tof sensor)
//     if(!doOnce){
//       // Wake up slave camera
//       digitalWrite(TO_SLAVE_PIN, HIGH);
//       digitalWrite(TO_SLAVE_PIN, LOW);
//       doOnce = true;
//     }
//     uint64_t timestamp = captureAndSave();    
//     if (timestamp < 0 && frameCount >= 20) continue; 
//     out_msg.timestamp = timestamp;
//     esp_err_t espnow_res = esp_now_send(slaveCameraAddress, (uint8_t*)&out_msg, sizeof(out_msg));
 
//     if (xSemaphoreTake(receiveSemaphore, pdMS_TO_TICKS(5000)) == pdTRUE) {
//       if(in_msg.timestamp > 10) {Serial.printf("TS difference %lu too large\n", in_msg.timestamp); continue; }
//         ts[frameCount++] = timestamp;
//     }
//     delay(4000);
//   } while (millis() - startTime < TOF_SENSOR_WAIT_TIME);

//   // Trigger WS sending on slave
//   out_msg.timestamp = -1;
//   esp_err_t espnow_res = esp_now_send(slaveCameraAddress, (uint8_t*)&out_msg, sizeof(out_msg));
//   Serial.printf("Send res %d\n", espnow_res);
//   Serial.printf("Captured %d frames in %lu ms\n", frameCount, millis() - startTime);

//   // — Phase 2: Reconnect WiFi + WS, then send saved frames —
//   connectToWiFi();

//   while (!wsConnected && millis() - lastReconnect > RECONNECT_MS) {
//     lastReconnect = millis();
//     connectWS();
//   }


//   for (int i = 0; i < frameCount; i++) {
//     sendFromSD(ts[i]);
//   }

//   // — Phase 3: Sleep until next motion event —
//   client.close();
//   WiFi.disconnect();
//   esp_light_sleep_start();
// }

