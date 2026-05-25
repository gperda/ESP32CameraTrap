#include "ota_update.h"

#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <HTTPUpdate.h>
#include <ArduinoJson.h>

bool performOTAIfAvailable(const char* firmwareDevice,
                           const char* firmwareVersion,
                           const char* githubRepo,
                           volatile bool* isUpToDate) {
  Serial.printf("[OTA] %s @ %s\n", firmwareDevice, firmwareVersion);

  WiFiClientSecure apiClient;
  apiClient.setInsecure();
  HTTPClient http;
  String releaseUrl = "https://api.github.com/repos/" + String(githubRepo) + "/releases/latest";
  http.begin(apiClient, releaseUrl);
  http.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);
  http.addHeader("User-Agent", "ESP32");
  http.addHeader("Accept", "application/vnd.github+json");
  Serial.printf("%s\n", releaseUrl);
  int code = http.GET();
  if (code != 200) {
    Serial.printf("[OTA] GitHub API returned %d\n", code);
    http.end();
    return false;
  }

  String body = http.getString();
  http.end();

  JsonDocument filter;
  filter["assets"][0]["name"]                 = true;
  filter["assets"][0]["browser_download_url"] = true;

  JsonDocument doc;
  DeserializationError err = deserializeJson(doc, body,
                                             DeserializationOption::Filter(filter));

  if (err) {
    Serial.printf("[OTA] JSON parse error: %s\n", err.c_str());
    return false;
  }

  String versionsUrl, binUrl;
  for (JsonObject asset : doc["assets"].as<JsonArray>()) {
    const char* name = asset["name"];
    if (name && strcmp(name, "versions.json") == 0) {
      versionsUrl = asset["browser_download_url"].as<String>();
    }
    String firmwareBinName = String(firmwareDevice) + ".bin";
    if (name && strcmp(name, firmwareBinName.c_str()) == 0) {
      binUrl = asset["browser_download_url"].as<String>();
    }
  }

  if (versionsUrl.isEmpty() || binUrl.isEmpty()) {
    Serial.println("[OTA] Required assets not found in release");
    return false;
  }

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
  if (err) {
    Serial.printf("[OTA] versions.json parse error: %s\n", err.c_str());
    return false;
  }

  const char* remoteVersion = verDoc[firmwareDevice].as<const char*>();
  if (!remoteVersion) {
    Serial.println("[OTA] Device key not found in versions.json");
    return false;
  }

  Serial.printf("[OTA] Remote: %s  Local: %s\n", remoteVersion, firmwareVersion);
  if (strcmp(remoteVersion, firmwareVersion) == 0) {
    Serial.println("[OTA] Already up to date");
    if (isUpToDate) {
      *isUpToDate = true;
    }
    return true;
  }

  Serial.printf("[OTA] Updating %s -> %s\n", firmwareVersion, remoteVersion);
  WiFiClientSecure binClient;
  binClient.setInsecure();
  httpUpdate.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);
  httpUpdate.rebootOnUpdate(false);
  Serial.printf("Looking for %s\n", binUrl);
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
