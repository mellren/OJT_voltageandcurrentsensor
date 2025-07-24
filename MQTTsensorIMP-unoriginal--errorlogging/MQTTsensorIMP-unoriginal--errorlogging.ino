// === Libraries ===
#include <Wire.h>
#include <Adafruit_ADS1X15.h>
#include <SPI.h>
#include <SD.h>
#include <RTClib.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <esp_system.h>
#include <ArduinoOTA.h>
#include <Preferences.h>
#include <ESPmDNS.h>

// === Wi-Fi Credentials ===
const char* ssid = "NBERIC";
const char* password = "123@nber!c";

// const char* ssid = "Xiaomao";
// const char* password = "L3prec1ati0n";

// === MQTT Broker Details ===
const char* mqtt_broker = "mqtt.arecmmsu.com";
const int mqtt_port = 1883;
const char* mqtt_username = "arec";
const char* mqtt_password = "arecmqtt";
const char* mqtt_client_id = "ESP32Client123";
const char* mqtt_publish_topic = "arec/data";

// === NTP Details ===
const char* ntpServer = "pool.ntp.org";
const long gmtOffset_sec = 8 * 3600;  // GMT+8
const int daylightOffset_sec = 0;

// === ADC Calibration & Constants ===
float acs712_zero_voltage = 2.5;
const float ACS712_SENSITIVITY = 0.100;
const float R1 = 470000.0;
const float R2 = 20000.0;
const float ADS_GAIN = 4.096;
const float ADS_MAX_READING = 32767.0;

Preferences prefs;

int wifiRetryCount = 0;
const int MAX_WIFI_RETRIES = 5;
bool wifiTry = false;
bool wasConnected = false;
bool wifiEnabled = true;

unsigned long lastWifiAttemptTime = 0;
const unsigned long WIFI_RETRY_INTERVAL = 10000;

unsigned long lastMQTTAttemptTime = 0;
const unsigned long MQTT_RETRY_INTERVAL = 10000; // Retry every 10 sec

unsigned long lastMQTTReconnectAttempt = 0;
const unsigned long MQTT_RETRY_DELAY = 5000; // Retry every 5 seconds

int mqttRetryCount = 0;
const int MAX_MQTT_RETRIES = 1;

bool mqttTry = true;

bool shouldSyncNTP = false;
unsigned long ntpSyncRequestTime = 0;
const unsigned long NTP_DELAY_AFTER_WIFI = 5000;

unsigned long lastRTCCheckMillis = 0;
const unsigned long RTC_CHECK_INTERVAL = 1000;  // 1 seconds
bool rtcPreviouslyFailed = false;

WiFiClient espClient;
PubSubClient client(espClient);
Adafruit_ADS1115 ads;
RTC_DS3231 rtc;

#define SD_CS 25
SPIClass spi = SPIClass(VSPI);

char filename[30];
int lastDay = -1;
float sumVoltage = 0, sumCurrent = 0, sumPower = 0, sumEnergy = 0;
int sampleCount = 0;
unsigned long lastSampleTime = 0;
int lastLoggedFiveMinute = -1;

void callback(char* topic, byte* payload, unsigned int length) {
  String message;

  for (unsigned int i = 0; i < length; i++) {
    message += (char)payload[i];
  }

  message.trim();
  if (message.length() == 0) return;

  if (message == "recalibrate") {
    Serial.println(F("MQTT Recalibration command received"));
    calibrateACS712();
  }
}

void logErrorToSD(const String& errorMsg) {
  File errorFile = SD.open("/error_log.txt", FILE_APPEND);
  if (errorFile) {
    unsigned long uptimeSec = millis() / 1000;
    errorFile.print("Uptime: ");
    errorFile.print(uptimeSec);
    errorFile.print("s - ");
    errorFile.println(errorMsg);
    errorFile.close();
    Serial.println("Logged error to SD: " + errorMsg);
  } else {
    Serial.println("Failed to open error log file.");
  }
}

void setup_wifi() {
  if (!wifiEnabled) return;
  Serial.println(F("Connecting to WiFi..."));
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED && wifiRetryCount < MAX_WIFI_RETRIES) {
    Serial.print(".");
    delay(2000);
    wifiRetryCount++;
  }
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("\nWiFi Failed. SD only.");
    wifiRetryCount = 0;
  } else {
    Serial.println("\nWiFi connected!");
    wasConnected = true;
  }
}

void setup_ota() {
  if (WiFi.status() == WL_CONNECTED) {
    if (MDNS.begin("esp32-energy-monitor")) {
      Serial.println("mDNS responder started");
    } else {
      Serial.println("Error setting up MDNS!");
    }

    ArduinoOTA.setHostname("esp32-energy-monitor");
    ArduinoOTA.setPassword("energymonitoring");
    ArduinoOTA.begin();
    Serial.println("OTA Ready");
  }
}


void reconnect() {
  // Only attempt if WiFi is connected and MQTT is NOT connected
  if (!wifiEnabled || WiFi.status() != WL_CONNECTED || client.connected() || !mqttTry) return;

  Serial.print("Connecting to MQTT... ");
  if (client.connect(mqtt_client_id, mqtt_username, mqtt_password)) {
    Serial.println("connected!");
    client.subscribe("arec/data");  // Only if subscription is needed right away
    mqttRetryCount = 0;
  } else {
    Serial.print("failed, rc=");
    Serial.println(client.state());
    mqttRetryCount++;
  }

  mqttTry = false;
}



void calibrateACS712() {
  Serial.println(F("Calibrating ACS712 Zero Voltage"));
  long total = 0;
  for (int i = 0; i < 1000; i++) {
    total += ads.readADC_SingleEnded(1);
    delay(2);
  }
  float avgRaw = (float)total / 1000.0;
  acs712_zero_voltage = (avgRaw * ADS_GAIN) / ADS_MAX_READING;
  prefs.putFloat("zeroOffset", acs712_zero_voltage);

  Serial.print("[Calib] Zero voltage: ");
  Serial.println(acs712_zero_voltage, 4);
  Serial.println("New Zero Voltage Offset Saved.");
  char mqttACS[256];
      snprintf(mqttACS, sizeof(mqttACS), "[Calib] Zero voltage: %.4f",
              acs712_zero_voltage);
  client.publish(mqtt_publish_topic, mqttACS);

}

void setup() {
  Serial.begin(115200);
  Wire.begin(22, 23);
  spi.begin(26, 14, 27, SD_CS);

  prefs.begin("logger", false);
  acs712_zero_voltage = prefs.getFloat("zeroOffset", 2.5);

  bool failed = false;
  rtcPreviouslyFailed = prefs.getBool("rtcLoop", false);
  bool rtcInitFailed = prefs.getBool("rtcInit", false);

  if (!SD.begin(SD_CS, spi, 1000000)) {
    Serial.println("SD Card Fail");
    delay(2000);
    ESP.restart();
  }

  if (!rtc.begin()) {
    if (!rtcInitFailed) {
      logErrorToSD("RTC init failed");
      prefs.putBool("rtcInit", true);
      prefs.end();
      delay(1000);
    }
    Serial.println("Restarting after RTC failure...");
    failed = true;
  } 
  if (rtcInitFailed  && rtc.begin()) {
    logErrorToSD("RTC re-initialized");
    prefs.putBool("rtcInit", false);
    prefs.end();
  }
  

  if (!ads.begin()) {
    logErrorToSD("ADS1115 init failed");
    failed = true;
  }


  if (failed) {
    Serial.println(F("Critical hardware check failed. Restarting..."));
    delay(2000);
    ESP.restart();
  } else Serial.println(F("All components are functional."));

  delay(1000);

  ads.setGain(GAIN_ONE);
  setup_wifi();
  setup_ota();
  client.setServer(mqtt_broker, mqtt_port);
  client.setCallback(callback); 
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
  
  if (rtc.lostPower()) {
    struct tm timeinfo;
    if (getLocalTime(&timeinfo)) {
      rtc.adjust(DateTime(timeinfo.tm_year + 1900, timeinfo.tm_mon + 1,
                          timeinfo.tm_mday, timeinfo.tm_hour, timeinfo.tm_min,
                          timeinfo.tm_sec));
    }
  }

  struct tm timeinfo;
  if (getLocalTime(&timeinfo)) {
    rtc.adjust(DateTime(timeinfo.tm_year + 1900, timeinfo.tm_mon + 1,
                        timeinfo.tm_mday, timeinfo.tm_hour, timeinfo.tm_min,
                        timeinfo.tm_sec));
    Serial.println("NTP Synced");
  }

  Serial.println("Setup done");
}


void processPreviousDayFile(int day, int month, int year) {
  String fname = "/" + String(day) + "-" + month + "-" + (year % 100) + ".csv";
  if (!SD.exists(fname)) return;

  File prevFile = SD.open(fname, FILE_READ);
  if (!prevFile || prevFile.size() == 0) return;

  float sumV = 0, sumA = 0, sumP = 0, sumE = 0;
  float maxV = -999, maxA = -999;
  int lines = 0;

  prevFile.readStringUntil('\n');
  while (prevFile.available()) {
    String line = prevFile.readStringUntil('\n');
    if (line.length() < 10) continue;

    int c1 = line.indexOf(',');
    int c2 = line.indexOf(',', c1 + 1);
    int c3 = line.indexOf(',', c2 + 1);
    int c4 = line.indexOf(',', c3 + 1);

    float V = line.substring(c1 + 1, c2).toFloat();
    float A = line.substring(c2 + 1, c3).toFloat();
    float P = line.substring(c3 + 1, c4).toFloat();
    float E = line.substring(c4 + 1).toFloat();

    sumV += V;
    sumA += A;
    sumP += P;
    sumE += E;
    if (V > maxV) maxV = V;
    if (A > maxA) maxA = A;
    lines++;
  }
  prevFile.close();

  if (lines == 0) return;

  File summaryFile = SD.open(fname, FILE_WRITE);
  if (summaryFile) {
    summaryFile.seek(summaryFile.size());
    summaryFile.println(F("----- Daily Summary -----"));
    summaryFile.printf("Average Voltage: %.3f V\n", sumV / lines);
    summaryFile.printf("Average Current: %.3f A\n", sumA / lines);
    summaryFile.printf("Average Power  : %.3f W\n", sumP / lines);
    summaryFile.printf("Max Voltage    : %.3f V\n", maxV);
    summaryFile.printf("Max Current    : %.3f A\n", maxA);
    summaryFile.printf("Total Energy   : %.3f Wh\n", sumE);
    summaryFile.close();
    Serial.println(F("Previous day summary written."));
  }
}

bool isSDCardStillPresent() {
  File testFile = SD.open(filename, FILE_APPEND);
  if (testFile) {
    testFile.close();
    return true;
  }
  return false;
}



void loop() {
  if (wifiEnabled){
    ArduinoOTA.handle();
  }

  // == Unified Serial Command Handler ==
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();

    if (cmd == "nvs_clear") {
      Serial.println("Clearing NVS data...");
      Preferences preferences;
      preferences.begin("logger", false);
      preferences.clear();  // clears all keys under namespace "logger"
      preferences.end();
      Serial.println("NVS cleared. Restarting...");
      delay(1000);
      ESP.restart();
    } else if (cmd == "recalibrate") {
      calibrateACS712(); 
    } else if (cmd == "wifi_off") {
      wifiEnabled = false;
      WiFi.disconnect(true);
      WiFi.mode(WIFI_OFF);
      Serial.println("[WiFi] Disabled. Logging to SD only.");
    } else if (cmd == "wifi_on") {
      wifiEnabled = true;
      wifiTry = true;
      Serial.println("[WiFi] Enabled. Will reconnect automatically.");
    } else if (cmd == "restart") {
      ESP.restart();
      Serial.println("Restarting...");
    } else {
      Serial.print("Unknown command: ");
      Serial.println(cmd);
    }
  }

  
  // == RTC Check Interval == 
  if (millis() - lastRTCCheckMillis >= RTC_CHECK_INTERVAL) {
    lastRTCCheckMillis = millis();

    if (!rtc.begin()) {
      // RTC not responding at all (e.g., disconnected)
      if (!rtcPreviouslyFailed) {
        logErrorToSD("RTC communication lost (not detected)");
        rtcPreviouslyFailed = true;
        prefs.putBool("rtcLoop", true);
        prefs.end();
      }
    } else {
      // RTC is back and responding
      if (rtcPreviouslyFailed) {
        logErrorToSD("RTC reconnected");
        rtcPreviouslyFailed = false;
        prefs.putBool("rtcLoop", false);
        prefs.end();
        delay(2000);
        struct tm timeinfo;
        if (getLocalTime(&timeinfo)) {
          rtc.adjust(DateTime(timeinfo.tm_year + 1900, timeinfo.tm_mon + 1,
                              timeinfo.tm_mday, timeinfo.tm_hour, timeinfo.tm_min,
                              timeinfo.tm_sec));
          Serial.println("NTP Synced");
        }
      }
    }

  }



  // == WiFi Reconnection == 
  if (wifiTry && wifiEnabled && WiFi.status() != WL_CONNECTED && millis() - lastWifiAttemptTime >= WIFI_RETRY_INTERVAL) {
    Serial.printf("Reconnecting to WiFi... Attempt #%d\n", wifiRetryCount + 1);
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);
    lastWifiAttemptTime = millis();
    wifiRetryCount++;
    if (wifiRetryCount >= MAX_WIFI_RETRIES) {
      Serial.println(F("Max retries reached. Retrying in 5 minutes. SD only."));
      WiFi.disconnect(true);
      WiFi.mode(WIFI_OFF);
      wifiTry = false;
      wifiRetryCount = 0;
    }
    
  }

  // == Wifi Status Change Checker ==
  if (WiFi.status() != WL_CONNECTED && wasConnected) {
      Serial.println(F("Wifi Disconnected."));
      wasConnected = false;
      wifiTry = true;
  } 
    if (WiFi.status() == WL_CONNECTED && !wasConnected) {
        Serial.println(F("Wifi Reconnected."));
        wasConnected = true;
        wifiTry = false;
        wifiRetryCount = 0;
        mqttTry = true;
        mqttRetryCount = 0;
        shouldSyncNTP = true;
        ntpSyncRequestTime = millis();
    }

    if (shouldSyncNTP && millis() - ntpSyncRequestTime > NTP_DELAY_AFTER_WIFI) {
      struct tm timeinfo;
      if (getLocalTime(&timeinfo)) {
        rtc.adjust(DateTime(timeinfo.tm_year + 1900, timeinfo.tm_mon + 1,
                            timeinfo.tm_mday, timeinfo.tm_hour, timeinfo.tm_min,
                            timeinfo.tm_sec));
        Serial.println("NTP Synced.");
      } else {
        Serial.println("NTP Sync failed.");
      }
      shouldSyncNTP = false;  // Only try once unless reconnected again
    }
  
  // == MQTT Reconnection == 
  if (wifiEnabled && WiFi.status() == WL_CONNECTED) {
    if (!client.connected()) {
      reconnect();
    } else {
      client.loop();  
    }
  }



  DateTime now = rtc.now();
  unsigned long currentMillis = millis();

  if (now.day() != lastDay) {
    int prevDay = (lastDay == -1) ? now.day() - 1 : lastDay;
    if (prevDay > 0) processPreviousDayFile(prevDay, now.month(), now.year());
    lastDay = now.day();
    String fname = "/" + String(now.day()) + "-" + now.month() + "-" + (now.year() % 100) + ".csv";
    fname.toCharArray(filename, sizeof(filename));
    if (!SD.exists(filename)) {
      File file = SD.open(filename, FILE_WRITE);
      if (file) {
        file.println(F("Time,AvgVoltage(V),AvgCurrent(A),AvgPower(W),TotalEnergy(Wh)"));
        file.close();
      }
    }
  }

  if (currentMillis - lastSampleTime >= 1000) {
    lastSampleTime = currentMillis;
    int16_t rawVolt = ads.readADC_SingleEnded(0);
    float adsVolt = (rawVolt * ADS_GAIN) / ADS_MAX_READING;
    float actualVoltage = adsVolt * ((R1 + R2) / R2);

    int16_t rawCurrent = ads.readADC_SingleEnded(1);
    float currentVolt = (rawCurrent * ADS_GAIN) / ADS_MAX_READING;
    float current = ((currentVolt - acs712_zero_voltage) / ACS712_SENSITIVITY) * -1;
    float calibratedCurrent = (1.4215 * current) + 0.0614;
    float power = actualVoltage * calibratedCurrent;
    float energy = power / 3600.0;

    sumVoltage += actualVoltage;
    sumCurrent += calibratedCurrent;
    sumPower += power;
    sumEnergy += energy;
    sampleCount++;

    char mqttMsg[256];
    snprintf(mqttMsg, sizeof(mqttMsg), "T:%s V:%.2f A:%.3f CalA:%.3f",
            now.timestamp(DateTime::TIMESTAMP_TIME),
            actualVoltage, current, calibratedCurrent);

    if (!isSDCardStillPresent()) {
      Serial.println("SD card removed or not accessible!");
      Serial.println("Restarting...");
      delay(2000);
      ESP.restart();
    }

    if (wifiEnabled && WiFi.status() == WL_CONNECTED)
      client.publish(mqtt_publish_topic, mqttMsg);

    Serial.println(mqttMsg);
  }

  if ((now.minute() % 5 == 0) && now.second() == 0 && now.minute() != lastLoggedFiveMinute && sampleCount > 0) {
    lastLoggedFiveMinute = now.minute();
    float avgV = sumVoltage / sampleCount;
    float avgA = sumCurrent / sampleCount;
    float avgP = sumPower / sampleCount;
    float totalE = sumEnergy;

    File file = SD.open(filename, FILE_APPEND);
    if (file) {
      file.printf("%s,%.3f,%.3f,%.3f,%.3f\n", now.timestamp(DateTime::TIMESTAMP_TIME), avgV, avgA, avgP, totalE);
      file.close();
      Serial.println(F("5-min data written to SD."));
    }
    sumVoltage = sumCurrent = sumPower = sumEnergy = 0;
    sampleCount = 0;
  }

  // == Reset Wifi Retrier ==
  if ((now.minute() % 5 == 0) && now.minute() != lastLoggedFiveMinute && now.second() == 0 && WiFi.status() != WL_CONNECTED && wifiEnabled) {
    wifiTry = true;
    wifiRetryCount = 0;
    mqttTry = true;
    mqttRetryCount = 0;
  }

  delay(50);
  
}
