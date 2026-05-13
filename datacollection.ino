#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <WiFi.h>
#include <time.h>

#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>
#include "DFRobot_C4001.h"

// ---------------- WIFI ----------------

const char* WIFI_SSID = "RUPAMPC";
const char* WIFI_PASS = "123737rd";

// India Standard Time = UTC + 5:30
const long GMT_OFFSET_SEC = 19800;
const int DAYLIGHT_OFFSET_SEC = 0;

// ---------------- PIN SETTINGS ----------------

#define C4001_SDA 6
#define C4001_SCL 7

#define OLED_SDA 8
#define OLED_SCL 9
#define OLED_ADDR 0x3C

#define RD03D_RX_PIN 4
#define RD03D_TX_PIN 5
#define RD03D_BAUD 256000

#define SD_CS   10
#define SD_SCK  12
#define SD_MISO 13
#define SD_MOSI 11

// ---------------- TIMING & SAMPLING ----------------

// 50ms interval = 20 Hz Sampling Rate
#define C4001_INTERVAL_MS   50
#define OLED_INTERVAL_MS    200
#define SERIAL_INTERVAL_MS  300
#define SD_LOG_INTERVAL_MS  50

#define WIFI_CONNECT_TIMEOUT_MS 8000
#define TIME_SYNC_TIMEOUT_MS    8000
#define WIFI_RETRY_MS          5000
#define TIME_RETRY_MS          5000

// ---------------- C4001 FILTER SETTINGS ----------------

#define C4001_MIN_RANGE_M       0.40
#define C4001_MAX_RANGE_M       4.00
#define C4001_MIN_ENERGY        3000
#define C4001_CONFIRM_FRAMES    3
#define C4001_CLEAR_FRAMES      6
#define C4001_DETECT_THRESHOLD  500

// ---------------- DATA COLLECTION STATES ----------------

enum CollectionState {
  STATE_IDLE,
  STATE_PREP_WAIT,
  STATE_COLLECTING
};

CollectionState currentState = STATE_IDLE;
uint32_t stateStartTime = 0;

const uint32_t PREP_TIME_MS = 10000;       // 10 seconds wait before collection
const uint32_t COLLECTION_TIME_MS = 30000; // 30 seconds of continuous collection
File logFile; // Global file object for continuous high-speed writing

// ---------------- OBJECTS ----------------

TwoWire C4001Wire = TwoWire(1);
HardwareSerial RDSerial(1);

Adafruit_SH1106G display = Adafruit_SH1106G(128, 64, &Wire, -1);
DFRobot_C4001_I2C c4001(&C4001Wire, DEVICE_ADDR_0);

// ---------------- STATUS ----------------

bool sdOK = false;
bool wifiOK = false;
bool timeOK = false;

// ---------------- RD-03D SINGLE TARGET ----------------

struct RDTarget {
  bool present = false;
  int16_t x = 0;
  int16_t y = 0;
  int16_t speed = 0;
  uint16_t resolution = 0;
  float distance = 0;
  float angle = 0;
};

RDTarget rdTarget;
uint8_t rdTargetCount = 0;

uint8_t rdBuf[30];
uint8_t rdPos = 0;

const uint8_t RD_HEADER[4] = {0xAA, 0xFF, 0x03, 0x00};
const uint8_t RD_FOOTER[2] = {0x55, 0xCC};

const uint8_t RD_SINGLE_TARGET_CMD[12] = {
  0xFD, 0xFC, 0xFB, 0xFA,
  0x02, 0x00,
  0x80, 0x00,
  0x04, 0x03, 0x02, 0x01
};

int16_t decodeRD03DValue(uint8_t lowByte, uint8_t highByte) {
  int16_t value = ((highByte & 0x7F) << 8) | lowByte;
  if ((highByte & 0x80) == 0) value = -value;
  return value;
}

void processRD03DFrame() {
  int16_t x = decodeRD03DValue(rdBuf[4], rdBuf[5]);
  int16_t y = decodeRD03DValue(rdBuf[6], rdBuf[7]);
  int16_t speed = decodeRD03DValue(rdBuf[8], rdBuf[9]);
  uint16_t res = ((uint16_t)rdBuf[11] << 8) | rdBuf[10];

  bool present = (x != 0 || y != 0 || res != 0);

  rdTarget.present = present;
  rdTarget.x = x;
  rdTarget.y = y;
  rdTarget.speed = speed;
  rdTarget.resolution = res;
  rdTarget.distance = sqrt((float)x * x + (float)y * y);
  rdTarget.angle = atan2((float)x, (float)y) * 180.0 / PI;

  rdTargetCount = present ? 1 : 0;
}

void readRD03D() {
  while (RDSerial.available()) {
    uint8_t b = RDSerial.read();

    if (rdPos < 4) {
      if (b == RD_HEADER[rdPos]) {
        rdBuf[rdPos++] = b;
      } else if (b == RD_HEADER[0]) {
        rdBuf[0] = b;
        rdPos = 1;
      } else {
        rdPos = 0;
      }
      continue;
    }

    rdBuf[rdPos++] = b;

    if (rdPos >= 30) {
      if (rdBuf[28] == RD_FOOTER[0] && rdBuf[29] == RD_FOOTER[1]) {
        processRD03DFrame();
      }
      rdPos = 0;
    }
  }
}

// ---------------- C4001 ----------------

struct C4001Data {
  bool present = false;

  int rawTargetNumber = 0;
  float rawRange = 0;
  float rawSpeed = 0;
  int rawEnergy = 0;

  int targetNumber = 0;
  float range = 0;
  float speed = 0;
  int energy = 0;
};

C4001Data cData;

// ---------------- WIFI + TIME ----------------

String getDateTimeString();

bool isTimeValid() {
  struct tm timeinfo;

  if (!getLocalTime(&timeinfo, 100)) {
    return false;
  }

  int year = timeinfo.tm_year + 1900;
  return year >= 2024;
}

bool waitForTimeSync(uint32_t timeoutMs) {
  uint32_t start = millis();

  while (millis() - start < timeoutMs) {
    if (isTimeValid()) {
      return true;
    }

    delay(250);
  }

  return false;
}

bool connectWiFi() {
  if (WiFi.status() == WL_CONNECTED) {
    wifiOK = true;
    return true;
  }

  Serial.println("Connecting WiFi...");

  WiFi.mode(WIFI_STA);
  WiFi.persistent(false);
  WiFi.setSleep(false);
  WiFi.disconnect(false, false);
  delay(200);

  WiFi.begin(WIFI_SSID, WIFI_PASS);

  uint32_t start = millis();

  while (WiFi.status() != WL_CONNECTED && millis() - start < WIFI_CONNECT_TIMEOUT_MS) {
    delay(250);
    Serial.print(".");
  }

  Serial.println();

  if (WiFi.status() == WL_CONNECTED) {
    wifiOK = true;
    Serial.print("WiFi OK, IP: ");
    Serial.println(WiFi.localIP());
    return true;
  }

  wifiOK = false;
  Serial.print("WiFi failed, status: ");
  Serial.println(WiFi.status());
  return false;
}

bool syncDateTime() {
  if (!connectWiFi()) {
    timeOK = false;
    return false;
  }

  Serial.println("Syncing time using NTP...");

  configTime(
    GMT_OFFSET_SEC,
    DAYLIGHT_OFFSET_SEC,
    "pool.ntp.org",
    "time.google.com",
    "time.nist.gov"
  );

  timeOK = waitForTimeSync(TIME_SYNC_TIMEOUT_MS);

  if (timeOK) {
    Serial.print("Time OK: ");
    Serial.println(getDateTimeString());
  } else {
    Serial.println("Time sync failed. Check internet access, DNS, firewall, or NTP blocking.");
  }

  return timeOK;
}

void maintainWiFiAndTime() {
  static uint32_t lastWifiTry = 0;
  static uint32_t lastTimeTry = 0;

  uint32_t nowMs = millis();

  if (WiFi.status() != WL_CONNECTED) {
    wifiOK = false;
    timeOK = false;

    if (nowMs - lastWifiTry >= WIFI_RETRY_MS) {
      lastWifiTry = nowMs;
      connectWiFi();
    }

    return;
  }

  wifiOK = true;

  if (isTimeValid()) {
    timeOK = true;
    return;
  }

  timeOK = false;

  if (nowMs - lastTimeTry >= TIME_RETRY_MS) {
    lastTimeTry = nowMs;
    syncDateTime();
  }
}

String getDateTimeString() {
  struct tm timeinfo;

  if (!getLocalTime(&timeinfo, 200)) {
    return "TIME_NOT_SYNCED";
  }

  char buf[24];
  strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S", &timeinfo);
  return String(buf);
}

// ---------------- SD CARD ----------------

void initSD() {
  SPI.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS);

  if (!SD.begin(SD_CS, SPI)) {
    Serial.println("SD card failed.");
    sdOK = false;
    return;
  }

  sdOK = true;
  Serial.println("SD card ready.");

  if (!SD.exists("/Projectmmwave_log.csv")) {
    File f = SD.open("/Projectmmwave_log.csv", FILE_WRITE);
    if (f) {
      f.println(
        "datetime,wifi,time_sync,"
        "rd_present,rd_distance_mm,rd_velocity_cm_s,rd_angle_deg,rd_x_mm,rd_y_mm,"
        "c4001_present,c4001_distance_m,c4001_velocity_m_s,c4001_energy,"
        "c4001_raw_targets,c4001_raw_distance_m,c4001_raw_velocity_m_s,c4001_raw_energy"
      );
      f.close();
    }
  }
}

void logToSD() {
  if (!sdOK || !logFile) return;

  logFile.print(getDateTimeString());
  logFile.print(",");
  logFile.print(wifiOK ? 1 : 0);
  logFile.print(",");
  logFile.print(timeOK ? 1 : 0);
  logFile.print(",");

  logFile.print(rdTarget.present ? 1 : 0);
  logFile.print(",");
  logFile.print(rdTarget.distance, 1);
  logFile.print(",");
  logFile.print(rdTarget.speed);
  logFile.print(",");
  logFile.print(rdTarget.angle, 1);
  logFile.print(",");
  logFile.print(rdTarget.x);
  logFile.print(",");
  logFile.print(rdTarget.y);
  logFile.print(",");

  logFile.print(cData.present ? 1 : 0);
  logFile.print(",");
  logFile.print(cData.range, 2);
  logFile.print(",");
  logFile.print(cData.speed, 2);
  logFile.print(",");
  logFile.print(cData.energy);
  logFile.print(",");

  logFile.print(cData.rawTargetNumber);
  logFile.print(",");
  logFile.print(cData.rawRange, 2);
  logFile.print(",");
  logFile.print(cData.rawSpeed, 2);
  logFile.print(",");
  logFile.println(cData.rawEnergy);
}

// ---------------- DISPLAY / SERIAL ----------------

bool c4001LooksValid(int targets, float range, float speed, int energy) {
  if (targets <= 0) return false;
  if (range < C4001_MIN_RANGE_M || range > C4001_MAX_RANGE_M) return false;
  if (energy < C4001_MIN_ENERGY) return false;

  return true;
}

void updateC4001() {
  static uint8_t confirmCount = 0;
  static uint8_t clearCount = 0;

  cData.rawTargetNumber = c4001.getTargetNumber();
  cData.rawRange = c4001.getTargetRange();
  cData.rawSpeed = c4001.getTargetSpeed();
  cData.rawEnergy = c4001.getTargetEnergy();

  bool validNow = c4001LooksValid(
    cData.rawTargetNumber,
    cData.rawRange,
    cData.rawSpeed,
    cData.rawEnergy
  );

  if (validNow) {
    if (confirmCount < C4001_CONFIRM_FRAMES) confirmCount++;
    clearCount = 0;
  } else {
    if (clearCount < C4001_CLEAR_FRAMES) clearCount++;
    confirmCount = 0;
  }

  if (confirmCount >= C4001_CONFIRM_FRAMES) {
    cData.present = true;
  }

  if (clearCount >= C4001_CLEAR_FRAMES) {
    cData.present = false;
  }

  if (cData.present) {
    cData.targetNumber = cData.rawTargetNumber;
    cData.range = cData.rawRange;
    cData.speed = cData.rawSpeed;
    cData.energy = cData.rawEnergy;
  } else {
    cData.targetNumber = 0;
    cData.range = 0;
    cData.speed = 0;
    cData.energy = 0;
  }
}

void printSerial() {
  Serial.print(getDateTimeString());
  Serial.print(" | WiFi:");
  Serial.print(wifiOK ? "OK" : "NO");
  Serial.print(" Time:");
  Serial.print(timeOK ? "OK" : "NO");

  Serial.print(" | RD:");
  Serial.print(rdTarget.present ? "YES" : "NO");
  Serial.print(" D=");
  Serial.print(rdTarget.distance, 0);
  Serial.print("mm V=");
  Serial.print(rdTarget.speed);
  Serial.print("cm/s A=");
  Serial.print(rdTarget.angle, 1);
  
  Serial.print(" | C4001:");
  Serial.print(cData.present ? "YES" : "NO");
  Serial.print(" D=");
  Serial.print(cData.range, 2);
  Serial.print("m V=");
  Serial.print(cData.speed, 2);
  Serial.print("m/s E=");
  Serial.println(cData.energy);
}

void updateDisplay() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SH110X_WHITE);

  display.setCursor(0, 0);
  display.print(getDateTimeString());

  // Show status of WiFi, Time, and CURRENT SD STATE
  display.setCursor(0, 10);
  display.print("W:");
  display.print(wifiOK ? "OK " : "NO ");
  display.print("SD: ");

  if (!sdOK) {
    display.print("FAIL");
  } else if (currentState == STATE_IDLE) {
    display.print("IDLE");
  } else if (currentState == STATE_PREP_WAIT) {
    int secLeft = (PREP_TIME_MS / 1000) - ((millis() - stateStartTime) / 1000);
    if (secLeft < 0) secLeft = 0;
    display.print("WAIT ");
    display.print(secLeft);
    display.print("s");
  } else if (currentState == STATE_COLLECTING) {
    int secLeft = (COLLECTION_TIME_MS / 1000) - ((millis() - stateStartTime) / 1000);
    if (secLeft < 0) secLeft = 0;
    display.print("REC ");
    display.print(secLeft);
    display.print("s");
  }

  display.setCursor(0, 20);
  display.print("RD D:");
  display.print(rdTarget.distance, 0);
  display.print("mm V:");
  display.print(rdTarget.speed);

  display.setCursor(0, 30);
  display.print("RD A:");
  display.print(rdTarget.angle, 1);
  display.print("deg");

  display.setCursor(0, 40);
  display.print("X:");
  display.print(rdTarget.x);
  display.print(" Y:");
  display.print(rdTarget.y);

  display.setCursor(0, 50);
  display.print("C D:");
  display.print(cData.range, 2);
  display.print("m V:");
  display.print(cData.speed, 1);

  display.display();
}

// ---------------- SETUP / LOOP ----------------

void setup() {
  Serial.begin(115200);
  delay(300);

  Serial.println("==================================================");
  Serial.println("Starting Human Activity Identification System");
  Serial.println("System will AUTO-RUN: 10s wait -> 30s collect -> repeat");
  Serial.println("==================================================");

  Wire.begin(OLED_SDA, OLED_SCL);
  Wire.setClock(400000);

  C4001Wire.begin(C4001_SDA, C4001_SCL);
  C4001Wire.setClock(400000);

  if (!display.begin(OLED_ADDR, true)) {
    Serial.println("SH1106 OLED not found.");
  } else {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SH110X_WHITE);
    display.setCursor(0, 0);
    display.println("Starting...");
    display.display();
  }

  RDSerial.setRxBufferSize(256);
  RDSerial.begin(RD03D_BAUD, SERIAL_8N1, RD03D_RX_PIN, RD03D_TX_PIN);
  delay(50);
  RDSerial.write(RD_SINGLE_TARGET_CMD, sizeof(RD_SINGLE_TARGET_CMD));
  Serial.println("RD-03D single-target mode enabled.");

  while (!c4001.begin()) {
    Serial.println("C4001 not found on SDA 6 / SCL 7.");
    delay(300);
  }

  Serial.println("C4001 connected.");

  c4001.setSensorMode(eSpeedMode);

  if (c4001.setDetectThres(40, 400, C4001_DETECT_THRESHOLD)) {
    Serial.println("C4001 detection threshold set.");
  } else {
    Serial.println("C4001 detection threshold failed.");
  }

  c4001.setFrettingDetection(eOFF);

  syncDateTime();
  initSD();

  Serial.println("\nSystem ready. Auto-mode starting...\n");
}

void loop() {
  static uint32_t lastC4001 = 0;
  static uint32_t lastDisplay = 0;
  static uint32_t lastSerial = 0;
  static uint32_t lastLog = 0;
  static uint32_t lastFlush = 0;

  uint32_t now = millis();

  // 1. Fully Automated State Machine
  if (currentState == STATE_IDLE) {
    // Only start if SD card successfully initialized
    if (sdOK) {
      currentState = STATE_PREP_WAIT;
      stateStartTime = now;
      Serial.println("\n>>> 10-SECOND PREPARATION PHASE... GET IN POSTURE! <<<\n");
    }
  } else if (currentState == STATE_PREP_WAIT) {
    if (now - stateStartTime >= PREP_TIME_MS) {
      // 10 seconds passed, switch to data collection
      currentState = STATE_COLLECTING;
      stateStartTime = now; 
      logFile = SD.open("/Projectmmwave_log.csv", FILE_APPEND);
      Serial.println("\n>>> STARTING 30-SECOND DATA COLLECTION (20 Hz) <<<\n");
    }
  } else if (currentState == STATE_COLLECTING) {
    if (now - stateStartTime >= COLLECTION_TIME_MS) {
      // 30 seconds passed, save file and go back to waiting
      currentState = STATE_PREP_WAIT;
      stateStartTime = now;
      if (logFile) {
        logFile.close();
      }
      Serial.println("\n>>> COLLECTION FINISHED. FILE SAVED. <<<\n");
      Serial.println(">>> 10-SECOND PREPARATION PHASE... ADJUST POSTURE! <<<\n");
    }
  }

  // 2. Normal background tasks
  readRD03D();
  maintainWiFiAndTime();

  if (now - lastC4001 >= C4001_INTERVAL_MS) {
    lastC4001 = now;
    updateC4001();
  }

  if (now - lastDisplay >= OLED_INTERVAL_MS) {
    lastDisplay = now;
    updateDisplay();
  }

  // Still print to serial to verify sensor health, regardless of state
  if (now - lastSerial >= SERIAL_INTERVAL_MS) {
    lastSerial = now;
    printSerial();
  }

  // 3. Log to SD (ONLY when we are in the COLLECTING state)
  if (now - lastLog >= SD_LOG_INTERVAL_MS) {
    lastLog = now;
    if (currentState == STATE_COLLECTING) {
      logToSD();
    }
  }
  
  // 4. Flush file buffer to SD Card periodically (every 1 second)
  if (currentState == STATE_COLLECTING && now - lastFlush >= 1000) {
    lastFlush = now;
    if (logFile) {
      logFile.flush();
    }
  }
}