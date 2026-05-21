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

const char* WIFI_SSID = "Your Wifi SSID";
const char* WIFI_PASS = "Your Wifi password";

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
#define C4001_INTERVAL_MS   10
#define OLED_INTERVAL_MS    200
#define SERIAL_INTERVAL_MS  500 // Slower serial output so we don't spam during counts
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

// ---------------- PROTOCOL CONSTANTS ----------------
const uint32_t PREP_TIME_SHORT_MS = 5000;    // 5s between repetitions
const uint32_t PREP_TIME_LONG_MS = 10000;    // 10s between activities
const uint32_t REP_COLLECTION_TIME_MS = 5000; // 5s collection per rep
const uint32_t TOTAL_ACTIVITY_TIME_MS = 600000; // 10 minutes total per activity (600,000 ms)

const int NUM_ACTIVITIES = 8;
const char* ACTIVITY_NAMES[NUM_ACTIVITIES] = {
  "Standing",
  "Sitting",
  "Sit_To_Stand",
  "Stand_To_Sit",
  "Walking",
  "Entry",
  "Exit",
  "No_Person"
};

// ---------------- DATA COLLECTION STATES ----------------

enum CollectionState {
  STATE_INIT,               // Wait for SD card/WiFi
  STATE_WAIT_USER_START,    // Ask "Continue?" (Yes/No) and wait for participant setup
  STATE_WAIT_PERSON_ID,     // Ask for Person ID (1-10)
  STATE_WAIT_CMD,           // Ask user to enter activity number (1-8)
  STATE_ACTIVITY_PREP_LONG, // 10s countdown before an activity begins
  STATE_REP_PREP_SHORT,     // 5s countdown before a repetition begins
  STATE_COLLECTING,         // Active 5s recording
  STATE_ACTIVITY_FINISHED,  // Activity is done, close files, ask next
  STATE_HALTED,             // User said "No" to continue
  STATE_PAUSED              // Paused via Stop command
};

CollectionState currentState = STATE_INIT;
CollectionState savedState = STATE_INIT; // Stores state to resume into
uint32_t stateStartTime = 0;
uint32_t pauseStartTime = 0; // Tracks when the system was paused
uint32_t activityTotalTime = 0; // Tracks total collection time for current activity

int currentActivityIndex = 0;   // 0-7 (corresponds to 1-8 input)
int currentPersonID = 1;        // Increments after a full session

File logFile;
char currentFileName[64];       // e.g., "/P1_Standing.csv"

uint32_t lastPrintSec = 0;      // For clean countdown printing

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
  if (!getLocalTime(&timeinfo, 100)) return false;
  int year = timeinfo.tm_year + 1900;
  return year >= 2024;
}

bool waitForTimeSync(uint32_t timeoutMs) {
  uint32_t start = millis();
  while (millis() - start < timeoutMs) {
    if (isTimeValid()) return true;
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
  configTime(GMT_OFFSET_SEC, DAYLIGHT_OFFSET_SEC, "pool.ntp.org", "time.google.com", "time.nist.gov");
  timeOK = waitForTimeSync(TIME_SYNC_TIMEOUT_MS);
  if (timeOK) {
    Serial.print("Time OK: ");
    Serial.println(getDateTimeString());
  } else {
    Serial.println("Time sync failed.");
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
  if (!getLocalTime(&timeinfo, 200)) return "TIME_NOT_SYNCED";
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
}

void createNewLogFile() {
  if (!sdOK) return;
  
  // Create filename like /P1_Standing.csv
  snprintf(currentFileName, sizeof(currentFileName), "/P%d_%s.csv", currentPersonID, ACTIVITY_NAMES[currentActivityIndex]);
  
  bool exists = SD.exists(currentFileName);
  logFile = SD.open(currentFileName, FILE_APPEND);
  
  if (logFile && !exists) {
      logFile.println(
        "datetime,activity,wifi,time_sync,"
        "rd_present,rd_distance_mm,rd_velocity_cm_s,rd_angle_deg,rd_x_mm,rd_y_mm,"
        "c4001_present,c4001_distance_m,c4001_velocity_m_s,c4001_energy,"
        "c4001_raw_targets,c4001_raw_distance_m,c4001_raw_velocity_m_s,c4001_raw_energy"
      );
      logFile.flush();
  }
}

void logToSD() {
  if (!sdOK || !logFile) return;

  logFile.print(getDateTimeString());
  logFile.print(",");
  logFile.print(ACTIVITY_NAMES[currentActivityIndex]);
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

  bool validNow = c4001LooksValid(cData.rawTargetNumber, cData.rawRange, cData.rawSpeed, cData.rawEnergy);

  if (validNow) {
    if (confirmCount < C4001_CONFIRM_FRAMES) confirmCount++;
    clearCount = 0;
  } else {
    if (clearCount < C4001_CLEAR_FRAMES) clearCount++;
    confirmCount = 0;
  }

  if (confirmCount >= C4001_CONFIRM_FRAMES) cData.present = true;
  if (clearCount >= C4001_CLEAR_FRAMES) cData.present = false;

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
  // Only print sensor data occasionally so we don't spam during countdowns
  if (currentState != STATE_COLLECTING) return; 

  Serial.print("C4001:");
  Serial.print(cData.present ? "Y" : "N");
  Serial.print(" D=");
  Serial.print(cData.range, 2);
  Serial.print("m V=");
  Serial.print(cData.speed, 2);
  
  Serial.print(" | RD:");
  Serial.print(rdTarget.present ? "Y" : "N");
  Serial.print(" D=");
  Serial.print(rdTarget.distance, 0);
  Serial.print("mm V=");
  Serial.print(rdTarget.speed);
  Serial.println("cm/s");
}

void updateDisplay() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SH110X_WHITE);

  display.setCursor(0, 0);
  display.print("P");
  display.print(currentPersonID);
  display.print(" ");
  display.print(getDateTimeString().substring(11)); // Just time

  // Show status
  display.setCursor(0, 10);
  if (!sdOK) display.print("SD:FAIL");
  else display.print("SD:OK");

  display.setCursor(0, 20);
  
  // State specific display
  int secLeft = 0;
  switch (currentState) {
    case STATE_WAIT_USER_START:
      display.print("READY? Type Yes");
      break;
    case STATE_WAIT_PERSON_ID:
      display.print("Enter Person ID");
      break;
    case STATE_WAIT_CMD:
      display.print("Enter Act # (1-9)");
      break;
    case STATE_ACTIVITY_PREP_LONG:
      secLeft = (PREP_TIME_LONG_MS / 1000) - ((millis() - stateStartTime) / 1000);
      display.print("PREP: ");
      display.print(secLeft);
      display.print("s");
      break;
    case STATE_REP_PREP_SHORT:
      secLeft = (PREP_TIME_SHORT_MS / 1000) - ((millis() - stateStartTime) / 1000);
      display.print("PREP: ");
      display.print(secLeft);
      display.print("s");
      break;
    case STATE_COLLECTING:
      secLeft = (REP_COLLECTION_TIME_MS / 1000) - ((millis() - stateStartTime) / 1000);
      display.print("REC: ");
      display.print(secLeft);
      display.print("s");
      display.setCursor(0, 30);
      display.print("Act: ");
      display.print(ACTIVITY_NAMES[currentActivityIndex]);
      break;
    case STATE_ACTIVITY_FINISHED:
      display.print("Activity Done");
      break;
    case STATE_HALTED:
      display.print("HALTED.");
      break;
    case STATE_PAUSED:
      display.print("PAUSED...");
      break;
    default:
      break;
  }

  // Sensor data
  if (currentState == STATE_COLLECTING || currentState == STATE_REP_PREP_SHORT) {
    display.setCursor(0, 40);
    display.print("RD D:"); display.print(rdTarget.distance, 0);
    display.print(" V:"); display.print(rdTarget.speed);
  
    display.setCursor(0, 50);
    display.print("C D:"); display.print(cData.range, 2);
    display.print(" V:"); display.print(cData.speed, 1);
  }

  display.display();
}

// ---------------- SETUP / LOOP ----------------

void setup() {
  Serial.begin(115200);
  delay(300);

  Serial.println("==================================================");
  Serial.println("Protocol Data Collection System");
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

  while (!c4001.begin()) {
    Serial.println("C4001 not found.");
    delay(300);
  }
  c4001.setSensorMode(eSpeedMode);
  c4001.setDetectThres(40, 400, C4001_DETECT_THRESHOLD);
  c4001.setFrettingDetection(eOFF);

  syncDateTime();
  initSD();

  Serial.println("\nSystem initialized.");
  currentState = STATE_WAIT_USER_START;
  
  // Clear serial buffer just in case
  while(Serial.available()) Serial.read();
}

void loop() {
  static uint32_t lastC4001 = 0;
  static uint32_t lastDisplay = 0;
  static uint32_t lastSerial = 0;
  static uint32_t lastLog = 0;
  static uint32_t lastFlush = 0;

  uint32_t now = millis();
  
  String input = "";

  // --- GLOBAL COMMAND PARSING (Stop / Resume) ---
  if (Serial.available()) {
    input = Serial.readStringUntil('\n');
    input.trim();

    if (input.equalsIgnoreCase("Stop") || input.equalsIgnoreCase("Pause")) {
      // Only allow pausing during active countdowns or collection
      if (currentState == STATE_ACTIVITY_PREP_LONG || 
          currentState == STATE_REP_PREP_SHORT || 
          currentState == STATE_COLLECTING) {
        
        savedState = currentState;
        currentState = STATE_PAUSED;
        pauseStartTime = now;
        
        Serial.println("\n=========================================");
        Serial.println(">>> SYSTEM PAUSED (STOPPED) <<<");
        Serial.println("Type 'Resume' to continue where you left off.");
        Serial.println("=========================================\n");
        
        input = ""; // Consume input so state machine doesn't process it
        lastPrintSec = 0; // Reset print flag
      } else if (currentState == STATE_PAUSED) {
        Serial.println("System is already paused.");
        input = "";
      }
    } 
    else if (input.equalsIgnoreCase("Resume") || input.equalsIgnoreCase("Start")) {
      if (currentState == STATE_PAUSED) {
        currentState = savedState;
        uint32_t pauseDuration = now - pauseStartTime;
        stateStartTime += pauseDuration; // Shift the start time forward to account for the pause
        
        Serial.println("\n=========================================");
        Serial.println(">>> SYSTEM RESUMED <<<");
        Serial.println("=========================================\n");
        
        input = ""; // Consume input
        lastPrintSec = 0; 
      } else {
        Serial.println("System is not paused.");
        input = "";
      }
    }
  }

  // ================= STATE MACHINE =================
  switch (currentState) {
      
    case STATE_WAIT_USER_START:
      if (lastPrintSec == 0) {
        Serial.println("\n=========================================");
        Serial.println("Ready to begin data collection session.");
        Serial.println("Type 'Yes' to continue or 'No' to halt.");
        Serial.println("Note: Type 'Stop' during collection to pause, and 'Resume' to continue.");
        Serial.println("=========================================\n");
        lastPrintSec = 1; // Used as a flag to not repeat print
      }
      
      if (input != "") {
        if (input.equalsIgnoreCase("Yes")) {
          currentState = STATE_WAIT_PERSON_ID;
          lastPrintSec = 0;
        } else if (input.equalsIgnoreCase("No")) {
          currentState = STATE_HALTED;
          Serial.println("System Halted.");
        }
      }
      break;

    case STATE_WAIT_PERSON_ID:
      if (lastPrintSec == 0) {
        Serial.println("\nWhich person data you want to collect? (e.g., 1, 2, 3, ... 10)");
        Serial.println("Enter a number:");
        lastPrintSec = 1;
      }
      
      if (input != "") {
        int pid = input.toInt();
        if (pid > 0) {
          currentPersonID = pid;
          Serial.printf("\n>>> Person ID set to: P%d <<<\n", currentPersonID);
          currentState = STATE_WAIT_CMD;
          lastPrintSec = 0;
        } else {
          Serial.println("Invalid input. Please enter a valid number (e.g., 1-10).");
        }
      }
      break;

    case STATE_WAIT_CMD:
      if (lastPrintSec == 0) {
        Serial.printf("\nSelect next activity to record for Person %d:\n", currentPersonID);
        for(int i=0; i<NUM_ACTIVITIES; i++) {
          Serial.printf("%d. %s\n", i+1, ACTIVITY_NAMES[i]);
        }
        Serial.println("9. Finish this person & ask for next person");
        Serial.println("Enter a number (1-9):");
        lastPrintSec = 1; 
      }
      
      if (input != "") {
        int cmd = input.toInt();
        if (cmd >= 1 && cmd <= 8) {
          currentActivityIndex = cmd - 1;
          
          // Open new file for this activity
          createNewLogFile();
          
          Serial.printf("\n>>> Selected: %s. Opening file: %s <<<\n", ACTIVITY_NAMES[currentActivityIndex], currentFileName);
          currentState = STATE_ACTIVITY_PREP_LONG;
          stateStartTime = now;
          activityTotalTime = 0;
          lastPrintSec = 0;
        } else if (cmd == 9) {
          currentState = STATE_WAIT_USER_START;
          lastPrintSec = 0;
        } else {
          Serial.println("Invalid input. Enter a number between 1 and 9.");
        }
      }
      break;

    case STATE_ACTIVITY_PREP_LONG:
      {
        uint32_t elapsed = now - stateStartTime;
        uint32_t secLeft = (PREP_TIME_LONG_MS / 1000) - (elapsed / 1000);
        
        if (secLeft != lastPrintSec) {
          Serial.printf("TAKE YOUR POSITION! Activity starts in %d...\n", secLeft);
          lastPrintSec = secLeft;
        }

        if (elapsed >= PREP_TIME_LONG_MS) {
          currentState = STATE_COLLECTING;
          stateStartTime = now;
          Serial.println("\n>>> RECORDING STARTED <<<\n");
          lastPrintSec = 0;
        }
      }
      break;

    case STATE_COLLECTING:
      {
        uint32_t elapsed = now - stateStartTime;
        uint32_t secLeft = (REP_COLLECTION_TIME_MS / 1000) - (elapsed / 1000);
        
        // Print 5,4,3,2,1 during collection if desired, though usually you just want data
        // Here we just let it run silently to not block serial.
        
        if (elapsed >= REP_COLLECTION_TIME_MS) {
          activityTotalTime += REP_COLLECTION_TIME_MS;
          
          if (activityTotalTime >= TOTAL_ACTIVITY_TIME_MS) {
             currentState = STATE_ACTIVITY_FINISHED;
          } else {
             currentState = STATE_REP_PREP_SHORT;
             stateStartTime = now;
             Serial.println("\n>>> RELAX. Next rep in 5 seconds... <<<\n");
             lastPrintSec = 0;
          }
        }
      }
      break;

    case STATE_REP_PREP_SHORT:
      {
        uint32_t elapsed = now - stateStartTime;
        uint32_t secLeft = (PREP_TIME_SHORT_MS / 1000) - (elapsed / 1000);
        
        if (secLeft != lastPrintSec) {
          Serial.printf("Prepare... %d\n", secLeft);
          lastPrintSec = secLeft;
        }

        if (elapsed >= PREP_TIME_SHORT_MS) {
          currentState = STATE_COLLECTING;
          stateStartTime = now;
          Serial.printf("\n>>> RECORDING STARTED (Total %d / %d ms) <<<\n", activityTotalTime, TOTAL_ACTIVITY_TIME_MS);
          lastPrintSec = 0;
        }
      }
      break;

    case STATE_ACTIVITY_FINISHED:
      if (logFile) {
        logFile.close();
      }
      Serial.printf("\n>>> Finished 10 minutes for %s <<<\n", ACTIVITY_NAMES[currentActivityIndex]);
      
      // Go back and ask for next command
      currentState = STATE_WAIT_CMD;
      lastPrintSec = 0;
      
      // If you want to automatically increment person after all 8 are done, 
      // you would need logic here to track which ones have been completed.
      // Currently, it just drops you back to the menu to select the next one manually.
      break;
      
    case STATE_HALTED:
      // Do nothing
      break;
      
    case STATE_PAUSED:
      // Waiting for "Resume" command. Handled at the top of loop()
      break;
      
    case STATE_INIT:
    default:
      break;
  }
  // ================= END STATE MACHINE =================


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
  
  // 4. Flush file buffer to SD Card periodically
  if (currentState == STATE_COLLECTING && now - lastFlush >= 1000) {
    lastFlush = now;
    if (logFile) {
      logFile.flush();
    }
  }
}
