#include <Arduino.h>
#include <Wire.h>
#include <HardwareSerial.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>
#include "DFRobot_C4001.h"

// ---------------------------------------------------------
// INCLDUE YOUR TRAINED MODEL HERE
// (Make sure RUPAM_model.h is in the same folder as this .ino file)
// ---------------------------------------------------------
#include "RUPAM_model.h"

// Instantiate the Random Forest Classifier
Eloquent::ML::Port::RandomForest clf;

// ---------------------------------------------------------
// PIN & SENSOR DEFINITIONS
// ---------------------------------------------------------
// C4001 mmWave Sensor (I2C Bus 0)
#define C4001_SDA 6
#define C4001_SCL 7

// C4001 Logic
#define C4001_CONFIRM_FRAMES 3
#define C4001_CLEAR_FRAMES   6

// SH1106 OLED Display (I2C Bus 1)
#define OLED_SDA 8
#define OLED_SCL 9
#define OLED_ADDR 0x3C // Default SH1106 I2C address

// Rd-03d mmWave Sensor (UART 1)
#define RD03D_RX_PIN 4
#define RD03D_TX_PIN 5
#define RD03D_BAUD 256000

// ---------------------------------------------------------
// GLOBAL OBJECTS & VARIABLES
// ---------------------------------------------------------
// Display instance using TwoWire (Wire1)
Adafruit_SH1106G display = Adafruit_SH1106G(128, 64, &Wire1, -1);

// C4001 instance (Use I2C Derived Class)
DFRobot_C4001_I2C c4001(&Wire, DEVICE_ADDR_0);

// Rd-03d Serial instance
HardwareSerial rdSerial(1);

// --- Sensor Variables (Mapped to Model Features) ---
float rd_distance_mm = 0.0;
float rd_velocity_cm_s = 0.0;
float rd_angle_deg = 0.0;
float rd_x_mm = 0.0;
float rd_y_mm = 0.0;

float c4001_distance_m = 0.0;
float c4001_velocity_m_s = 0.0;
float c4001_energy = 0.0;

// --- EMA (Noise Reduction) Variables ---
const float EMA_ALPHA = 0.3;
float smoothed_features[8] = {0, 0, 0, 0, 0, 0, 0, 0};
bool first_reading = true;

// --- Missing C4001 Data Struct ---
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

// --- Missing RD03D Parsing Variables ---
uint8_t rdBuf[30];
uint8_t rdPos = 0;
const uint8_t RD_HEADER[4] = {0xAA, 0xFF, 0x03, 0x00};
const uint8_t RD_FOOTER[2] = {0x55, 0xCC};

// Timing
unsigned long lastInferenceTime = 0;
const int INFERENCE_INTERVAL_MS = 100; // Run model every 100ms (10 Hz)

// Function Prototypes to prevent scope errors
void updateDisplay(String activity);
void performInference();

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n--- ESP32-S3 ML Human Activity Inference ---");

  // 1. Initialize I2C Bus 0 for C4001
  Wire.begin(C4001_SDA, C4001_SCL);
  
  // Initialize C4001
  if (!c4001.begin()) {
    Serial.println("C4001 initialization failed!");
  } else {
    Serial.println("C4001 initialized.");
  }

  // 2. Initialize I2C Bus 1 for OLED SH1106
  Wire1.begin(OLED_SDA, OLED_SCL);
  if (!display.begin(OLED_ADDR, true)) {
    Serial.println("OLED initialization failed!");
  } else {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SH110X_WHITE);
    display.setCursor(0, 0);
    display.println("System Booting...");
    display.println("Loading ML Model...");
    display.display();
  }

  // 3. Initialize UART 1 for Rd-03d
  rdSerial.begin(RD03D_BAUD, SERIAL_8N1, RD03D_RX_PIN, RD03D_TX_PIN);

  Serial.println("Initialization Complete. Waiting for sensor data...");
}

void loop() {
  // 1. Constantly update sensors in the background
  updateC4001();
  readRD03D();

  // 2. Perform Inference at a fixed interval
  unsigned long currentMillis = millis();
  if (currentMillis - lastInferenceTime >= INFERENCE_INTERVAL_MS) {
    lastInferenceTime = currentMillis;
    performInference();
  }
}

// ---------------------------------------------------------
// ML INFERENCE PIPELINE
// ---------------------------------------------------------
void performInference() {
  float raw_features[8] = {
    rd_distance_mm, 
    rd_velocity_cm_s, 
    rd_angle_deg, 
    rd_x_mm, 
    rd_y_mm, 
    c4001_distance_m, 
    c4001_velocity_m_s, 
    c4001_energy
  };

  // Apply Exponential Moving Average (EMA)
  for (int i = 0; i < 8; i++) {
    if (first_reading) {
      smoothed_features[i] = raw_features[i]; 
    } else {
      smoothed_features[i] = (EMA_ALPHA * raw_features[i]) + ((1.0 - EMA_ALPHA) * smoothed_features[i]);
    }
  }
  first_reading = false;

  // Run Model Prediction
  String predicted_activity = clf.predictLabel(smoothed_features);

  // --- POST-PROCESSING STATE MACHINE (LATCHING) ---
  static String held_activity = "INITIALIZING";

  if (predicted_activity == "STAND TO SIT") {
    // Show transition text, confirm with negative (or zero) velocity
    if (rd_velocity_cm_s <= 0) { 
      held_activity = "STAND TO SIT";
    }
  } 
  else if (predicted_activity == "SIT TO STAND") {
    // Show transition text, confirm with positive (or zero) velocity
    if (rd_velocity_cm_s >= 0) {
      held_activity = "SIT TO STAND";
    }
  }
  else {
    // For WALKING, SITTING, STANDING, NO PERSON, ENTRY, EXIT
    // Let the ML directly dictate the screen so all classes show properly.
    held_activity = predicted_activity;
  }

  // Display Results
  Serial.print("Raw ML: ");
  Serial.print(predicted_activity);
  Serial.print(" | Final Activity: ");
  Serial.println(held_activity);
  
  updateDisplay(held_activity);
}

// ---------------------------------------------------------
// SENSOR READING FUNCTIONS
// ---------------------------------------------------------
bool c4001LooksValid(int targets, float range, float speed, int energy) {
  if (targets <= 0) return false;
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
    cData.range = 0.0;
    cData.speed = 0.0;
    cData.energy = 0;
  }

  // UPDATE GLOBAL ML FEATURES
  c4001_distance_m = cData.range;
  c4001_velocity_m_s = cData.speed;
  c4001_energy = cData.energy;
}

int16_t decodeRD03DValue(uint8_t lowByte, uint8_t highByte) {
  int16_t value = ((highByte & 0x7F) << 8) | lowByte;
  if ((highByte & 0x80) == 0) value = -value;
  return value;
}

void processRD03DFrame() {
  int16_t x = decodeRD03DValue(rdBuf[4], rdBuf[5]);
  int16_t y = decodeRD03DValue(rdBuf[6], rdBuf[7]);
  int16_t speed = decodeRD03DValue(rdBuf[8], rdBuf[9]);

  // UPDATE GLOBAL ML FEATURES
  rd_x_mm = x;
  rd_y_mm = y;
  rd_velocity_cm_s = speed;
  rd_distance_mm = sqrt((float)x * x + (float)y * y);
  rd_angle_deg = atan2((float)x, (float)y) * 180.0 / PI;
}

void readRD03D() {
  while (rdSerial.available()) {
    uint8_t b = rdSerial.read();

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

// ---------------------------------------------------------
// OLED DISPLAY UPDATE
// ---------------------------------------------------------
void updateDisplay(String activity) {
  display.clearDisplay();
  
  // Header
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println("Human Sensing AI");
  display.drawLine(0, 10, 128, 10, SH110X_WHITE);

  // Main Activity
  display.setTextSize(2);
  display.setCursor(0, 20);
  display.println(activity);

  // Sensor Debug Info (Now including Velocity)
  display.setTextSize(1);
  display.setCursor(0, 45);
  display.print("RD: "); display.print(rd_distance_mm, 0); display.print("mm ");
  display.print("V:"); display.print(rd_velocity_cm_s, 0); display.println("cm/s");
  
  display.setCursor(0, 55);
  display.print("C4: "); display.print(c4001_distance_m, 2); display.print("m ");
  display.print("V:"); display.print(c4001_velocity_m_s, 1); display.println("m/s");

  display.display();
}