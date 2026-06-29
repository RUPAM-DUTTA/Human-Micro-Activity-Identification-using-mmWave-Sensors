#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>

// Display and Sensor Libraries
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>

// Bluetooth LE Libraries
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

#include "rd03d_model.h" 

/* ==========================================
 * PIN SETTINGS
 * ========================================== */
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

/* ==========================================
 * TIMING INTERVALS
 * ========================================== */
#define INFERENCE_INTERVAL_MS 100  // 10 Hz
#define OLED_INTERVAL_MS      350 // ~3 Hz
#define BLE_INTERVAL_MS       500 // 2 Hz

unsigned long lastDisplay = 0;
unsigned long lastBLEUpdate = 0;

/* ==========================================
 * EMA FILTER VARIABLES (STABILIZATION)
 * FINE TUNING: Lowered Alphas to smooth out jitter. Slower, but highly stable.
 * ========================================== */
const float EMA_ALPHA_DIST = 0.10; // Was 0.2
const float EMA_ALPHA_VEL  = 0.15; // Was 0.3

float smooth_rd_dist = 0;
float smooth_rd_speed = 0;

/* ==========================================
 * STATE MACHINE SETTINGS
 * ========================================== */
enum AppState {
    STATE_NO_PERSON,
    STATE_ENTRY,
    STATE_WALKING,
    STATE_STANDING,
    STATE_STAND_TO_SIT,
    STATE_SITTING,
    STATE_SIT_TO_STAND,
    STATE_EXIT
};
AppState currentState = STATE_NO_PERSON;
unsigned long stateTimer = 0;

enum EnvelopeType { 
    ENV_ZERO, 
    ENV_WALKING, 
    ENV_POS_PULSE,  // Represents: 0 --> Positive --> 0 
    ENV_NEG_PULSE,  // Represents: 0 --> Negative --> 0
    ENV_UNCLEAR 
};

/* ==========================================
 * BLUETOOTH LE (BLE) SETTINGS
 * ========================================== */
#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"

BLEServer* pServer = NULL;
BLECharacteristic* pCharacteristic = NULL;
bool deviceConnected = false;
bool oldDeviceConnected = false;

class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
    };
    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
    }
};

/* ==========================================
 * OLED & ML Model Configuration
 * ========================================== */
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1

TwoWire I2C_OLED = TwoWire(0);
Adafruit_SH1106G display = Adafruit_SH1106G(SCREEN_WIDTH, SCREEN_HEIGHT, &I2C_OLED, OLED_RESET);

Eloquent::ML::Port::RandomForest model;

// Feature Configuration - UPDATED TO 41 FOR RD-03D ONLY
const int WINDOW_SIZE = 50;
const int NUM_FEATURES = 41; // 40 stat features (5 buffers * 8 stats) + 1 previous state

// Sliding Buffers for the 5 raw RD-03D signals
float rd_vel_buf[WINDOW_SIZE] = {0};
float rd_dist_buf[WINDOW_SIZE] = {0};
float rd_angle_buf[WINDOW_SIZE] = {0};
float rd_x_buf[WINDOW_SIZE] = {0};
float rd_y_buf[WINDOW_SIZE] = {0};

int buffer_index = 0;
bool buffer_full = false;

// Initialize with LabelEncoder's value for 'No_Person' (Typically 2)
int prev_activity_encoded = 2; 

// STANDARD SCALER PARAMETERS - REDUCED TO 41 FEATURES
const float scaler_means[41] = {
  1.958272, 1.981244, 9.880944, 264.965280, 16.203711, -14.930004, 18.885695, 33.815699, 1117.759415, 1109.139235, 211.744899, 102162.756106, 1155.204475, 801.085020, 1484.732258, 683.647238, 0.880707, 0.609746, 5.449460, 69.332677, 10.604903, -7.871649, 10.645795, 18.517444, 26.845616, 24.162111, 127.343850, 37952.618778, 251.127061, -192.965823, 257.664800, 450.630623, 1067.912005, 1058.160772, 205.577572, 97233.923656, 1105.649890, 761.508507, 1425.108125, 663.599618, 3.439374
};

const float scaler_scales[41] = {
  18.195207, 19.284282, 12.935696, 628.424623, 18.221035, 27.278614, 31.514079, 39.930954, 590.793098, 608.547205, 239.430269, 184483.594361, 603.361182, 577.267851, 843.271830, 811.081906, 10.567441, 11.166464, 6.295718, 125.197608, 8.241876, 13.266236, 16.668036, 21.340402, 287.834636, 297.836779, 147.431891, 69939.055353, 240.204559, 370.007952, 442.194125, 573.328499, 581.965482, 598.713410, 234.460627, 178187.059865, 593.251581, 562.092374, 830.223227, 790.272337, 2.281709
};

const char* ACTIVITY_LABELS[] = {
  "Entry", "Exit", "No_Person", "Sit_To_Stand", 
  "Sitting", "Stand_To_Sit", "Standing", "Walking"
};

/* ==========================================
 * MULTI-CORE IPC
 * ========================================== */
SemaphoreHandle_t dataMutex;

float shared_rd_speed = 0.0, shared_rd_distance = 0.0, shared_rd_angle = 0.0;
float shared_rd_x = 0.0, shared_rd_y = 0.0;
bool shared_rd_present = false;

String shared_currentActivity = "No_Person"; // Boot straight into No_Person
float shared_currentConfidence = 100.0; 

/* ==========================================
 * BACKTRACKING & LOGGING BUFFER
 * ========================================== */
const int HISTORY_SIZE = 150; // Increased to 15 seconds of history at 10Hz due to slower ML
struct LogEntry {
    unsigned long timestamp;
    char originalActivity[16];
    char correctedActivity[16];
    char triggerEvent[24]; // Stores what caused the backtrack
    float confidence;
    float rd_dist;
    float rd_vel;
    bool isCorrected;
};

LogEntry historyBuffer[HISTORY_SIZE];
int historyHead = 0;
int historyTail = 0;
int historyCount = 0;

/* ==========================================
 * PREDICTION SMOOTHING
 * FINE TUNING: Massive increase in smoothing window to drastically slow down 
 * decision making and eliminate false predictions.
 * ========================================== */
#define SMOOTHING_WINDOW 30 // 3.0 seconds at 10Hz to commit to a decision
String predictionHistory[SMOOTHING_WINDOW];
int historyIndex = 0;

String getSmoothedPrediction(String newPred, float &outConfidence, String currentStableState) {
    predictionHistory[historyIndex] = newPred;
    historyIndex = (historyIndex + 1) % SMOOTHING_WINDOW;

    String bestPred = currentStableState;
    int maxCount = 0;
    
    for(int i = 0; i < SMOOTHING_WINDOW; i++) {
        if(predictionHistory[i] == "" || predictionHistory[i] == "Loading...") continue;
        int count = 0;
        for(int j = 0; j < SMOOTHING_WINDOW; j++) {
            if(predictionHistory[i] == predictionHistory[j]) count++;
        }
        if(count > maxCount) {
            maxCount = count;
            bestPred = predictionHistory[i];
        }
    }
    
    outConfidence = ((float)maxCount / SMOOTHING_WINDOW) * 100.0;
    
    // HYSTERESIS GATE: Only allow state change if we are > 70% confident 
    // over the 3-second voting window.
    if (outConfidence >= 70.0) {
        return bestPred;
    }
    return currentStableState; // Otherwise, hold the last stable prediction
}

/* ==========================================
 * GLOBAL SENSOR VARIABLES
 * ========================================== */
const float SENSOR_MOUNT_HEIGHT_MM = 1000.0; // Configurable: Radar height from floor in mm (e.g., 1 meter)
const float SENSOR_TILT_ANGLE_DEG = 0.0;     // Configurable: Pitch angle of the sensor (0 = straight ahead)

struct RDTarget {
  bool present = false;
  int16_t x = 0;  int16_t y = 0;  int16_t speed = 0;
  uint16_t resolution = 0;
  float distance = 0; float angle = 0;
};
RDTarget rdTarget;

HardwareSerial RDSerial(1);
const uint8_t RD_HEADER[4] = {0xAA, 0xFF, 0x03, 0x00};
const uint8_t RD_FOOTER[2] = {0x55, 0xCC};
uint8_t rdBuf[30];
int rdPos = 0;
const uint8_t RD_SINGLE_TARGET_CMD[12] = {
  0xFD, 0xFC, 0xFB, 0xFA, 0x02, 0x00, 0x80, 0x00, 0x04, 0x03, 0x02, 0x01
};

void processAndLog(unsigned long timestamp, String activity, float conf, float rd_dist, float rd_vel);
void updateDisplay();
void readRD03D();

/* ==========================================
 * BACKTRACKING ALGORITHM LOGIC
 * ========================================== */
void applyBacktracking(const char* triggeringEvent, const char* correctPastState, const char* stopCondition1, const char* stopCondition2) {
    if (historyCount == 0) return;

    int current = (historyHead - 1 + HISTORY_SIZE) % HISTORY_SIZE;
    int count = 0;
    
    while (count < historyCount) {
        if (strcmp(historyBuffer[current].correctedActivity, stopCondition1) == 0 ||
            strcmp(historyBuffer[current].correctedActivity, stopCondition2) == 0) {
            break;
        }

        if (strcmp(historyBuffer[current].correctedActivity, correctPastState) != 0 && 
            strcmp(historyBuffer[current].correctedActivity, triggeringEvent) != 0 && 
            strcmp(historyBuffer[current].correctedActivity, "Loading...") != 0) {
            
            strncpy(historyBuffer[current].correctedActivity, correctPastState, 15);
            historyBuffer[current].correctedActivity[15] = '\0';
            
            strncpy(historyBuffer[current].triggerEvent, triggeringEvent, 23);
            historyBuffer[current].triggerEvent[23] = '\0';
            
            historyBuffer[current].isCorrected = true;
        }
        
        current = (current - 1 + HISTORY_SIZE) % HISTORY_SIZE;
        count++;
    }
}


/* ==========================================
 * FEATURE EXTRACTION
 * ========================================== */
void sortArray(float* arr, int n) {
    for (int i = 0; i < n - 1; i++) {
        for (int j = 0; j < n - i - 1; j++) {
            if (arr[j] > arr[j + 1]) {
                float temp = arr[j]; arr[j] = arr[j + 1]; arr[j + 1] = temp;
            }
        }
    }
}

void extract_stats(float* buffer, float* feature_array, int start_idx) {
    float sum = 0.0, sq_sum = 0.0;
    float min_val = buffer[0], max_val = buffer[0];
    float temp_buf[WINDOW_SIZE];

    for (int i = 0; i < WINDOW_SIZE; i++) {
        float val = buffer[i];
        temp_buf[i] = val;
        sum += val;
        sq_sum += (val * val);
        if (val < min_val) min_val = val;
        if (val > max_val) max_val = val;
    }

    float mean = sum / WINDOW_SIZE;
    float rms = sqrt(sq_sum / WINDOW_SIZE);
    float range_val = max_val - min_val;

    float var_sum = 0.0;
    for (int i = 0; i < WINDOW_SIZE; i++) {
        var_sum += (buffer[i] - mean) * (buffer[i] - mean);
    }
    float variance = var_sum / (WINDOW_SIZE - 1);
    if(WINDOW_SIZE == 1) variance = 0;
    float std_dev = sqrt(variance);

    sortArray(temp_buf, WINDOW_SIZE);
    float median = (WINDOW_SIZE % 2 == 0) ? (temp_buf[WINDOW_SIZE / 2 - 1] + temp_buf[WINDOW_SIZE / 2]) / 2.0 : temp_buf[WINDOW_SIZE / 2];

    feature_array[start_idx + 0] = mean;
    feature_array[start_idx + 1] = median;
    feature_array[start_idx + 2] = std_dev;
    feature_array[start_idx + 3] = variance;
    feature_array[start_idx + 4] = rms;
    feature_array[start_idx + 5] = min_val;
    feature_array[start_idx + 6] = max_val;
    feature_array[start_idx + 7] = range_val;
}

/* ==========================================
 * STRICT KINEMATIC ENVELOPE EVALUATOR
 * FINE TUNING: Increased MOTION_THRESH and decreased ZERO_THRESH to
 * create a wider dead-band, ignoring fidgeting and micro-movements.
 * ========================================== */
EnvelopeType evaluateEnvelope() {
    if (!buffer_full) return ENV_UNCLEAR;

    int pos_frames = 0;
    int neg_frames = 0;
    int recent_zeros = 0;

    const float MOTION_THRESH = 8.0; // Was 6.0
    const float ZERO_THRESH = 2.5;   // Was 3.5 
    const int LOOKBACK = 35;         // Increased lookback
    const int SETTLE_FRAMES = 15;    // Require longer settle time

    for (int i = WINDOW_SIZE - SETTLE_FRAMES; i < WINDOW_SIZE; i++) {
        if (abs(rd_vel_buf[i]) <= ZERO_THRESH) recent_zeros++;
    }
    bool settled = (recent_zeros >= (SETTLE_FRAMES - 3)); 

    for (int i = WINDOW_SIZE - LOOKBACK; i < WINDOW_SIZE; i++) {
        float v = rd_vel_buf[i];
        if (v > MOTION_THRESH) pos_frames++;
        else if (v < -MOTION_THRESH) neg_frames++;
    }

    if (pos_frames >= 5 && neg_frames >= 5) return ENV_WALKING;
    if (pos_frames >= 5 && neg_frames <= 1 && settled) return ENV_POS_PULSE;
    if (neg_frames >= 5 && pos_frames <= 1 && settled) return ENV_NEG_PULSE;
    if (pos_frames <= 1 && neg_frames <= 1 && settled) return ENV_ZERO;

    return ENV_UNCLEAR;
}

/* ==========================================
 * CORE 0: ML & FSM TASK
 * ========================================== */
void mlTask(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(INFERENCE_INTERVAL_MS);

    int presence_counter = 0;
    int continuous_ghost_frames = 0; 
    bool isPresent = false;

    static float ref_stand_y = 0;
    static float ref_stand_angle = 0;
    static bool has_stand_ref = false;

    static AppState lastTrackedState = STATE_NO_PERSON;
    static String stableMLState = "No_Person";

    for (;;) {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
        unsigned long now = millis();

        float rd_s, rd_d, rd_a, rd_x, rd_y; bool rd_p;

        if (xSemaphoreTake(dataMutex, portMAX_DELAY)) {
            rd_s = shared_rd_speed; 
            rd_d = shared_rd_distance;
            rd_a = shared_rd_angle;
            rd_x = shared_rd_x;
            rd_y = shared_rd_y;
            rd_p = shared_rd_present;
            xSemaphoreGive(dataMutex);
        }

        // 1. UPDATE BUFFERS
        if (!buffer_full) {
            rd_vel_buf[buffer_index]    = rd_s;
            rd_dist_buf[buffer_index]   = rd_d;
            rd_angle_buf[buffer_index]  = rd_a;
            rd_x_buf[buffer_index]      = rd_x;
            rd_y_buf[buffer_index]      = rd_y;

            buffer_index++;
            if (buffer_index >= WINDOW_SIZE) buffer_full = true;
        } else {
            for (int i = 0; i < WINDOW_SIZE - 1; i++) {
                rd_vel_buf[i]    = rd_vel_buf[i + 1];
                rd_dist_buf[i]   = rd_dist_buf[i + 1];
                rd_angle_buf[i]  = rd_angle_buf[i + 1];
                rd_x_buf[i]      = rd_x_buf[i + 1];
                rd_y_buf[i]      = rd_y_buf[i + 1];
            }
            rd_vel_buf[WINDOW_SIZE - 1]    = rd_s;
            rd_dist_buf[WINDOW_SIZE - 1]   = rd_d;
            rd_angle_buf[WINDOW_SIZE - 1]  = rd_a;
            rd_x_buf[WINDOW_SIZE - 1]      = rd_x;
            rd_y_buf[WINDOW_SIZE - 1]      = rd_y;
        }

        // 2. GHOST REJECTION
        bool rd_valid = rd_p && (rd_d > 100.0 && rd_d < 5000.0);
        if (rd_p && !rd_valid) {
            continuous_ghost_frames++;
            if (continuous_ghost_frames >= 20) {
                Serial.println("GHOST MODE DETECTED: Restarting System...");
                delay(100); ESP.restart(); 
            }
        } else {
            continuous_ghost_frames = 0;
        }

        if (rd_valid) {
            presence_counter++;
            if (presence_counter > 20) presence_counter = 20; 
        } else {
            presence_counter -= 2; 
            if (presence_counter < 0) presence_counter = 0;
        }

        if (presence_counter >= 12) isPresent = true; // Stricter presence confirmation
        else if (presence_counter <= 3) isPresent = false;

        // 3. FEATURE EXTRACTION & ML
        String rawML = "No_Person";
        String smoothedML = "No_Person";
        float localConf = 100.0;
        
        float mean_angle = 0, var_angle = 0;
        float mean_x = 0, var_x = 0;
        float mean_y = 0, var_y = 0;
        float estimated_z = 0; 

        if (buffer_full && isPresent) {
            float raw_features[NUM_FEATURES]; 
            extract_stats(rd_vel_buf,    raw_features, 0);
            extract_stats(rd_dist_buf,   raw_features, 8);
            extract_stats(rd_angle_buf,  raw_features, 16);
            extract_stats(rd_x_buf,      raw_features, 24);
            extract_stats(rd_y_buf,      raw_features, 32);
            raw_features[40] = (float)prev_activity_encoded;

            mean_angle = raw_features[16]; var_angle = raw_features[19];
            mean_x     = raw_features[24]; var_x     = raw_features[27];
            mean_y     = raw_features[32]; var_y     = raw_features[35];

            float z_delta = mean_y * sin((mean_angle + SENSOR_TILT_ANGLE_DEG) * PI / 180.0);
            estimated_z = SENSOR_MOUNT_HEIGHT_MM + z_delta;

            float scaled_features[NUM_FEATURES];
            for (int i = 0; i < NUM_FEATURES; i++) {
                scaled_features[i] = (raw_features[i] - scaler_means[i]) / scaler_scales[i];
            }

            int current_activity_encoded = model.predict(scaled_features);
            rawML = ACTIVITY_LABELS[current_activity_encoded];
            prev_activity_encoded = current_activity_encoded;

            // USE THE GATED SMOOTHER
            smoothedML = getSmoothedPrediction(rawML, localConf, stableMLState);
            stableMLState = smoothedML; // Update our firm ground truth
        }

        // ==========================================
        // EXPLICIT FINITE STATE MACHINE (FSM)
        // FINE TUNING: Increased transition delay timers
        // ==========================================
        unsigned long elapsed = now - stateTimer;
        EnvelopeType env = evaluateEnvelope();

        bool is_spatially_stable = false;
        if (buffer_full) {
            // Stricter Variance requirements to declare spatial stability
            is_spatially_stable = (var_y < 250.0 && var_x < 250.0 && var_angle < 8.0);
        }

        if (!isPresent) {
            if (currentState != STATE_NO_PERSON && currentState != STATE_EXIT) {
                currentState = STATE_EXIT;
                stateTimer = now;
            } else if (currentState == STATE_EXIT && elapsed > 2500) { // Was 1500
                currentState = STATE_NO_PERSON;
                has_stand_ref = false; 
                for(int i=0; i < SMOOTHING_WINDOW; i++) predictionHistory[i] = "No_Person";
                stableMLState = "No_Person";
            }
        } else {
            if (currentState == STATE_STANDING && env == ENV_ZERO && is_spatially_stable && smoothedML == "Standing") {
                if (!has_stand_ref) {
                    ref_stand_y = mean_y; ref_stand_angle = mean_angle;
                    has_stand_ref = true;
                } else {
                    ref_stand_y = 0.98 * ref_stand_y + 0.02 * mean_y; // Slowed down reference adaptation
                    ref_stand_angle = 0.98 * ref_stand_angle + 0.02 * mean_angle;
                }
            }

            if (currentState == STATE_NO_PERSON || currentState == STATE_EXIT) {
                currentState = STATE_ENTRY;
                stateTimer = now;
            }
            else if (currentState == STATE_ENTRY) {
                if (elapsed > 2500) { // Was 1500
                    if (buffer_full) {
                        if (env == ENV_WALKING || smoothedML == "Walking") {
                            currentState = STATE_WALKING;
                        } else {
                            bool is_physically_sitting = (estimated_z < 850.0);
                            if (smoothedML == "Sitting" || is_physically_sitting) {
                                currentState = STATE_SITTING;
                            } else {
                                currentState = STATE_STANDING; 
                                ref_stand_y = mean_y;
                                ref_stand_angle = mean_angle;
                                has_stand_ref = true;
                            }
                        }
                    } else {
                        currentState = STATE_STANDING; 
                    }
                    stateTimer = now;
                }
            }
            else if (currentState == STATE_WALKING || currentState == STATE_STANDING) {
                if (env == ENV_WALKING) {
                    currentState = STATE_WALKING;
                    stateTimer = now; 
                } 
                else if (currentState == STATE_WALKING) {
                    if (abs(rd_s) >= 4.0) stateTimer = now; 
                    else if (env == ENV_ZERO || elapsed > 3500) { // Wait 3.5s of stop to confirm Standing
                        currentState = STATE_STANDING;
                        stateTimer = now;
                    }
                }

                if (env == ENV_POS_PULSE) {
                    currentState = STATE_STAND_TO_SIT;
                    stateTimer = now;
                }
                else if (currentState == STATE_STANDING && env == ENV_ZERO && is_spatially_stable && elapsed > 3000) {
                    if (has_stand_ref) {
                        float delta_y = abs(mean_y - ref_stand_y);
                        float delta_angle = abs(mean_angle - ref_stand_angle);
                        if ((delta_y > 250.0 || delta_angle > 12.0) && smoothedML == "Sitting") {
                            currentState = STATE_SITTING; 
                            stateTimer = now;
                        }
                    } else {
                        // High confidence requirement to bypass ref check
                        if (smoothedML == "Sitting" && localConf > 85.0) {
                            currentState = STATE_SITTING; 
                            stateTimer = now;
                        }
                    }
                }
            }
            else if (currentState == STATE_STAND_TO_SIT) {
                if (elapsed > 3000) { // Extended to 3s to ensure they have sat completely
                    currentState = STATE_SITTING;
                    stateTimer = now;
                }
            }
            else if (currentState == STATE_SITTING) {
                // If a negative pulse (standing) is seen, OR if walking envelope begins while sitting
                if (env == ENV_NEG_PULSE || (env == ENV_WALKING && abs(rd_s) > 6.0)) {
                    currentState = STATE_SIT_TO_STAND;
                    stateTimer = now;
                } 
                else if (env == ENV_ZERO && is_spatially_stable && elapsed > 3000) {
                    if (has_stand_ref) {
                        float delta_y_to_stand = abs(mean_y - ref_stand_y);
                        float delta_angle_to_stand = abs(mean_angle - ref_stand_angle);
                        if (delta_y_to_stand < 150.0 && delta_angle_to_stand < 8.0 && smoothedML == "Standing") {
                            currentState = STATE_STANDING;
                            stateTimer = now;
                        }
                    } else {
                        // High confidence requirement
                        if (smoothedML == "Standing" && localConf > 85.0) {
                            currentState = STATE_STANDING; 
                            stateTimer = now;
                        }
                    }
                }
            }
            else if (currentState == STATE_SIT_TO_STAND) {
                if (elapsed > 1500) { // Hold for 1.5s to explicitly log the transition
                    if (env == ENV_WALKING || smoothedML == "Walking") {
                        currentState = STATE_WALKING;
                    } else {
                        currentState = STATE_STANDING;
                    }
                    stateTimer = now;
                }
            }
        }

        // ==========================================
        // APPLY BACKTRACKING ON STATE TRANSITIONS
        // ==========================================
        if (currentState != lastTrackedState) {
            if (currentState == STATE_SIT_TO_STAND) {
                applyBacktracking("Sit_To_Stand", "Sitting", "Stand_To_Sit", "Walking");
            } 
            else if (currentState == STATE_STAND_TO_SIT) {
                applyBacktracking("Stand_To_Sit", "Standing", "Sit_To_Stand", "Walking");
            }
            else if (currentState == STATE_SITTING && lastTrackedState == STATE_STANDING) {
                applyBacktracking("Auto_Correct_to_Sit", "Sitting", "Stand_To_Sit", "Walking");
            }
            else if (currentState == STATE_STANDING && lastTrackedState == STATE_SITTING) {
                applyBacktracking("Auto_Correct_to_Stand", "Standing", "Sit_To_Stand", "Walking");
            }
            lastTrackedState = currentState;
        }

        // ==========================================
        // MAP FSM TO DISPLAY STRING
        // ==========================================
        String displayActivity;
        switch (currentState) {
            case STATE_NO_PERSON: displayActivity = "No_Person"; break;
            case STATE_ENTRY: displayActivity = "Entry"; break;
            case STATE_WALKING: displayActivity = "Walking"; break;
            case STATE_STANDING: displayActivity = (buffer_full) ? "Standing" : "Loading..."; break;
            case STATE_STAND_TO_SIT: displayActivity = "Stand_To_Sit"; break;
            case STATE_SITTING: displayActivity = "Sitting"; break;
            case STATE_SIT_TO_STAND: displayActivity = "Sit_To_Stand"; break;
            case STATE_EXIT: displayActivity = "Exit"; break;
        }

        if (currentState == STATE_NO_PERSON || currentState == STATE_ENTRY || currentState == STATE_EXIT || !buffer_full) {
            localConf = 100.0;
        }

        if (xSemaphoreTake(dataMutex, portMAX_DELAY)) {
            shared_currentActivity = displayActivity;
            shared_currentConfidence = localConf;
            xSemaphoreGive(dataMutex);
        }

        // Log everything into the unified delayed buffer log
        processAndLog(now, displayActivity, localConf, rd_d, rd_s);
        
        Serial.print("Activity: "); Serial.print(displayActivity);
        Serial.print(" | FSM: "); Serial.print((int)currentState);
        Serial.print(" | Z: "); Serial.print(estimated_z, 0); 
        Serial.print(" | RD: "); Serial.print(rd_d); Serial.print("mm, "); Serial.println(rd_s);
    }
}

/* ==========================================
 * SETUP
 * ========================================== */
void setupBLE() {
  BLEDevice::init("ESP32_ML_Sensor");
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  BLEService *pService = pServer->createService(SERVICE_UUID);

  pCharacteristic = pService->createCharacteristic(
                        CHARACTERISTIC_UUID,
                        BLECharacteristic::PROPERTY_READ   |
                        BLECharacteristic::PROPERTY_NOTIFY
                      );

  pCharacteristic->addDescriptor(new BLE2902());
  pService->start();

  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  
  pAdvertising->setScanResponse(true); 
  pAdvertising->setMinPreferred(0x06);
  
  BLEDevice::startAdvertising();
}

void setup() {
  Serial.begin(115200);
  unsigned long startWait = millis();
  while (millis() - startWait < 1000) { }
  
  Serial.println("Initializing ESP32-S3 Pipeline (High Stability Mode)...");
  dataMutex = xSemaphoreCreateMutex();

  I2C_OLED.begin(OLED_SDA, OLED_SCL);
  I2C_OLED.setClock(400000);

  RDSerial.setRxBufferSize(256);
  RDSerial.begin(RD03D_BAUD, SERIAL_8N1, RD03D_RX_PIN, RD03D_TX_PIN);
  
  unsigned long startRD = millis();
  while (millis() - startRD < 50) { }
  RDSerial.write(RD_SINGLE_TARGET_CMD, sizeof(RD_SINGLE_TARGET_CMD));

  if (!display.begin(OLED_ADDR, true)) {
    Serial.println("SH1106 OLED allocation failed!");
  } else {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SH110X_WHITE);
    display.setCursor(0, 10);
    display.println("  AI Inference  ");
    display.println("  Initializing... ");
    display.display();
  }

  setupBLE();

  // Initialize the Unified Log with Header
  SPI.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS);
  if (!SD.begin(SD_CS)) {
    Serial.println("SD Init failed!");
  } else {
    if (!SD.exists("/HAR_Log.csv")) {
        File f = SD.open("/HAR_Log.csv", FILE_WRITE);
        if(f) { 
            f.println("Timestamp_ms,Instant_Prediction,Final_Prediction,Was_Corrected,Trigger_Event,Confidence,RD_Distance,RD_Velocity"); 
            f.close(); 
        }
    }
  }

  xTaskCreatePinnedToCore(mlTask, "ML_Task", 16384, NULL, 1, NULL, 0);
  Serial.println("Setup Complete.");
}

/* ==========================================
 * MAIN LOOP
 * ========================================== */
void loop() {
  unsigned long now = millis();

  readRD03D();

  if (rdTarget.present) {
      smooth_rd_dist = (EMA_ALPHA_DIST * rdTarget.distance) + ((1.0 - EMA_ALPHA_DIST) * smooth_rd_dist);
      smooth_rd_speed = (EMA_ALPHA_VEL * (float)rdTarget.speed) + ((1.0 - EMA_ALPHA_VEL) * smooth_rd_speed);
  } else {
      smooth_rd_dist = (EMA_ALPHA_DIST * 0) + ((1.0 - EMA_ALPHA_DIST) * smooth_rd_dist);
      smooth_rd_speed = (EMA_ALPHA_VEL * 0) + ((1.0 - EMA_ALPHA_VEL) * smooth_rd_speed);
  }

  if (xSemaphoreTake(dataMutex, portMAX_DELAY)) {
      shared_rd_speed = smooth_rd_speed; 
      shared_rd_distance = smooth_rd_dist; 
      shared_rd_angle = rdTarget.angle;
      shared_rd_x = (float)rdTarget.x;
      shared_rd_y = (float)rdTarget.y;
      shared_rd_present = rdTarget.present;
      xSemaphoreGive(dataMutex);
  }

  if (now - lastDisplay >= OLED_INTERVAL_MS) {
    lastDisplay = now;
    updateDisplay();
  }

  if (!deviceConnected && oldDeviceConnected) {
      delay(500); 
      pServer->startAdvertising(); 
      oldDeviceConnected = deviceConnected;
  }
  if (deviceConnected && !oldDeviceConnected) {
      oldDeviceConnected = deviceConnected;
  }

  if (deviceConnected && (now - lastBLEUpdate >= BLE_INTERVAL_MS)) {
      lastBLEUpdate = now;
      String act; float conf;
      float rd_d, rd_s;
      
      if (xSemaphoreTake(dataMutex, portMAX_DELAY)) {
          act = shared_currentActivity;
          conf = shared_currentConfidence;
          rd_d = shared_rd_distance;
          rd_s = shared_rd_speed;
          xSemaphoreGive(dataMutex);
      }
      
      String blePayload = "{\"activity\":\"" + act + "\", \"confidence\":" + String(conf, 1) + 
                          ", \"rd_d\":" + String(rd_d, 0) + ", \"rd_s\":" + String(rd_s, 2) + "}";
      
      int payloadLen = blePayload.length();
      int offset = 0;
      while (offset < payloadLen) {
          int chunkSize = (payloadLen - offset > 20) ? 20 : (payloadLen - offset);
          String chunk = blePayload.substring(offset, offset + chunkSize);
          pCharacteristic->setValue(chunk.c_str());
          pCharacteristic->notify();
          offset += chunkSize;
          delay(10); 
      }
  }

  vTaskDelay(1); 
}

/* ==========================================
 * SENSOR PARSING FUNCTIONS
 * ========================================== */
int16_t decodeRD03DValue(uint8_t lowByte, uint8_t highByte) {
  int16_t value = ((highByte & 0x7F) << 8) | lowByte;
  if ((highByte & 0x80) == 0) value = -value;
  return value;
}

void processRD03DFrame() {
  int16_t x = decodeRD03DValue(rdBuf[4], rdBuf[5]);
  int16_t y = decodeRD03DValue(rdBuf[6], rdBuf[7]);
  int16_t new_speed = decodeRD03DValue(rdBuf[8], rdBuf[9]);
  uint16_t res = ((uint16_t)rdBuf[11] << 8) | rdBuf[10];

  if (rdTarget.present && abs(new_speed - rdTarget.speed) > 200) {
      return; 
  }

  rdTarget.present = (x != 0 || y != 0 || res != 0);
  rdTarget.x = x;
  rdTarget.y = y;
  rdTarget.speed = new_speed;
  rdTarget.resolution = res;
  rdTarget.distance = sqrt((float)x * x + (float)y * y);
  rdTarget.angle = atan2((float)x, (float)y == 0 ? 0.001 : (float)y) * 180.0 / PI; 
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

/* ==========================================
 * SD LOGGING ROUTINES (Unified output)
 * ========================================== */
void processAndLog(unsigned long timestamp, String activity, float conf, float rd_dist, float rd_vel) {
    
    // 1. Manage the History Buffer for Final Unified Output
    if (historyCount == HISTORY_SIZE) {
        LogEntry oldest = historyBuffer[historyTail];
        File unifiedLog = SD.open("/HAR_Log.csv", FILE_APPEND);
        if (unifiedLog) {
            unifiedLog.print(oldest.timestamp); unifiedLog.print(",");
            unifiedLog.print(oldest.originalActivity); unifiedLog.print(",");
            unifiedLog.print(oldest.correctedActivity); unifiedLog.print(",");
            unifiedLog.print(oldest.isCorrected ? "YES" : "NO"); unifiedLog.print(",");
            unifiedLog.print(oldest.isCorrected ? oldest.triggerEvent : "None"); unifiedLog.print(",");
            unifiedLog.print(oldest.confidence); unifiedLog.print(",");
            unifiedLog.print(oldest.rd_dist); unifiedLog.print(",");
            unifiedLog.println(oldest.rd_vel);
            unifiedLog.close();
        }
        
        historyTail = (historyTail + 1) % HISTORY_SIZE;
        historyCount--;
    }

    // 2. Push new entry to head
    historyBuffer[historyHead].timestamp = timestamp;
    
    strncpy(historyBuffer[historyHead].originalActivity, activity.c_str(), 15);
    historyBuffer[historyHead].originalActivity[15] = '\0';
    
    strncpy(historyBuffer[historyHead].correctedActivity, activity.c_str(), 15);
    historyBuffer[historyHead].correctedActivity[15] = '\0';

    historyBuffer[historyHead].triggerEvent[0] = '\0';
    
    historyBuffer[historyHead].confidence = conf;
    historyBuffer[historyHead].rd_dist = rd_dist;
    historyBuffer[historyHead].rd_vel = rd_vel;
    historyBuffer[historyHead].isCorrected = false;

    historyHead = (historyHead + 1) % HISTORY_SIZE;
    historyCount++;
}


/* ==========================================
 * OLED DISPLAY
 * ========================================== */
void updateDisplay() {
  String act; float conf;
  if (xSemaphoreTake(dataMutex, portMAX_DELAY)) {
      act = shared_currentActivity;
      conf = shared_currentConfidence;
      xSemaphoreGive(dataMutex);
  }

  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.print("ML: Random Forest");
  display.drawLine(0, 10, SCREEN_WIDTH, 10, SH110X_WHITE);
  
  display.setCursor(0, 14); display.print("Act:");
  display.setCursor(64, 14); 
  display.print("Conf: "); display.print((int)conf); display.print("%");

  display.setCursor(0, 30);
  display.setTextSize(2); 
  
  if (act == "Sit_To_Stand") { display.print("Sit->Stand"); }
  else if (act == "Stand_To_Sit") { display.print("Stand->Sit"); }
  else if (act == "No_Person") { display.print("No Person"); } 
  else if (act == "Loading...") { display.setTextSize(1); display.print("Loading Buffer..."); } 
  else { display.print(act); }

  display.setTextSize(1);
  display.setCursor(0, 52);
  display.print("RD: "); 
  display.print(rdTarget.distance, 0); display.print("mm ");
  display.print((float)rdTarget.speed, 1); display.print("c/s");
  
  display.display();
}