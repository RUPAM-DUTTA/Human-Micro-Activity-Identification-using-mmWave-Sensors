Human Micro Activity Identification Using mmWave Sensors

📌 Project Overview

This project focuses on identifying human micro-activities and transitional movements using advanced mmWave (millimeter-wave) radar sensors. Unlike traditional optical cameras, mmWave sensors provide a non-intrusive, privacy-preserving method for continuous activity monitoring.

By leveraging a dual-sensor approach (Ai-Thinker Rd-03d and DFRobot C4001) driven by an ESP32-S3 Pico, the system captures fine-grained spatial and velocity data. This repository includes the complete pipeline: from an automated data collection rig that logs to an SD card, to a real-time on-device Machine Learning deployment recognizing 8 distinct activities.

🎯 Recognized Activities

The on-device Random Forest model is trained to classify the following 8 states in real-time (at 10 Hz):

NO PERSON (Empty room)

ENTRY

EXIT

STANDING

SITTING

WALKING

STAND TO SIT (Transitional)

SIT TO STAND (Transitional)

🛠️ Hardware Setup & Pin Mapping

The system is built on the ESP32-S3 Pico to handle dual I2C buses, hardware serial, and SPI concurrently.

Component

Interface

ESP32-S3 Pin

Notes

DFRobot C4001

I2C (Bus 0)

SDA: 6, SCL: 7

Configured for eSpeedMode

OLED (SH1106)

I2C (Bus 1)

SDA: 8, SCL: 9

Address 0x3C, displays system state

Ai-Thinker Rd-03d

UART1

RX: 4, TX: 5

Baud 256000, single-target mode

MicroSD Module

SPI

CS: 10, SCK: 12, MISO: 13, MOSI: 11

For raw CSV data logging

Power: The system runs independently on a rechargeable battery with an integrated power switch.

🏗️ Software Architecture

The project is split into two main phases, each with its own dedicated firmware:

Phase 1: Automated Data Collection (datacollection.ino)

Designed to gather high-quality training data without manual intervention.

NTP Time Sync: Connects to WiFi (RUPAMPC) to pull accurate GMT+5:30 timestamps.

Automated State Machine: * Prep Phase: 10-second wait to allow the subject to get into posture.

Collect Phase: 30-second continuous data collection at 20Hz (50ms interval).

Logging: Writes comprehensive CSV files (Projectmmwave_log.csv) directly to the SD card including distance, velocity, angle, spatial coordinates, and energy readings from both sensors.

Phase 2: Real-Time Inference (Rupam1632mmdeploy.ino)

Deploys the trained Random Forest classifier directly onto the ESP32-S3.

Signal Processing: Uses an Exponential Moving Average (EMA, $\alpha=0.3$) to smooth live raw features and reduce sensor noise.

Inference Engine: Utilizes the EloquentTinyML framework (RUPAM_model.h) to predict activities from 8 features at 10Hz.

Latching Logic: Includes custom post-processing to confirm transitional states (e.g., verifying negative velocity for "STAND TO SIT") before updating the OLED.

🚀 Getting Started

Prerequisites

Arduino IDE with the ESP32 board package installed.

Required Arduino Libraries (Install via Library Manager):

Adafruit GFX Library

Adafruit_SH110X

DFRobot_C4001 (Ensure this is in your src or libraries folder)

SD and SPI (Built-in)

Wire and HardwareSerial (Built-in)

Installation & Deployment

Clone this repository:

git clone [https://github.com/yourusername/MicroActivity-mmWave.git](https://github.com/yourusername/MicroActivity-mmWave.git)


For Data Collection:

Open datacollection.ino.

Update the WIFI_SSID and WIFI_PASS variables.

Upload to the ESP32-S3 Pico, insert a formatted MicroSD card, and open the Serial Monitor (115200 baud).

For ML Inference:

Open Rupam1632mmdeploy.ino.

Ensure RUPAM_model.h is in the same directory.

Upload to the ESP32-S3 Pico. The OLED will immediately begin displaying real-time predictions.

📊 Data Features for Machine Learning

The model expects an 8-feature array representing a fused state of both sensors:

rd_distance_mm (Rd-03d)

rd_velocity_cm_s (Rd-03d)

rd_angle_deg (Rd-03d)

rd_x_mm (Rd-03d)

rd_y_mm (Rd-03d)

c4001_distance_m (C4001)

c4001_velocity_m_s (C4001)

c4001_energy (C4001)

📝 License

This project is open-source. Feel free to fork, modify, and integrate into your own IoT / Smart Home projects!
