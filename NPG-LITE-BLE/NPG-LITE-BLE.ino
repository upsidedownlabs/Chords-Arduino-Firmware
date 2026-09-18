/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program. If not, see <https://www.gnu.org/licenses/>.

   BLE connectivity adapted from the ESP32 BLE Server example by Random Nerd Tutorials:
   https://randomnerdtutorials.com/esp32-bluetooth-low-energy-ble-arduino-ide/.

   Copyright (c) 2025 Krishnanshu Mittal - krishnanshu@upsidedownlabs.tech
   Copyright (c) 2025 Deepak Khatri - deepak@upsidedownlabs.tech
   Copyright (c) 2025 Upside Down Labs - contact@upsidedownlabs.tech

   At Upside Down Labs, we create open‐source DIY neuroscience hardware and software.
   Our mission is to make neuroscience affordable and accessible for everyone.
   By supporting us with your purchase, you help spread innovation and open science.
   Thank you for being part of this journey with us!
*/

#include <Arduino.h>
#include <math.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLE2902.h>
#include <BLEUtils.h>
#include <Adafruit_NeoPixel.h>
#include <sdkconfig.h>
#include "hal/efuse_hal.h"
#include "esp_idf_version.h"
#include "esp_mac.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

// ADC includes
#include "esp_bt.h"                 // release Classic BT memory
#include "esp_adc/adc_continuous.h" // ADC continuous (DMA) driver
#include "hal/adc_types.h"          // adc_atten_t, bit width, etc.
#include "soc/soc_caps.h"           // SOC_ADC_DIGI_RESULT_BYTES

// Supported Playmates
#define PROTO_PLAYMATE 0      // Proto (3 BioAmp channels, no buzzer or vibration motor)
#define VIBZ_PLAYMATE 1       // Vibz (3 BioAmp channels, buzzer and vibration motor)
#define VIBZ_PLUS_PLAYMATE 2  // Vibz Plus (6 BioAmp channels, buzzer and vibration motor)

// ----- Chip-specific Pin Definitions -----
// Use the ESP-IDF config macros to detect the chip.
#if defined(CONFIG_IDF_TARGET_ESP32C6)
// Store chip revision number (for optional raw fixup if needed)
uint32_t chiprev = efuse_hal_chip_revision();
#define GAIN_PIN 14
#define LED_BUILTIN 7
#define BUZZER_PIN 8
#define PIXEL_PIN 15
#define PIXEL_COUNT 6
#elif defined(CONFIG_IDF_TARGET_ESP32C3)
#define LED_BUILTIN 6
#define BUZZER_PIN 8
#define PIXEL_PIN 3
#define PIXEL_COUNT 4
#else
#error "Unsupported board: Please target either ESP32-C6 or ESP32-C3 in your Board Manager."
#endif

#define NUM_CHANNELS_MAX 7        // Max channels supported (Vibz Plus Playmate)
#define BLE_PAYLOAD_BUFFERS 2     // Number of buffers for BLE payloads to prevent overflow (Change if you need more buffers)
#define PIXEL_BRIGHTNESS 7        // Brightness of Neopixel LED
#define BLOCK_COUNT 10            // Batch size: 10 samples per notification
#define SAMP_RATE 500.0           // Sampling rate per channel (500 Hz)
#define BATTERY_PIN A6            // Battery voltage pin
#define BOOT_MIN_BATTERY 10.0     // Minimum battery percentage to boot
#define STREAMING_MIN_BATTERY 5.0 // Minimum battery percentage to start streaming

// Global variables for Channel count and packet size
static uint8_t NUM_CHANNELS = 4;      // Number of BioAmp channels + 1 channel for battery
static uint8_t SINGLE_SAMPLE_LEN = 0; // Each sample: (No. of bioAmp channels * 2 bytes) + 1 counter
static uint16_t NEW_PACKET_LEN = 0;   // Packet length (BLOCK_COUNT * SINGLE_SAMPLE_LEN)
static uint8_t Playmate = PROTO_PLAYMATE; // PROTO_PLAYMATE, VIBZ_PLAYMATE, or VIBZ_PLUS_PLAYMATE

// Recompute packet sizes to adjust for channel count changes
static inline void recomputePacketSizes()
{
  SINGLE_SAMPLE_LEN = (uint8_t)(2 * (NUM_CHANNELS - 1) + 1);
  NEW_PACKET_LEN = (uint16_t)(BLOCK_COUNT * SINGLE_SAMPLE_LEN);
}

// Onboard Neopixel at PIXEL_PIN
Adafruit_NeoPixel pixels(PIXEL_COUNT, PIXEL_PIN, NEO_GRB + NEO_KHZ800);

// Battery monitoring variables
static unsigned long lastBatteryCheck = 0;
static const unsigned long BATTERY_CHECK_INTERVAL = 10000; // Interval in milliseconds
static BLEServer *pBLEServer = nullptr;                    // Store server reference for disconnect

// LUT for 1S LiPo (Voltage in ascending order)
const float voltageLUT[] = {
    3.27, 3.61, 3.69, 3.71, 3.73, 3.75, 3.77, 3.79, 3.80, 3.82,
    3.84, 3.85, 3.87, 3.91, 3.95, 3.98, 4.02, 4.08, 4.11, 4.15, 4.20};

const int percentLUT[] = {
    0, 5, 10, 15, 20, 25, 30, 35, 40, 45,
    50, 55, 60, 65, 70, 75, 80, 85, 90, 95, 100};

const int lutSize = sizeof(voltageLUT) / sizeof(voltageLUT[0]);

// Linear interpolation function
float interpolatePercentage(float voltage)
{
  // Handle out-of-range voltages
  if (voltage <= voltageLUT[0])
    return 0;
  if (voltage >= voltageLUT[lutSize - 1])
    return 100;

  // Find the nearest LUT entries
  int i = 0;
  while (i < lutSize - 1 && voltage > voltageLUT[i + 1])
    i++;

  // Interpolate
  float v1 = voltageLUT[i], v2 = voltageLUT[i + 1];
  int p1 = percentLUT[i], p2 = percentLUT[i + 1];
  return p1 + (voltage - v1) * (p2 - p1) / (v2 - v1);
}

// BLE UUIDs
#define SERVICE_UUID "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define DATA_CHAR_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"    // For ADC data (Notify only)
#define CONTROL_CHAR_UUID "0000ff01-0000-1000-8000-00805f9b34fb" // For commands (Read/Write/Notify)
#define BATTERY_CHAR_UUID "f633d0ec-46b4-43c1-a39f-1ca06d0602e1" // For battery status (Notify only)

// ----- Global Variables -----
uint8_t blePayload[BLE_PAYLOAD_BUFFERS][BLOCK_COUNT * (2 * (NUM_CHANNELS_MAX - 1) + 1)] = {0};
static uint8_t payload_wr = 0;   // buffer currently being filled
static uint8_t payload_rd = 0;   // next full buffer to notify
static uint8_t payload_full = 0; // number of full buffers ready
static uint8_t sampleIndex = 0;  // How many samples accumulated in current batch
volatile bool streaming = false; // True when "START" command is received
uint8_t mac[6];                  // Array to store 6-byte MAC address

// Flags to start/stop adc_continuous_mode
static volatile bool adc_start_requested = false;
static volatile bool adc_stop_requested = false;

BLECharacteristic *pDataCharacteristic;
BLECharacteristic *pControlCharacteristic;
BLECharacteristic *pBatteryCharacteristic;

// Global sample counter (each sample's packet counter)
uint8_t overallCounter = 0;

// Battery monitoring - stores latest ADC reading from A6
static volatile uint16_t latestBatteryRaw = 0;

// Battery averaging: log one battery ADC value per completed data packet
// Average all collected samples once per battery check interval
static uint32_t batteryWinStartMs = 0;
static uint32_t batteryWinSum = 0;
static uint16_t batteryWinCount = 0;
static uint16_t batteryAvgToSend = 0; // 0 when not ready yet
static uint16_t isCharging = 0;       // 0 when not charging
static uint8_t lastBatteryPct = 255;  // 255 is unset
static uint8_t consecutiveChargingCheck = 3;
static TaskHandle_t ledBlinkTask = NULL;
static volatile int ledBlinkCycles = -1; // -1=off, 0=indefinite, >0 = that many cycles

// Sample assembly state (reset on start/stop/disconnect)
static uint16_t last_vals[NUM_CHANNELS_MAX] = {0};
static uint32_t have_mask = 0;
static inline void resetSampleState()
{
  have_mask = 0;
  for (uint8_t i = 0; i < NUM_CHANNELS_MAX; i++)
  {
    last_vals[i] = 0;
  }
}

// ----- ADC DMA (continuous mode) globals -----
static adc_continuous_handle_t adc_handle = nullptr;
static bool adc_started = false;
static SemaphoreHandle_t adc_data_semaphore = nullptr;
// Helper macros to parse DMA results as TYPE2 format on C3/C6
#define ADC_OUTPUT_TYPE ADC_DIGI_OUTPUT_FORMAT_TYPE2
#define ADC_GET_CHANNEL(p) ((p)->type2.channel)
#define ADC_GET_DATA(p) ((p)->type2.data)

// Forward declarations
static void adc_dma_init();
static void adc_dma_start();
static void adc_dma_stop();
static void handle_adc_dma_and_notify();
static inline uint16_t fix_raw_if_needed(uint16_t raw);

// ----- BLE Server Callbacks -----
class MyServerCallbacks : public BLEServerCallbacks
{
  void onConnect(BLEServer *pServer) override
  {
    pixels.setPixelColor(0, pixels.Color(0, PIXEL_BRIGHTNESS, 0)); // Green
    pixels.show();
    digitalWrite(LED_BUILTIN, HIGH);
    vTaskDelay(200 / portTICK_PERIOD_MS);
    digitalWrite(LED_BUILTIN, LOW);

    // Apply +9 dBm to the active connection
    BLEDevice::setPower(ESP_PWR_LVL_P9, ESP_BLE_PWR_TYPE_CONN_HDL0);
    BLEDevice::stopAdvertising(); // Explicitly stop advertising
  }
#if defined(CONFIG_BLUEDROID_ENABLED)
  void onConnect(BLEServer *pServer, esp_ble_gatts_cb_param_t *param) override
  {
    // Request a 7.5-15 ms connection interval while preserving the
    // connection's current latency and supervision timeout.
    pServer->updateConnParams(
        param->connect.remote_bda,
        0x0006, // 6 * 1.25 ms = 7.5 ms
        0x000C, // 12 * 1.25 ms = 15 ms
        param->connect.conn_params.latency,
        param->connect.conn_params.timeout);
  }
#elif defined(CONFIG_NIMBLE_ENABLED)
    void onConnect(BLEServer *pServer, struct ble_gap_conn_desc *desc) override {
    pServer->requestConnParams(
      desc->conn_handle,
      6,   // 6 * 1.25 ms = 7.5 ms
      12,  // 12 * 1.25 ms = 15 ms
      desc->conn_latency,
      desc->supervision_timeout);
  }
#endif
  void onDisconnect(BLEServer *pServer) override
  {
    pixels.setPixelColor(0, pixels.Color(PIXEL_BRIGHTNESS, 0, 0)); // Red on disconnect
    pixels.show();
    // Vibrate twice on disconnect
    digitalWrite(LED_BUILTIN, HIGH);
    vTaskDelay(200 / portTICK_PERIOD_MS);
    digitalWrite(LED_BUILTIN, LOW);
    vTaskDelay(100 / portTICK_PERIOD_MS);
    digitalWrite(LED_BUILTIN, HIGH);
    vTaskDelay(200 / portTICK_PERIOD_MS);
    digitalWrite(LED_BUILTIN, LOW);

    streaming = false;
    // Reset payload state and battery states on disconnect
    sampleIndex = 0;
    payload_wr = 0;
    payload_rd = 0;
    payload_full = 0;
    resetSampleState();
    lastBatteryPct = 255;
    isCharging = 0;
    ledBlinkCycles = -1;

    adc_stop_requested = true; // Request stop
    BLEDevice::startAdvertising();
  }
};

// ----- BLE Control Characteristic Callback -----
// Handles incoming commands ("START", "STOP", "WHORU", "STATUS")
class ControlCallback : public BLECharacteristicCallbacks
{
  void onWrite(BLECharacteristic *characteristic) override
  {
    String cmd = characteristic->getValue();
    cmd.trim();
    cmd.toUpperCase();

    if (cmd == "START")
    {
      pixels.setPixelColor(0, pixels.Color(0, 0, PIXEL_BRIGHTNESS)); // Blue
      pixels.show();
      overallCounter = 0;
      sampleIndex = 0;
      payload_wr = 0;
      payload_rd = 0;
      payload_full = 0;
      resetSampleState();

      // Start battery averaging window on streaming start
      batteryWinStartMs = millis();
      batteryWinSum = 0;
      batteryWinCount = 0;
      batteryAvgToSend = 0;
      isCharging = 0;
      lastBatteryPct = 255;
      ledBlinkCycles = -1;

      streaming = true;
      adc_start_requested = true; // Request start
    }
    else if (cmd == "STOP")
    {
      pixels.setPixelColor(0, pixels.Color(0, PIXEL_BRIGHTNESS, 0)); // Green
      pixels.show();
      streaming = false;
      resetSampleState();
      adc_stop_requested = true; // Request stop
    }
    else if (cmd == "WHORU")
    {
      characteristic->setValue("NPG-LITE");
      characteristic->notify();
    }
    else if (cmd == "STATUS")
    {
      characteristic->setValue(streaming ? "RUNNING" : "STOPPED");
      characteristic->notify();
    }
    else
    {
      characteristic->setValue("UNKNOWN COMMAND");
      characteristic->notify();
    }
  }
};

void neoPixelTask(void *parameter)
{
  while (true)
  {
    if (ledBlinkCycles == -1)
    {
      vTaskDelay(100 / portTICK_PERIOD_MS);
      continue;
    }

    uint8_t cycles = 0;
    uint8_t fader = 100;
    bool decreasing = true;

    // run indefinitely when ledBlinkCycles == 0, else run ledBlinkCycles times
    while (ledBlinkCycles != -1 && (ledBlinkCycles == 0 || cycles < (uint8_t)ledBlinkCycles))
    {
      pixels.setPixelColor(PIXEL_COUNT - 1, pixels.Color(fader, 0, 0));
      pixels.show();
      vTaskDelay(20 / portTICK_PERIOD_MS);

      if (decreasing)
      {
        fader = fader - 2;
        if (fader < 10)
        {
          decreasing = false;
        }
      }
      else
      {
        fader = fader + 2;
        if (fader > 100)
        {
          decreasing = true;
          cycles++;
        }
      }
    }

    // If it was a finite request, clear neopixel after completion
    if (ledBlinkCycles > 0)
    {
      pixels.clear();
      pixels.show();
      ledBlinkCycles = -1; // stop repeating the 10-cycle blink forever
    }
  }
}

// -------Battery Functions-------

// Check battery status while streaming and notify every 10 seconds
void checkBatteryAndDisconnect()
{
  if (batteryAvgToSend == 0)
    return;

  float voltage = (batteryAvgToSend / 1000.0) * 2; // for ESP32C6 v0.1
  voltage = voltage - 0.02;
  float percentage = ceil(interpolatePercentage(voltage));

  // Send decreased battery percentage immediately
  // Send increased battery percentage after 4 consecutive increases
  uint8_t currentBatteryPct = (uint8_t)percentage;

  if (lastBatteryPct == 255)
  {
    // First valid percentage
    lastBatteryPct = currentBatteryPct;
    isCharging = 0;
  }
  else if (currentBatteryPct <= lastBatteryPct)
  {
    lastBatteryPct = currentBatteryPct;
    isCharging = 0;
  }
  else // currentBatteryPct > lastBatteryPct
  {
    if (isCharging >= consecutiveChargingCheck)
    {
      lastBatteryPct = currentBatteryPct;
    }
    isCharging++;
  }

  // Send battery percentage as single byte
  uint8_t batteryByte = lastBatteryPct;
  pBatteryCharacteristic->setValue(&batteryByte, 1);
  pBatteryCharacteristic->notify();
  if (percentage > 70.0)
  {
    ledBlinkCycles = -1;
    pixels.setPixelColor(PIXEL_COUNT - 1, pixels.Color(0, PIXEL_BRIGHTNESS, 0)); // Green when above 70%
    pixels.show();
  }
  else if (percentage <= 70.0 && percentage > 20.0)
  {
    ledBlinkCycles = -1;
    pixels.setPixelColor(PIXEL_COUNT - 1, pixels.Color(15, 4, 0)); // Orange when between 20 and 70
    pixels.show();
  }
  else if (percentage <= 20.0 && percentage > 10.0)
  {
    ledBlinkCycles = -1;
    pixels.setPixelColor(PIXEL_COUNT - 1, pixels.Color(PIXEL_BRIGHTNESS, 0, 0)); // Red when below 20%
    pixels.show();
  }
  else if (percentage <= 10.0 && percentage >= STREAMING_MIN_BATTERY)
  {
    ledBlinkCycles = 0; // blink until battery is in the range of 5-10%
  }
  else if (percentage < STREAMING_MIN_BATTERY)
  {
    // Stop streaming
    streaming = false;

    // Stop ADC since loop() won't run again before deep sleep
    adc_dma_stop();

    // Stop advertising to save power while blinking before deep sleep
    BLEDevice::stopAdvertising();

    // Disconnect BLE client if connected
    if (pBLEServer != nullptr && pBLEServer->getConnectedCount() > 0)
    {
      // Get connection ID from first connected client
      std::map<uint16_t, conn_status_t> peerDevices = pBLEServer->getPeerDevices(false);
      for (auto const &entry : peerDevices)
      {
        uint16_t connId = entry.first;
        pBLEServer->disconnect(connId); // Use public disconnect method
      }
    }
    sleepWhenLowBattery();
  }
}

// Set device to deep sleep when battery is low
void sleepWhenLowBattery()
{
  ledBlinkCycles = 10;         // request 10 cycles (task runs the algo)
  while (ledBlinkCycles != -1) // wait until task finishes
  {
    vTaskDelay(50 / portTICK_PERIOD_MS);
  }
  esp_deep_sleep_start(); // Enter deep sleep after blinking sequence
}

// Check battery on boot and go to deep sleep if battery < BOOT_MIN_BATTERY
void checkInitialBattery()
{
  int count = 0;
  float sum = 0.0;
  unsigned long startMillis = millis();
  while (millis() - startMillis < 100) // Collect battery voltage samples for 100ms
  {
    int analogValue = analogRead(BATTERY_PIN);
    sum += analogValue;
    count++;
  }
  // Avoid divide-by-zero
  if (count == 0)
    return;

  float initialBatteryRaw = sum / count;
  float voltage = (initialBatteryRaw / 1000.0) * 2; // for ESP32C6 v0.1
  voltage = voltage - 0.02;
  float initialBatteryPercentage = ceil(interpolatePercentage(voltage)); // Calculate battery percentage from LUT

  // If battery is low, slowly blink the neopixel
  if (initialBatteryPercentage < BOOT_MIN_BATTERY)
  {
    sleepWhenLowBattery();
  }
}

// --------Check Playmate-------

void checkPlaymate()
{
  pinMode(LED_BUILTIN, INPUT_PULLUP);
  pinMode(BUZZER_PIN, INPUT_PULLUP);
  if(digitalRead(LED_BUILTIN) == HIGH && digitalRead(BUZZER_PIN) == HIGH)
  {
    Playmate = PROTO_PLAYMATE;
  }
  else
  {
    Playmate = VIBZ_PLAYMATE; // Assume Vibz until Vibz Plus is detected
    // Check for Vibz Plus Playmate
    pinMode(A3, INPUT_PULLUP);
    pinMode(A4, INPUT_PULLUP);
    pinMode(A5, INPUT_PULLUP);
    unsigned long start = millis();
    while (millis() - start < 100) 
    {
      if (digitalRead(A3) == LOW || digitalRead(A4) == LOW || digitalRead(A5) == LOW) 
      {
        Playmate = VIBZ_PLUS_PLAYMATE;
        break;
      }
    }
    // Restore high-impedance inputs before ADC use
    pinMode(A3, INPUT);
    pinMode(A4, INPUT);
    pinMode(A5, INPUT);
  }
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);
  // Configure active channels and packet sizes
  if (Playmate == VIBZ_PLUS_PLAYMATE)
    NUM_CHANNELS = 7;
  else
    NUM_CHANNELS = 4;
  recomputePacketSizes();
}

void setup()
{
  // ----- LEDs -----
  pixels.begin();

  xTaskCreatePinnedToCore(neoPixelTask, "NeoPixelTask", 2048, NULL, 1, &ledBlinkTask, 0);

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  setCpuFrequencyMhz(80);

  checkPlaymate();

  checkInitialBattery(); // Check initial battery status

  pixels.setPixelColor(0, pixels.Color(PIXEL_BRIGHTNESS, 0, 0)); // Red (power on)
  pixels.show();

  // Create binary semaphore for ADC data ready signaling
  adc_data_semaphore = xSemaphoreCreateBinary();

  if (adc_data_semaphore == nullptr)
  {
    while (1)
      ; // Halt
  }

  esp_read_mac(mac, ESP_MAC_EFUSE_FACTORY);

  // ----- BLE-only memory footprint (free Classic BT) -----
  esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT);

  // ----- Initialize BLE -----
  char deviceName[36];
  if (Playmate == VIBZ_PLUS_PLAYMATE)
    sprintf(deviceName, "NPG-Lite-6CH:%02X:%02X", mac[4], mac[5]);
  else
    sprintf(deviceName, "NPG-Lite-3CH:%02X:%02X", mac[4], mac[5]);
  BLEDevice::init(deviceName);

  // larger MTU for efficiency (doesn't change packet format)
  BLEDevice::setMTU(500);

  pBLEServer = BLEDevice::createServer();
  pBLEServer->setCallbacks(new MyServerCallbacks());
  BLEService *pService = pBLEServer->createService(SERVICE_UUID);

  // Data Characteristic (Notify only) for ADC data
  pDataCharacteristic = pService->createCharacteristic(
      DATA_CHAR_UUID,
      BLECharacteristic::PROPERTY_NOTIFY);
  pDataCharacteristic->addDescriptor(new BLE2902());

  // Control Characteristic (Read/Write/Notify)
  pControlCharacteristic = pService->createCharacteristic(
      CONTROL_CHAR_UUID,
      BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_NOTIFY);
  pControlCharacteristic->setCallbacks(new ControlCallback());

  // Battery Characteristic (Read/Notify)
  pBatteryCharacteristic = pService->createCharacteristic(
      BATTERY_CHAR_UUID,
      BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
  pBatteryCharacteristic->addDescriptor(new BLE2902());

  pService->start();

  BLEDevice::startAdvertising();
}

void loop()
{
  
  if (adc_stop_requested)
  {
    adc_dma_stop();
    adc_stop_requested = false;
  }

  // Handle start/stop requests of adc_continuous_mode
  if (adc_start_requested)
  {
    adc_dma_start();
    adc_start_requested = false;
  }

  if (streaming)
  {
    // Battery check only when streaming (every battery check interval)
    unsigned long currentMillis = millis();
    if (currentMillis - lastBatteryCheck >= BATTERY_CHECK_INTERVAL)
    {
      lastBatteryCheck = currentMillis;
      checkBatteryAndDisconnect();
    }

    // Block until semaphore is given by ISR 
    if (xSemaphoreTake(adc_data_semaphore, portMAX_DELAY) == pdTRUE)
    {
      handle_adc_dma_and_notify();
    }
  }
  else
  {
    // Longer delay when idle 
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

// ----- ADC DMA implementation -----

// Maps physical ADC channel id → logical index 0..NUM_CHANNELS-1
static int8_t hw2idx[10];

static const uint8_t hw_chs_4[4] = {0, 1, 2, 6};          // Configuration for 3 BioAmp Channels
static const uint8_t hw_chs_7[7] = {0, 1, 2, 3, 4, 5, 6}; // Configuration for 6 BioAmp Channels

static void adc_dma_init()
{

  static adc_digi_pattern_config_t pattern[NUM_CHANNELS_MAX];
  const uint8_t *hw_chs = (NUM_CHANNELS == 7) ? hw_chs_7 : hw_chs_4; // Use appropriate channel configuration

  // Build pattern from the single channel list
  for (int i = 0; i < NUM_CHANNELS; i++)
  {
    pattern[i].atten = ADC_ATTEN_DB_11;
    pattern[i].channel = hw_chs[i]; // Use physical channel id
    pattern[i].unit = ADC_UNIT_1;
    pattern[i].bit_width = ADC_BITWIDTH_12;
  }

  for (int i = 0; i < (int)sizeof(hw2idx); i++)
    hw2idx[i] = -1;
  for (int i = 0; i < NUM_CHANNELS; i++)
    hw2idx[hw_chs[i]] = i;

  // Create driver handle and configure continuous conversion
  adc_continuous_handle_cfg_t handle_cfg = {
      .max_store_buf_size = NUM_CHANNELS * SOC_ADC_DIGI_RESULT_BYTES * BLOCK_COUNT * 5,
      .conv_frame_size = NUM_CHANNELS * SOC_ADC_DIGI_RESULT_BYTES * BLOCK_COUNT,
#if defined(ESP_IDF_VERSION) && (ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 3, 0))
      .flags = {.flush_pool = 1}, // Only for newer IDF that supports it
#endif
  };
  if (adc_handle == nullptr)
  {
    ESP_ERROR_CHECK(adc_continuous_new_handle(&handle_cfg, &adc_handle));
  }

  adc_continuous_evt_cbs_t cbs = {
      .on_conv_done = [](adc_continuous_handle_t handle,
                         const adc_continuous_evt_data_t *edata,
                         void *user_data) -> bool
      {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        xSemaphoreGiveFromISR(adc_data_semaphore, &xHigherPriorityTaskWoken);
        return (xHigherPriorityTaskWoken == pdTRUE); // Yield if higher priority task woken
      },
  };
  ESP_ERROR_CHECK(adc_continuous_register_event_callbacks(adc_handle, &cbs, nullptr));

  adc_continuous_config_t dig_cfg = {
      .pattern_num = NUM_CHANNELS,
      .adc_pattern = pattern,
      // total sample rate = per-channel * number of channels
      .sample_freq_hz = (uint32_t)(SAMP_RATE * NUM_CHANNELS), // 2000 SPS total
      .conv_mode = ADC_CONV_SINGLE_UNIT_1,
      .format = ADC_OUTPUT_TYPE, // TYPE2 (unit, channel, data)
  };
  ESP_ERROR_CHECK(adc_continuous_config(adc_handle, &dig_cfg));
}

static void adc_dma_start()
{
  // Reinitialize ADC if it was deinited
  if (adc_handle == nullptr)
  {
    adc_dma_init();
  }

  if (adc_handle && !adc_started)
  {
    ESP_ERROR_CHECK(adc_continuous_start(adc_handle));
    adc_started = true;
  }
}

static void adc_dma_stop()
{
  if (adc_handle && adc_started)
  {
    ESP_ERROR_CHECK(adc_continuous_stop(adc_handle));
    adc_started = false;

    // DEINITIALIZE ADC to save power
    ESP_ERROR_CHECK(adc_continuous_deinit(adc_handle));
    adc_handle = nullptr;
  }
}

static inline uint16_t fix_raw_if_needed(uint16_t raw)
{
#if defined(CONFIG_IDF_TARGET_ESP32C6)
  // Optional: match your prior scaling workaround for C6 rev1 if needed
  if (chiprev == 1)
  {
    // scale raw (0..~3249) to 0..4095
    uint32_t v = (uint32_t)raw * 4095u / 3249u;
    if (v > 4095u)
      v = 4095u;
    return (uint16_t)v;
  }
#endif
  return raw;
}

static void handle_adc_dma_and_notify()
{
  // Assemble Channel data into sample packets
  // use global last_vals/have_mask
  const uint32_t FULL_MASK = (1u << NUM_CHANNELS) - 1;
  uint8_t dma_buf[NUM_CHANNELS_MAX * SOC_ADC_DIGI_RESULT_BYTES * BLOCK_COUNT];

  // Drain ADC driver until empty OR until our payload buffers are full
  while (payload_full < BLE_PAYLOAD_BUFFERS)
  {
    // Read whatever DMA has buffered
    uint32_t ret_len = 0;

    esp_err_t ret = adc_continuous_read(adc_handle, dma_buf, sizeof(dma_buf), &ret_len, 0);
    if (ret != ESP_OK || ret_len == 0)
    {
      break;
    }

    for (uint32_t i = 0; i + SOC_ADC_DIGI_RESULT_BYTES <= ret_len; i += SOC_ADC_DIGI_RESULT_BYTES)
    {
      auto *p = (const adc_digi_output_data_t *)&dma_buf[i];
      uint8_t ch_hw = ADC_GET_CHANNEL(p); // physical channel id from TYPE2
      uint16_t raw = ADC_GET_DATA(p);

      // map physical channel → logical index (0..NUM_CHANNELS-1)
      int8_t idx = (ch_hw < (uint8_t)sizeof(hw2idx)) ? hw2idx[ch_hw] : -1;
      if (idx >= 0 && idx < (int8_t)NUM_CHANNELS)
      {
        // Apply fix only to BioAmp channels (A0-A5), NOT battery (A6)
        if (idx < (NUM_CHANNELS - 1))
        {
          last_vals[idx] = fix_raw_if_needed(raw);
        }
        else
        {
          last_vals[idx] = raw; // Battery channel: use raw value
        }
        have_mask |= (1u << idx);

        // Track latest battery raw for reference/debug (averaging happens per completed packet)
        if (idx == (NUM_CHANNELS - 1))
        {
          latestBatteryRaw = last_vals[idx];
        }
      }

      // When we have all channels, emit one record into current payload buffer
      if (have_mask == FULL_MASK)
      {
        // If all payload buffers are full, stop reading
        if (payload_full >= BLE_PAYLOAD_BUFFERS)
        {
          have_mask = 0;
          break;
        }

        // Calculate offset in blePayload buffer
        uint16_t offset = sampleIndex * SINGLE_SAMPLE_LEN;

        // Write counter directly to blePayload buffer
        blePayload[payload_wr][offset] = overallCounter;
        overallCounter = (overallCounter + 1) & 0xFF;

        // Big-endian packing directly to blePayload buffer
        for (uint8_t c = 0; c < NUM_CHANNELS - 1; c++)
        {
          uint16_t v = last_vals[c];
          blePayload[payload_wr][offset + 1 + c * 2] = (uint8_t)((v >> 8) & 0xFF);
          blePayload[payload_wr][offset + 1 + c * 2 + 1] = (uint8_t)(v & 0xFF);
        }

        sampleIndex++;

        if (sampleIndex >= BLOCK_COUNT)
        {
          sampleIndex = 0;

          // Log one battery ADC value per completed packet
          if (NUM_CHANNELS > 0)
          {
            uint16_t batt = last_vals[NUM_CHANNELS - 1];
            batteryWinSum += batt;
            batteryWinCount++;
          }

          // Every BATTERY_CHECK_INTERVAL, compute window average and update latch (decreasing-only)
          uint32_t nowMs = millis();
          if ((uint32_t)(nowMs - batteryWinStartMs) >= (uint32_t)BATTERY_CHECK_INTERVAL)
          {
            if (batteryWinCount > 0)
            {
              uint16_t currentAvg = (uint16_t)(batteryWinSum / batteryWinCount);
              batteryAvgToSend = currentAvg;
            }

            // Reset window
            batteryWinStartMs = nowMs;
            batteryWinSum = 0;
            batteryWinCount = 0;
          }

          payload_full++;
          payload_wr = (payload_wr + 1) % BLE_PAYLOAD_BUFFERS;

          // If all payload buffers are full, stop reading
          if (payload_full >= BLE_PAYLOAD_BUFFERS)
          {
            have_mask = 0;
            break;
          }
        }

        have_mask = 0; // reset for next triplet
      }
    }
  }
  while (payload_full > 0 && streaming)
  {
    pDataCharacteristic->setValue(blePayload[payload_rd], NEW_PACKET_LEN);
    pDataCharacteristic->notify();

    payload_rd = (uint8_t)((payload_rd + 1) % BLE_PAYLOAD_BUFFERS);
    payload_full--;
  }
}
