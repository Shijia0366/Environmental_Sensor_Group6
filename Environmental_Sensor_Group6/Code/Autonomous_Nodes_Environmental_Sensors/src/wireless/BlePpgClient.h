#pragma once

#include <Arduino.h>
#include <BLEDevice.h>

// UUIDs must match your PPG peripheral
#define PPG_SERVICE_UUID        "d3aa6a66-a623-4c76-80a5-a66004acf0bf"
#define PPG_CHARACTERISTIC_UUID "2df03c7d-8ac5-493e-9d88-ac467aed4ad0"

class BlePpgClient {
public:
  BlePpgClient();

  // Call in setup(): Initialize BLE (Non-blocking scan)
  void begin(const char* hostName = "esp32s3-host");

  // Call repeatedly in loop(): Handles Scan/Connect/Reconnect
  void loop();

  // Is currently connected to PPG peripheral
  bool isConnected() const { return connected; }

  // Has received valid HR / SpO2 data
  bool hasValidData() const { return dataValid; }

  uint32_t heartRate() const { return hr; }   // bpm
  uint32_t spo2() const { return spo2Val; }   // %

  // Public to allow access from static callback
  void onNotify(uint8_t* data, size_t length);

private:
  friend class PpgAdvertisedDeviceCallbacks;

  void connectToServer();

  BLEUUID serviceUUID;
  BLEUUID charUUID;

  BLEClient* pClient      = nullptr;
  BLERemoteCharacteristic* pRemoteChar  = nullptr;
  BLEAdvertisedDevice* pFoundDevice = nullptr;

  bool connected   = false;
  bool doConnect   = false;
  bool dataValid   = false;

  float hr         = 0.0f;
  float spo2Val    = 0.0f;

  // Last connection attempt time (controls reconnect interval)
  unsigned long lastConnectAttemptMs = 0;
  const unsigned long RECONNECT_INTERVAL_MS = 5000; // Retry every 5s on failure

  // ===== Scan Scheduling =====
  unsigned long lastScanMs            = 0;     // Timestamp when last scan ended
  const unsigned long SCAN_INTERVAL_MS = 5000; // Interval between scans (ms)
  const uint32_t      SCAN_DURATION_S  = 2;    // Duration of each scan (seconds)
};