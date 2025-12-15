#include "BlePpgClient.h"

// ======== Global Pointer for Notify Callback forwarding ========
static BlePpgClient* g_ppgClientInstance = nullptr;

// ======== Scan Callback: Prepare to connect if service found ========
class PpgAdvertisedDeviceCallbacks : public BLEAdvertisedDeviceCallbacks {
public:
  explicit PpgAdvertisedDeviceCallbacks(BlePpgClient* parent) : _parent(parent) {}

  // Note: Passed by value, no override used to maintain compatibility
  void onResult(BLEAdvertisedDevice advertisedDevice) {
    // Check for target Service UUID
    if (advertisedDevice.haveServiceUUID() &&
        advertisedDevice.isAdvertisingService(_parent->serviceUUID)) {

      Serial.print("[BLE-Host] Found PPG device: ");
      Serial.println(advertisedDevice.toString().c_str());

      // Stop scanning
      BLEDevice::getScan()->stop();

      // Save device reference
      if (_parent->pFoundDevice != nullptr) {
        delete _parent->pFoundDevice;
        _parent->pFoundDevice = nullptr;
      }
      _parent->pFoundDevice = new BLEAdvertisedDevice(advertisedDevice);

      _parent->doConnect = true;
      _parent->lastConnectAttemptMs = millis();
    }
  }

private:
  BlePpgClient* _parent;
};

// ======== Notify Callback: Standard C function ========
static void ppgNotifyCallback(
  BLERemoteCharacteristic* pRemoteCharacteristic,
  uint8_t* data,
  size_t length,
  bool isNotify
) {
  (void)pRemoteCharacteristic;
  (void)isNotify;

  if (g_ppgClientInstance) {
    g_ppgClientInstance->onNotify(data, length);
  }
}

// ======== BlePpgClient Implementation ========

BlePpgClient::BlePpgClient()
  : serviceUUID(PPG_SERVICE_UUID), charUUID(PPG_CHARACTERISTIC_UUID) {}

// Call in setup() (Scan is not started indefinitely here)
void BlePpgClient::begin(const char* hostName) {
  Serial.println("[BLE-Host] Init BLE client...");

  g_ppgClientInstance = this;      // Allows global callback to find instance

  BLEDevice::init(hostName);

  BLEScan* pScan = BLEDevice::getScan();
  pScan->setAdvertisedDeviceCallbacks(new PpgAdvertisedDeviceCallbacks(this));
  pScan->setInterval(160);  // Scan interval & window
  pScan->setWindow(80);
  pScan->setActiveScan(true);

  // Do not start() here to avoid blocking setup
  lastScanMs = millis() - SCAN_INTERVAL_MS;  // Trigger immediate scan in first loop
  
  Serial.println("[BLE-Host] BLE initialized, scan will run in loop()");
}

// Call repeatedly in loop() (Non-blocking scheduler)
void BlePpgClient::loop() {
  if (connected) {
    // Connected: Data arrives via notify asynchronously, nothing to do here
    return;
  }

  unsigned long now = millis();

  // Target found, attempt connection at intervals
  if (doConnect && pFoundDevice != nullptr) {
    if (now - lastConnectAttemptMs >= RECONNECT_INTERVAL_MS) {
      connectToServer();
      lastConnectAttemptMs = now;

      // If failed, restart scan after a delay
      if (!connected) {
        Serial.println("[BLE-Host] Re-start scan after failed connect");
        lastScanMs = now;  // Wait before scanning again
      }
    }
  } else {
    // No target found or no pending connection: Start short periodic scan
    if (now - lastScanMs >= SCAN_INTERVAL_MS) {
      Serial.println("[BLE-Host] Scan start");
      // Note: start(duration, is_continue) blocks for 'duration' seconds,
      // but since it's occasional in loop, it won't freeze setup.
      BLEDevice::getScan()->start(SCAN_DURATION_S, false);
      Serial.println("[BLE-Host] Scan done");
      lastScanMs = now;
    }
  }
}

// Connect to PPG Server
void BlePpgClient::connectToServer() {
  if (!pFoundDevice) {
    return;
  }

  Serial.println("[BLE-Host] Connecting to PPG server...");

  // Create or reuse BLEClient
  if (pClient == nullptr) {
    pClient = BLEDevice::createClient();
  }

  // connect() requires BLEAdvertisedDevice*
  if (!pClient->connect(pFoundDevice)) {
    Serial.println("[BLE-Host] Failed to connect");
    connected = false;
    return;
  }

  Serial.println("[BLE-Host] Connected, getting service...");

  BLERemoteService* pRemoteService = pClient->getService(serviceUUID);
  if (pRemoteService == nullptr) {
    Serial.println("[BLE-Host] Failed to find service");
    pClient->disconnect();
    connected = false;
    return;
  }

  pRemoteChar = pRemoteService->getCharacteristic(charUUID);
  if (pRemoteChar == nullptr) {
    Serial.println("[BLE-Host] Failed to find characteristic");
    pClient->disconnect();
    connected = false;
    return;
  }

  // Register notify callback
  if (pRemoteChar->canNotify()) {
    pRemoteChar->registerForNotify(ppgNotifyCallback);
    Serial.println("[BLE-Host] Notification enabled");
  } else {
    Serial.println("[BLE-Host] Characteristic cannot notify");
  }

  connected = true;
  dataValid = false; // Wait for first frame
  Serial.println("[BLE-Host] PPG client ready");
}

// Notify handler for "HR,SpO2" text
void BlePpgClient::onNotify(uint8_t* data, size_t length) {
  if (length == 0) return;

  char buf[40] = {0};
  size_t copyLen = (length < sizeof(buf) - 1) ? length : sizeof(buf) - 1;
  memcpy(buf, data, copyLen);
  buf[copyLen] = '\0';

  // Format: "72.5,97.8"
  char* comma = strchr(buf, ',');
  if (!comma) return;

  *comma = '\0';
  const char* hrStr   = buf;
  const char* spo2Str = comma + 1;

  float hrVal    = atof(hrStr);
  float spo2Val_ = atof(spo2Str);

  // Validity check
  if (hrVal > 0.0f && hrVal < 250.0f &&
      spo2Val_ > 0.0f && spo2Val_ <= 100.0f) {

    hr      = hrVal;
    spo2Val = spo2Val_;
    dataValid = true;

    Serial.print("[BLE-Host] RX HR/SpO2: ");
    Serial.print(hr);
    Serial.print(" bpm, ");
    Serial.print(spo2Val);
    Serial.println(" %");
  }
}