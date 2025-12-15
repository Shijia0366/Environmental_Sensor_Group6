#pragma once
#include <Arduino.h>
#include <NimBLEDevice.h>

/** === Your slave Bluetooth definition === */
static constexpr char   ACC_NAME[]     = "C3-LP-ACC";
static constexpr char   PPG_NAME[]     = "C3-LP-PPG";
static constexpr char   ACC_SVC_UUID[] = "8b37e000-6d6a-4a90-903b-22f43a1e2001";
static constexpr char   ACC_CHR_UUID[] = "8b37e001-6d6a-4a90-903b-22f43a1e2001";
static constexpr char   PPG_SVC_UUID[] = "fae6b800-1a5f-4c6b-8a28-2a42b2d60001";
static constexpr char   PPG_CHR_UUID[] = "fae6b801-1a5f-4c6b-8a28-2a42b2d60001";

/** Data source identifier */
enum class Src : uint8_t { ACC = 1, PPG = 2 };


struct BlePacket {
  Src      src;
  uint16_t len;
  uint8_t  data[100]; 
};


class BLEManager {
public:
  // qDepth
  static void begin(size_t qDepth = 32, uint16_t scanMs = 4000, uint16_t scanRestMs = 2000);
  static void loopTick();                       // Lightweight state machine propulsion
  static bool getPacket(BlePacket &out);       
  static bool accConnected();
  static bool ppgConnected();

  // For callback class to call
  static void handleAdv(NimBLEAdvertisedDevice* adv);
  static void onClientDisconnect(NimBLEClient* c);

private:
  BLEManager() = delete; 

  // Scan & Connect Process
  static void startScan();
  static void tryConnectTarget(const char* name, const char* svcUuid,
                               const char* chrUuid, Src srcTag);


  struct ClientCtx {
  public:
    NimBLEAddress addr;
    bool   haveAddr = false;
    Src src;
    const char* name;
    const char* svcUuid;
    const char* chrUuid;

    NimBLEClient*               client   = nullptr;
    NimBLERemoteCharacteristic* chr      = nullptr;
    bool                        wanted   = false;   // Determine if it is in the target list
    bool                        connected= false;
    uint32_t                    backoffMs= 1000;    // Reconnection index declines

    ClientCtx() = default;
    ClientCtx(Src s, const char* n, const char* svc, const char* chr)
      : src(s), name(n), svcUuid(svc), chrUuid(chr) {}
  };

  static void onDisconnect(ClientCtx& ctx);
  static void subscribe(ClientCtx& ctx);

  // Received Notify callback
  static void notifyCB(NimBLERemoteCharacteristic* c, uint8_t* data, size_t len, bool isNotify);

  static QueueHandle_t sQueue;

  static ClientCtx sAcc, sPpg;

  // Scan parameters and status
  static uint32_t sScanEndMs;
  static uint16_t sScanMs, sScanRestMs;
  static bool sScanning;
};
