#include "ble_manager.h"

QueueHandle_t BLEManager::sQueue = nullptr;

BLEManager::ClientCtx BLEManager::sAcc(Src::ACC, ACC_NAME, ACC_SVC_UUID, ACC_CHR_UUID);
BLEManager::ClientCtx BLEManager::sPpg(Src::PPG, PPG_NAME, PPG_SVC_UUID, PPG_CHR_UUID);

uint32_t BLEManager::sScanEndMs = 0;
uint16_t BLEManager::sScanMs    = 4000;
uint16_t BLEManager::sScanRestMs= 2000;
bool BLEManager::sScanning      = false;

static NimBLEScan* gScan = nullptr;


class AdvCb : public NimBLEAdvertisedDeviceCallbacks {
  void onResult(NimBLEAdvertisedDevice* adv) override {
    BLEManager::handleAdv(adv);
  }
};

class ClientCb : public NimBLEClientCallbacks {
  void onConnect(NimBLEClient* c) override {

  }
  void onDisconnect(NimBLEClient* c) override {
    BLEManager::onClientDisconnect(c);
  }
};

// static instance
static AdvCb    sAdvCb;
static ClientCb sClientCb;

/** ---- public interface ---- */
void BLEManager::begin(size_t qDepth, uint16_t scanMs, uint16_t scanRestMs) {
  sScanMs = scanMs; sScanRestMs = scanRestMs;

  NimBLEDevice::init("S3-HOST");
  NimBLEDevice::setPower(ESP_PWR_LVL_N0);
  NimBLEDevice::setMTU(100);

  sQueue = xQueueCreate(qDepth, sizeof(BlePacket));

  gScan = NimBLEDevice::getScan();
  gScan->setInterval(80); // 50 ms
  gScan->setWindow(40);   // 25 ms
  gScan->setActiveScan(true);

  sAcc.wanted = true;
  sPpg.wanted = true;

  startScan();
}

void BLEManager::loopTick() {
  const uint32_t now = millis();

  // Scan window control (non-blocking)
  if (sScanning && now > sScanEndMs) {
    gScan->stop();
    sScanning = false;
    sScanEndMs = now + sScanRestMs;
  } else if (!sScanning && now > sScanEndMs) {
    if (!sAcc.connected || !sPpg.connected) startScan();
  }

  // Disconnection and reconnection (index rollback)
  auto tryReconnect = [&](ClientCtx& ctx) {
    if (!ctx.wanted || ctx.connected) return;
    static uint32_t nextTryAcc = 0, nextTryPpg = 0;
    uint32_t &nextTry = (ctx.src == Src::ACC) ? nextTryAcc : nextTryPpg;
    if (millis() < nextTry) return;

    tryConnectTarget(ctx.name, ctx.svcUuid, ctx.chrUuid, ctx.src);
    if (!ctx.connected) {
      ctx.backoffMs = min<uint32_t>(ctx.backoffMs * 2, 15000);
      nextTry = millis() + ctx.backoffMs;
    } else {
      ctx.backoffMs = 1000;
      nextTry = 0;
    }
  };

  tryReconnect(sAcc);
  tryReconnect(sPpg);
}

bool BLEManager::getPacket(BlePacket &out) {
  if (sQueue == nullptr) return false;
  return xQueueReceive(sQueue, &out, 0) == pdTRUE;
}

bool BLEManager::accConnected() { return sAcc.connected; }
bool BLEManager::ppgConnected() { return sPpg.connected; }

/** ---- Internal: Scanning/Processing ---- */
void BLEManager::startScan() {
  if (sScanning) return;
  gScan->setAdvertisedDeviceCallbacks(&sAdvCb, false);
  gScan->start(sScanMs / 1000.0, false);
  sScanning = true;
  sScanEndMs = millis() + sScanMs;
}

void BLEManager::handleAdv(NimBLEAdvertisedDevice* adv) {
  std::string name = adv->getName();
  if (sAcc.wanted && !sAcc.connected && name == sAcc.name) {
    sAcc.addr = adv->getAddress(); sAcc.haveAddr = true;
    tryConnectTarget(sAcc.name, sAcc.svcUuid, sAcc.chrUuid, Src::ACC);
  } else if (sPpg.wanted && !sPpg.connected && name == sPpg.name) {
    sPpg.addr = adv->getAddress(); sPpg.haveAddr = true;
    tryConnectTarget(sPpg.name, sPpg.svcUuid, sPpg.chrUuid, Src::PPG);
  }
}

/** ----Internal: Connections/Subscriptions ---- */
void BLEManager::tryConnectTarget(const char* name, const char* svcUuid,
                                  const char* chrUuid, Src srcTag) {
  ClientCtx& ctx = (srcTag == Src::ACC) ? sAcc : sPpg;

  if (ctx.connected && ctx.client && ctx.client->isConnected()) return;

  if (gScan->isScanning()) gScan->stop();

  if (!ctx.client) {
    ctx.client = NimBLEDevice::createClient();
    ctx.client->setClientCallbacks(&sClientCb, false);
    ctx.client->setConnectTimeout(3);
  }

  // —— Key point: Prioritize by address, then directly connect by name. —— //
  bool ok = false;
  if (ctx.haveAddr) {
    ok = ctx.client->connect(ctx.addr);
  } else {
    ok = ctx.client->connect(name);
  }
  if (!ok) { onDisconnect(ctx); return; }

  auto* svc = ctx.client->getService(svcUuid);
  if (!svc) { ctx.client->disconnect(); onDisconnect(ctx); return; }

  ctx.chr = svc->getCharacteristic(chrUuid);
  if (!ctx.chr || !ctx.chr->canNotify()) {
    ctx.client->disconnect();
    onDisconnect(ctx);
    return;
  }

  subscribe(ctx);
}

void BLEManager::subscribe(ClientCtx& ctx) {
  if (!ctx.chr->subscribe(true, BLEManager::notifyCB)) {
    ctx.client->disconnect();
    onDisconnect(ctx);
    return;
  }
  ctx.connected = true;
  ctx.backoffMs = 1000;
}

void BLEManager::onDisconnect(ClientCtx& ctx) {
  ctx.connected = false;
  ctx.chr = nullptr;
  startScan();
}

void BLEManager::onClientDisconnect(NimBLEClient* c) {
  ClientCtx* p = nullptr;
  if (c == sAcc.client) p = &sAcc;
  else if (c == sPpg.client) p = &sPpg;
  if (p) onDisconnect(*p);
}


void BLEManager::notifyCB(NimBLERemoteCharacteristic* c, uint8_t* data, size_t len, bool) {
  Src src = (c == sAcc.chr) ? Src::ACC : Src::PPG;

  while (len) {
    BlePacket pkt;
    pkt.src = src;
    pkt.len = (uint16_t)min(len, sizeof(pkt.data));
    memcpy(pkt.data, data, pkt.len);

    // Non-blocking enqueue
    if (xQueueSend(sQueue, &pkt, 0) != pdTRUE) {
      BlePacket dropOld;
      xQueueReceive(sQueue, &dropOld, 0);       // Discard the oldest
      (void)xQueueSend(sQueue, &pkt, 0);        // Try joining the team again
    }

    data += pkt.len;
    len  -= pkt.len;
  }
}
