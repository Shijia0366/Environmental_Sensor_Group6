#pragma once
#include <Arduino.h>
#include "ble_manager.h"   


struct AccSample {
  uint32_t ts_ms = 0;
  float ax = NAN, ay = NAN, az = NAN;   // m/s^2
};

struct PpgSample {
  uint32_t ts_ms = 0;
  bool hasRed = false, hasIr = false;
  uint32_t red = 0;   // 24-bit 
  uint32_t ir  = 0;  
};

// ---------- parser ----------
class BleStreams {
public:
// ppgSpo2=true indicates 6 bytes per sample (RED+IR), false indicates 3 bytes (IR only)
// accLsbMg is the LSB corresponding to mg in the current range/mode of LIS2DE12 (e.g., ±2g 8-bit, commonly 16 mg/LSB)
  void begin(bool ppgSpo2, float accLsbMg) {
    setPpgMode(ppgSpo2);
    setAccScaleMg(accLsbMg);
    reset();
  }

  void setPpgMode(bool spo2Mode) { ppg_spo2_ = spo2Mode; }
  void setAccScaleMg(float lsbMg) { acc_lsb_mps2_ = (lsbMg * 1e-3f) * 9.80665f; }
  void reset();

  // Pass the packets retrieved by BLEManager into the system; it will automatically route them to ACC/PPG for resolution.
  void ingest(const BlePacket& pkt);

  // Pop the parsed sample in a non-blocking manner; return true if available.
  bool popAcc(AccSample& out);
  bool popPpg(PpgSample& out);

private:
  // ----Internal: ACC/PPG parsing ----
  void processAcc_(const uint8_t* data, size_t len);
  void processPpg_(const uint8_t* data, size_t len);
  static inline uint32_t be24_(const uint8_t* p) {
    return ( (uint32_t)p[0] << 16 ) | ( (uint32_t)p[1] << 8 ) | (uint32_t)p[2];
  }

  // ----carry----
  uint8_t acc_carry_[2];  size_t acc_carry_n_ = 0;   // ACC is 3-byte aligned, with a maximum of 2 bytes reserved.
  uint8_t ppg_carry_[5];  size_t ppg_carry_n_ = 0;   // PPG is 6-byte aligned, with a maximum of 5 bytes reserved.
  bool    ppg_spo2_ = true;

  // ---- Scaling ----
  float acc_lsb_mps2_ = (16.0f * 1e-3f) * 9.80665f;  // Default 16 mg/LSB → m/s^2
  static constexpr int ACC_Q = 128;
  static constexpr int PPG_Q = 128;

  AccSample acc_q_[ACC_Q]; volatile int acc_h_ = 0, acc_t_ = 0;
  PpgSample ppg_q_[PPG_Q]; volatile int ppg_h_ = 0, ppg_t_ = 0;

  void accPush_(const AccSample& s) {
    int nx = (acc_h_ + 1) % ACC_Q;
    if (nx == acc_t_) acc_t_ = (acc_t_ + 1) % ACC_Q; 
    acc_q_[acc_h_] = s; acc_h_ = nx;
  }
  bool accPop_(AccSample& s) {
    if (acc_h_ == acc_t_) return false;
    s = acc_q_[acc_t_]; acc_t_ = (acc_t_ + 1) % ACC_Q; return true;
  }
  void ppgPush_(const PpgSample& s) {
    int nx = (ppg_h_ + 1) % PPG_Q;
    if (nx == ppg_t_) ppg_t_ = (ppg_t_ + 1) % PPG_Q; 
    ppg_q_[ppg_h_] = s; ppg_h_ = nx;
  }
  bool ppgPop_(PpgSample& s) {
    if (ppg_h_ == ppg_t_) return false;
    s = ppg_q_[ppg_t_]; ppg_t_ = (ppg_t_ + 1) % PPG_Q; return true;
  }
};
