#include "ble_streams.h"
#include <string.h>

void BleStreams::reset() {
  acc_carry_n_ = 0; ppg_carry_n_ = 0;
  acc_h_ = acc_t_ = 0;
  ppg_h_ = ppg_t_ = 0;
}

void BleStreams::ingest(const BlePacket& pkt) {
  if (pkt.src == Src::ACC) {
    processAcc_(pkt.data, pkt.len);
  } else if (pkt.src == Src::PPG) {
    processPpg_(pkt.data, pkt.len);
  }
}

bool BleStreams::popAcc(AccSample& out) { return accPop_(out); }
bool BleStreams::popPpg(PpgSample& out) { return ppgPop_(out); }

// --------ACC: Grouped in sets of 3 bytes (x, y, z, int8) --------
void BleStreams::processAcc_(const uint8_t* data, size_t len) {
  if (acc_carry_n_) {
    size_t need = 3 - acc_carry_n_;
    size_t take = (len < need) ? len : need;
    memcpy(acc_carry_ + acc_carry_n_, data, take);
    acc_carry_n_ += take; data += take; len -= take;
    if (acc_carry_n_ == 3) {
      int8_t x = (int8_t)acc_carry_[0];
      int8_t y = (int8_t)acc_carry_[1];
      int8_t z = (int8_t)acc_carry_[2];
      AccSample s; s.ts_ms = millis();
      s.ax = x * acc_lsb_mps2_;
      s.ay = y * acc_lsb_mps2_;
      s.az = z * acc_lsb_mps2_;
      accPush_(s);
      acc_carry_n_ = 0;
    }
  }


  while (len >= 3) {
    int8_t x = (int8_t)data[0];
    int8_t y = (int8_t)data[1];
    int8_t z = (int8_t)data[2];
    AccSample s; s.ts_ms = millis();
    s.ax = x * acc_lsb_mps2_;
    s.ay = y * acc_lsb_mps2_;
    s.az = z * acc_lsb_mps2_;
    accPush_(s);
    data += 3; len -= 3;
  }


  if (len) { memcpy(acc_carry_, data, len); acc_carry_n_ = len; }
}

// --------PPG: SPO2 mode 6 bytes/sample (RED, IR), HR-only 3 bytes/sample (IR) --------
void BleStreams::processPpg_(const uint8_t* data, size_t len) {
  size_t group = ppg_spo2_ ? 6 : 3;

  if (ppg_carry_n_) {
    size_t need = group - ppg_carry_n_;
    size_t take = (len < need) ? len : need;
    memcpy(ppg_carry_ + ppg_carry_n_, data, take);
    ppg_carry_n_ += take; data += take; len -= take;
    if (ppg_carry_n_ == group) {
      PpgSample s; s.ts_ms = millis();
      if (ppg_spo2_) {
        s.hasRed = s.hasIr = true;
        s.red = be24_(ppg_carry_ + 0);
        s.ir  = be24_(ppg_carry_ + 3);
      } else {
        s.hasRed = false; s.hasIr = true;
        s.ir  = be24_(ppg_carry_ + 0);
      }
      ppgPush_(s);
      ppg_carry_n_ = 0;
    }
  }

  while (len >= group) {
    PpgSample s; s.ts_ms = millis();
    if (ppg_spo2_) {
      s.hasRed = s.hasIr = true;
      s.red = be24_(data + 0);
      s.ir  = be24_(data + 3);
      data += 6; len -= 6;
    } else {
      s.hasRed = false; s.hasIr = true;
      s.ir  = be24_(data + 0);
      data += 3; len -= 3;
    }
    ppgPush_(s);
  }


  if (len) { memcpy(ppg_carry_, data, len); ppg_carry_n_ = len; }
}
