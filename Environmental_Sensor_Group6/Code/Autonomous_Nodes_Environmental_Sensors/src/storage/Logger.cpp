#include "Logger.h"
#include "../include/config.h"  // CSV_FILE_PATH / CSV_HEADER / LOG_FLUSH_INTERVAL_MS

Status Logger::begin(SdCard& card, const char* csvPath, const char* header) {
  card_    = &card;
  mounted_ = card.mounted();
  if (!mounted_) return Status::Err("SD not mounted");

  path_ = csvPath;

  // Ensure directory exists and open in append mode (keep open)
  auto res = card.openAppend(path_.c_str());
  if (!res.ok) return Status::Err(res.err);

  file_ = res.value;

  // Write header if file is empty
  writeHeaderIfEmpty_();

  lastFlushMs_ = millis();
  return Status::Ok();
}

void Logger::writeHeaderIfEmpty_() {
  if (!file_) return;
  if (file_.size() == 0) {
    file_.println(CSV_HEADER);
    file_.flush();
  }
}

Status Logger::appendCSV(const DataRecord& r) {
  if (!ready()) return Status::Err("logger not ready");

  char buf[200];   // Fixed stack buffer, avoid heap allocation
  int n = snprintf(buf, sizeof(buf),
    "%lu,%.2f,%.1f,%.1f,%.0f,%d,%.1f,%.1f,%.7f,%.7f,%.1f,%d,%.1f,%.1f,%.1f,%d,%d",
    r.timestamp_ms,
    r.temp_c, r.hum_pct, r.press_hpa, r.gas_ohm, r.AQI,
    r.dist_mm, r.speed,
    r.lat_deg, r.lon_deg, r.alt_m, r.gps_fix ? 1 : 0,
    r.acc_x, r.acc_y, r.acc_z,
    r.HR, r.SPO2
  );

  if (n <= 0 || n >= (int)sizeof(buf)) {
    return Status::Err("snprintf overflow");
  }
  if (file_.println(buf)) return Status::Ok();
  return Status::Err("file write failed");
}


void Logger::flushIfNeeded() {
  if (!file_) return;
  const unsigned long now = millis();
  if (now - lastFlushMs_ >= LOG_FLUSH_INTERVAL_MS) {
    file_.flush();
    lastFlushMs_ = now;
  }
}

void Logger::end() {
  if (file_) {
    file_.flush();
    file_.close();
  }
  file_ = File();
  mounted_ = false;
}