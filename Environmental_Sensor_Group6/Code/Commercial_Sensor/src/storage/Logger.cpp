#include "Logger.h"
#include "../include/config.h"  // CSV_FILE_PATH / CSV_HEADER / LOG_FLUSH_INTERVAL_MS

Status Logger::begin(SdCard& card, const char* csvPath, const char* header) {
  card_    = &card;
  mounted_ = card.mounted();
  if (!mounted_) return Status::Err("SD not mounted");

  path_ = csvPath;

  // Ensure the directory exists and open it in append mode.
  auto res = card.openAppend(path_.c_str());
  if (!res.ok) return Status::Err(res.err);

  file_ = res.value;

  // If the file is empty, write the table header.
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

// Directly call the built-in toCSV() function of DataRecord
String Logger::formatCSV_(const DataRecord& r) {
  return r.toCSV();
}

Status Logger::appendCSV(const DataRecord& r) {
  if (!ready()) return Status::Err("logger not ready");

  // Get the complete CSV lines
  String line = r.toCSV();

  if (file_.println(line)) {
    return Status::Ok();
  } else {
    return Status::Err("file write failed");
  }
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