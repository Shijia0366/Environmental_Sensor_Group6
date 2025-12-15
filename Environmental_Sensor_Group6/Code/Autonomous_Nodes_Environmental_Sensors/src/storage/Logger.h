#pragma once
#include <Arduino.h>
#include <SD.h>
#include "../models/Result.h"
#include "../models/DataRecord.h"
#include "../storage/SdCard.h"

// Default if not defined in config.h
#ifndef LOG_FLUSH_INTERVAL_MS
  #define LOG_FLUSH_INTERVAL_MS 2000
#endif

class Logger {
public:
  // Depends on SdCard being successfully mounted
  Status begin(SdCard& card, const char* csvPath, const char* header);

  // Append a CSV row (Non-blocking; formats and writes once)
  Status appendCSV(const DataRecord& r);

  // Periodic flush (Reduces data loss risk on power fail)
  void flushIfNeeded();

  // Explicitly close
  void end();

  bool ready() const { return file_ && mounted_; }

private:
  SdCard* card_ = nullptr;
  File    file_;
  String  path_;
  bool    mounted_ = false;

  unsigned long lastFlushMs_ = 0;

  void writeHeaderIfEmpty_();   // Write header if file is empty
};