#pragma once
#include <Arduino.h>
#include <SD.h>
#include "../models/Result.h"
#include "../models/DataRecord.h"
#include "../storage/SdCard.h"

#ifndef LOG_FLUSH_INTERVAL_MS
  #define LOG_FLUSH_INTERVAL_MS 2000
#endif

class Logger {
public:
  Status begin(SdCard& card, const char* csvPath, const char* header);

  Status appendCSV(const DataRecord& r);
 
  void flushIfNeeded();

  void end();

  bool ready() const { return file_ && mounted_; }

private:
  SdCard* card_ = nullptr;
  File    file_;
  String  path_;
  bool    mounted_ = false;

  unsigned long lastFlushMs_ = 0;

  void writeHeaderIfEmpty_();   
  static String formatCSV_(const DataRecord& r);
};
