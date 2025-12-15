#pragma once
#include <Arduino.h>
#include <SD.h>
#include "../models/Result.h"   // Provides Status/Result<T>
#include "../hal/Buses.h"       // Provides HAL::getSPI()

const  uint32_t SD_PERIOD_MS = 10000;  // 10 seconds

class SdCard {
public:
  // Initialize at low speed, then switch to working frequency (params from config.h)
  Status begin(uint8_t csPin, uint32_t initHz = 0, uint32_t workHz = 0);

  // Optional: Unmount/Cleanup
  void end();

  // Simple File/Directory Utilities
  bool   exists(const char* path) const;
  bool   mkdirs(const char* path) const;   // Recursive create

  // Open file (Append/Write), caller is responsible for close()
  Result<File> openAppend(const char* path) const;
  Result<File> openWrite(const char* path, bool truncate = false) const;

  // Check if card is mounted
  bool mounted() const { return mounted_; }

private:
  uint8_t cs_ = 255;
  bool    mounted_ = false;

  // Internal implementation for recursive mkdir
  bool mkdirs_(const String& path) const;
};