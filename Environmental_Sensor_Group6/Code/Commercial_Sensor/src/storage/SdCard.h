#pragma once
#include <Arduino.h>
#include <SD.h>
#include "../models/Result.h"   
#include "../hal/Buses.h"       

const  uint32_t SD_PERIOD_MS = 5000;  // 5 s

class SdCard {
public:
  // After power-on, perform low-speed initialization, then switch to the operating frequency.
  Status begin(uint8_t csPin, uint32_t initHz = 0, uint32_t workHz = 0);

  void end();


  bool   exists(const char* path) const;
  bool   mkdirs(const char* path) const;  

  // Opening a file (Append/Write) is the responsibility of the caller
  Result<File> openAppend(const char* path) const;
  Result<File> openWrite(const char* path, bool truncate = false) const;

  bool mounted() const { return mounted_; }

private:
  uint8_t cs_ = 255;
  bool    mounted_ = false;
  bool mkdirs_(const String& path) const;
};
