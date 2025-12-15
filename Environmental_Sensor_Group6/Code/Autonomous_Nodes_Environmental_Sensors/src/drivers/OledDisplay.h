#pragma once
#include <Arduino.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "../models/DataRecord.h"
#include "../models/Result.h"
#include "../hal/Buses.h"
#include "config.h"

// Defaults can be overridden in config.h
#ifndef OLED_TEXT_SIZE
  #define OLED_TEXT_SIZE 2
#endif
#ifndef OLED_CONTRAST
  #define OLED_CONTRAST 0xCF   // 0x00~0xFF
#endif
#ifndef OLED_PRE_INIT_DELAY_MS
  #define OLED_PRE_INIT_DELAY_MS 0
#endif
#ifndef OLED_POST_BEGIN_DELAY_MS
  #define OLED_POST_BEGIN_DELAY_MS 0
#endif

static const uint32_t PAGE_MS = 10000; 

class OledDisplay {
public:
  // rstPin = -1 means no hardware reset pin
  Status begin(TwoWire& w, uint8_t addr, int8_t rstPin = -1);

  // Render a single frame (Simple 4-line example)
  void render(const DataRecord& r);

  void drawOledPaged(const DataRecord& r);

  // Simple error message
  void showError(const String& msg);

  // Rotate screen 180 degrees
  void Rotate180();

  // Optional: Adjust contrast
  void setContrast(uint8_t value);

  bool isReady() const { return ready_; }

private:
  // Dynamically allocated to pass &w; avoids binding to global Wire
  Adafruit_SSD1306* oled_ = nullptr;
  uint8_t addr_ = 0x3C;
  int8_t  rst_  = -1;
  bool    ready_ = false;

  void hardReset_();
  bool tryBegin_(uint8_t vccMode);
  void drawBootFrame_();
};