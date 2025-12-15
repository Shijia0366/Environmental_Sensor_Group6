#pragma once
#include <Arduino.h>
#include <Adafruit_BME680.h>
#include "../models/Result.h"   // Status definition
#include "../hal/Buses.h"       // TwoWire reference
#include "config.h"             // BME_ADDR etc.

// Apply defaults if not configured in config.h
#ifndef BME688_TEMP_OS
  #define BME688_TEMP_OS BME680_OS_8X
#endif
#ifndef BME688_HUM_OS
  #define BME688_HUM_OS  BME680_OS_2X
#endif
#ifndef BME688_PRES_OS
  #define BME688_PRES_OS BME680_OS_4X
#endif
#ifndef BME688_IIR_SIZE
  #define BME688_IIR_SIZE BME680_FILTER_SIZE_3
#endif
#ifndef BME688_HEATER_TEMP_C
  #define BME688_HEATER_TEMP_C 320   // °C
#endif
#ifndef BME688_HEATER_MS
  #define BME688_HEATER_MS  150      // ms
#endif

class BME688Sensor {
public:
  // Initialize with I2C bus and address
  Status begin(TwoWire& w, uint8_t addr);

  // Read measurement results. Returns Status::Ok() on success.
  // Note: Pressure is returned in hPa (Library returns Pa / 100)
  //       Gas is returned in Ohms
  Status read(float& temp_c, float& hum_pct, float& press_hpa, float& gas_ohm);

  bool isReady() const { return ready_; }

private:
  Adafruit_BME680 bme;
  bool ready_ = false;
};