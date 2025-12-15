#pragma once
#include <Arduino.h>
#include <Adafruit_GPS.h>   // Adafruit GPS Library
#include "../models/Result.h"
#include "../hal/Buses.h"
#include "config.h"

// Defaults can be overridden in config.h
#ifndef GPS_I2C_ADDR
  #define GPS_I2C_ADDR   GPS_ADDR
#endif

#ifndef GPS_UPDATE_HZ
  #define GPS_UPDATE_HZ  1            // 1Hz Update (Stable)
#endif

#ifndef GPS_OUTPUT_SENTENCES
  #define GPS_OUTPUT_SENTENCES PMTK_SET_NMEA_OUTPUT_RMCGGA // RMC+GGA only (Efficient)
#endif

class GpsSensor {
public:
  // Initialize (I2C)
  Status begin(TwoWire& w, uint8_t addr = GPS_I2C_ADDR);

  // Non-blocking poll: Call in every loop iteration
  void poll();

  // Get latest cached data (fix=false means no lock)
  Status get(double& lat_deg, double& lon_deg, float& alt_m, bool& fix, double& speed) const;

  bool isReady() const { return ready_; }

private:
  // Note: Initialized with I2C wire
  Adafruit_GPS gps_{&Wire};

  bool   ready_   = false;
  bool   hasFix_  = false;
  double lastLat_ = NAN;
  double lastLon_ = NAN;
  float  lastAlt_ = NAN;
  double speed_   = NAN;

  // Convert Hz macro to PMTK command
  void configureUpdateRate_();
};