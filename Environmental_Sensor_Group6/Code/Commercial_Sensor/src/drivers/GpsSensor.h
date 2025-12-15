#pragma once
#include <Arduino.h>
#include <Adafruit_GPS.h>   // Adafruit GPS Library
#include "../models/Result.h"
#include "../hal/Buses.h"
#include "config.h"

#ifndef GPS_I2C_ADDR
  #define GPS_I2C_ADDR   GPS_ADDR    
#endif

#ifndef GPS_UPDATE_HZ
  #define GPS_UPDATE_HZ  1            // 1Hz 
#endif

#ifndef GPS_OUTPUT_SENTENCES
  #define GPS_OUTPUT_SENTENCES PMTK_SET_NMEA_OUTPUT_RMCGGA 
#endif

class GpsSensor {
public:
 
  Status begin(TwoWire& w, uint8_t addr = GPS_I2C_ADDR);

  // Non-blocking polling
  void poll();

  Status get(double& lat_deg, double& lon_deg, float& alt_m, bool& fix, double& speed) const;

  bool isReady() const { return ready_; }

private:
  Adafruit_GPS gps_{&Wire};

  bool   ready_   = false;
  bool   hasFix_  = false;
  double lastLat_ = NAN;
  double lastLon_ = NAN;
  float  lastAlt_ = NAN;
  double  speed_ = NAN;


  void configureUpdateRate_();
};
