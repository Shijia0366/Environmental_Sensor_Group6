#include "GpsSensor.h"

Status GpsSensor::begin(TwoWire& w, uint8_t addr) {
  // Bind the internal GPS object to the specified Wire
  gps_ = Adafruit_GPS(&w);

  //I2C initialization
  if (!gps_.begin(addr)) {
    ready_ = false;
    return Status::Err("GPS I2C init failed");
  }

  gps_.sendCommand(GPS_OUTPUT_SENTENCES);

  configureUpdateRate_();

  ready_  = true;
  hasFix_ = false;
  lastLat_ = NAN;
  lastLon_ = NAN;
  lastAlt_ = NAN;

  return Status::Ok();
}

void GpsSensor::configureUpdateRate_() {
  switch (GPS_UPDATE_HZ) {
    case 10: gps_.sendCommand(PMTK_SET_NMEA_UPDATE_10HZ); break;
    case 5:  gps_.sendCommand(PMTK_SET_NMEA_UPDATE_5HZ);  break;
    case 2:  gps_.sendCommand(PMTK_SET_NMEA_UPDATE_2HZ);  break;
    case 1:  gps_.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);  break;
    default: gps_.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);  break;
  }
}

void GpsSensor::poll() {
  if (!ready_) return;

  // Non-blocking read
  for (int i = 0; i < 32; ++i) {   
    gps_.read();
  }

  if (gps_.newNMEAreceived()) {
  
    if (!gps_.parse(gps_.lastNMEA())) return;

    // refresh cache
    hasFix_ = gps_.fix;                         // Is there a location
    if (hasFix_) {
      // Adafruit_GPS 提供 latitude/longitude（degrees）与 altitude (m)
      lastLat_ = gps_.latitudeDegrees;                  // degrees
      lastLon_ = gps_.longitudeDegrees;                 // degrees
      lastAlt_ = gps_.altitude;                  // meters (from GGA)
      speed_   = gps_.speed *0.514444f *1.852f;
    }
  }
}

Status GpsSensor::get(double& lat_deg, double& lon_deg, float& alt_m, bool& fix, double& speed) const {
  if (!ready_) return Status::Err("GPS not initialized");

  fix    = hasFix_;
  lat_deg = lastLat_;
  lon_deg = lastLon_;
  alt_m  = lastAlt_;
  speed  = speed_;
  return Status::Ok();
}
