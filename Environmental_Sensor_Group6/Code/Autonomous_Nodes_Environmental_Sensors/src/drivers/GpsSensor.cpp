#include "GpsSensor.h"

Status GpsSensor::begin(TwoWire& w, uint8_t addr) {
  // Bind internal GPS object to the specific Wire
  gps_ = Adafruit_GPS(&w);

  // I2C Init (Adafruit_GPS supports I2C begin(addr))
  if (!gps_.begin(addr)) {
    ready_ = false;
    return Status::Err("GPS I2C init failed");
  }

  // Select output sentences (RMC+GGA)
  gps_.sendCommand(GPS_OUTPUT_SENTENCES);

  // Set update rate
  configureUpdateRate_();

  ready_   = true;
  hasFix_  = false;
  lastLat_ = NAN;
  lastLon_ = NAN;
  lastAlt_ = NAN;

  return Status::Ok();
}

void GpsSensor::configureUpdateRate_() {
  // NMEA update frequency (must match position fix rate)
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

  // Non-blocking read: read multiple bytes to drain buffer faster
  for (int i = 0; i < 32; ++i) {   
    gps_.read();
  }

  // Parse if a complete NMEA sentence is received
  if (gps_.newNMEAreceived()) {
    // Discard frame if parsing fails
    if (!gps_.parse(gps_.lastNMEA())) return;

    // Update cache
    hasFix_ = gps_.fix;
    if (hasFix_) {
      lastLat_ = gps_.latitudeDegrees;        // degrees
      lastLon_ = gps_.longitudeDegrees;       // degrees
      lastAlt_ = gps_.altitude;               // meters
      
      // Note: Original logic preserved. 
      // gps_.speed is in Knots. 
      // 1 Knot = 0.514444 m/s
      // 1 Knot = 1.852 km/h
      speed_   = gps_.speed * 0.514444f * 1.852f; 
    }
  }
}

Status GpsSensor::get(double& lat_deg, double& lon_deg, float& alt_m, bool& fix, double& speed) const {
  if (!ready_) return Status::Err("GPS not initialized");

  fix     = hasFix_;
  lat_deg = lastLat_;
  lon_deg = lastLon_;
  alt_m   = lastAlt_;
  speed   = speed_;
  return Status::Ok();
}