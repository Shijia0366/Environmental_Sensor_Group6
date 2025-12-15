#pragma once  
#include <Arduino.h>

struct DataRecord {
  uint32_t timestamp_ms = 0;

  // BME688
  float temp_c    = NAN;
  float hum_pct   = NAN;
  float press_hpa = NAN;
  float gas_ohm   = NAN;
  int AQI = 0;

  // ToF
  float dist_mm   = NAN;

  // GPS
  double lat_deg = NAN;
  double lon_deg = NAN;
  float  alt_m = NAN;
  bool   gps_fix = false;
  float  speed = NAN;

  // IMU (Acc + Gyro)
  float acc_x = NAN;
  float acc_y = NAN;
  float acc_z = NAN;
  float gyro_x = NAN;
  float gyro_y = NAN;
  float gyro_z = NAN;

  // Pulse Sensor
  int signal = 0;  
  int bpm    = 0; 
 

  String toCSV() const {
    String s;
    s.reserve(180);
    s += String(timestamp_ms); s += ",";
    s += String(temp_c, 2);    s += ",";
    s += String(hum_pct, 2);   s += ",";
    s += String(press_hpa, 2); s += ",";
    s += String(gas_ohm, 0);   s += ",";
    s += String(AQI);          s += ",";
    s += String(dist_mm, 1);   s += ",";
    s += String(speed, 1);     s += ",";
    s += String(lat_deg, 7);   s += ",";
    s += String(lon_deg, 7);   s += ",";
    s += String(alt_m, 1);     s += ",";
    s += (gps_fix ? "1" : "0");s += ",";
    
    s += String(acc_x, 2);     s += ",";
    s += String(acc_y, 2);     s += ",";
    s += String(acc_z, 2);     s += ",";
    s += String(gyro_x, 2);    s += ",";
    s += String(gyro_y, 2);    s += ",";
    s += String(gyro_z, 2);    s += ",";
    s += String(signal);       s += ",";
    s += String(bpm);          
    
    return s;
  }
};