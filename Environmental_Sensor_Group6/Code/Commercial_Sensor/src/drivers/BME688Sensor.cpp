#include "BME688Sensor.h"

Status BME688Sensor::begin(TwoWire& w, uint8_t addr) {
  if (!bme.begin(addr, &w)) {
    ready_ = false;
    return Status::Err("BME688 not found");
  }

  // Oversampling and Filtering Configuration
  bme.setTemperatureOversampling(BME688_TEMP_OS);
  bme.setHumidityOversampling(BME688_HUM_OS);
  bme.setPressureOversampling(BME688_PRES_OS);
  bme.setIIRFilterSize(BME688_IIR_SIZE);
  bme.setGasHeater(BME688_HEATER_TEMP_C, BME688_HEATER_MS);

  ready_ = true;
  return Status::Ok();
}

Status BME688Sensor::read(float& t, float& h, float& p_hpa, float& gas_ohm) {
  if (!ready_) return Status::Err("BME688 not initialized");

  const uint32_t now = millis();


  enum { IDLE, MEASURING };
  static uint8_t  st              = IDLE;
  static uint32_t ready_at_ms     = 0;     
  static uint32_t next_allowed_ms = 0;    
  static uint32_t backoff_ms      = 0;     

  // Read frequency
  const uint32_t MIN_PERIOD_MS = 250;

  if ((int32_t)(now - next_allowed_ms) < 0) {
    return Status::Err("");  
  }

  if (st == IDLE) {
    // ——Start sampling (non-blocking), return to "Waiting" —— 
    uint32_t t_done = bme.beginReading();   
    if (t_done == 0) {
      // Startup failed: Exponential backoff to avoid frequent bus occupation.
      backoff_ms = (backoff_ms == 0) ? 1000 : (backoff_ms < 4000 ? backoff_ms * 2 : 4000);
      next_allowed_ms = now + backoff_ms;
      return Status::Err("BME688 beginReading() failed");
    }
    ready_at_ms     = t_done;
    st              = MEASURING;
    next_allowed_ms = now + MIN_PERIOD_MS;  // After successful startup, the earliest next startup time
    return Status::Err("");                 // Sampling in progress: No new data available at the moment.
  }


  if ((int32_t)(now - ready_at_ms) < 0) {
    return Status::Err("");                 
  }

  
  if (bme.endReading()) {
    t       = bme.temperature;              // °C
    h       = bme.humidity;                 // %RH
    p_hpa   = bme.pressure / 100.0f;        // Pa -> hPa
    gas_ohm = bme.gas_resistance;           // Ω

    st = IDLE;
    backoff_ms = 0;                         
    return Status::Ok();
  } else {
// Data retrieval failed: Set backoff and return to idle state, waiting for the next start.
    st = IDLE;
    backoff_ms = (backoff_ms == 0) ? 1000 : (backoff_ms < 4000 ? backoff_ms * 2 : 4000);
    next_allowed_ms = now + backoff_ms;
    return Status::Err("BME688 endReading() failed");
  }
}