#include "BME688Sensor.h"

Status BME688Sensor::begin(TwoWire& w, uint8_t addr) {
  // Initialize I2C using Adafruit_BME680 library
  if (!bme.begin(addr, &w)) {
    ready_ = false;
    return Status::Err("BME688 not found");
  }

  // Oversampling and filter configuration
  bme.setTemperatureOversampling(BME688_TEMP_OS);
  bme.setHumidityOversampling(BME688_HUM_OS);
  bme.setPressureOversampling(BME688_PRES_OS);
  bme.setIIRFilterSize(BME688_IIR_SIZE);

  // Gas heater settings (temp/duration), otherwise gas_resistance might read 0
  bme.setGasHeater(BME688_HEATER_TEMP_C, BME688_HEATER_MS);

  ready_ = true;
  return Status::Ok();
}

Status BME688Sensor::read(float& t, float& h, float& p_hpa, float& gas_ohm) {
  if (!ready_) return Status::Err("BME688 not initialized");

  const uint32_t now = millis();

  // —— Internal State Machine ——
  enum { IDLE, MEASURING };
  static uint8_t  st              = IDLE;
  static uint32_t ready_at_ms     = 0;     // Expected completion timestamp
  static uint32_t next_allowed_ms = 0;     // Rate limiting / Backoff target
  static uint32_t backoff_ms      = 0;     // Backoff duration after failure

  // Reading frequency (Min interval on success)
  const uint32_t MIN_PERIOD_MS = 250;

  // Check if we are allowed to try yet (Rate limiting)
  if ((int32_t)(now - next_allowed_ms) < 0) {
    return Status::Err("");  // Return empty error so caller doesn't log it
  }

  if (st == IDLE) {
    // —— Step 1: Start sampling (non-blocking) —— 
    uint32_t t_done = bme.beginReading();   // Returns completion timestamp (0 = fail)
    if (t_done == 0) {
      // Start failed: Exponential backoff to avoid bus congestion
      backoff_ms = (backoff_ms == 0) ? 1000 : (backoff_ms < 4000 ? backoff_ms * 2 : 4000);
      next_allowed_ms = now + backoff_ms;
      return Status::Err("BME688 beginReading() failed");
    }
    ready_at_ms     = t_done;
    st              = MEASURING;
    next_allowed_ms = now + MIN_PERIOD_MS;  // Earliest next start time
    return Status::Err("");                 // Sampling in progress
  }

  // st == MEASURING: Check if time is up
  if ((int32_t)(now - ready_at_ms) < 0) {
    return Status::Err("");                 // Not ready, keep waiting
  }

  // —— Step 2: Retrieve results (non-blocking) —— 
  if (bme.endReading()) {
    t       = bme.temperature;              // °C
    h       = bme.humidity;                 // %RH
    p_hpa   = bme.pressure / 100.0f;        // Pa -> hPa
    gas_ohm = bme.gas_resistance;           // Ohm

    st = IDLE;
    backoff_ms = 0;                         // Reset backoff on success
    return Status::Ok();
  } else {
    // Fetch failed: Set backoff and reset to IDLE
    st = IDLE;
    backoff_ms = (backoff_ms == 0) ? 1000 : (backoff_ms < 4000 ? backoff_ms * 2 : 4000);
    next_allowed_ms = now + backoff_ms;
    return Status::Err("BME688 endReading() failed");
  }
}