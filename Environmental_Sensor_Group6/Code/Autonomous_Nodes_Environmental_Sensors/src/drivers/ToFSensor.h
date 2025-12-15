#pragma once
#include <Arduino.h>
#include <vl53l4cx_class.h>     // ST ULD Driver Header
#include "../models/Result.h"
#include "../hal/Buses.h"
#include "config.h"

// Can be overridden in config.h
#ifndef TOF_TIMING_BUDGET_MS
  #define TOF_TIMING_BUDGET_MS 20   // Timing budget (affects accuracy/speed)
#endif
#ifndef TOF_INTERMEAS_MS
  #define TOF_INTERMEAS_MS     20   // Inter-measurement period (must be >= timing budget)
#endif
#ifndef TOF_TIMEOUT_MS
  #define TOF_TIMEOUT_MS       30   // Read timeout (prevents blocking main loop)
#endif

class ToFSensor {
public:
  // xshutPin: -1 if not connected.
  // intPin: -1 to disable interrupt (fallback to polling).
  Status begin(TwoWire& w, uint8_t i2cAddr, int8_t xshutPin = -1, int8_t intPin = -1);

  // Non-blocking read: Output mm if ready. Returns Err if timeout or not ready.
  Status read(float& dist_mm);

  // Optional: Stop/Start ranging (Useful for sleep/power saving or reconfiguration)
  Status stop();
  Status start();

  bool isReady() const { return ready_; }

  // === Forwarding interfaces matching ST examples ===
  int InitSensor(uint8_t newAddr);
  int VL53L4CX_Off();
  int VL53L4CX_StartMeasurement();
  int VL53L4CX_StopMeasurement();
  int VL53L4CX_ClearInterruptAndStartMeasurement();
  int VL53L4CX_GetMeasurementDataReady(uint8_t* isReady);
  int VL53L4CX_GetMultiRangingData(VL53L4CX_MultiRangingData_t* data);

  // Common configuration interfaces
  int VL53L4CX_SetDistanceMode(uint8_t mode); // e.g. VL53L4CX_DISTANCEMODE_MEDIUM
  int VL53L4CX_SetMeasurementTimingBudgetMicroSeconds(uint32_t us);

private:
  VL53L4CX* tof_ = nullptr;     // Dynamically allocated to inject specific TwoWire
  uint8_t   addr_ = 0x29;
  int8_t    xshut_ = -1;
  int8_t    intPin_ = -1;
  bool      ready_ = false;

  // Interrupt flag (FALLING trigger); ESP32 ISR requires IRAM_ATTR
  volatile bool intFired_ = false;
  static void IRAM_ATTR isrThunk_(void* arg);  // For attachInterrupt
  void IRAM_ATTR onInt_();                     // Actual setter function

  // Internal helpers
  void hwReset_();
  Status configureRanging_();
};