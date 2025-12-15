#pragma once
#include <Arduino.h>
#include <vl53l4cx_class.h>    
#include "../models/Result.h"
#include "../hal/Buses.h"
#include "config.h"

#ifndef TOF_TIMING_BUDGET_MS
  #define TOF_TIMING_BUDGET_MS 20   // Distance measurement time budget
#endif
#ifndef TOF_INTERMEAS_MS
  #define TOF_INTERMEAS_MS     20   //Frame interval (must be >= time budget)
#endif
#ifndef TOF_TIMEOUT_MS
  #define TOF_TIMEOUT_MS       30   // Read wait timeout (to avoid blocking the main loop)
#endif

class ToFSensor {
public:
  //`xshutPin` can be -1 to indicate no connection; `intPin` can be -1 to indicate no interruption (backoff polling).
  Status begin(TwoWire& w, uint8_t i2cAddr, int8_t xshutPin = -1, int8_t intPin = -1);

  // Non-blocking read: Outputs the distance in mm
  Status read(float& dist_mm);


  Status stop();
  Status start();

  bool isReady() const { return ready_; }

  int InitSensor(uint8_t newAddr);
  int VL53L4CX_Off();
  int VL53L4CX_StartMeasurement();
  int VL53L4CX_StopMeasurement();
  int VL53L4CX_ClearInterruptAndStartMeasurement();
  int VL53L4CX_GetMeasurementDataReady(uint8_t* isReady);
  int VL53L4CX_GetMultiRangingData(VL53L4CX_MultiRangingData_t* data);

  int VL53L4CX_SetDistanceMode(uint8_t mode); 
  int VL53L4CX_SetMeasurementTimingBudgetMicroSeconds(uint32_t us);


private:
  VL53L4CX* tof_ = nullptr;     // Dynamic allocation
  uint8_t   addr_ = 0x29;
  int8_t    xshut_ = -1;
  int8_t    intPin_ = -1;
  bool      ready_ = false;

  // Interruption flag
  volatile bool intFired_ = false;
  static void IRAM_ATTR isrThunk_(void* arg);  // attachInterrupt
  void IRAM_ATTR onInt_();                     

  void hwReset_();
  Status configureRanging_();
};
