#include "ToFSensor.h"

Status ToFSensor::begin(TwoWire& w, uint8_t i2cAddr, int8_t xshutPin, int8_t intPin) {
  addr_  = i2cAddr;
  xshut_ = xshutPin;
  intPin_= intPin;

  hwReset_();
  if (tof_) { delete tof_; tof_ = nullptr; }
  tof_ = new VL53L4CX(&w, xshut_);

  // Power on → Power off
  tof_->begin();
  tof_->VL53L4CX_Off();

  // Initialization address
  if (tof_->InitSensor(addr_) != 0) {
    ready_ = false;
    return Status::Err("ToF InitSensor failed");
  }

  // Configure ranging parameters and modes
  if (!configureRanging_().ok) {
    ready_ = false;
    return Status::Err("ToF configure failed");
  }

  // Optional interrupts: GPIO configuration
  if (intPin_ >= 0) {
    pinMode(intPin_, INPUT); 
    intFired_ = false;
    attachInterruptArg(intPin_, &ToFSensor::isrThunk_, this, FALLING);
  }

  ready_ = true;
  return Status::Ok();
}

Status ToFSensor::configureRanging_() {
  int st = 0;

  //Distance Mode: MEDIUM (Suitable for indoor environments, balancing speed and distance)
  st |= tof_->VL53L4CX_SetDistanceMode(VL53L4CX_DISTANCEMODE_MEDIUM);
  if (st) return Status::Err("ToF set distance mode failed");

  //Time budget: determines the measurement speed
  st |= tof_->VL53L4CX_SetMeasurementTimingBudgetMicroSeconds(TOF_TIMING_BUDGET_MS * 1000UL);
  if (st) return Status::Err("ToF set timing budget failed");

  // Start continuous ranging
  st |= tof_->VL53L4CX_StartMeasurement();
  if (st) return Status::Err("ToF start measurement failed");

  return Status::Ok();
}

void ToFSensor::hwReset_() {
  if (xshut_ < 0) return;
  pinMode(xshut_, OUTPUT);
  digitalWrite(xshut_, LOW);   
  delay(2);                    
  digitalWrite(xshut_, HIGH);  
  delay(5);          }          

// Interruption of ISR bridge
void IRAM_ATTR ToFSensor::isrThunk_(void* arg) {
  reinterpret_cast<ToFSensor*>(arg)->onInt_();
}
void IRAM_ATTR ToFSensor::onInt_() {
  intFired_ = true;
}

// ==========================================================
// non-blocking
Status ToFSensor::read(float& dist_mm) {
  if (!ready_ || !tof_) return Status::Err("ToF not initialized");

  // 1. Check data ready
  uint8_t dataReady = 0;
  if (intPin_ >= 0) {
      if (digitalRead(intPin_) == LOW || intFired_) {
          dataReady = 1;
          intFired_ = false; 
      }
  } else {
      tof_->VL53L4CX_GetMeasurementDataReady(&dataReady);
  }

  if (!dataReady) return Status::Err(""); // Not ready, skip

  // 2. Read data
  VL53L4CX_MultiRangingData_t data;
  tof_->VL53L4CX_GetMultiRangingData(&data);

  // 3. Clear interrupt immediately
  tof_->VL53L4CX_ClearInterruptAndStartMeasurement();

  // 4. Process data
  // Even if no object is found, we successfully "read" that there is nothing.
  if (data.NumberOfObjectsFound > 0) {
      // Check status (0 = Valid)
      if (data.RangeData[0].RangeStatus == 0) {
          dist_mm = data.RangeData[0].RangeMilliMeter;
      } else {
          // Object found but invalid data (e.g. sigma fail), treat as out of range
          dist_mm = 0; 
      }
  } else {
      // No object found, distance is 0 (or you could set to max range e.g. 2000)
      dist_mm = 0;
  }

  // Always return Ok if we actually communicated with the sensor
  return Status::Ok();
}

Status ToFSensor::stop() {
  if (!ready_) return Status::Err("ToF not initialized");
  tof_->VL53L4CX_StopMeasurement();
  if (intPin_ >= 0) detachInterrupt(intPin_);
  return Status::Ok();
}

Status ToFSensor::start() {
  if (!ready_) return Status::Err("ToF not initialized");
  if (intPin_ >= 0) {
    intFired_ = false;
    attachInterruptArg(intPin_, &ToFSensor::isrThunk_, this, FALLING);
  }
  int st = tof_->VL53L4CX_StartMeasurement();
  if (st != 0) return Status::Err("ToF restart failed");
  return Status::Ok();
}

int ToFSensor::InitSensor(uint8_t newAddr) {
  if (!tof_) return -1;
  addr_ = newAddr;
  return tof_->InitSensor(addr_);
}

int ToFSensor::VL53L4CX_Off() {
  if (!tof_) return -1;
  tof_->VL53L4CX_Off();
  return 0;
}

int ToFSensor::VL53L4CX_StartMeasurement() {
  if (!tof_) return -1;
  return tof_->VL53L4CX_StartMeasurement();
}

int ToFSensor::VL53L4CX_StopMeasurement() {
  if (!tof_) return -1;
  return tof_->VL53L4CX_StopMeasurement();
}

int ToFSensor::VL53L4CX_ClearInterruptAndStartMeasurement() {
  if (!tof_) return -1;
  return tof_->VL53L4CX_ClearInterruptAndStartMeasurement();
}

int ToFSensor::VL53L4CX_GetMeasurementDataReady(uint8_t* isReady) {
  if (!tof_ || !isReady) return -1;
  return tof_->VL53L4CX_GetMeasurementDataReady(isReady);
}

int ToFSensor::VL53L4CX_GetMultiRangingData(VL53L4CX_MultiRangingData_t* data) {
  if (!tof_ || !data) return -1;
  return tof_->VL53L4CX_GetMultiRangingData(data);
}

int ToFSensor::VL53L4CX_SetDistanceMode(uint8_t mode) {
  if (!tof_) return -1;
  return tof_->VL53L4CX_SetDistanceMode(mode);
}

int ToFSensor::VL53L4CX_SetMeasurementTimingBudgetMicroSeconds(uint32_t us) {
  if (!tof_) return -1;
  return tof_->VL53L4CX_SetMeasurementTimingBudgetMicroSeconds(us);
}