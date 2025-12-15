#include "ToFSensor.h"

Status ToFSensor::begin(TwoWire& w, uint8_t i2cAddr, int8_t xshutPin, int8_t intPin) {
  addr_  = i2cAddr;
  xshut_ = xshutPin;
  intPin_= intPin;

  // XSHUT Hardware Reset (Optional)
  hwReset_();

  // Dynamically create VL53L4CX, passing TwoWire and XSHUT
  if (tof_) { delete tof_; tof_ = nullptr; }
  tof_ = new VL53L4CX(&w, xshut_);

  // Power on sequence (as per standard flow)
  tof_->begin();
  tof_->VL53L4CX_Off();

  // Initialize Sensor with address
  if (tof_->InitSensor(addr_) != 0) {
    ready_ = false;
    return Status::Err("ToF InitSensor failed");
  }

  // Configure ranging parameters and mode
  if (!configureRanging_().ok) {
    ready_ = false;
    return Status::Err("ToF configure failed");
  }

  // Interrupt Setup: GPIO1 output data ready (Low Active)
  if (intPin_ >= 0) {
    // pinMode(intPin_, INPUT_PULLUP);   // Common default
    pinMode(intPin_, INPUT); 
    intFired_ = false;
    attachInterruptArg(intPin_, &ToFSensor::isrThunk_, this, FALLING);
  }

  ready_ = true;
  return Status::Ok();
}

Status ToFSensor::configureRanging_() {
  int st = 0;

  // Distance Mode: SHORT / MEDIUM / LONG
  st |= tof_->VL53L4CX_SetDistanceMode(VL53L4CX_DISTANCEMODE_MEDIUM);
  if (st) return Status::Err("ToF set distance mode failed");

  // Timing Budget (us)
  st |= tof_->VL53L4CX_SetMeasurementTimingBudgetMicroSeconds(TOF_TIMING_BUDGET_MS * 1000UL);
  if (st) return Status::Err("ToF set timing budget failed");

  // Continuous Ranging
  st |= tof_->VL53L4CX_StartMeasurement();
  if (st) return Status::Err("ToF start measurement failed");

  return Status::Ok();
}


void ToFSensor::hwReset_() {
  if (xshut_ < 0) return;
  pinMode(xshut_, OUTPUT);
  digitalWrite(xshut_, LOW);   // Power Off
  delay(2);                    // tRESET >= 1ms
  digitalWrite(xshut_, HIGH);  // Power On
  delay(5);                    // Power stabilization
}

// Interrupt ISR Thunk
void IRAM_ATTR ToFSensor::isrThunk_(void* arg) {
  reinterpret_cast<ToFSensor*>(arg)->onInt_();
}
void IRAM_ATTR ToFSensor::onInt_() {
  intFired_ = true;
}

Status ToFSensor::read(float& dist_mm) {
  if (!ready_) return Status::Err("ToF not initialized");

  // Check Data Ready: Prefer Interrupt flag, then Register polling
  uint32_t t0 = millis();
  while (true) {
    bool ready = false;

    if (intPin_ >= 0) {
      // INT Low or ISR fired means ready
      if (!digitalRead(intPin_) || intFired_) {
        ready = true;
        intFired_ = false; // Consume flag
      }
    } else {
      // Poll ULD data-ready
      uint8_t newReady = 0;
      int st = tof_->VL53L4CX_GetMeasurementDataReady(&newReady);
      if (st == 0 && newReady) ready = true;
    }

    if (ready) break;
    if (millis() - t0 > TOF_TIMEOUT_MS) return Status::Err("ToF timeout");
    
    delay(1); // Avoid busy loop
  }

  // Read Multi-Object Data (Taking object #0)
  VL53L4CX_MultiRangingData_t data;
  int st = tof_->VL53L4CX_GetMultiRangingData(&data);
  if (st != 0 || data.NumberOfObjectsFound < 1) {
    // Clear interrupt and proceed to next frame
    tof_->VL53L4CX_ClearInterruptAndStartMeasurement();
    return Status::Err("ToF invalid frame");
  }

  dist_mm = data.RangeData[0].RangeMilliMeter;

  // Clear interrupt and restart next frame (Crucial for Continuous mode)
  tof_->VL53L4CX_ClearInterruptAndStartMeasurement();

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

// === Forwarding Implementation to match ST Examples ===
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

// === Common Configuration (Optional) ===
int ToFSensor::VL53L4CX_SetDistanceMode(uint8_t mode) {
  if (!tof_) return -1;
  return tof_->VL53L4CX_SetDistanceMode(mode);
}

int ToFSensor::VL53L4CX_SetMeasurementTimingBudgetMicroSeconds(uint32_t us) {
  if (!tof_) return -1;
  return tof_->VL53L4CX_SetMeasurementTimingBudgetMicroSeconds(us);
}