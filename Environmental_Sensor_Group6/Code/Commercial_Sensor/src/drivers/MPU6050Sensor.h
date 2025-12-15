#pragma once
#include <Arduino.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include "../models/Result.h"
#include "../hal/Buses.h"
#include "config.h"

#ifndef MPU_I2C_ADDR
  #define MPU_I2C_ADDR MPU6050_I2CADDR_DEFAULT
#endif
#ifndef MPU_ACCEL_RANGE
  #define MPU_ACCEL_RANGE MPU6050_RANGE_8_G
#endif
#ifndef MPU_GYRO_RANGE
  #define MPU_GYRO_RANGE MPU6050_RANGE_500_DEG
#endif
#ifndef MPU_FILTER_BW
  #define MPU_FILTER_BW MPU6050_BAND_5_HZ
#endif
#ifndef MPU_MIN_PERIOD_MS
  #define MPU_MIN_PERIOD_MS 20
#endif

class MPU6050Sensor {
public:
    struct MPUData {
        float ax, ay, az; // m/s^2
        float gx, gy, gz; // rad/s
        float temp;       // degC
    };

    Status begin(TwoWire& w, uint8_t addr = MPU_I2C_ADDR);
    Status read(MPUData& data);
    
    // Calibration function
    void calibrate(); 

    bool isReady() const { return ready_; }

private:
    Adafruit_MPU6050 mpu;
    bool ready_ = false;

    // Used to store deviation values
    float offset_ax_ = 0;
    float offset_ay_ = 0;
    float offset_az_ = 0;
    float offset_gx_ = 0;
    float offset_gy_ = 0;
    float offset_gz_ = 0;
};