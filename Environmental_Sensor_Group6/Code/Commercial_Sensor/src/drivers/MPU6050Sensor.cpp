#include "MPU6050Sensor.h"

Status MPU6050Sensor::begin(TwoWire& w, uint8_t addr) {
    if (!mpu.begin(addr, &w)) {
        ready_ = false;
        return Status::Err("MPU6050 not found");
    }

    mpu.setAccelerometerRange(MPU_ACCEL_RANGE);
    mpu.setGyroRange(MPU_GYRO_RANGE);
    mpu.setFilterBandwidth(MPU_FILTER_BW);

// After successful initialization, a calibration will be performed automatically.
    calibrate();

    ready_ = true;
    return Status::Ok();
}

//Calibration function implementation
void MPU6050Sensor::calibrate() {
    Serial.println("Calibrating MPU6050... Keep it still and flat!");
    
    float sum_ax=0, sum_ay=0, sum_az=0;
    float sum_gx=0, sum_gy=0, sum_gz=0;
    const int num_samples = 100;

    // Read data 100 times and calculate the average.
    for(int i=0; i<num_samples; i++) {
        sensors_event_t a, g, temp;
        mpu.getEvent(&a, &g, &temp);
        
        sum_ax += a.acceleration.x;
        sum_ay += a.acceleration.y;
        sum_az += a.acceleration.z;
        sum_gx += g.gyro.x;
        sum_gy += g.gyro.y;
        sum_gz += g.gyro.z;
        delay(5);
    }

    // Calculate the deviation
    offset_ax_ = sum_ax / num_samples;
    offset_ay_ = sum_ay / num_samples;
    offset_az_ = (sum_az / num_samples) - 9.81;
    // gyroscope
    offset_gx_ = sum_gx / num_samples;
    offset_gy_ = sum_gy / num_samples;
    offset_gz_ = sum_gz / num_samples;

    Serial.printf("Offsets -> Acc: %.2f, %.2f, %.2f\n", offset_ax_, offset_ay_, offset_az_);
}

Status MPU6050Sensor::read(MPUData& data) {
    if (!ready_) return Status::Err("MPU6050 not init");

    const uint32_t now = millis();
    static uint32_t next_read_ms = 0;
    
    if ((int32_t)(now - next_read_ms) < 0) {
        return Status::Err(""); 
    }

    sensors_event_t a, g, temp;
    
    if (mpu.getEvent(&a, &g, &temp)) {
        data.ax = a.acceleration.x - offset_ax_;
        data.ay = a.acceleration.y - offset_ay_;
        data.az = a.acceleration.z - offset_az_;
        
        data.gx = g.gyro.x - offset_gx_;
        data.gy = g.gyro.y - offset_gy_;
        data.gz = g.gyro.z - offset_gz_;
        
        data.temp = temp.temperature;

        next_read_ms = now + MPU_MIN_PERIOD_MS;
        return Status::Ok();
    } else {
        return Status::Err("MPU getEvent failed");
    }
}