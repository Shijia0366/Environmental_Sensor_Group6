#pragma once
#include <Arduino.h>
#include <PulseSensorPlayground.h> 
#include "../models/Result.h"
#include "config.h"

class PulseSensorDriver {
public:
 
    struct PulseData {
        int signal; 
        int bpm;   
        bool beat;  
    };

    Status begin(uint8_t pin = PULSE_SENSOR_PIN);

    Status read(PulseData& data);

private:
    PulseSensorPlayground pulseSensor;
    bool ready_ = false;
};