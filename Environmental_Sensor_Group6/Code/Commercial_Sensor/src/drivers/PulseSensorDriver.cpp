#include "PulseSensorDriver.h"

// Define the minimum reporting interval (ms).
#ifndef PULSE_REPORT_PERIOD_MS
  #define PULSE_REPORT_PERIOD_MS 20
#endif

Status PulseSensorDriver::begin(uint8_t pin) {
    pulseSensor.analogInput(pin);
    pulseSensor.setThreshold(PULSE_THRESHOLD); 

    if (pulseSensor.begin()) {
        ready_ = true;
        return Status::Ok();
    } else {
        ready_ = false;
        return Status::Err("PulseSensor init failed");
    }
}

Status PulseSensorDriver::read(PulseData& data) {
    if (!ready_) return Status::Err("PulseSensor not init");


    if (pulseSensor.sawNewSample()) {
        // keep library running
    }

    //  Non-blocking flow control
    const uint32_t now = millis();
    static uint32_t next_report_ms = 0;

    if ((int32_t)(now - next_report_ms) < 0) {
        return Status::Err(""); 
    }

    // Acquire signal
    int currentSignal = pulseSensor.getLatestSample();
    data.signal = currentSignal; 
    static uint32_t last_finger_detected_time = 0; 
    const uint32_t FINGER_TIMEOUT_MS = 2000;       

    if (currentSignal > 3000) {
// --- Situation A: Strong signal (peak) ---
// This indicates the finger is definitely there, update the timestamp.
        last_finger_detected_time = now;
        
        // Normal data reading
        data.bpm  = pulseSensor.getBeatsPerMinute();
        data.beat = pulseSensor.sawStartOfBeat();
    } 
    else {
       // --- Situation B: Weak signal (may be a low point in the signal, or the finger may have been removed) ---
       // Check: How long has it been since the last time the signal was strong
        if (now - last_finger_detected_time > FINGER_TIMEOUT_MS) {
            // No signal for more than 2 seconds -> The finger has really been removed.
            data.bpm  = 0;
            data.beat = false;
        } else {
            data.bpm  = pulseSensor.getBeatsPerMinute(); 
            data.beat = false; 
        }
    }

    next_report_ms = now + PULSE_REPORT_PERIOD_MS;

    return Status::Ok();
}