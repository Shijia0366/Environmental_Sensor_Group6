
#pragma once

#include <Arduino.h>
#include "../models/DataRecord.h"

void WifiIf_beginAP(const char* ssid = nullptr,
                    const char* password = nullptr);

void WifiIf_setRecord(const DataRecord& rec);

void WifiIf_loop();

// Returns whether the AP/HTTP server has started successfully.
bool WifiIf_ready();
