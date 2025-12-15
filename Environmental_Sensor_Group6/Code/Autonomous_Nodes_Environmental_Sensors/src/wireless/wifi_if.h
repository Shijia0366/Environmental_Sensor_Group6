// src/wireless/wifi_if.h
#pragma once

#include <Arduino.h>
#include "../models/DataRecord.h"

// Start AP Mode + HTTP Server
// Uses default macros WIFI_AP_SSID / WIFI_AP_PASSWORD if arguments are null
void WifiIf_beginAP(const char* ssid = nullptr,
                    const char* password = nullptr);

// Update the latest data record (Call after new DataRecord is acquired)
// Note: Call in normal loop/task, do not call in ISR
void WifiIf_setRecord(const DataRecord& rec);

// Call frequently in app_loop() (Non-blocking)
// Internally calls WebServer::handleClient()
void WifiIf_loop();

// Returns if AP / HTTP server has successfully started
bool WifiIf_ready();