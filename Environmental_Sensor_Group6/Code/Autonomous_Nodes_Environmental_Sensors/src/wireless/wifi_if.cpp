// src/wireless/wifi_if.cpp
#include "wifi_if.h"

#include <WiFi.h>
#include <WebServer.h>
#include "config.h"   // Can define WIFI_AP_SSID / WIFI_AP_PASSWORD here
#include <math.h>     // isnan, isinf


// Allow config.h to override default SSID / Password
#ifndef WIFI_AP_SSID
  #define WIFI_AP_SSID      "ENV-S3-AP"
#endif

#ifndef WIFI_AP_PASSWORD
  #define WIFI_AP_PASSWORD  "12345678"
#endif

// ---- Internal Static Objects ----
static WebServer       s_server(80);
static bool            s_started   = false;   // Is AP + server running
static DataRecord      s_latest{};            // Latest data frame
static bool            s_hasRecord = false;   // Is there any valid data yet

// ---- Helper: Safely convert double to JSON number (NaN/Inf -> null) ----
static String safeJsonNumber(double v, int decimals) {
  if (isnan(v) || isinf(v)) {
    return "null";  // Note: No quotes, actual null in JSON
  }
  return String(v, decimals);
}

// ---- Convert DataRecord to JSON String (All fields) ----
static String buildJsonFromRecord(const DataRecord& r) {
  String s;
  s.reserve(256);

  s += '{';

  // Timestamp
  s += "\"timestamp_ms\":"; s += String(r.timestamp_ms); s += ',';

  // Environmental Sensors
  s += "\"temp_c\":";    s += safeJsonNumber(r.temp_c,   2); s += ',';
  s += "\"hum_pct\":";   s += safeJsonNumber(r.hum_pct,  1); s += ',';
  s += "\"press_hpa\":"; s += safeJsonNumber(r.press_hpa,1); s += ',';
  s += "\"gas_ohm\":";   s += safeJsonNumber(r.gas_ohm,  0); s += ',';
  s += "\"AQI\":";       s += String(r.AQI);                  s += ',';

  // ToF
  s += "\"dist_mm\":";   s += safeJsonNumber(r.dist_mm,   1); s += ',';

  // GPS
  s += "\"speed\":";     s += safeJsonNumber(r.speed,     1); s += ',';
  s += "\"lat_deg\":";   s += safeJsonNumber(r.lat_deg,   7); s += ',';
  s += "\"lon_deg\":";   s += safeJsonNumber(r.lon_deg,   7); s += ',';
  s += "\"alt_m\":";     s += safeJsonNumber(r.alt_m,     1); s += ',';

  s += "\"gps_fix\":";   s += (r.gps_fix ? "true" : "false"); s += ',';

  // Acceleration
  s += "\"acc_x\":";     s += safeJsonNumber(r.acc_x,     1); s += ',';
  s += "\"acc_y\":";     s += safeJsonNumber(r.acc_y,     1); s += ',';
  s += "\"acc_z\":";     s += safeJsonNumber(r.acc_z,     1); s += ',';

  // PPG (Integer counts, no NaN handling needed usually)
  s += "\"ppg_heart_rate\":";   s += safeJsonNumber(r.HR, 1); s += ',';
  s += "\"ppg_SPO2\":";    s += safeJsonNumber(r.SPO2, 1);

  s += '}';

  return s;
}

// ---- HTTP Handler: Root path, returns auto-refreshing page ----
static void handleRoot() {
  // Use raw string literal for HTML + JS
  String page = F(R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <meta charset="utf-8">
  <title>ENV S3 Telemetry</title>
  <style>
    body { font-family: system-ui, -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif;
           margin: 0; padding: 16px; background: #111; color: #eee; }
    h1   { font-size: 20px; margin: 0 0 8px; }
    #status { font-size: 13px; color: #aaa; margin-bottom: 8px; }
    pre  { background: #000; padding: 12px; border-radius: 8px; font-size: 13px;
           overflow-x: auto; }
    .badge { display: inline-block; padding: 2px 6px; border-radius: 999px;
             background: #333; font-size: 11px; margin-left: 6px; }
  </style>
</head>
<body>
  <h1>Environmental Sensor (ESP32-S3) <span class="badge">Live 2s</span></h1>
  <div id="status">Connecting...</div>
  <pre id="data">{ }</pre>

  <script>
    async function update() {
      const statusEl = document.getElementById('status');
      const dataEl   = document.getElementById('data');
      try {
        // Add timestamp to prevent caching
        const res = await fetch('/json?ts=' + Date.now());
        if (!res.ok) {
          throw new Error('HTTP ' + res.status);
        }
        const json = await res.json();
        dataEl.textContent = JSON.stringify(json, null, 2);
        const now = new Date();
        statusEl.textContent = 'Last update: ' + now.toLocaleTimeString();
      } catch (e) {
        statusEl.textContent = 'Error: ' + e;
      }
    }

    // Update immediately
    update();
    // Update every 200ms
    setInterval(update, 200);
  </script>
</body>
</html>
)rawliteral");

  s_server.send(200, "text/html", page);
}

// ---- HTTP Handler: /json returns JSON data ----
static void handleJson() {
  if (!s_hasRecord) {
    s_server.send(200, "application/json", "{}");
  } else {
    String json = buildJsonFromRecord(s_latest);
    s_server.send(200, "application/json", json);
  }
}

// ---- Public Interface Implementation ----

void WifiIf_beginAP(const char* ssid, const char* password) {
  if (s_started) return;   // Prevent re-initialization

  Serial.println(F("[wifi] Init AP mode..."));

  WiFi.mode(WIFI_AP);

  const char* useSsid = ssid     ? ssid     : WIFI_AP_SSID;
  const char* usePass = password ? password : WIFI_AP_PASSWORD;

  bool ok = WiFi.softAP(useSsid, usePass);
  if (!ok) {
    Serial.println(F("[wifi] softAP start FAILED"));
    s_started = false;
    return;
  }

  IPAddress ip = WiFi.softAPIP();
  Serial.println(F("[wifi] softAP started"));
  Serial.print(F("  SSID: ")); Serial.println(useSsid);
  Serial.print(F("  PASS: ")); Serial.println(usePass);
  Serial.print(F("  IP  : ")); Serial.println(ip);   // Usually 192.168.4.1

  // Register HTTP Routes
  s_server.on("/",     handleRoot);
  s_server.on("/json", handleJson);

  s_server.begin();
  Serial.println(F("[wifi] HTTP server started on port 80"));

  s_started = true;
}

void WifiIf_setRecord(const DataRecord& rec) {
  // Simple struct copy, low overhead
  s_latest    = rec;
  s_hasRecord = true;
}

void WifiIf_loop() {
  if (!s_started) return;
  // Non-blocking client handling
  s_server.handleClient();
}

bool WifiIf_ready() {
  return s_started;
}