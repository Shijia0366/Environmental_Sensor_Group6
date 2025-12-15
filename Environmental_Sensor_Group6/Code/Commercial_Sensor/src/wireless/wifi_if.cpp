#include "wifi_if.h"

#include <WiFi.h>
#include <WebServer.h>
#include "config.h"  
#include <math.h>   

#ifndef WIFI_AP_SSID
  #define WIFI_AP_SSID      "ENV-S3-AP"
#endif

#ifndef WIFI_AP_PASSWORD
  #define WIFI_AP_PASSWORD  "12345678"
#endif

// ---- Internal static object ----
static WebServer       s_server(80);
static bool            s_started   = false;   
static DataRecord      s_latest{};           
static bool            s_hasRecord = false;  

// ---- Safely convert floating-point numbers to JSON numbers (NaN/Inf -> null) ----
static String safeJsonNumber(double v, int decimals) {
  if (isnan(v) || isinf(v)) {
    return "null";  
  }
  return String(v, decimals);
}

// ---- Convert DataRecord to a JSON string (all fields) ----
static String buildJsonFromRecord(const DataRecord& r) {
  String s;
  s.reserve(512); // Increase reserved space to prevent memory reallocation due to an increase in fields.

  s += '{';
  s += "\"timestamp_ms\":"; s += String(r.timestamp_ms); s += ',';

  // BME688
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

  // ACC
  s += "\"acc_x\":";     s += safeJsonNumber(r.acc_x,     2); s += ',';
  s += "\"acc_y\":";     s += safeJsonNumber(r.acc_y,     2); s += ',';
  s += "\"acc_z\":";     s += safeJsonNumber(r.acc_z,     2); s += ',';
  s += "\"gyro_x\":";    s += safeJsonNumber(r.gyro_x,    2); s += ',';
  s += "\"gyro_y\":";    s += safeJsonNumber(r.gyro_y,    2); s += ',';
  s += "\"gyro_z\":";    s += safeJsonNumber(r.gyro_z,    2); s += ',';

  // Pulse Sensor 
  s += "\"signal\":";    s += String(r.signal); s += ',';
  s += "\"bpm\":";       s += String(r.bpm);
  s += '}';
  return s;
}


static void handleRoot() {
  String page = F(R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <meta charset="utf-8">
  <title>ENV S3 Telemetry</title>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <style>
    body { font-family: system-ui, -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif;
           margin: 0; padding: 16px; background: #111; color: #eee; }
    h1   { font-size: 20px; margin: 0 0 8px; }
    #status { font-size: 13px; color: #aaa; margin-bottom: 8px; }
    pre  { background: #000; padding: 12px; border-radius: 8px; font-size: 13px;
           overflow-x: auto; white-space: pre-wrap; word-wrap: break-word; }
    .badge { display: inline-block; padding: 2px 6px; border-radius: 999px;
             background: #333; font-size: 11px; margin-left: 6px; }
  </style>
</head>
<body>
  <h1>Environmental Sensor <span class="badge">Live</span></h1>
  <div id="status">Connecting...</div>
  <pre id="data">{ }</pre>

  <script>
    async function update() {
      const statusEl = document.getElementById('status');
      const dataEl   = document.getElementById('data');
      try {
        // 加上时间戳避免浏览器缓存
        const res = await fetch('/json?ts=' + Date.now());
        if (!res.ok) {
          throw new Error('HTTP ' + res.status);
        }
        const json = await res.json();
        dataEl.textContent = JSON.stringify(json, null, 2);
        const now = new Date();
        statusEl.textContent = 'Last update: ' + now.toLocaleTimeString();
        statusEl.style.color = '#aaa';
      } catch (e) {
        statusEl.textContent = 'Error: ' + e;
        statusEl.style.color = 'red';
      }
    }

    // 先立即更新一次
    update();
    // 然后每 200ms 更新一次
    setInterval(update, 200);
  </script>
</body>
</html>
)rawliteral");

  s_server.send(200, "text/html", page);
}

// ---- HTTP processing: /json directly returns JSON data. ----
static void handleJson() {
  if (!s_hasRecord) {
    s_server.send(200, "application/json", "{}");
  } else {
    String json = buildJsonFromRecord(s_latest);
    s_server.send(200, "application/json", json);
  }
}

// ---- External interface implementation ----

void WifiIf_beginAP(const char* ssid, const char* password) {
  if (s_started) return;   

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
  Serial.print(F("  IP  : ")); Serial.println(ip);   

  s_server.on("/",     handleRoot);
  s_server.on("/json", handleJson);

  s_server.begin();
  Serial.println(F("[wifi] HTTP server started on port 80"));

  s_started = true;
}

void WifiIf_setRecord(const DataRecord& rec) {
  
  s_latest  = rec;
  s_hasRecord = true;
}

void WifiIf_loop() {
  if (!s_started) return;

  s_server.handleClient();
}

bool WifiIf_ready() {
  return s_started;
}