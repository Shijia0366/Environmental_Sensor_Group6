#include <Arduino.h>
#include "config.h"

#include "hal/Buses.h"
#include "storage/SdCard.h"
#include "storage/Logger.h"

#include "drivers/BME688Sensor.h"
#include "drivers/GpsSensor.h"
#include "drivers/ToFSensor.h"
#include "drivers/OledDisplay.h"

#include "wireless/wifi_if.h"
#include "wireless/BlePpgClient.h"

#include "models/Result.h"
#include "models/DataRecord.h"

// ---- Global Objects (Singleton) ----
static SdCard        g_sd;
static Logger        g_logger;

static BME688Sensor  g_bme;
static GpsSensor     g_gps;
static ToFSensor     g_tof;
static OledDisplay   g_oled;

BlePpgClient ppgClient;

static DataRecord    rec{};  

// OLED update interval control
static unsigned long g_lastLogMs = 0;

// ToF parameter and interrupt
volatile bool g_tof_ready = false;
void IRAM_ATTR tof_isr() { g_tof_ready = true; }

// SD write interval control
static uint32_t last_sd_ms = 0;

// Note: To write immediately on startup, set last_sd_ms = millis() - SD_PERIOD_MS in setup()

// ---- Helper to read sensors safely ----
static void readSensorsOnce(DataRecord& rec) {
  // Timestamp
  rec.timestamp_ms = millis();

  // ToF (Non-blocking with timeout)
  if (g_tof_ready) {
    g_tof_ready = false;

    VL53L4CX_MultiRangingData_t m;
    if (g_tof.VL53L4CX_GetMultiRangingData(&m) == 0) {
      // Clear interrupt and start next frame immediately
      g_tof.VL53L4CX_ClearInterruptAndStartMeasurement();

      // Select closest valid target (RangeStatus==0)
      int n = m.NumberOfObjectsFound;
      int best = -1; uint16_t best_mm = 0xFFFF;
      for (int j = 0; j < n; ++j) {
        const auto &rd = m.RangeData[j];
        if (rd.RangeStatus == 0 && rd.RangeMilliMeter < best_mm) {
          best_mm = rd.RangeMilliMeter; best = j;
        }
      }
      if (best >= 0) {
        rec.dist_mm = (float)best_mm;   
      }

    } else {
      // Read failed: Soft restart measurement (no re-init)
      g_tof.VL53L4CX_ClearInterruptAndStartMeasurement();
      g_tof.VL53L4CX_StopMeasurement();
      g_tof.VL53L4CX_StartMeasurement();
      delay(60); // Warmup
    }
  }

    // GPS: Polled in loop, get data here
  double lat, lon, speed; float alt; bool fix;
  if (g_gps.get(lat, lon, alt, fix, speed).ok) {
    rec.lat_deg = lat;
    rec.lon_deg = lon;
    rec.alt_m   = alt;
    rec.gps_fix = fix;
    // =========== GPS Speed Smart Filter ===========
    
    // 1. Check Fix
    // If indoors (Fix=0), force speed to 0
    if (!fix) {
        rec.speed = 0.0;
    } 
    else {
        // 2. Filter drift if fixed
        // Low threshold to filter static jitter but keep slow walking data
        float speed_threshold = 0.6; 
        
        if (speed < speed_threshold) {
            rec.speed = 0.0;
        } else {
            rec.speed = speed;
        }
    }
    // ==============================================
    
  } else {
    // GPS module not readable
    rec.gps_fix = false;
    rec.speed   = 0.0; 
  }

// BME
  float t,h,p,gas;
  if (g_bme.read(t,h,p,gas).ok) {

    // =========== Static + Dynamic Thermal Compensation (V3) ===========
    
    // 1. Parameters (Calibrated based on 44min test)
    float start_offset = 2.5;      
    float max_stable_offset = 7.5; 
    
    // 2. Thermal equilibrium time (extended to 45 mins for smoother curve)
    // 45 mins = 45 * 60 * 1000 = 2700000
    unsigned long warmup_time_ms = 1200000; 
    
    // 3. Calculate dynamic offset
    float current_total_offset = 0.0;
    unsigned long uptime = millis();
    
    if (uptime >= warmup_time_ms) {
        // Warmup done, use max offset
        current_total_offset = max_stable_offset;
    } else {
        // Still warming up
        float dynamic_range = max_stable_offset - start_offset; 
        float progress = (float)uptime / (float)warmup_time_ms;
        
        current_total_offset = start_offset + (dynamic_range * progress);
    }

    // 4. Apply compensation
    rec.temp_c = t - current_total_offset;

    // ==========================================================

    rec.hum_pct   = h + 17;
    rec.press_hpa = p;
    rec.gas_ohm   = gas;


    // =========== UK AQI (Logarithmic Calculation) ===========
    // 1. Humidity Compensation
    // MOX Sensor: Higher humidity = lower resistance (false bad air indication)
    // Baseline: 40%, Compensate 0.5% per 1% humidity increase
    float humidity_baseline = 40.0;
    float humidity_diff = rec.hum_pct - humidity_baseline;
    
    // Compensate resistance if humidity > 40%
    float gas_compensated = rec.gas_ohm;
    if (humidity_diff > 0) {
        gas_compensated = rec.gas_ohm * (1.0 + (humidity_diff * 0.005));
    }
    
    // 1. Define baselines (Safety margins included)
    float baseline_clean = 85000.0;  // Clean air (AQI 1)
    float baseline_dirty = 10000.0;  // Polluted air (AQI 10)

    // 2. Log10 calculation
    double log_clean = log10(baseline_clean);
    double log_dirty = log10(baseline_dirty);
    double log_gas   = log10(rec.gas_ohm);

    // 3. Boundary checks
    if (rec.gas_ohm >= baseline_clean) {
        rec.AQI = 1;
    } else if (rec.gas_ohm <= baseline_dirty) {
        rec.AQI = 10;
    } else {
        // 4. Logarithmic interpolation
        // Calculate position in [Dirty, Clean] log interval (0.0 to 1.0)
        double ratio = (log_gas - log_dirty) / (log_clean - log_dirty);
        
        // 5. Map to 1-10
        // Note: Higher resistance = Better air, so higher ratio = lower AQI
        int calculated_aqi = 10 - (int)(ratio * 9.0);
        
        // Clamping
        if (calculated_aqi < 1) calculated_aqi = 1;
        if (calculated_aqi > 10) calculated_aqi = 10;
        
        rec.AQI = calculated_aqi;
    }
    // ====================================================
  }
  
  // BLE
  if (ppgClient.isConnected() && ppgClient.hasValidData()) {
      // Store received HR / SpO2
      rec.HR   = ppgClient.heartRate();
      rec.SPO2 = ppgClient.spo2();
    }
}

// ---- Setup/Loop Adapter ----
void app_setup() {
  Serial.begin(115200);
  delay(50);

  // 1) Buses
  HAL::beginI2C();   // Handles power/delay/freq internally
  HAL::beginSPI();

  // 2) SD
{
  auto st = g_sd.begin(SD_CS, SD_INIT_FREQ_HZ, SD_WORK_FREQ_HZ);
  if (st.ok) {
    Serial.println(F("[sd] mounted"));
    auto lst = g_logger.begin(g_sd, CSV_FILE_PATH, CSV_HEADER);
    if (lst.ok) {
      Serial.println(F("[log] ready"));
    } else {
      Serial.print(F("[log] fail: "));
      Serial.println(lst.err);
    }
  } else {
    Serial.print(F("[sd] fail: "));
    Serial.println(st.err);
  }
}

// 3) Sensors
{
  auto st = g_bme.begin(HAL::getWire(), BME_ADDR);
  if (st.ok) {
    Serial.println(F("[bme] ok"));
  } else {
    Serial.print(F("[bme] fail: "));
    Serial.println(st.err);
  }
}
{
  auto st = g_gps.begin(HAL::getWire(), GPS_ADDR);
  if (st.ok) {
    Serial.println(F("[gps] ok"));
  } else {
    Serial.print(F("[gps] fail: "));
    Serial.println(st.err);
  }
}
{
  auto st = g_tof.begin(HAL::getWire(), ToF_ADDR, TOF_XSHUT_PIN, TOF_INT_PIN);
  if (!st.ok) {
    Serial.print(F("[tof] fail: "));
    Serial.println(st.err);
    return;
  }

  int rc = 0;
  g_tof.VL53L4CX_StopMeasurement();             // Safety stop

  if ((rc = g_tof.VL53L4CX_StartMeasurement())) { Serial.printf("[tof] start=%d\n", rc); return; }
  g_tof.VL53L4CX_ClearInterruptAndStartMeasurement();           // Key: Clear once before first frame

  attachInterrupt(TOF_INT_PIN, tof_isr, FALLING);               // DRDY active low
  Serial.println(F("[tof] ok"));
}

// 4) OLED
{
  auto st = g_oled.begin(HAL::getWire(), OLED_ADDR, OLED_RESET);
  if (st.ok) {
    Serial.println(F("[oled] ok"));
  } else {
    Serial.print(F("[oled] fail: "));
    Serial.println(st.err);
  }
}
  g_oled.Rotate180();
  g_lastLogMs = millis();

  // 5) BLE: Init last
  ppgClient.begin("esp32s3-host");

  // 6) WiFi
  WifiIf_beginAP(); 

  // Default values (avoiding semantic errors)
  const uint32_t now = millis();
  rec.timestamp_ms = now;
  rec.temp_c = NAN; rec.hum_pct = NAN; rec.press_hpa = NAN; rec.gas_ohm = NAN;
  rec.dist_mm = NAN; rec.lat_deg = NAN; rec.lon_deg = NAN; rec.alt_m = NAN; rec.gps_fix = false;
  rec.speed = NAN; rec.AQI = 0; rec.acc_x = NAN; rec.acc_y = NAN; rec.acc_z = NAN; rec.HR = 0; rec.SPO2 = 0;
}

void app_loop() {
  // 1) Poll GPS frequently (non-blocking)
  g_gps.poll();

  // 2) Interval check: Sample + Display + Log
  const unsigned long now = millis();
  if (now - g_lastLogMs >= LOG_INTERVAL_MS) {
    g_lastLogMs = now;
    // rec.timestamp_ms = now;

    readSensorsOnce(rec);

    // OLED Update (if initialized)
    if (g_oled.isReady()) {
      g_oled.drawOledPaged(rec);
    }

    WifiIf_setRecord(rec);

    // Log to CSV
    if ((uint32_t)(millis() - last_sd_ms) >= SD_PERIOD_MS) {
      last_sd_ms += SD_PERIOD_MS;               // Accumulate to avoid jitter/drift

      if (g_logger.ready()) {
        auto st = g_logger.appendCSV(rec);      // Write row
        if (!st.ok) {
          Serial.println(String("[log] write failed: ") + st.err);
        }
        g_logger.flushIfNeeded();               
      }
    }
  }
  WifiIf_loop();
  ppgClient.loop();
}