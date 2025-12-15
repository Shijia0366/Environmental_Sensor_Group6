#include <Arduino.h>
#include "config.h"

#include "hal/Buses.h"
#include "storage/SdCard.h"
#include "storage/Logger.h"

#include "drivers/BME688Sensor.h"
#include "drivers/GpsSensor.h"
#include "drivers/ToFSensor.h"
#include "drivers/OledDisplay.h"
#include "drivers/MPU6050Sensor.h"

#include "wireless/ble_manager.h"
#include "wireless/ble_streams.h"
#include "wireless/wifi_if.h"

#include "models/Result.h"
#include "models/DataRecord.h"

#include "drivers/PulseSensorDriver.h"

static SdCard        g_sd;
static Logger        g_logger;

static BME688Sensor  g_bme;
static GpsSensor     g_gps;
static ToFSensor     g_tof;
static OledDisplay   g_oled;
static MPU6050Sensor g_mpu;
static PulseSensorDriver g_pulse;

static BleStreams    g_streams;

static constexpr bool  PPG_SPO2_MODE = true; 
static constexpr float ACC_LSB_MG     = 16.0f; 

static DataRecord    rec{};  

// OLED sampling beat
static unsigned long g_lastLogMs = 0;

// ToF parameter and interrupt
//volatile bool g_tof_ready = false;
//void IRAM_ATTR tof_isr() { g_tof_ready = true; }

// SD write frequency
static uint32_t last_sd_ms = 0;

static void readSensorsOnce(DataRecord& rec) {
  rec.timestamp_ms = millis();

 // ToF Read
  float dist;
  // The driver will automatically handle polling.
  if (g_tof.read(dist).ok) {
      rec.dist_mm = dist;
  }

  // GPS: poll is called frequently in the loop; get is called once at the sampling point.
  double lat, lon, speed; float alt; bool fix;
  if (g_gps.get(lat, lon, alt, fix, speed).ok) {
    rec.lat_deg = lat;
    rec.lon_deg = lon;
    rec.alt_m   = alt;
    rec.gps_fix = fix;
    // =========== GPS Smart Filter ===========
    
// 1.check if there is a location (Fix)
// If ndoors (Fix=0), regardless of the speed displayed, it will be forced to zero.
    if (!fix) {
        rec.speed = 0.0;
    } 
    else {
// 2. If location services are available, check if the drift is extremely small.
// Since the actual walking speed might only be 0.8~1.5, the threshold needs to be set very low.
// Ultimately set to 0.6 (filtering out jitter when completely stationary, retaining slow walking data).
        float speed_threshold = 0.6; 
        
        if (speed < speed_threshold) {
            rec.speed = 0.0;
        } else {
            rec.speed = speed;
        }
    }
    // ========================================================
    
  } else {
 // Situation where the GPS module cannot read data
    rec.gps_fix = false;
    rec.speed   = 0.0; 
  }

// BME
  float t,h,p,gas;
  if (g_bme.read(t,h,p,gas).ok) {

    // =========== Static + Dynamic Dual Thermal Compensation Algorithm ===========
    
    float start_offset = 0.8;      
    float max_stable_offset = 7.5; 
    
    // Define thermal equilibrium time
    unsigned long warmup_time_ms = 2700000; 
    
    // Calculate the current dynamic compensation part
    float current_total_offset = 0.0;
    unsigned long uptime = millis();
    
    if (uptime >= warmup_time_ms) {
        current_total_offset = max_stable_offset;
    } else {
        float dynamic_range = max_stable_offset - start_offset; 
        float progress = (float)uptime / (float)warmup_time_ms;
        
        current_total_offset = start_offset + (dynamic_range * progress);
    }
    rec.temp_c = t - current_total_offset;
    
    rec.hum_pct   = h + 17;
    rec.press_hpa = p;
    rec.gas_ohm   = gas;


    // =========== AQI  ===========
    // Humidity Compensation
    // 0.005 is the compensation coefficient.
    float humidity_baseline = 40.0;
    float humidity_diff = rec.hum_pct - humidity_baseline;

    float gas_compensated = rec.gas_ohm;
    if (humidity_diff > 0) {
        gas_compensated = rec.gas_ohm * (1.0 + (humidity_diff * 0.005));
    }
    //Define the benchmark based on your measured data.
    float baseline_clean = 85000.0; //AQI 1
    float baseline_dirty = 10000.0;  //AQI 10

    // Log10
    // The comparison is of orders of magnitude, not specific numerical values.
    double log_clean = log10(baseline_clean);
    double log_dirty = log10(baseline_dirty);
    double log_gas   = log10(rec.gas_ohm);

    // Boundary processing
    if (rec.gas_ohm >= baseline_clean) {
        rec.AQI = 1;
    } else if (rec.gas_ohm <= baseline_dirty) {
        rec.AQI = 10;
    } else {
        // Logarithmic interpolation calculation
        double ratio = (log_gas - log_dirty) / (log_clean - log_dirty);
        
        // ratio = 1.0 (Clean) -> AQI = 1
        // ratio = 0.0 (Dirty) -> AQI = 10
        int calculated_aqi = 10 - (int)(ratio * 9.0);
        if (calculated_aqi < 1) calculated_aqi = 1;
        if (calculated_aqi > 10) calculated_aqi = 10;
        
        rec.AQI = calculated_aqi;
    }
    // ====================================================
  } 
  
  MPU6050Sensor::MPUData mpu_data;
  if (g_mpu.read(mpu_data).ok) {
  
      rec.acc_x = mpu_data.ax;
      rec.acc_y = mpu_data.ay;
      rec.acc_z = mpu_data.az - 10;

      rec.gyro_x = mpu_data.gx;
      rec.gyro_y = mpu_data.gy;
      rec.gyro_z = mpu_data.gz;
  }
  // PulseSensor 
  PulseSensorDriver::PulseData p_data;
  if (g_pulse.read(p_data).ok) {
      rec.signal = p_data.signal;
      rec.bpm    = p_data.bpm;
  }
}


// ---- setup/loop  ----
void app_setup() {
  Serial.begin(115200);
  delay(50);

  // 1) Buses
  HAL::beginI2C();   
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
  // MPU_ADDR  0x68
  auto st = g_mpu.begin(HAL::getWire(), MPU_ADDR);
  if (st.ok) {
    Serial.println(F("[mpu] ok"));
  } else {
    Serial.print(F("[mpu] fail: "));
    Serial.println(st.err);
  }
}

// Pulse Sensor
{
  // use config.h  PULSE_SENSOR_PIN (A3)
  auto st = g_pulse.begin(); 
  if (st.ok) {
    Serial.println(F("[pulse] ok"));
  } else {
    Serial.print(F("[pulse] fail: ")); // Even if it fails, do not return in the setup, to avoid freezing subsequent steps.
    Serial.println(st.err);
  }
}

{

  auto st = g_tof.begin(HAL::getWire(), ToF_ADDR, TOF_XSHUT_PIN, -1);
  
  if (!st.ok) {
    Serial.print(F("[tof] fail: "));
    Serial.println(st.err);
   
  }


  g_tof.stop(); 
  g_tof.start();
  
  Serial.println(F("[tof] ok"));
}

// OLED
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

  // WiFi
  WifiIf_beginAP(); 
  const uint32_t now = millis();
  rec.timestamp_ms = now;
  rec.temp_c = NAN; rec.hum_pct = NAN; rec.press_hpa = NAN; rec.gas_ohm = NAN;
  rec.dist_mm = NAN; rec.lat_deg = NAN; rec.lon_deg = NAN; rec.alt_m = NAN; rec.gps_fix = false;
  rec.speed = NAN; rec.AQI = 0; rec.acc_x = NAN; rec.acc_y = NAN; rec.acc_z = NAN; rec.signal = 0; rec.bpm = 0;
}

void app_loop() {
  // GPS polling should be as frequent as possible without blocking.
  g_gps.poll();

  const unsigned long now = millis();
  if (now - g_lastLogMs >= LOG_INTERVAL_MS) {
    g_lastLogMs = now;

    readSensorsOnce(rec);

    // OLED refresh
    if (g_oled.isReady()) {
      g_oled.drawOledPaged(rec);
    }

    WifiIf_setRecord(rec);

    // Record CSV
    if ((uint32_t)(millis() - last_sd_ms) >= SD_PERIOD_MS) {
      last_sd_ms += SD_PERIOD_MS;               // Use "cumulative" to avoid shaking and missing beats.

      if (g_logger.ready()) {
        auto st = g_logger.appendCSV(rec);      
        if (!st.ok) {
          Serial.println(String("[log] write failed: ") + st.err);
        }
        g_logger.flushIfNeeded();               
      }
    }
  }
  WifiIf_loop();
}
