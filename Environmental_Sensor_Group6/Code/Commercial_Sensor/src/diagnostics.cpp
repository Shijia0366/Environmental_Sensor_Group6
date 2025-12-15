#include <Arduino.h>
#include "config.h"
#include "hal/Buses.h"

#include "drivers/OledDisplay.h"
#include "drivers/BME688Sensor.h"
#include "drivers/GpsSensor.h"
#include "drivers/ToFSensor.h"

#include "storage/SdCard.h"
#include "storage/Logger.h"
#include "models/DataRecord.h"
#include "models/Result.h"


static OledDisplay  d_oled;
static BME688Sensor d_bme;
static GpsSensor    d_gps;
static ToFSensor    d_tof;
static SdCard       d_sd;
bool ToF_Init = 0;


static void i2c_scan(TwoWire& w) {
  Serial.println(F("\n[I2C] Scanning..."));
  uint8_t found = 0;
  for (uint8_t addr = 1; addr < 127; addr++) {
    w.beginTransmission(addr);
    uint8_t err = w.endTransmission();
    if (err == 0) {
      Serial.printf("  - Found 0x%02X\n", addr);
      found++;
    }
    delay(2);
  }
  if (!found) Serial.println(F("  (no device found)"));
}

// ---- Test 1: OLED ----
static void test_oled() {
  Serial.println(F("\n[TEST] OLED"));
  auto st1 = d_oled.begin(HAL::getWire(), OLED_ADDR, OLED_RESET);
  if (!st1.ok) { Serial.println(String("  FAIL: ")+st1.err); return; }
  DataRecord rec{};
  rec.temp_c=23.4; rec.hum_pct=45; rec.press_hpa=1008; rec.gas_ohm=12345;
  rec.dist_mm=456; rec.gps_fix=false;
  d_oled.render(rec);
  d_oled.setContrast(0xCF);
  Serial.println(F("  PASS: SSD1306 rendered demo"));
}

// ---- Test 2: BME688 ----
static void test_bme() {
  Serial.println(F("\n[TEST] BME688"));
  auto st = d_bme.begin(HAL::getWire(), BME_ADDR);
  if (!st.ok) { Serial.println(String("  FAIL: ")+st.err); return; }
  float t,h,p,gas;
  st = d_bme.read(t,h,p,gas);
  if (!st.ok) { Serial.println(String("  FAIL: ")+st.err); return; }
  Serial.printf("  PASS: T=%.2fC H=%.1f%% P=%.1f hPa Gas=%.0f ohm\n", t,h,p,gas);
}

// ---- Test 3: GPS (I2C, non-blocking polling) ----
static void test_gps(unsigned long wait_s = 5) {
  Serial.println(F("\n[TEST] GPS (I2C)"));
  auto st = d_gps.begin(HAL::getWire(), GPS_ADDR);
  if (!st.ok) { Serial.println(String("  FAIL: ")+st.err); return; }
  Serial.printf("  Info: polling %lus for NMEA...\n", wait_s);

  unsigned long t0 = millis();
  uint32_t nmea_cnt = 0;
  bool gotFix = false;
  double lat= NAN, lon = NAN, speed = NAN; float alt = NAN; bool fix=false;

  while (millis() - t0 < wait_s*1000UL) {
  for (int i = 0; i < 50; ++i) {
    d_gps.poll(); 
  }
  if (d_gps.get(lat, lon, alt, fix, speed).ok) {
    if (!isnan(lat) && !isnan(lon)) gotFix = gotFix || fix;
  }
  delay(20);
  nmea_cnt++;
}

  Serial.printf("  NMEA windows checked: %lu\n", (unsigned long)nmea_cnt);

    Serial.printf("  Last: lat=%.6f lon=%.6f alt=%.1fm fix=%d\n", lat,lon,alt,(int)fix);

  Serial.println(gotFix ? F("  PASS: got valid GPS stream (fix may still take longer outdoors)") 
                        : F("  WARN: NMEA received but no fix yet (try outdoors)"));
}

// ---- Test 4: ToF ----
static void test_tof(unsigned tries = 10) {  
  Serial.println(F("\n[TEST] VL53L4CX read x10 (d_tof)"));
  int rc = 0;
  
  if(ToF_Init == 0){


  auto st = d_tof.begin(HAL::getWire(), ToF_ADDR, TOF_XSHUT_PIN, TOF_INT_PIN);
  if (!st.ok) { Serial.println(String("  FAIL: ") + st.err); return; }
  
  
  // if ((rc = d_tof.VL53L4CX_Off()))              { Serial.printf("  FAIL: off=%d\n",   rc); return; }
  // if ((rc = d_tof.InitSensor(ToF_ADDR)))        { Serial.printf("  FAIL: init=%d\n",  rc); return; }

  d_tof.VL53L4CX_SetDistanceMode(VL53L4CX_DISTANCEMODE_MEDIUM);
  d_tof.VL53L4CX_SetMeasurementTimingBudgetMicroSeconds(50000); // 50 ms 


  if ((rc = d_tof.VL53L4CX_StartMeasurement())) { Serial.printf("  FAIL: start=%d\n", rc); return; }
  d_tof.VL53L4CX_ClearInterruptAndStartMeasurement();  
  delay(60);  // Warm-up: ≈ timing budget (50ms) + margin
  ToF_Init = 1;
  }

  for (unsigned i = 0; i < tries; ++i) {
    uint8_t NewDataReady = 0;
    const uint32_t timeout_ms = (i == 0) ? 1200 : 500;
    uint32_t t0 = millis();
    do {
      rc = d_tof.VL53L4CX_GetMeasurementDataReady(&NewDataReady);
      if (rc) break;
      if (!NewDataReady) delay(2);
    } while (!NewDataReady && (millis() - t0 < timeout_ms));

    if (rc || !NewDataReady) {
      Serial.printf("  #%u: not ready (rc=%d) -> restart measure\n", i, rc);
      // Soft reboot for measurement to avoid the setting remaining unchanged in the future.
      d_tof.VL53L4CX_ClearInterruptAndStartMeasurement();
      d_tof.VL53L4CX_StopMeasurement();
      d_tof.VL53L4CX_StartMeasurement();
      delay(60);
      continue;
    }

    // Read one frame & immediately clear interrupt and start the next frame
    VL53L4CX_MultiRangingData_t m;
    rc = d_tof.VL53L4CX_GetMultiRangingData(&m);
    d_tof.VL53L4CX_ClearInterruptAndStartMeasurement();

    if (rc) {
      Serial.printf("  #%u: read rc=%d\n", i, rc);
      continue;
    }

    int n = m.NumberOfObjectsFound;
    if (n <= 0) {
      Serial.printf("  #%u: no object\n", i);
    } else {
      int best = 0; uint16_t best_mm = m.RangeData[0].RangeMilliMeter;
      for (int j = 1; j < n; ++j) {
        if (m.RangeData[j].RangeMilliMeter < best_mm) { best = j; best_mm = m.RangeData[j].RangeMilliMeter; }
      }
      const auto &rd = m.RangeData[best];
      Serial.printf("  #%u: D=%u mm (objs=%d, status=%d, S=%.3f, Amb=%.3f)\n",
                    i, (unsigned)rd.RangeMilliMeter, n, rd.RangeStatus,
                    (float)rd.SignalRateRtnMegaCps/65536.0f,
                    (float)rd.AmbientRateRtnMegaCps/65536.0f);
    }

    delay(20);
  }

  Serial.println(F("  DONE"));
}




// ---- Test 5: SD Card (Mount + Write + Read Check) ----
static void test_sd() {
  Serial.println(F("\n[TEST] SD card (SPI)"));
  auto st = d_sd.begin(SD_CS, SD_INIT_FREQ_HZ, SD_WORK_FREQ_HZ);
  if (!st.ok) { Serial.println(String("  FAIL: ")+st.err); return; }
  Serial.println(F("  SD mounted"));

  const char* path = "/logs/diag_test.txt";

  {
    auto res = d_sd.openWrite(path, /*truncate*/true);
    if (!res.ok) { Serial.println(String("  FAIL openWrite: ")+res.err); return; }
    File f = res.value;
    f.println("hello sd");
    f.flush();
    f.close();
  }
  {
    File f = SD.open(path, FILE_READ);
    if (!f) { Serial.println(F("  FAIL: reopen read")); return; }
    String s = f.readString();
    f.close();
    s.trim();
    if (s == "hello sd") {
      Serial.println(F("  PASS: readback matched"));
    } else {
      Serial.printf("  FAIL: readback mismatch: '%s'\n", s.c_str());
    }
  }
}

// ---- Test 0: I2C Scan + Power-On Timing Verification ----
static void test_i2c_scan() {
  Serial.println(F("\n[TEST] I2C scan + power"));
  HAL::beginI2C();  
  i2c_scan(HAL::getWire());
}

static void print_menu() {
  Serial.println(F("\n========== DIAGNOSTICS =========="));
  Serial.println(F("[0] I2C scan"));
  Serial.println(F("[1] OLED"));
  Serial.println(F("[2] BME688"));
  Serial.println(F("[3] GPS (5s)"));
  Serial.println(F("[4] ToF (10 samples)"));
  Serial.println(F("[5] SD card"));
  Serial.println(F("[a] ALL (0->5)"));
  Serial.println(F("[q] Quit diag (reboot to app)"));
  Serial.println(F("================================="));
  Serial.print(F("Select: "));
}

static void run_all() {
  test_i2c_scan();
  test_oled();
  test_bme();
  test_gps(5);
  test_tof(10);
  test_sd();
  Serial.println(F("\n[ALL] done."));
}

// ---- External interfaces: diag_setup / diag_loop (for main to call) ----
void diag_setup() {
  Serial.begin(115200);
  delay(50);
  Serial.println(F("\n[diag] boot"));
  HAL::beginI2C();
  HAL::beginSPI();
  print_menu();
}

void diag_loop() {
  static int state = 0;
  if (Serial.available()) {
    char c = (char)Serial.read();
    Serial.println(c);
    switch (c) {
      case '0': test_i2c_scan(); break;
      case '1': test_oled();     break;
      case '2': test_bme();      break;
      case '3': test_gps(5);     break;
      case '4': test_tof(10);    break;
      case '5': test_sd();       break;
      case 'a': run_all();       break;
      case 'q': Serial.println(F("Reboot to exit diag.")); delay(200); esp_restart(); break; //需要同时xshut tof，否则tof不会重置，但是mcu重置了ToF_Init的状态
      default:  Serial.println(F("Unknown.")); break;
    }
    print_menu();
  }
  d_gps.poll();
}
