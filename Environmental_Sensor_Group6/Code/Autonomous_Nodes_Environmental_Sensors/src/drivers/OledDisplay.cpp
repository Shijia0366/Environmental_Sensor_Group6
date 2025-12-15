#include "OledDisplay.h"

Status OledDisplay::begin(TwoWire& w, uint8_t addr, int8_t rstPin) {
  addr_ = addr;
  rst_  = rstPin;

  if (OLED_PRE_INIT_DELAY_MS > 0) delay(OLED_PRE_INIT_DELAY_MS);

  // Dynamic allocation to ensure correct TwoWire instance is used
  if (oled_) { delete oled_; oled_ = nullptr; }
  // oled_ = new Adafruit_SSD1306(128, 64, &w, rst_);
  oled_ = new Adafruit_SSD1306(128, 64, &w);

  // Try SWITCHCAPVCC first, if fail, try EXTERNALVCC
  if (!tryBegin_(SSD1306_SWITCHCAPVCC)) {
    // hardReset_();
    if (!tryBegin_(SSD1306_EXTERNALVCC)) {
      ready_ = false;
      return Status::Err("OLED init failed");
    }
  }

  if (OLED_POST_BEGIN_DELAY_MS > 0) delay(OLED_POST_BEGIN_DELAY_MS);

  // Configurable contrast
  setContrast(OLED_CONTRAST);

  // Boot frame (optional)
  // drawBootFrame_();

  ready_ = true;
  return Status::Ok();
}

bool OledDisplay::tryBegin_(uint8_t vccMode) {
  if (!oled_) return false;
  if (!oled_->begin(vccMode, addr_)) return false;
  oled_->clearDisplay();
  oled_->setTextSize(OLED_TEXT_SIZE);
  oled_->setTextColor(SSD1306_WHITE);
  oled_->display();
  return true;
}

void OledDisplay::hardReset_() {
  if (rst_ < 0) return;
  pinMode(rst_, OUTPUT);
  digitalWrite(rst_, HIGH); delay(10);
  digitalWrite(rst_, LOW);  delay(15);
  digitalWrite(rst_, HIGH); delay(50);
}

void OledDisplay::drawBootFrame_() {
  if (!oled_) return;
  oled_->clearDisplay();
  oled_->setCursor(0, 0);
  oled_->println(F("LeafSpace"));
  oled_->println(F("SSD1306 OK"));
  oled_->drawRect(0, 0, 128, 64, SSD1306_WHITE);
  oled_->display();
}

void OledDisplay::Rotate180() {
  if (!oled_) return;
  oled_->setRotation(2);
  oled_->clearDisplay();     // Clear buffer
  oled_->display();          // Actual clear
  // Redraw content with new coordinates after this
}

void OledDisplay::setContrast(uint8_t value) {
  if (!oled_) return;
  oled_->ssd1306_command(SSD1306_SETCONTRAST);
  oled_->ssd1306_command(value);
  oled_->ssd1306_command(SSD1306_DISPLAYON);
}

void OledDisplay::drawOledPaged(const DataRecord& r) {
  if (!ready_ || !oled_) return;
  
  // Pagination State
  static uint8_t  page = 0;
  static uint32_t t0   = 0;
  uint32_t now = millis();
  if (now - t0 >= PAGE_MS) { t0 = now; page = (page + 1) % 5; }   // Total 5 pages

  // Unified Page Drawing
  oled_->clearDisplay();
  oled_->setTextSize(OLED_TEXT_SIZE);
  oled_->setTextColor(SSD1306_WHITE, SSD1306_BLACK);
  oled_->setCursor(0, 0);
  
  switch (page) {
    case 0:   // Env + AQI
      oled_->printf("Tem:%.1f\n", r.temp_c);
      oled_->printf("Hum:%.0f%%\n", r.hum_pct);
      oled_->printf("Pre:%.0f\n", r.press_hpa);
      if (!isnan(r.gas_ohm)) oled_->printf("Gas:%.1fk\n", r.gas_ohm / 1000.0);
      else                   oled_->printf("Gas:--\n");
      break;

    case 1:   // Location
      oled_->printf("GPS:%s\n", r.gps_fix ? "FIX" : "NO");
      oled_->printf("Lat:%.3f\n", r.lat_deg);
      oled_->printf("Lon:%.3f\n", r.lon_deg);
      oled_->printf("Alt:%.1fm\n", r.alt_m);
      break;

    case 2:   // Gas + Distance + Speed
      oled_->printf("AQI: %d\n", r.AQI);
      oled_->printf("Dis: %.0fmm\n", r.dist_mm);
      oled_->printf("Spd: %.1f\n", r.speed);

      oled_->println(F("-------"));
      break;

   case 3: // Acc (IMU)
      // %5.1f: width 5 chars (including dot), right aligned.
      // Ensures "-9.8" and " 9.8" take same space, avoiding jitter.
      oled_->println(F("- ACC -")); 
      oled_->printf("X: %5.1f\n", r.acc_x);
      oled_->printf("Y: %5.1f\n", r.acc_y);
      oled_->printf("Z: %5.1f\n", r.acc_z);
      break;
  
   case 4: // Pulse Sensor Page
      oled_->println(F("- PPG -"));
      oled_->printf("HR: %.1fbpm\n", r.HR);
      oled_->printf("SPO2: %.1f%%\n", r.SPO2);
      oled_->println(F("-------"));
      break;
}
oled_->display();
}

void OledDisplay::showError(const String& msg) {
  if (!oled_) return;
  oled_->clearDisplay();
  oled_->setTextSize(OLED_TEXT_SIZE);
  oled_->setCursor(0, 0);
  oled_->println(F("ERR:"));
  oled_->println(msg);
  oled_->display();
}