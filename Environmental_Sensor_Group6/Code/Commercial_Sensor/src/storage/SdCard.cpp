#include "SdCard.h"
#include "config.h"

Status SdCard::begin(uint8_t csPin, uint32_t initHz, uint32_t workHz) {
  cs_ = csPin;

  
  if (initHz == 0) initHz = SD_INIT_FREQ_HZ;
  if (workHz == 0) workHz = SD_WORK_FREQ_HZ;


  #if defined(SD_POWER_DELAY_MS) && (SD_POWER_DELAY_MS > 0)
    delay(SD_POWER_DELAY_MS);
  #endif


  #if defined(SD_CD_PIN) && (SD_CD_PIN >= 0)
    pinMode(SD_CD_PIN, INPUT_PULLUP);
    delay(SD_CD_DEBOUNCE_MS);
    bool inserted = digitalRead(SD_CD_PIN) == (SD_CD_ACTIVE_LOW ? LOW : HIGH);
    if (!inserted) return Status::Err("No SD card detected");
  #endif

// Low-speed initialization (≤400kHz)
  if (!SD.begin(cs_, HAL::getSPI(), initHz)) {
    mounted_ = false;
    return Status::Err("SD init failed (low speed)");
  }
  mounted_ = true;

// Speeding up operation: The most compatible way is to remount it to the operating frequency.
  if (workHz != initHz) {
    SD.end();
    mounted_ = SD.begin(cs_, HAL::getSPI(), workHz);
    if (!mounted_) return Status::Err("SD reinit failed (work speed)");
  }

  //Create a default log directory
  mkdirs_("/logs");
  return Status::Ok();
}

void SdCard::end() {
  if (mounted_) {
    SD.end();
    mounted_ = false;
  }
}

bool SdCard::exists(const char* path) const {
  return SD.exists(path);
}

bool SdCard::mkdirs_(const String& fullPath) const {
  if (fullPath.length() == 0 || fullPath == "/") return true;
  String path = fullPath;
  if (path[0] != '/') path = "/" + path;


  int idx = 1;
  while (idx < (int)path.length()) {
    int next = path.indexOf('/', idx);
    if (next < 0) next = path.length();
    String sub = path.substring(0, next);
    if (!SD.exists(sub)) {
      if (!SD.mkdir(sub)) return false;
    }
    idx = next + 1;
  }
  return true;
}

bool SdCard::mkdirs(const char* path) const {
  return mkdirs_(String(path));
}

Result<File> SdCard::openAppend(const char* path) const {
  String p(path);
  int slash = p.lastIndexOf('/');
  if (slash > 0) {
    String dir = p.substring(0, slash);
    if (!mkdirs_(dir)) return Result<File>::Err("mkdirs() failed");
  }
  File f = SD.open(path, FILE_APPEND);
  if (!f) return Result<File>::Err("openAppend() failed");
  return Result<File>::Ok(f);
}

Result<File> SdCard::openWrite(const char* path, bool truncate) const {

  String p(path);
  int slash = p.lastIndexOf('/');
  if (slash > 0) {
    String dir = p.substring(0, slash);
    if (!mkdirs_(dir)) return Result<File>::Err("mkdirs() failed");
  }
  if (truncate && SD.exists(path)) SD.remove(path);
  File f = SD.open(path, FILE_WRITE);
  if (!f) return Result<File>::Err("openWrite() failed");
  return Result<File>::Ok(f);
}
