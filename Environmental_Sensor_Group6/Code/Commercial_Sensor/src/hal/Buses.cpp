#include "Buses.h"
#include "config.h"   // define the PIN_I2C_POWER / I2C_FREQUENCY / I2C_POWER_DELAY_MS / I2C_BUS_SETTLE_MS

namespace {
  TwoWire* wire = &Wire;
  SPIClass* spi  = &SPI;
}

namespace HAL {
  TwoWire& getWire() { return *wire; }
  SPIClass& getSPI() { return *spi; }

  bool beginI2C() {
    // 1) power up the I2C domain (meet OLED module pull-up RC charging and other power sequencing needs)
    #ifdef PIN_I2C_POWER
      pinMode(PIN_I2C_POWER, OUTPUT);
      digitalWrite(PIN_I2C_POWER, HIGH);
    #endif
    #ifdef I2C_POWER_DELAY_MS
      delay(I2C_POWER_DELAY_MS);
    #endif

    // 2) start the I2C controller (use board-level default pins first; if custom pins are needed, change to Wire.begin(SDA,SCL))
    getWire().begin();

    // 3) set the I2C frequency
    #ifdef I2C_FREQUENCY
      getWire().setClock(I2C_FREQUENCY);
    #endif

    // 4) additional stability wait after begin()
    #ifdef I2C_BUS_SETTLE_MS
      delay(I2C_BUS_SETTLE_MS);
    #endif

    return true;
  }

  bool beginSPI() {
    getSPI().begin();
    return true;
  }
}
