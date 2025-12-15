#pragma once
#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>

// HAL: Hardware Abstraction Layer for shared buses
namespace HAL {
  // I2C / SPI singletons (references to global instances)
  TwoWire& getWire();      // default I2C (Wire)
  SPIClass& getSPI();      // default SPI (SPI)

  // Init functions (use timings defined in config.h)
  bool beginI2C();         // powers I2C domain, waits, begin Wire, setClock
  bool beginSPI();         // begin SPI, optionally with custom pins in Buses.cpp

  // TwoWire& getWire1();
  // bool beginI2C1();
}
