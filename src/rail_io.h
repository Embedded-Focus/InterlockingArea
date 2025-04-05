#pragma once

#include <Arduino.h>
#include <Wire.h>

class OutputModule {
public:
  OutputModule(uint8_t addr) : addr{addr} {}
  inline void reset() { pin_states = 0; }
  inline void set_pin_on(uint8_t pin) { pin_states |= (0x01 << pin); }
  inline void commit() {
    // first, set all pin states according to their values
    Wire.beginTransmission(addr);
    Wire.write(pin_states);
    Wire.endTransmission();

    delay(200);

    // then, reset all pin values so that coils are no more supplied with
    // current
    Wire.beginTransmission(addr);
    Wire.write(0x00); // TODO might need inversion
    Wire.endTransmission();
  }

private:
  uint8_t const addr;
  uint8_t pin_states = 0;
};
