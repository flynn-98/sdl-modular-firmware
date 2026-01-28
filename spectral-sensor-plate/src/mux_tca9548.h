#pragma once
#include <Arduino.h>
#include <Wire.h>
#include "TCA9548.h"   // Rob Tillaart library

// Manager for two TCA9548A/PWR muxes controlling 16 identical-address sensors (0x39).
// Enforces that only ONE mux channel is enabled globally at a time.
class Tca9548MuxManager {
public:
  enum class Status : uint8_t {
    Ok = 0,
    BadSensorIndex,
    NotBegun,
    NotConnected,
    I2COpFail,
    VerifyFail,
  };

  struct Config {
    TwoWire* wire = &Wire;

    // Mux I2C addresses
    uint8_t mux1_addr = 0x70;
    uint8_t mux2_addr = 0x71;

    // Optional reset pins wired to mux RESET (active-low)
    int reset1_gpio = -1;
    int reset2_gpio = -1;

    // Optional I2C recovery pins (your bus pins)
    int sda_gpio = -1;
    int scl_gpio = -1;

    // Retry policy for mux ops
    uint8_t i2c_retries = 3;
    uint16_t retry_delay_ms = 2;

    // Wire timeout (Arduino-ESP32)
    uint16_t wire_timeout_ms = 50;

    // Attempt bus recovery on failures
    bool enable_bus_recovery = true;

    // Enable forced readback verification of channel masks
    bool verify_after_select = true;
  };

  explicit Tca9548MuxManager(const Config& cfg);

  // Call once in setup() AFTER Wire.begin()
  Status begin();

  // Open channel corresponding to sensor index (1..16). Closes any previously open channel.
  Status selectSensor(uint8_t sensorIndex);

  // Disable all channels on both muxes.
  Status disableAll();

  // Reset mux #1, mux #2, or both (if reset pins are provided)
  Status resetMux1();
  Status resetMux2();
  Status resetBoth();

  // Returns currently selected sensor index, or 0 if none.
  uint8_t currentSensor() const { return _currentSensor; }

  // Optional: I2C bus recovery (clock SCL, issue STOP)
  Status recoverI2CBus();

private:
  struct Route {
    uint8_t muxId;     // 1 or 2
    uint8_t channel;   // 0..7
  };

  Config _cfg;
  uint8_t _currentSensor = 0;
  bool _begun = false;

  // Rob Tillaart library instances
  TCA9548 _mux1;
  TCA9548 _mux2;

  Route routeForSensor(uint8_t sensorIndex) const;

  Status setExclusive(uint8_t muxId, uint8_t channel);

  template<typename Fn>
  Status withRetries(Fn fn);

  bool isMuxesConnected();
};
