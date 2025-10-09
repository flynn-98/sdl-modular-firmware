#pragma once
#include <Arduino.h>
#include <Wire.h>
#include <SensirionI2CSFX6XXX.h>

// Default I2C address (ADDR floating or GND)
#ifndef SFC6XXX_I2C_ADDR
#define SFC6XXX_I2C_ADDR 0x24
#endif

class SFC6000D {
public:
  // Initialize the device object with a given I2C bus and address.
  // Pass Wire (reference), not &Wire.
  bool begin(TwoWire& wire = Wire, uint8_t i2c_addr = SFC6XXX_I2C_ADDR);

  // Turn controller ON: start continuous measurement for CO2.
  bool mfcOn();

  // Turn controller OFF: set setpoint to 0 sccm and stop measurement.
  bool mfcOff();

  // Set a new mass flow setpoint in sccm (converts to slm internally).
  bool mfcSetFlowSccm(float sccm);

  // Read measured mass flow in sccm. Averages 'avg_samples' readings.
  bool mfcReadFlowSccm(float& out_sccm, uint16_t avg_samples = 10, uint16_t inter_sample_ms = 2);

  // Optional: read temperature in °C (if you care).
  bool readTemperatureC(float& out_degC);

  // Accessors
  uint8_t address() const { return _addr; }

private:
  SensirionI2cSfx6xxx _dev;
  uint8_t _addr = SFC6XXX_I2C_ADDR;
  bool _begun = false;

  static inline bool ok(int16_t err) { return err == 0; }
  bool logIfError(int16_t err, const __FlashStringHelper* where);
};
