#pragma once
#include <Arduino.h>
#include <Wire.h>

// Sensirion I2C driver for SFX6XXX (MFC/MFM)
#include <SensirionI2CSFX6XXX.h>  // from https://github.com/Sensirion/arduino-i2c-sfx6xxx

// ---- Config ----
// Default I2C address is 0x24 when ADDR is floating or tied low.
// Change if you strap a different address.
#ifndef SFC6XXX_I2C_ADDR
#define SFC6XXX_I2C_ADDR 0x24
#endif

// Choose the "startXContinuousMeasurement" for your gas calibration.
// Options typically include startO2ContinuousMeasurement(), startN2ContinuousMeasurement(), startAirContinuousMeasurement(), etc.
// See the library's example and header for the exact list. Defaults to Air.
#ifndef SFC6XXX_START_MEAS_FN
#define SFC6XXX_START_MEAS_FN startAirContinuousMeasurement
#endif

class SFC6000D {
public:
  // wire: pass Wire (default) or a custom TwoWire
  bool begin(TwoWire* wire = &Wire, uint8_t i2c_addr = SFC6XXX_I2C_ADDR);

  // Turn controller “on”: starts continuous measurement (for readback) and leaves last setpoint as-is.
  // Returns true on success.
  bool mfcOn();

  // Turn controller “off”: sets setpoint to 0 sccm and stops continuous measurement.
  bool mfcOff();

  // Set a new mass flow setpoint in sccm (e.g., 250.0 == 0.25 slm)
  bool mfcSetFlowSccm(float sccm);

  // Read current measured flow in sccm. Returns true on success.
  bool mfcReadFlowSccm(float& out_sccm, uint16_t avg_samples = 10);

  // Optional helpers
  bool reset();
  uint8_t address() const { return _addr; }

private:
  SensirionI2cSfx6xxx _dev;
  uint8_t _addr = SFC6XXX_I2C_ADDR;
  bool _begun = false;

  bool check(int16_t err, const __FlashStringHelper* where);
};
