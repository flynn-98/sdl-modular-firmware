#include "SFC6000D.h"

static char g_errmsg[96];

bool SFC6000D::begin(TwoWire* wire, uint8_t i2c_addr) {
  _addr = i2c_addr;
  _dev.begin(*wire, _addr);
  // Not strictly required, but a soft reset helps if the sensor was left running
  // (SFC stays powered from 24 V even when MCU reboots).
  if (!reset()) {
    // continue anyway; some firmwares refuse reset while measuring
  }
  _begun = true;
  return true;
}

bool SFC6000D::mfcOn() {
  if (!_begun) return false;
  // Make sure any previous loop is stopped, then start the measurement loop (for readback).
  int16_t err = _dev.stopContinuousMeasurement();
  if (err != NO_ERROR && err != STATUS_CODE_EXECUTION_FAILURE) {
    // ignore benign “nothing to stop” errors
    if (!check(err, F("stopContinuousMeasurement"))) return false;
  }
  err = _dev.SFC6XXX_START_MEAS_FN();  // startAirContinuousMeasurement() by default
  return check(err, F("start*ContinuousMeasurement"));
}

bool SFC6000D::mfcOff() {
  if (!_begun) return false;

  // Setpoint = 0 (close valve)
  int16_t err = _dev.setSetpoint(0.0f);
  if (!check(err, F("setSetpoint(0)"))) return false;

  // Stop the measurement loop to save a few mA on I2C activity
  err = _dev.stopContinuousMeasurement();
  if (err != NO_ERROR && err != STATUS_CODE_EXECUTION_FAILURE) {
    return check(err, F("stopContinuousMeasurement"));
  }
  return true;
}

bool SFC6000D::mfcSetFlowSccm(float sccm) {
  if (!_begun) return false;
  if (sccm < 0.0f) sccm = 0.0f;  // basic guard
  // Library takes setpoint in sccm for SFC6xxx devices.
  int16_t err = _dev.setSetpoint(sccm);
  return check(err, F("setSetpoint(sccm)"));
}

bool SFC6000D::mfcReadFlowSccm(float& out_sccm, uint16_t avg_samples) {
  if (!_begun) return false;

  // Use the averaged read; fall back to single if small sample count
  if (avg_samples < 2) {
    int16_t err = _dev.readMeasuredValue(out_sccm);
    return check(err, F("readMeasuredValue"));
  } else {
    int16_t err = _dev.readAveragedMeasuredValue(avg_samples, out_sccm);
    return check(err, F("readAveragedMeasuredValue"));
  }
}

bool SFC6000D::reset() {
  int16_t err = _dev.deviceReset();
  // Some firmware keeps measuring across MCU resets; small delay after reset helps.
  delay(5);
  // If reset isn’t supported while measuring, don’t fail the whole init.
  if (err != NO_ERROR) {
    errorToString(err, g_errmsg, sizeof(g_errmsg));
    // Uncomment if you want to see the reason on the console:
    // Serial.printf("[SFC] deviceReset(): %s\n", g_errmsg);
    // Don’t treat as fatal
  }
  return true;
}

bool SFC6000D::check(int16_t err, const __FlashStringHelper* where) {
  if (err == NO_ERROR) return true;
  errorToString(err, g_errmsg, sizeof(g_errmsg));
  // Minimal, non-spammy logging. Replace with your BLE logger if you prefer.
  Serial.print(F("[SFC] "));
  Serial.print(where);
  Serial.print(F(" -> "));
  Serial.println(g_errmsg);
  return false;
}
