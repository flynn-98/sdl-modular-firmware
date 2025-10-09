#include "sensirion.h"

bool SFC6000D::begin(TwoWire& wire, uint8_t i2c_addr) {
  _addr = i2c_addr;
  _dev.begin(wire, _addr);
  _begun = true;
  return true;
}

bool SFC6000D::mfcOn() {
  if (!_begun) return false;
  // Best-effort stop first (in case it was already running)
  (void)_dev.stopContinuousMeasurement();

  // CO2-only as requested
  int16_t err = _dev.startCo2ContinuousMeasurement();
  return logIfError(err, F("startCo2ContinuousMeasurement"));
}

bool SFC6000D::mfcOff() {
  if (!_begun) return false;

  // Set setpoint to 0 slm (0 sccm) and reset pointer as required by the driver docs
  int16_t err = _dev.updateSetpoint(0.0f);      // slm
  if (!ok(err)) return logIfError(err, F("updateSetpoint(0 slm)"));
  (void)_dev.resetPointerToMeasurementBuffer();

  // Best-effort stop
  (void)_dev.stopContinuousMeasurement();
  return true;
}

bool SFC6000D::mfcSetFlowSccm(float sccm) {
  if (!_begun) return false;
  if (sccm < 0.0f) sccm = 0.0f;

  const float slm = sccm / 1000.0f;             // driver expects slm
  int16_t err = _dev.updateSetpoint(slm);
  if (!ok(err)) return logIfError(err, F("updateSetpoint(slm)"));

  // Per driver note: reset pointer after updating setpoint
  (void)_dev.resetPointerToMeasurementBuffer();
  return true;
}

bool SFC6000D::mfcReadFlowSccm(float& out_sccm, uint16_t avg_samples, uint16_t inter_sample_ms) {
  if (!_begun) return false;
  if (avg_samples == 0) avg_samples = 1;

  float sum_slm = 0.0f;
  uint16_t good = 0;

  for (uint16_t i = 0; i < avg_samples; ++i) {
    float slm = NAN;
    int16_t err = _dev.readFlow(slm);           // returns slm
    if (ok(err)) {
      sum_slm += slm;
      ++good;
    } else {
      // Non-fatal: skip this sample, keep trying
      logIfError(err, F("readFlow"));
    }
    if (inter_sample_ms) delay(inter_sample_ms);
  }

  if (good == 0) return false;
  const float avg_slm = sum_slm / good;
  out_sccm = avg_slm * 1000.0f;                 // convert to sccm
  return true;
}

bool SFC6000D::readTemperatureC(float& out_degC) {
  if (!_begun) return false;
  int16_t err = _dev.readTemperature(out_degC);
  return logIfError(err, F("readTemperature"));
}

bool SFC6000D::logIfError(int16_t err, const __FlashStringHelper* where) {
  if (ok(err)) return true;
  Serial.print(F("[SFC] "));
  Serial.print(where);
  Serial.print(F(" err="));
  Serial.println(err);
  return false;
}
