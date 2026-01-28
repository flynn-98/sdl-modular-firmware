#include "as7341_array.h"

static constexpr uint8_t AS7341_ADDR = AS7341_I2CADDR_DEFAULT; // 0x39

AS7341Array::AS7341Array(Tca9548MuxManager& muxMgr, TwoWire* wire)
: _mux(muxMgr), _wire(wire) {}

AS7341Array::Status AS7341Array::begin(const Settings& settings) {
  _settings = settings;

  // Ensure mux is up
  if (_mux.currentSensor() == 0) {
    // ok; mux may still be begun; begin() belongs to mux module setup
  }

  // Initialise all sensors once.
  for (uint8_t i = 1; i <= 16; i++) {
    auto st = initOneSensor(i);
    if (st != Status::Ok) {
      _begun = false;
      return st;
    }
  }

  // Leave mux in safe state
  _mux.disableAll();
  _begun = true;
  return Status::Ok;
}

AS7341Array::Status AS7341Array::applySettingsAll(const Settings& settings) {
  _settings = settings;

  for (uint8_t i = 1; i <= 16; i++) {
    auto st = select(i);
    if (st != Status::Ok) return st;

    // Apply configuration (no begin())
    if (!_as.setGain(_settings.gain)) return Status::SensorI2CError;
    if (!_as.setATIME(_settings.atime)) return Status::SensorI2CError;
    if (!_as.setASTEP(_settings.astep)) return Status::SensorI2CError;

    // LED (internal)
    if (!_as.enableLED(_settings.enable_sensor_led)) return Status::SensorI2CError;
    if (_settings.enable_sensor_led) {
      if (!_as.setLEDCurrent(_settings.led_current_ma)) return Status::SensorI2CError;
    }

    // Measurement engine
    if (!_as.enableSpectralMeasurement(_settings.keep_measurement_enabled))
      return Status::SensorI2CError;
  }

  _mux.disableAll();
  return Status::Ok;
}

AS7341Array::Status AS7341Array::select(uint8_t sensorIndex) {
  auto ms = _mux.selectSensor(sensorIndex);
  if (ms != Tca9548MuxManager::Status::Ok) return Status::MuxError;
  return Status::Ok;
}

AS7341Array::Status AS7341Array::initOneSensor(uint8_t sensorIndex) {
  auto st = select(sensorIndex);
  if (st != Status::Ok) return st;

  // We must call begin() while the channel is selected, otherwise we’d be
  // probing nothing / the wrong sensor.
  if (!_as.begin(AS7341_ADDR, _wire)) {
    return Status::SensorNotFound;
  }

  // Apply initial configuration tuned for bright backlight through liquids:
  // - Low gain to avoid saturation
  // - Moderate integration time; tweak later if needed
  if (!_as.setGain(_settings.gain)) return Status::SensorI2CError;
  if (!_as.setATIME(_settings.atime)) return Status::SensorI2CError;
  if (!_as.setASTEP(_settings.astep)) return Status::SensorI2CError;

  // Internal LED driver is usually irrelevant if you have an external LED panel.
  if (!_as.enableLED(_settings.enable_sensor_led)) return Status::SensorI2CError;
  if (_settings.enable_sensor_led) {
    if (!_as.setLEDCurrent(_settings.led_current_ma)) return Status::SensorI2CError;
  }

  // Enable measurement so later reads don't need re-arming
  if (!_as.enableSpectralMeasurement(_settings.keep_measurement_enabled))
    return Status::SensorI2CError;

  return Status::Ok;
}

void AS7341Array::flashIndicatorLED() {
  if (!_settings.enable_sensor_led) return;

  _as.enableLED(true);
  _as.setLEDCurrent(5);        // very dim
  delay(5);                    // 5 ms = very visible
  _as.enableLED(false);
}

AS7341Array::Status AS7341Array::readRaw12(uint16_t raw12[12]) {
  // Adafruit readAllChannels(uint16_t*) returns bool
  if (!_as.readAllChannels(raw12)) return Status::SensorI2CError;
  return Status::Ok;
}

void AS7341Array::unpackRaw(const uint16_t raw12[12], Reading& out) {
  // Adafruit library internally stores 12 values; common ordering is:
  // [F1,F2,F3,F4,CLEAR,NIR,F5,F6,F7,F8,CLEAR_2,NIR_2]
  // We expose F1..F8 + CLEAR + NIR, taking the first CLEAR/NIR pair.
  out.f[0] = raw12[0];
  out.f[1] = raw12[1];
  out.f[2] = raw12[2];
  out.f[3] = raw12[3];
  out.clear = raw12[4];
  out.nir   = raw12[5];
  out.f[4] = raw12[6];
  out.f[5] = raw12[7];
  out.f[6] = raw12[8];
  out.f[7] = raw12[9];

  out.valid = true;
  out.likely_saturated = false;
  out.denom = 0.0f;

  // init normalized fields
  for (int i = 0; i < 8; i++) out.nf[i] = 0.0f;
  out.nclear = 0.0f;
  out.nnir = 0.0f;
}

void AS7341Array::normalize(Reading& io) {
  float denom = 0.0f;

  if (_settings.normalize_by_visible_sum) {
    for (int i = 0; i < 8; i++) denom += (float)io.f[i];
  } else {
    denom = (float)io.clear;
  }

  io.denom = denom;

  if (denom <= 0.0f) {
    // leave normalized at 0
    return;
  }

  for (int i = 0; i < 8; i++) io.nf[i] = (float)io.f[i] / denom;

  if (_settings.include_clear_nir_in_norm) {
    io.nclear = (float)io.clear / denom;
    io.nnir   = (float)io.nir   / denom;
  }
}

bool AS7341Array::saturationHeuristic(const Reading& r) const {
  // Heuristic only: if any channel is very close to 16-bit max, you're saturated.
  // Real saturation status exists in STATUS2, but Adafruit lib doesn't expose it directly.
  const uint16_t SAT = 65500;
  if (r.clear >= SAT || r.nir >= SAT) return true;
  for (int i = 0; i < 8; i++) if (r.f[i] >= SAT) return true;
  return false;
}

AS7341Array::Status AS7341Array::readSensor(uint8_t sensorIndex, Reading& out) {
  if (!_begun) return Status::NotBegun;
  if (sensorIndex < 1 || sensorIndex > 16) return Status::BadSensorIndex;

  auto st = select(sensorIndex);
  if (st != Status::Ok) return st;

  // show selected LED
  flashIndicatorLED();

  uint16_t raw12[12] = {0};
  st = readRaw12(raw12);
  if (st != Status::Ok) return st;

  unpackRaw(raw12, out);
  normalize(out);

  out.likely_saturated = saturationHeuristic(out);
  if (out.likely_saturated) {
    // Not a hard error; caller may still want the data.
    return Status::SaturatedHint;
  }

  return Status::Ok;
}

AS7341Array::Status AS7341Array::readAll(Reading out[16]) {
  if (!_begun) return Status::NotBegun;

  Status worst = Status::Ok;
  for (uint8_t i = 1; i <= 16; i++) {
    auto st = readSensor(i, out[i - 1]);
    // keep going even if one sensor saturates/errors
    if (st != Status::Ok && worst == Status::Ok) worst = st;
  }

  // Leave mux safe/off after scan
  _mux.disableAll();
  return worst;
}

AS7341Array::Status AS7341Array::shutdownAll() {
  // Power down every sensor. This takes time but is deterministic.
  for (uint8_t i = 1; i <= 16; i++) {
    auto st = select(i);
    if (st != Status::Ok) {
      // still try to close mux channels
      _mux.disableAll();
      return st;
    }

    // Disable measurement + LED + power
    _as.enableLED(false);
    _as.enableSpectralMeasurement(false);
    _as.powerEnable(false);
  }

  _mux.disableAll();
  _begun = false;
  return Status::Ok;
}

AS7341Array::Status AS7341Array::wakeAll() {
  // Optional: if you ever see stuck bus issues
  // _mux.recoverI2CBus();
  // _mux.resetBoth();

  // Ensure clean mux state
  if (_mux.disableAll() != Tca9548MuxManager::Status::Ok) return Status::MuxError;

  for (uint8_t i = 1; i <= 16; i++) {
    auto st = select(i);
    if (st != Status::Ok) return st;

    // Bring sensor back up
    _as.powerEnable(true);

    // Re-apply config (don’t assume retention after power-down)
    if (!_as.setGain(_settings.gain)) return Status::SensorI2CError;
    if (!_as.setATIME(_settings.atime)) return Status::SensorI2CError;
    if (!_as.setASTEP(_settings.astep)) return Status::SensorI2CError;

    if (!_as.enableLED(_settings.enable_sensor_led)) return Status::SensorI2CError;
    if (_settings.enable_sensor_led) {
      if (!_as.setLEDCurrent(_settings.led_current_ma)) return Status::SensorI2CError;
    }

    if (!_as.enableSpectralMeasurement(_settings.keep_measurement_enabled))
      return Status::SensorI2CError;
  }

  _mux.disableAll();
  _begun = true;
  return Status::Ok;
}