#include "mux_tca9548.h"

// ---------- Your sensor-to-mux mapping ----------
// Mux #1 0x70 channels 0..7 -> Sensor#: 8, 6, 4, 2, 1, 3, 5, 7
// Mux #2 0x71 channels 0..7 -> Sensor#: 16,14,12,10, 9,11,13,15

Tca9548MuxManager::Tca9548MuxManager(const Config& cfg)
: _cfg(cfg),
  _mux1(cfg.mux1_addr, cfg.wire),
  _mux2(cfg.mux2_addr, cfg.wire)
{}

bool Tca9548MuxManager::isMuxesConnected() {
  // TCA9548::isConnected() uses Wire beginTransmission/endTransmission
  return _mux1.isConnected() && _mux2.isConnected();
}

Tca9548MuxManager::Status Tca9548MuxManager::begin() {
  if (!_cfg.wire) return Status::NotBegun;

  _cfg.wire->setTimeOut(_cfg.wire_timeout_ms);

  // Hook up reset pins if provided (active-low assumed by library)
  if (_cfg.reset1_gpio >= 0) _mux1.setResetPin((uint8_t)_cfg.reset1_gpio);
  if (_cfg.reset2_gpio >= 0) _mux2.setResetPin((uint8_t)_cfg.reset2_gpio);

  // Force reads from device for verification (and cache coherency)
  _mux1.setForced(true);
  _mux2.setForced(true);

  // begin() also disables channels by default mask 0x00
  if (!_mux1.begin(0x00) || !_mux2.begin(0x00)) {
    _begun = false;
    return Status::NotConnected;
  }

  // Final sanity: ensure both are reachable
  if (!isMuxesConnected()) {
    _begun = false;
    return Status::NotConnected;
  }

  auto st = disableAll();
  _begun = (st == Status::Ok);
  return st;
}

Tca9548MuxManager::Route Tca9548MuxManager::routeForSensor(uint8_t sensorIndex) const {
  Route r{0, 0xFF};
  if (sensorIndex < 1 || sensorIndex > 16) return r;

  static const uint8_t mux1_ch_to_sensor[8] = {8, 6, 4, 2, 1, 3, 5, 7};
  static const uint8_t mux2_ch_to_sensor[8] = {16,14,12,10,9,11,13,15};

  for (uint8_t ch = 0; ch < 8; ch++) {
    if (mux1_ch_to_sensor[ch] == sensorIndex) return Route{1, ch};
    if (mux2_ch_to_sensor[ch] == sensorIndex) return Route{2, ch};
  }
  return r; // should never hit if mapping is complete
}

template<typename Fn>
Tca9548MuxManager::Status Tca9548MuxManager::withRetries(Fn fn) {
  Status last = Status::I2COpFail;
  for (uint8_t i = 0; i < _cfg.i2c_retries; i++) {
    last = fn();
    if (last == Status::Ok) return Status::Ok;

    if (_cfg.enable_bus_recovery) {
      recoverI2CBus();  // best-effort
    }
    delay(_cfg.retry_delay_ms);
  }
  return last;
}

Tca9548MuxManager::Status Tca9548MuxManager::disableAll() {
  if (!_cfg.wire) return Status::NotBegun;

  return withRetries([&]() -> Status {
    bool ok1 = _mux1.disableAllChannels();
    bool ok2 = _mux2.disableAllChannels();
    if (!ok1 || !ok2) return Status::I2COpFail;

    if (_cfg.verify_after_select) {
      uint8_t m1 = _mux1.getChannelMask();
      uint8_t m2 = _mux2.getChannelMask();
      if (m1 != 0x00 || m2 != 0x00) return Status::VerifyFail;
    }

    _currentSensor = 0;
    return Status::Ok;
  });
}

Tca9548MuxManager::Status Tca9548MuxManager::setExclusive(uint8_t muxId, uint8_t channel) {
  if (!_begun) return Status::NotBegun;
  if (channel > 7 || (muxId != 1 && muxId != 2)) return Status::BadSensorIndex;

  return withRetries([&]() -> Status {
    // Enforce global exclusivity: disable both, then enable exactly one channel
    if (!_mux1.disableAllChannels()) return Status::I2COpFail;
    if (!_mux2.disableAllChannels()) return Status::I2COpFail;

    bool ok = false;
    if (muxId == 1) ok = _mux1.selectChannel(channel);
    else            ok = _mux2.selectChannel(channel);
    if (!ok) return Status::I2COpFail;

    if (_cfg.verify_after_select) {
      uint8_t m1 = _mux1.getChannelMask();
      uint8_t m2 = _mux2.getChannelMask();

      uint8_t expected1 = (muxId == 1) ? (uint8_t)(1u << channel) : 0x00;
      uint8_t expected2 = (muxId == 2) ? (uint8_t)(1u << channel) : 0x00;

      if (m1 != expected1 || m2 != expected2) return Status::VerifyFail;
    }

    return Status::Ok;
  });
}

Tca9548MuxManager::Status Tca9548MuxManager::selectSensor(uint8_t sensorIndex) {
  if (!_begun) return Status::NotBegun;

  auto r = routeForSensor(sensorIndex);
  if (r.channel == 0xFF) return Status::BadSensorIndex;

  auto st = setExclusive(r.muxId, r.channel);
  if (st == Status::Ok) _currentSensor = sensorIndex;
  else _currentSensor = 0;

  return st;
}

Tca9548MuxManager::Status Tca9548MuxManager::resetMux1() {
  if (_cfg.reset1_gpio < 0) return disableAll();  // no pin -> best effort
  _mux1.reset();
  delay(2);
  // library reset doesn’t change cached mask; disableAll re-syncs/forces readback
  return disableAll();
}

Tca9548MuxManager::Status Tca9548MuxManager::resetMux2() {
  if (_cfg.reset2_gpio < 0) return disableAll();
  _mux2.reset();
  delay(2);
  return disableAll();
}

Tca9548MuxManager::Status Tca9548MuxManager::resetBoth() {
  if (_cfg.reset1_gpio >= 0) _mux1.reset();
  if (_cfg.reset2_gpio >= 0) _mux2.reset();
  delay(2);
  return disableAll();
}

Tca9548MuxManager::Status Tca9548MuxManager::recoverI2CBus() {
  if (_cfg.sda_gpio < 0 || _cfg.scl_gpio < 0) return Status::Ok;

  // Release lines
  pinMode(_cfg.sda_gpio, INPUT_PULLUP);
  pinMode(_cfg.scl_gpio, INPUT_PULLUP);
  delay(1);

  // If SDA is stuck low, clock SCL up to 9 pulses to free it
  if (digitalRead(_cfg.sda_gpio) == LOW) {
    pinMode(_cfg.scl_gpio, OUTPUT_OPEN_DRAIN);
    pinMode(_cfg.sda_gpio, INPUT_PULLUP);

    for (int i = 0; i < 9; i++) {
      digitalWrite(_cfg.scl_gpio, HIGH);
      delayMicroseconds(5);
      digitalWrite(_cfg.scl_gpio, LOW);
      delayMicroseconds(5);
      if (digitalRead(_cfg.sda_gpio) == HIGH) break;
    }
  }

  // Generate a STOP: SDA low -> SCL high -> SDA high
  pinMode(_cfg.sda_gpio, OUTPUT_OPEN_DRAIN);
  pinMode(_cfg.scl_gpio, OUTPUT_OPEN_DRAIN);

  digitalWrite(_cfg.sda_gpio, LOW);
  delayMicroseconds(5);
  digitalWrite(_cfg.scl_gpio, HIGH);
  delayMicroseconds(5);
  digitalWrite(_cfg.sda_gpio, HIGH);
  delayMicroseconds(5);

  // Back to pullups
  pinMode(_cfg.sda_gpio, INPUT_PULLUP);
  pinMode(_cfg.scl_gpio, INPUT_PULLUP);

  return Status::Ok;
}
