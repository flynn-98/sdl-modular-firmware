#pragma once
#include <Arduino.h>
#include <Wire.h>

#include "mux_tca9548.h"
#include "Adafruit_AS7341.h"

// Manages 16x AS7341 behind 2x TCA9548 muxes (all sensors at 0x39).
// Uses muxMgr.selectSensor(idx) to make exactly one sensor visible at a time.
class AS7341Array {
public:
  enum class Status : uint8_t {
    Ok = 0,
    BadSensorIndex,
    MuxError,
    SensorNotFound,
    SensorI2CError,
    NotBegun,
    SaturatedHint,
  };

  // "Best guess" settings for strong white LED backlight through liquids:
  // - Keep gain low to avoid saturation.
  // - Keep integration time moderate; increase only if signal is too noisy.
  struct Settings {
    as7341_gain_t gain = AS7341_GAIN_1X;

    // Integration settings:
    // TINT ~= (ATIME + 1) * (ASTEP + 1) * 2.78us
    // These defaults give a moderate integration time suitable for scanning 16 sensors.
    uint8_t  atime = 29;     // moderate
    uint16_t astep = 599;    // moderate

    // Sensor LED driver (AS7341 internal LED pin). Often unused if you have external LED panels.
    bool enable_sensor_led = false;
    uint16_t led_current_ma = 4;  // safe low default if enabled

    // Keep spectral measurement enabled after init for fastest reads.
    bool keep_measurement_enabled = true;

    // Normalization mode:
    // - If true, normalize F1..F8 by sum(F1..F8).
    // - If false, normalize by CLEAR.
    bool normalize_by_visible_sum = true;

    // If true, also include CLEAR + NIR in normalized outputs (using same denominator).
    bool include_clear_nir_in_norm = true;
  };

  static Settings defaults() { return Settings(); }

  // Output container for one read.
  // Adafruit lib reads 12 slots; we expose the useful 10 channels (F1..F8 + CLEAR + NIR).
  struct Reading {
    // Raw channels
    uint16_t f[8];       // F1..F8 in wavelength order
    uint16_t clear;      // CLEAR
    uint16_t nir;        // NIR

    // Normalized channels (0..1-ish as float)
    float nf[8];
    float nclear;
    float nnir;

    // Denominator used for normalization
    float denom;

    // Simple flags
    bool valid;
    bool likely_saturated;  // heuristic only
  };

  AS7341Array(Tca9548MuxManager& muxMgr, TwoWire* wire = &Wire);

  // Configure defaults/settings + initialise all sensors once.
  Status begin();  // uses defaults
  Status begin(const Settings& settings);

  // Apply settings to all sensors (without re-calling begin()).
  Status applySettingsAll(const Settings& settings);

  // Read one sensor (1..16).
  Status readSensor(uint8_t sensorIndex, Reading& out);

  // Convenience: read all 16 sensors into provided array[16]
  Status readAll(Reading out[16]);

  // Power down all sensors and close mux channels.
  Status shutdownAll();

  // Wake all sensors after shutdown.
  Status wakeAll();

  // Last used settings
  const Settings& settings() const { return _settings; }

private:
  Tca9548MuxManager& _mux;
  TwoWire* _wire;
  Settings _settings;
  bool _begun = false;

  // Reused driver instance (talks to whichever sensor is currently mux-selected)
  Adafruit_AS7341 _as;

  // Internal helpers
  Status initOneSensor(uint8_t sensorIndex);
  Status select(uint8_t sensorIndex);
  void flashIndicatorLED();

  Status readRaw12(uint16_t raw12[12]);

  void unpackRaw(const uint16_t raw12[12], Reading& out);
  void normalize(Reading& io);

  bool saturationHeuristic(const Reading& r) const;
};
