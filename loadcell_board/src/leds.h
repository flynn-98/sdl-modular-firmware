#pragma once
#include <Arduino.h>

struct RGB {
  uint8_t r;
  uint8_t g;
  uint8_t b;
};

// Logical color IDs — extend as needed.
enum LedColor {
  ORANGE,
  NEUTRAL,
  BLUETOOTH,
  TRANSMIT,
  ERROR,
  HAPPY,
  WARNING,
  SLEEP,
  CO2,
  CALIBRATING,
  SEARCHING,
  LOW_BATTERY,
  OFF,
  COUNT
};

// Call once in setup (replaces pixels.begin())
void led_init(uint8_t pin, uint16_t numPixels = 3);

// Same signature and behavior as before
void pulseLEDs(LedColor color, int pulses = 1, int stepDelay = 5);

class LedAnimations {
public:
  // Setup the strip and spinner layout.
  // - pin: GPIO for the NeoPixel data
  // - numPixels: total count
  // - centerIdx: index of the center LED
  // - outerIdx: pointer to array of outer ring indices (clockwise)
  // - outerCount: number of indices in outerIdx
  // - spinColor: color used for the spinner
  void begin(uint8_t pin,
             uint16_t numPixels,
             uint8_t centerIdx,
             const uint8_t* outerIdx,
             uint8_t outerCount);

  // Call frequently for non-blocking spinner updates.
  void run(LedColor spinColor = ORANGE);

  // Blocking pulse fade for status indications.
  void pulse(LedColor color, int pulses = 1, int stepDelay = 5);

  // Run spinner for `seconds`, then overwrite with a pulse of `pulseColor`.
  void spinnerTimer(float seconds, LedColor pulseColor = ORANGE, LedColor spinColor = BLUETOOTH);

private:
  // Opaque impl to keep header light
  struct Impl;
  Impl* impl_ = nullptr;
};
