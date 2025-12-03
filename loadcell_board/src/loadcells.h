#pragma once
#include <Arduino.h>
#include "leds.h"

// Numbered channels for your four HX711s
static const uint8_t NUM_CHANNELS = 4;

// Hardcoded channel identifiers (for 1..60 across full system, pick any unique numbers)
extern uint8_t LC_IDS[NUM_CHANNELS];

// Pin map (matches your provided pinout)
extern const uint8_t LC_DOUT[NUM_CHANNELS];
extern const uint8_t LC_SCK [NUM_CHANNELS];

// Calibration scale factors (units per ADC count). Replace with your own!
extern float LC_SCALE[NUM_CHANNELS];   // e.g., grams per unit
extern long  LC_OFFSET[NUM_CHANNELS];  // stored offset after tare

void loadcellsInit();
void loadcellsTareAll();                          // tare every channel
bool loadcellsReadAll(float outMass[NUM_CHANNELS]); // returns true if all fresh
bool loadcellsCalibrateAll(float knownMassKg, uint8_t btn_pin, LedAnimations &leds, uint16_t settleMsPerStep = 2000);
