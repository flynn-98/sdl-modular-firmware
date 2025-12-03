#pragma once
#include <Arduino.h>

static bool opened = false;

// Initialize NVS/Preferences (safe to call multiple times)
void storageInit();

// Save/load calibration for up to `count` channels.
// Returns true on success (and data applied), false if nothing stored yet or invalid.
bool storageLoadCalib(float* scales, long* offsets, uint8_t count);
bool storageSaveCalib(const float* scales, const long* offsets, uint8_t count);

// Optional: clear stored calibration (factory reset)
bool storageClearCalib();
