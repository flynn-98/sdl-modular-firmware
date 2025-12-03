#include "storage.h"
#include <Preferences.h>

static Preferences prefs;
static const char* NVS_NS = "cb_cal";   // NVS namespace
static const char* KEY_CNT = "count";
static const char* KEY_SCL = "scales";
static const char* KEY_OFF = "offsets";

void storageInit() {
  // open RW; it’s fine to keep Preferences open for the app lifetime
  if (opened) return;
  // RW mode; begin() returns true on success
  opened = prefs.begin(NVS_NS, /*readOnly=*/false);

	if (!opened) {
		Serial.println("Failed to open NVS");
	}
}

void storageEnd() {
	if (!opened) return;

	prefs.end();
	opened = false;
}

bool storageLoadCalib(float* scales, long* offsets, uint8_t count) {
  storageInit();
  uint8_t storedCount = prefs.getUChar(KEY_CNT, 0);
  if (storedCount == 0 || storedCount > count) return false;

  size_t needS = storedCount * sizeof(float);
  size_t needO = storedCount * sizeof(long);

  // readBytes returns bytes actually read; must match requested length
  if (prefs.getBytesLength(KEY_SCL) != needS) return false;
  if (prefs.getBytesLength(KEY_OFF) != needO) return false;

  size_t gotS = prefs.getBytes(KEY_SCL, scales, needS);
  size_t gotO = prefs.getBytes(KEY_OFF, offsets, needO);
  if (gotS != needS || gotO != needO) return false;

	storageEnd();

  // If caller supports more channels than stored, leave the rest as-is
  return true;
}

bool storageSaveCalib(const float* scales, const long* offsets, uint8_t count) {
  if (count == 0) return false;

	storageInit();
  bool ok = true;
  ok &= prefs.putUChar(KEY_CNT, count) == 1;
  ok &= prefs.putBytes(KEY_SCL, scales, count * sizeof(float)) == (count * sizeof(float));
  ok &= prefs.putBytes(KEY_OFF, offsets, count * sizeof(long))  == (count * sizeof(long));
	storageEnd();
  return ok;
}

bool storageClearCalib() {
  storageInit();
  bool ok = true;
  ok &= prefs.remove(KEY_CNT);
  ok &= prefs.remove(KEY_SCL);
  ok &= prefs.remove(KEY_OFF);
	storageEnd();
  return ok;
}
