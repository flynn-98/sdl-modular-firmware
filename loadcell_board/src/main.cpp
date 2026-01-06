#include <Arduino.h>
#include "leds.h"
#include "loadcells.h"
#include "ble.h"

#include <string>
#include <stdio.h>

// --- Pins ---
static const uint8_t PIN_BUTTON_TARE = 42;   // active LOW with internal pull-up

// --- State ---
static const uint16_t BTN_DEBOUNCE_MS = 30;
static const uint16_t BTN_TARE_TRIGGER_MS = 60;

// --- add near top of main.cpp, after other consts ---
static const float   CAL_KNOWN_LOAD_KG  = 1.0f;       // EDIT to your calibration mass
static const uint32_t BTN_LONG_MS       = 5000;      // 5 seconds to enter calibration
static bool     btnDown = false;
static uint32_t btnDownAt = 0;

float masses[NUM_CHANNELS] = {0};
String action;
#define MSG_BUF_SIZE 256

#define SERVICE_UUID        "fc193d64-a716-4734-8002-abe9205b4dc8"
#define CHARACTERISTIC_UUID "e0535719-06e3-46e6-a89a-4dfeb0c4f01a"
#define DEVICE_NAME         "LoadCellBoard"

// --- LED config (moved implementation to led.*) ---
#define NUMPIXELS 2
#define LEDPIN 41
static const uint8_t OUTER[2] = {0,1};
static const uint8_t CENTER_IDX = 0; //not used
LedAnimations LEDS;

// ------ Prototypes ------
static void handleButton();
void buildMassMessage(char* buffer, size_t bufferSize, const float masses[], size_t n);
// Route arg reads: prefer BLE message args when present, else Serial.
static inline String readArg(char delim = ',') {
  if (ble_message_received()) return ble_read_arg(delim);
  return Serial.readStringUntil(delim);
}
static inline void respond(const String& s) { ble_respond(s); }

void setup() {
  Serial.begin(115200);
  delay(200);

  Serial.println("Load Cell Board V1");

  // LED init
  pinMode(LEDPIN, OUTPUT);
  LEDS.begin(LEDPIN, NUMPIXELS, CENTER_IDX, OUTER, 6);

  // Button
  pinMode(PIN_BUTTON_TARE, INPUT_PULLUP);

  Serial.println("Setting up load cells..");

  // Load cells
  loadcellsInit();
  loadcellsTareAll();                 // initial tare on boot

  // BLE
  ble_begin(DEVICE_NAME, SERVICE_UUID, CHARACTERISTIC_UUID);

  Serial.println("Available functions:");
  Serial.println("readCells()");
  Serial.println("tare()");
  Serial.println("statusCheck()");

  Serial.println("# Ready");
  LEDS.pulse(HAPPY);
}

void loop() {
  handleButton();

  delay(200);

  // Read either BLE message or Serial command
  if (ble_message_received() || Serial.available() > 0) {
    LEDS.pulse(TRANSMIT);

    if (!ble_message_received()) {
      action = Serial.readStringUntil('(');
    } else {
      action = ble_get_action();
    }
    if (action == "readCells") {
      (void)readArg(')');

      bool ok = loadcellsReadAll(masses);
      if (!ok) {
        LEDS.pulse(ERROR, 2);
        respond("Failed to read load cells");
      }
      else {
        char msg[MSG_BUF_SIZE];
        buildMassMessage(msg, sizeof(msg), masses, NUM_CHANNELS);
        respond(msg);
      }
    }
    else if (action == "tare") {
      (void)readArg(')');
      loadcellsTareAll();
      respond("# Done");
    }
    else if (action == "statusCheck") {
      (void)readArg(')');
      respond("# Ready");
      LEDS.pulse(BLUETOOTH);
    }
    else {
      (void)readArg(')');
      Serial.println("Unknown command: " + action);
      LEDS.pulse(ERROR, 2);
    }
  }
}

static void handleButton() {
  bool pressed = (digitalRead(PIN_BUTTON_TARE) == LOW);
  uint32_t now = millis();

  // edge detection (debounced)
  static bool stable = false;
  static bool last = false;
  static uint32_t lastChange = 0;

  if (pressed != last && (now - lastChange) > BTN_DEBOUNCE_MS) {
    lastChange = now;
    last = pressed;
    stable = true;
  } else {
    stable = false;
  }

  // track long press
  if (pressed && !btnDown) { btnDown = true; btnDownAt = now; }
  if (!pressed && btnDown) {
    // released
    uint32_t held = now - btnDownAt;
    btnDown = false;

    if (held >= BTN_LONG_MS) {
      // LONG PRESS -> CALIBRATE (No-load then known load, per channel)
      Serial.println("Entering calibration mode..");
      bool ok = loadcellsCalibrateAll(CAL_KNOWN_LOAD_KG, PIN_BUTTON_TARE, LEDS);
      if (!ok) LEDS.pulse(ERROR, 2);
      delay(1000); 
    } else if (held > BTN_TARE_TRIGGER_MS) {
      // SHORT PRESS -> TARE ALL
      Serial.println("Tare all readings..");
      LEDS.pulse(SEARCHING, 2);
      loadcellsTareAll();
      LEDS.pulse(ORANGE, 2);
    }
  }
}

void buildMassMessage(char* buffer, size_t bufferSize,
                      const float masses[], size_t n)
{
    size_t len = 0;

    len += snprintf(buffer + len, bufferSize - len, "Masses: ");

    for (size_t i = 0; i < n; ++i)
    {
        len += snprintf(buffer + len,
                        bufferSize - len,
                        "%.3f", masses[i]);

        if (i < n - 1)
            len += snprintf(buffer + len,
                            bufferSize - len,
                            ", ");
    }
}