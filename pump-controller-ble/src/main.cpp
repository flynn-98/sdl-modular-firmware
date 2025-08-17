#include <Arduino.h>
#include <esp_sleep.h>
#include <Adafruit_NeoPixel.h>
#include <Adafruit_SHT4x.h>
#include <ArduinoOTA.h>

// C99 libraries
#include <cstdlib>
#include <string.h>
#include <time.h>

// Libraries for MQTT client and WiFi connection
#include <WiFi.h>

// BLE
#include <Preferences.h>
#include <NimBLEDevice.h>

#include <AccelStepper.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define ADC1 GPIO_NUM_1
#define ADC2 GPIO_NUM_2
#define ADC3 GPIO_NUM_4

#define SDA GPIO_NUM_42
#define SCL GPIO_NUM_41

#define PWM1 GPIO_NUM_40
#define DIR1 GPIO_NUM_39

#define PWM2 GPIO_NUM_38
#define DIR2 GPIO_NUM_37

#define PWM3 GPIO_NUM_36
#define DIR3 GPIO_NUM_35

#define PWM4 GPIO_NUM_48
#define DIR4 GPIO_NUM_47

#define STEP5 GPIO_NUM_16
#define DIR5 GPIO_NUM_8
#define EN5 GPIO_NUM_15

#define STEP6 GPIO_NUM_6
#define DIR6 GPIO_NUM_7
#define EN6 GPIO_NUM_5

#define STEP7 GPIO_NUM_14
#define DIR7 GPIO_NUM_13
#define EN7 GPIO_NUM_21

#define STEP8 GPIO_NUM_11
#define DIR8 GPIO_NUM_10
#define EN8 GPIO_NUM_12

#define LEDPIN GPIO_NUM_9
#define SERVO1 GPIO_NUM_18
#define SERVO2 GPIO_NUM_17

#define BUTTON GPIO_NUM_0

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

AccelStepper STEPPER1(AccelStepper::DRIVER, STEP5, DIR5); 
AccelStepper STEPPER2(AccelStepper::DRIVER, STEP6, DIR6);
AccelStepper STEPPER3(AccelStepper::DRIVER, STEP7, DIR7);
AccelStepper STEPPER4(AccelStepper::DRIVER, STEP8, DIR8);

// Array of pointers to stepper objects
AccelStepper* steppers[4] = {&STEPPER1, &STEPPER2, &STEPPER3, &STEPPER4};

// SHT40-CD1B-R3 settings
#define SHT4x_DEFAULT_ADDR 0x46
Adafruit_SHT4x sht4 = Adafruit_SHT4x();
sensors_event_t hum, temp;

// NEO_KHZ800 800 KHz bitstream (most NeoPixel products w/WS2812 LEDs)
// NEO_GRB Pixels are wired for GRB bitstream (most NeoPixel products)
#define NUMPIXELS 7
Adafruit_NeoPixel pixels(NUMPIXELS, LEDPIN);

struct RGB {
  uint8_t r;
  uint8_t g;
  uint8_t b;
};

enum LedColor {
  BRIGHT_WHITE,
  BLUE,
  PURPLE,
  RED,
  GREEN,
  YELLOW,
  ORANGE,
  OFF
};

const RGB colorMap[] = {
  {255, 255, 255},  // BRIGHT WHITE 
  {84, 139, 227},   // SOFT BLUE (Spinner)
  {189, 84, 227},   // SOFT PURPLE (Received message)
  {227, 84, 84},    // SOFT RED (Error)
  {84, 227, 125},   // SOFT GREEN (Start up success)
  {250, 239, 27},   // SOFT YELLOW
  {255, 166, 51},   // SOFT ORANGE (Motor finished)
  {0, 0, 0}         // OFF
};

// ---- Animator (non-blocking) ----
struct SpinnerPulse {
  enum State { SPIN, CENTER_PULSE } state = SPIN;

  Adafruit_NeoPixel* strip;
  const uint8_t* outer;     // indices of 6 ring LEDs (clockwise)
  uint8_t outerCount;
  uint8_t centerIdx;

  // Colors (by enum lookup into colorMap)
  LedColor spinColor = BLUE;     // ring

  // Timing / behavior
  uint16_t stepIntervalMs = 70;      // spinner step duration
  uint8_t  tailLen = 3;              // head + trailing LEDs
  uint16_t pulseUpMs = 300;          // fade-in time
  uint16_t pulseDownMs = 350;        // fade-out time

  // Internals
  unsigned long tLastStep = 0;
  unsigned long tStateStart = 0;
  uint8_t head = 0;                  // 0..outerCount-1

  SpinnerPulse(Adafruit_NeoPixel* s,
               const uint8_t* outerIdx, uint8_t nOuter, uint8_t center)
  : strip(s), outer(outerIdx), outerCount(nOuter), centerIdx(center) {}

  inline uint32_t mkColor(const RGB& c, uint8_t scale = 255) const {
    // scale (0..255), integer math
    uint8_t r = (uint16_t)c.r * scale / 255;
    uint8_t g = (uint16_t)c.g * scale / 255;
    uint8_t b = (uint16_t)c.b * scale / 255;
    return strip->Color(r, g, b);
  }

  // Tail brightness profile (head -> tail)
  inline uint8_t tailLevel(uint8_t d) const {
    static const uint8_t levels[6] = {255, 140, 60, 25, 10, 4};
    return (d < 6) ? levels[d] : 0;
  }

  void begin(LedColor spinC) {
    spinColor  = spinC;
    head = 0;
    tLastStep = millis();
    tStateStart = tLastStep;

    for (uint8_t i = 0; i < strip->numPixels(); ++i) strip->setPixelColor(i, 0);
    strip->show();
  }

  void run() {
    unsigned long now = millis();
    if ((now - tLastStep) >= stepIntervalMs) {
          tLastStep = now;
          stepSpinnerFrame();
        }
  }

  void stepSpinnerFrame() {
    head = (head + 1) % outerCount;

    // clear ring
    for (uint8_t i = 0; i < outerCount; ++i)
      strip->setPixelColor(outer[i], 0);

    // head + tail
    for (uint8_t t = 0; t < tailLen; ++t) {
      uint8_t idx = (head + outerCount - t) % outerCount;
      uint8_t lvl = tailLevel(t);
      if (lvl == 0) break;
      strip->setPixelColor(outer[idx], mkColor(colorMap[spinColor], lvl));
    }

    // center off during spin
    strip->setPixelColor(centerIdx, 0);
    strip->show();
  }
};

const uint8_t OUTER[6] = {0,1,2,3,4,5};
SpinnerPulse anim(&pixels, OUTER, 6, 6);

// Arrays to map motor index to DIR/PWM pins
const int pwm_pins[4] = {PWM1, PWM2, PWM3, PWM4};
const int dir_pins[4] = {DIR1, DIR2, DIR3, DIR4};

const int en_pins[4] = {EN5, EN6, EN7, EN8};

const float STEPS_REV = 200.0;
const float MICROSTEPS = 4.0;
const float ML_REV = 0.094; //ml/rev
const float back_flow = 0.02; //ml

const float MAX_ACCEL = 300.0 * MICROSTEPS; //microsteps/s2
const float MAX_SPEED = 425.0 * MICROSTEPS; //microsteps/s2

// BLE additions
static NimBLECharacteristic* pCharacteristic;
bool messageReceived = false;

#define SERVICE_UUID        "7b5c5357-7ddd-44a4-8419-bc6b6bd4b74b"
#define CHARACTERISTIC_UUID "42bb8b48-f737-4435-adba-f578eba53675"
#define DEVICE_NAME         "PumpController"

// --- BLE/command shared state ---
static bool bleConnected = false;
static bool wasConnected = false;

static uint32_t lastAdvKick = 0;
static bool advRestartPending = false;
static uint32_t advRetryAt = 0;

// For BLE-originated commands: "action(arg1,arg2,...)"
static String bleParams;
static size_t blePos = 0;  // current cursor into bleParams

// put function declarations here:
void pwmDrivingSignal(int motor, int power);
long volToSteps(float vol);
void spinnerTimer(float seconds);
void runSteppers();
void driveStepper(int motor, float vol, float back_flow = back_flow);
void driveAllSteppers(float volumes[4], float back_flow = back_flow);
void writeToOLED(String message = "No message.");
void updateEnvironmentReadings();
void pulseLEDs(LedColor color, int pulses = 1, int stepDelay = 5);
void startBLEProvisioning();
static String nextBleToken(char delim = ',');
static inline String readArg(char delim = ',');
static inline void respond(const String& s);
static inline void scheduleAdvRestart(uint32_t delayMs = 400);

float measured_temp = 0.0;
float measured_hum = 0.0;

float speed;
float vol;
float volumes[4];
float pwm;
float seconds;
int motor;

String action;
String buffer;

unsigned long CurrentTime;
unsigned long ElapsedTime;

unsigned long LastCall = 0;
const unsigned long screenReset = 120;

class commandCallback : public NimBLECharacteristicCallbacks {
  void onWrite(NimBLECharacteristic* pChar, NimBLEConnInfo& /*connInfo*/) override {
    std::string value = pChar->getValue();
    if (value.empty()) return;

    String payload(value.c_str());
    payload.trim();
    int open = payload.indexOf('(');
    int close = payload.lastIndexOf(')');
    if (open <= 0 || close < 0 || close <= open) {
      // malformed input; ignore
      return;
    }

    action    = payload.substring(0, open);
    bleParams = payload.substring(open + 1, close); // "a,b,c"
    blePos    = 0;

    messageReceived = true; // main loop will consume bleParams
  }
};

class ServerCallbacks : public NimBLEServerCallbacks {
  void onConnect(NimBLEServer* /*s*/) { 
    Serial.println("BLE Connected"); 
  }
  void onDisconnect(NimBLEServer* /*s*/) { 
    Serial.println("BLE Disconnected; re-advertising");
    // Use the same advertising object; older NimBLEs are picky here.
    NimBLEAdvertising* adv = NimBLEDevice::getAdvertising();
    bool ok = adv->start();
    Serial.println(ok ? "[BLE] Advertising restarted" : "[BLE] Advertising restart FAILED");
  }
};

void setup() {
  // put your setup code here, to run once:

  // set up pins not handled by other libraries
  for (int i = 0; i < 4; i++) {
    pinMode(pwm_pins[i], OUTPUT);
    pinMode(dir_pins[i], OUTPUT);
    pinMode(en_pins[i], OUTPUT);

    // set default OFFs
    digitalWrite(pwm_pins[i], LOW);
    digitalWrite(dir_pins[i], LOW);
    digitalWrite(en_pins[i], HIGH);

    ledcSetup(i, 10000, 10); // Channel i, 10kHz, 10-bit resolution
    ledcAttachPin(pwm_pins[i], i);

    // default stepper speed settings
    steppers[i]->setAcceleration(MAX_ACCEL);
    steppers[i]->setMaxSpeed(MAX_SPEED);
  }

  Serial.begin(115200);

  Wire.begin(SDA, SCL, 400000);
  sht4.begin(&Wire);
  sht4.setPrecision(SHT4X_HIGH_PRECISION);

  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println("Pump Controller w/ 2x Transfer Pumps");
  }
  else {
    Serial.println("Pump Controller w/ OLED Screen + 4x Waste Pumps");
  }

  writeToOLED("Initialising pumpbot..");

  // Begin neopixels
  pinMode(LEDPIN, OUTPUT);
  pixels.begin();
  anim.begin(BLUE);

  pinMode(BUTTON, INPUT);

  writeToOLED("Initialisation successful!");

  Serial.println("Available functions:");
  Serial.println("singleStepperPump(int motor, float volume [ml])");
  Serial.println("multiStepperPump(float volume1 [ml], float volume2 [ml], float volume3 [ml], float volume4 [ml])");
  Serial.println("transferPump(int motor, float power [+-%], float time [s])");
  Serial.println("getTemperature()");
  Serial.println("getHumidity()");
  Serial.println("statusCheck()");
  Serial.println("displayMessage(String message)");

  // Begin bluetooth
  startBLEProvisioning();

  pulseLEDs(GREEN, 1, 20);
  writeToOLED("Waiting for command..");
}

void loop() {
  // Main code here, to run repeatedly on a loop
  delay(1000);
  
  // Wait until data received from PC, via Serial (USB) of BLE
  if (messageReceived || Serial.available() > 0) {
    pulseLEDs(PURPLE);

    // data structure to receive = action(var1, var2..)
    if (!messageReceived) {
      action = Serial.readStringUntil('(');
    }
    // else action already updated in ble callback
      
    if (action == "singleStepperPump") {
      motor = readArg(',').toInt();      
      vol   = readArg(')').toFloat();           
      driveStepper(motor, vol);
      respond("# Pump action complete");         
    }
    else if (action == "multiStepperPump") {
      volumes[0] = readArg(',').toFloat();
      volumes[1] = readArg(',').toFloat();
      volumes[2] = readArg(',').toFloat();
      volumes[3] = readArg(')').toFloat();
      driveAllSteppers(volumes);
      respond("# Pump action complete");
    }
    else if (action == "transferPump") {
      motor   = readArg(',').toInt();
      pwm     = readArg(',').toFloat();
      seconds = readArg(')').toFloat();
      pwmDrivingSignal(motor, pwm);
      spinnerTimer(seconds);
      pwmDrivingSignal(motor, 0);
      respond("# Transfer complete");
    }
    else if (action == "getTemperature") {
      (void)readArg(')'); // consume ')'
      updateEnvironmentReadings();
      respond(String(measured_temp, 2));
    }
    else if (action == "getHumidity") {
      (void)readArg(')');
      updateEnvironmentReadings();
      respond(String(measured_hum, 2));
    }
    else if (action == "statusCheck") {
      (void)readArg(')');
      respond("# Controller available");
      pulseLEDs(GREEN, 2);
    }
    else if (action == "displayMessage") {
      buffer = readArg(')');
      writeToOLED(buffer);
      LastCall = ceil(millis() / 1000);
      respond("# Message received");
    }
    else {
      // Report back to PC if confused
      Serial.println("Unknown command: " + action);
      pulseLEDs(RED, 3);
    }

    // Reset BLE related vars
    blePos = 0;
    bleParams = "";
    messageReceived = false;
  }

  else {
    CurrentTime = ceil( millis() / 1000 );
    ElapsedTime = CurrentTime - LastCall;

    if (ElapsedTime > screenReset) {
        writeToOLED("Waiting...");
        LastCall = CurrentTime;
    }

    NimBLEServer* srv = NimBLEDevice::getServer();
    bleConnected = (srv && srv->getConnectedCount() > 0);

    // Edge: connected -> disconnected
    if (wasConnected && !bleConnected) {
      Serial.println("[BLE] Detected disconnect; scheduling re-advertise");
      scheduleAdvRestart();  // ~0.4s works well on macOS
    }
    // Edge: disconnected -> connected cancels any pending restart
    if (!wasConnected && bleConnected) {
      advRestartPending = false;
    }

    wasConnected = bleConnected;

    // Fire the scheduled restart (once), with idempotent check
    if (advRestartPending && (int32_t)(millis() - advRetryAt) >= 0) {
      NimBLEAdvertising* adv = NimBLEDevice::getAdvertising();
      if (adv && !adv->isAdvertising() && !bleConnected) {
        bool ok = adv->start();
        Serial.println(ok ? "[BLE] Advertising restarted" : "[BLE] Advertising restart FAILED");
      }
      advRestartPending = false;  // done either way
    }

    // --- BLE advertising watchdog (covers missed/late disconnects) ---
    //if (CurrentTime - lastAdvKick > 2) {
    //  lastAdvKick = CurrentTime;
    //  NimBLEServer* srv = NimBLEDevice::getServer();
    //  if (srv && srv->getConnectedCount() == 0) {
    //    bool ok = NimBLEDevice::getAdvertising()->start(); // harmless if already advertising
    //    Serial.println(ok ? "BLE Command channel restarted!" : "BLE advertising START FAILED!");
    //  }
    //}

  }

}

long volToSteps(float vol) {
    return floor(MICROSTEPS * STEPS_REV * vol / ML_REV);
};

void driveStepper(int motor, float vol, float back_flow) {
  // Volume in ml

  // Validate motor index
  if (motor < 1 || motor > 4) {
    Serial.println("Error: Invalid stepper index (must be 1-4)");
    return;
  }

  // Enable driver
  digitalWrite(en_pins[motor - 1], LOW);

  // Shift forward to account for last back_flow
  steppers[motor - 1]->move(volToSteps(back_flow));
  runSteppers();

  // Set target
  steppers[motor - 1]->move(volToSteps(vol));

  // Run to target position (blocking)
  runSteppers();

  // Shift backwards to prevent drips
  steppers[motor - 1]->move(volToSteps(-1 * back_flow));
  runSteppers();

  // Disable driver to save power/heat
  digitalWrite(en_pins[motor - 1], HIGH);

  // Overwrite spinner
  pulseLEDs(ORANGE);
}

void runSteppers() {
  // Enable all drivers
  for (int i = 0; i < 4; i++) {
    digitalWrite(en_pins[i], LOW);
  }

  // Run all steppers until all are done
  bool anyRunning = true;
  do {
    anim.run();

    anyRunning = false;
    for (int i = 0; i < 4; i++) {
      if (steppers[i]->distanceToGo() != 0) {
        steppers[i]->run();
        anyRunning = true;
      }
      else {
        // Disable when done
        digitalWrite(en_pins[i], HIGH);
      }
    }
  } while (anyRunning);
}

void spinnerTimer(float seconds) {
  unsigned long secondsLong = (long)ceil(seconds);
  CurrentTime = ceil( millis() / 1000 );

  do {
    anim.run();
  }
  while ((ceil( millis() / 1000 ) - CurrentTime) < secondsLong);
  
  // Overwrite spinner
  pulseLEDs(ORANGE);
}

void driveAllSteppers(float volumes[4], float back_flow) {
  // Volume in ml

  // Shift forward to account for last back_flow
  for (int i = 0; i < 4; i++) {
    steppers[i]->move(volToSteps(back_flow));
  }
  runSteppers();

  // Set targets
  for (int i = 0; i < 4; i++) {
    steppers[i]->move(volToSteps(volumes[i]));
  }
  runSteppers();

  // Shift backwards to prevent drips
  for (int i = 0; i < 4; i++) {
    steppers[i]->move(volToSteps(-1 * back_flow));
  }
  runSteppers();

  // Overwrite spinner
  pulseLEDs(ORANGE);
}

void pwmDrivingSignal(int motor, int power) {
  power = constrain(power, -100, 100);

  // Check valid motor index
  if (motor < 1 || motor > 4) {
    Serial.println("Error: Invalid motor index. Must be 1–4.");
    return;
  }

  int dir_pin = dir_pins[motor - 1];

  if (power >= 0) {
    digitalWrite(dir_pin, HIGH);  // Forward
    ledcWrite(motor - 1, 1023 - map(power, 0, 100, 0, 1023));
  } else {
    digitalWrite(dir_pin, LOW);   // Reverse
    ledcWrite(motor - 1, map(-power, 0, 100, 0, 1023));
  }
}

void pulseLEDs(LedColor color, int pulses, int stepDelay) 
{
  RGB target = colorMap[color];

  for (int p = 0; p < pulses; p++) {
    // Fade in
    for (int i = 0; i <= 255; i += 5) {
      for (int j = 0; j < NUMPIXELS; j++) {
        pixels.setPixelColor(j,
          (target.r * i) / 255,
          (target.g * i) / 255,
          (target.b * i) / 255
        );
      }
      pixels.show();
      delay(stepDelay);
    }

    // Fade out
    for (int i = 255; i >= 0; i -= 5) {
      for (int j = 0; j < NUMPIXELS; j++) {
        pixels.setPixelColor(j,
          (target.r * i) / 255,
          (target.g * i) / 255,
          (target.b * i) / 255
        );
      }
      pixels.show();
      delay(stepDelay);
    }
  }
}

void updateEnvironmentReadings() {
  sht4.getEvent(&hum, &temp);

  measured_temp = temp.temperature;
  measured_hum = hum.relative_humidity;
}

void writeToOLED(String message) {
  updateEnvironmentReadings();
  display.clearDisplay();

  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  display.setCursor(0, 0);
  display.print(message);

  display.setCursor(0, 15);
  display.print("----------");

  // Temperature
  display.setCursor(0, 30);
  display.print(measured_temp, 1); // 1 decimal place
  display.write(247);     // ° symbol
  display.print("C");

  // Humidity
  display.setCursor(0, 45);
  display.print(measured_hum, 1);
  display.print("%RH");

  display.display(); // Push buffer to display
}

void startBLEProvisioning() {
  Serial.println("Attempting to start BLE command channel..");

  NimBLEDevice::init(DEVICE_NAME);
  NimBLEDevice::setPower(ESP_PWR_LVL_P9);
  NimBLEDevice::setSecurityAuth(false, false, true);
  // Optional: bump MTU if you plan longer messages
  // NimBLEDevice::setMTU(128);

  NimBLEServer* pServer = NimBLEDevice::createServer();
  pServer->setCallbacks(new ServerCallbacks());
  NimBLEService* pService = pServer->createService(SERVICE_UUID);

  pCharacteristic = pService->createCharacteristic(
      CHARACTERISTIC_UUID,
      NIMBLE_PROPERTY::READ     |
      NIMBLE_PROPERTY::WRITE    |
      NIMBLE_PROPERTY::WRITE_NR |
      NIMBLE_PROPERTY::NOTIFY
  );
  pCharacteristic->setCallbacks(new commandCallback());

  pService->start();

  NimBLEAdvertising* ad = NimBLEDevice::getAdvertising();
  ad->addServiceUUID(SERVICE_UUID);
  ad->setAppearance(0x00);

  NimBLEAdvertisementData advData;
  advData.addServiceUUID(SERVICE_UUID);

  NimBLEAdvertisementData scanResp;
  scanResp.setName(DEVICE_NAME);

  ad->setAdvertisementData(advData);
  ad->setScanResponseData(scanResp);

  bool ok = ad->start();
  Serial.println(ok ? "[BLE] Initial advertising started" : "[BLE] Advertising restart FAILED");
}


static String nextBleToken(char delim) {
  // Return next token delimited by delim from bleParams
  if (blePos >= bleParams.length()) return String("");
  int end = bleParams.indexOf(delim, blePos);
  String token;
  if (end < 0) { token = bleParams.substring(blePos); blePos = bleParams.length(); }
  else { token = bleParams.substring(blePos, end); blePos = end + 1; }
  token.trim();
  return token;
}

static inline String readArg(char delim) {
  if (messageReceived) {
    return nextBleToken(delim);
  } else {
    return Serial.readStringUntil(delim);
  }
}

static inline void respond(const String& s) {
  Serial.println(s);
  if (!pCharacteristic) return;
  pCharacteristic->setValue(s.c_str());
  // Notify unconditionally; if nobody subscribed, this is a no-op on older NimBLE
  pCharacteristic->notify();
}

// Restart scheduler to readvertise bluetooth after disconnect
static inline void scheduleAdvRestart(uint32_t delayMs) {
  advRestartPending = true;
  advRetryAt = millis() + delayMs;   // debounce so CoreBluetooth fully releases
}