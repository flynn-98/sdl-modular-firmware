#include <Arduino.h>
#include <esp_sleep.h>
#include <Adafruit_NeoPixel.h>
#include <Adafruit_SHT4x.h>
#include <ArduinoOTA.h>
#include <WiFi.h>

#include <AccelStepper.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// Local modules
#include "ble.h"
#include "led_animations.h"
#include "sensirion.h"

// ---------------- Pinout ----------------
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
#define DIR5  GPIO_NUM_8
#define EN5   GPIO_NUM_15

#define STEP6 GPIO_NUM_6
#define DIR6  GPIO_NUM_7
#define EN6   GPIO_NUM_5

#define STEP7 GPIO_NUM_14
#define DIR7  GPIO_NUM_13
#define EN7   GPIO_NUM_21

#define STEP8 GPIO_NUM_11
#define DIR8  GPIO_NUM_10
#define EN8   GPIO_NUM_12

#define LEDPIN  GPIO_NUM_9
#define SERVO1  GPIO_NUM_18
#define SERVO2  GPIO_NUM_17

#define BUTTON  GPIO_NUM_0

// ---------------- Steppers ----------------
AccelStepper STEPPER1(AccelStepper::DRIVER, STEP5, DIR5); 
AccelStepper STEPPER2(AccelStepper::DRIVER, STEP6, DIR6);
AccelStepper STEPPER3(AccelStepper::DRIVER, STEP7, DIR7);
AccelStepper STEPPER4(AccelStepper::DRIVER, STEP8, DIR8);

AccelStepper* steppers[4] = {&STEPPER1, &STEPPER2, &STEPPER3, &STEPPER4};
const int pwm_pins[4] = {PWM1, PWM2, PWM3, PWM4};
const int dir_pins[4] = {DIR1, DIR2, DIR3, DIR4};
const int en_pins[4]  = {EN5, EN6, EN7, EN8};

// ---------------- Sensors ----------------
#define SHT4x_DEFAULT_ADDR 0x46
Adafruit_SHT4x sht4 = Adafruit_SHT4x();
sensors_event_t hum, temp;

// ---------------- MFC ----------------
static SFC6000D MFC;
// ---------------- LED Animations ----------------
#define NUMPIXELS 7
static const uint8_t OUTER[6] = {0,1,2,3,4,5};
static const uint8_t CENTER_IDX = 6;
LedAnimations LEDS;

// ---------------- Kinematics ----------------
const float STEPS_REV   = 200.0;
const float MICROSTEPS  = 4.0;
const float ML_REV      = 0.4;   // ml/rev
const float back_flow   = 0;    // ml
const float default_flow_rate = 200.0; // ml/s
const float MAX_ACCEL   = 500.0 * MICROSTEPS; // microsteps/s^2

// ---------------- BLE UUIDs/Names ----------------
#define SERVICE_UUID        "41ae6296-8eb6-4e65-a114-2de9e1038255"
#define CHARACTERISTIC_UUID "b9caaaad-5236-4813-9bbe-826a79745934"
#define DEVICE_NAME         "MEAController"

// ---------------- Globals used in loop/handlers ----------------
float measured_temp = 0.0;
float measured_hum  = 0.0;

float flow_rate;
float vol;
float volumes[4];
float pwm;
float seconds;
int   motor;

String action;
String buffer;

unsigned long CurrentTime;
unsigned long ElapsedTime;
unsigned long LastCall = 0;
const unsigned long screenReset = 120;

// ------ Prototypes ------
void valveSignal(int valve, bool state);
long volToSteps(float vol);
void runSteppers();
void driveStepper(int motor, float vol, float flow_rate = default_flow_rate, float back_flow = back_flow);
void driveAllSteppers(float volumes[4], float flow_rate = default_flow_rate, float back_flow = back_flow);
void updateEnvironmentReadings();

// Route arg reads: prefer BLE message args when present, else Serial.
static inline String readArg(char delim = ',') {
  if (ble_message_received()) return ble_read_arg(delim);
  return Serial.readStringUntil(delim);
}

static inline void respond(const String& s) { ble_respond(s); }

// ---------------- Setup ----------------
void setup() {
  Serial.begin(115200);

  // GPIO + PWM + steppers
  for (int i = 0; i < 4; i++) {
    pinMode(pwm_pins[i], OUTPUT);
    pinMode(dir_pins[i], OUTPUT);
    pinMode(en_pins[i], OUTPUT);

    digitalWrite(pwm_pins[i], LOW);
    digitalWrite(dir_pins[i], LOW);
    digitalWrite(en_pins[i], HIGH);

    steppers[i]->setAcceleration(MAX_ACCEL);
    steppers[i]->setMaxSpeed((float)volToSteps(default_flow_rate));
  }

  // I2C + sensors + display
  Wire.begin(SDA, SCL, 400000);
  sht4.begin(&Wire);
  sht4.setPrecision(SHT4X_HIGH_PRECISION);

  MFC.begin(Wire);
  bool ok = MFC.mfcOn();
  Serial.println(ok ? "MFC enabled" : "Failed to enable MFC");

  // LEDs
  pinMode(LEDPIN, OUTPUT);
  LEDS.begin(LEDPIN, NUMPIXELS, CENTER_IDX, OUTER, 6, BLUE);

  pinMode(BUTTON, INPUT);

  Serial.println("Available functions:");
  Serial.println("singleStepperPump(int motor, float volume [ml], float flow_rate [ml/s])");
  Serial.println("multiStepperPump(float v1 [ml], float v2 [ml], float v3 [ml], float v4 [ml], float flow_rate [ml/s])");
  Serial.println("valveTimer(int valve, float seconds)");
  Serial.println("valveOpen(int valve)");
  Serial.println("valveClose(int valve)");
  Serial.println("getTemperature()");
  Serial.println("getHumidity()");
  Serial.println("statusCheck()");
  Serial.println("mfcOn()");
  Serial.println("mfcOff()");
  Serial.println("mfcSetFlow(float sccm)");
  Serial.println("mfcGetFlow()");
  Serial.println("mfcForceClose()");
  Serial.println("mfcForceCloseReset()");

  // BLE
  ble_begin(DEVICE_NAME, SERVICE_UUID, CHARACTERISTIC_UUID);

  if (ok) {
    LEDS.spinnerTimer(1, GREEN);
  }
  else {
    LEDS.spinnerTimer(1, RED);
  }
}

// ---------------- Loop ----------------
void loop() {
  delay(1000);

  // Read either BLE message or Serial command
  if (ble_message_received() || Serial.available() > 0) {
    LEDS.pulse(PURPLE);

    if (!ble_message_received()) {
      action = Serial.readStringUntil('(');
    } else {
      action = ble_get_action();
    }

    if (action == "singleStepperPump") {
      motor     = readArg(',').toInt();
      vol       = readArg(',').toFloat();
      flow_rate = readArg(')').toFloat();

      driveStepper(motor, vol, flow_rate);
      respond("# Pump action complete");
    }
    else if (action == "multiStepperPump") {
      volumes[0] = readArg(',').toFloat();
      volumes[1] = readArg(',').toFloat();
      volumes[2] = readArg(',').toFloat();
      volumes[3] = readArg(',').toFloat();
      flow_rate  = readArg(')').toFloat();

      driveAllSteppers(volumes, flow_rate);
      respond("# Pump action complete");
    }
    else if (action == "valveTimer") {
      motor   = readArg(',').toInt();
      seconds = readArg(')').toFloat();

      valveSignal(motor, true);
      LEDS.spinnerTimer(seconds, ORANGE);
      valveSignal(motor, false);
      respond("# Valve action complete");
    }
    else if (action == "valveOpen") {
      motor   = readArg(')').toInt();

      // turn off all valves first (in pairs)
      //for (int i = 1; i < 6; i+=2) {
      //  valveSignal(i, false);
      //}

      valveSignal(motor, true);
      respond("# Valve open");
    }
    else if (action == "valveClose") {
      motor   = readArg(')').toInt();
 
      valveSignal(motor, false);
      respond("# Valve closed");
    }
    else if (action == "getTemperature") {
      (void)readArg(')');
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
    }
    else if (action == "mfcOn") {
      (void)readArg(')');
      bool ok = MFC.mfcOn();
      respond(ok ? "# MFC on" : "MFC on failed");
    }
    else if (action == "mfcOff") {
      (void)readArg(')');
      bool ok = MFC.mfcOff();
      respond(ok ? "# MFC off" : "MFC off failed");
    }
    else if (action == "mfcSetFlow") {
      float sccm = readArg(')').toFloat();   // e.g. 250.0 = 0.25 slm
      bool ok = MFC.mfcSetFlowSccm(sccm);
      respond(ok ? "# Setpoint updated" : "Setpoint failed");
    }
    else if (action == "mfcGetFlow") {
      (void)readArg(')');
      float flow = NAN;
      bool ok = MFC.mfcReadFlowSccm(flow, 10);
      if (ok) respond(String(flow, 2)); else respond(String(0));
    }
    else if (action == "mfcForceClose") {
      (void)readArg(')');
      respond(MFC.mfcForceCloseValve() ? "# Valve forced CLOSED" : "Force-close failed");
    }
    else if (action == "mfcForceCloseReset") {
      (void)readArg(')');
      respond(MFC.mfcResetForceCloseValve() ? "# Valve control restored" : "Reset force-close failed");
    }
    else {
      Serial.println("Unknown command: " + action);
      LEDS.pulse(RED, 3);
    }

    // Clear BLE state (if any) for next cycle
    if (ble_message_received()) ble_clear_message();
  }
  else {
    CurrentTime = ceil(millis() / 1000.0);
    ElapsedTime = CurrentTime - LastCall;

    if (ElapsedTime > screenReset) {
      LastCall = CurrentTime;
    }

    // BLE housekeeping (connection status + re-adv)
    ble_loop();
  }
}

// ---------------- Implementation ----------------

long volToSteps(float vol) {
  return floor(MICROSTEPS * STEPS_REV * vol / ML_REV);
}

void runSteppers() {
  // Enable all drivers
  for (int i = 0; i < 4; i++) digitalWrite(en_pins[i], LOW);

  bool anyRunning;
  do {
    LEDS.run(); // non-blocking spinner
    anyRunning = false;
    for (int i = 0; i < 4; i++) {
      if (steppers[i]->distanceToGo() != 0) {
        steppers[i]->run();
        anyRunning = true;
      } else {
        digitalWrite(en_pins[i], HIGH); // disable when done
      }
    }
  } while (anyRunning);
}

void driveStepper(int motor, float vol, float flow_rate, float back_flow_local) {
  if (motor < 1 || motor > 4) {
    Serial.println("Error: Invalid stepper index (must be 1-4)");
    return;
  }

  digitalWrite(en_pins[motor - 1], LOW);
  steppers[motor - 1]->setMaxSpeed((float)volToSteps(flow_rate));

  steppers[motor - 1]->move(volToSteps(vol + back_flow_local));
  runSteppers();

  steppers[motor - 1]->move(volToSteps(-1 * back_flow_local));
  runSteppers();

  digitalWrite(en_pins[motor - 1], HIGH);

  LEDS.pulse(ORANGE);
}

void driveAllSteppers(float volumes[4], float flow_rate, float back_flow_local) {
  for (int i = 0; i < 4; i++) {
    if (volumes[i] != 0) {
      steppers[i]->setMaxSpeed((float)volToSteps(flow_rate));
      steppers[i]->move(volToSteps(volumes[i] + back_flow_local));
    }
  }
  runSteppers();

  for (int i = 0; i < 4; i++) {
    if (volumes[i] != 0) {
      steppers[i]->move(volToSteps(-1 * back_flow_local));
    }
  }
  runSteppers();

  LEDS.pulse(ORANGE);
}

void valveSignal(int valve, bool state) {
  // integer div + remainder = 12345678 -> 11223344
  int idx = ( (int)valve/2 ) + valve%2 - 1;

  int dir_pin = dir_pins[idx];
  int pwm_pin = pwm_pins[idx];

  if (valve < 1 || valve > 6) {
    Serial.println("Error: Invalid valve index. Must be 1–6.");
    return;
  }
  else if (state==false)
  {
    digitalWrite(dir_pin, LOW);
    digitalWrite(pwm_pin, LOW);
  }
  else if (valve%2 != 0)
  {
    // Odd number (trigger high on left pin)
    digitalWrite(dir_pin, LOW);
    digitalWrite(pwm_pin, HIGH);
  }
  else {
    // Even number (trigger high on right pin)
    digitalWrite(dir_pin, HIGH);
    digitalWrite(pwm_pin, LOW);
  }
  
}

void updateEnvironmentReadings() {
  sht4.getEvent(&hum, &temp);
  measured_temp = temp.temperature;
  measured_hum  = hum.relative_humidity;
}
