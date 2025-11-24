#include <Arduino.h>
#include <esp_sleep.h>
#include <Adafruit_NeoPixel.h>
#include <Adafruit_SHT4x.h>
#include <ArduinoOTA.h>
#include <WiFi.h>

#include <AccelStepper.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#include <DFRobot_PH.h>
#include <EEPROM.h>

// Local modules
#include "ble.h"
#include "led_animations.h"

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

// ---------------- LED Animations ----------------
#define NUMPIXELS 7
static const uint8_t OUTER[6] = {0,1,2,3,4,5};
static const uint8_t CENTER_IDX = 6;
LedAnimations LEDS;

// ---------------- Kinematics ----------------
const float STEPS_REV   = 200.0;
const float MICROSTEPS  = 4.0;
const float ML_REV      = 0.094;   // ml/rev
const float back_flow   = 0.02;    // ml

const float default_flow_rate = 0.3; // ml/s
const float ecms_flow_rate = 0.01;
const float max_ecms_flow_rate = 0.1;

const float MAX_ACCEL   = 500.0 * MICROSTEPS; // microsteps/s^2

// ---------------- BLE UUIDs/Names ----------------
#define SERVICE_UUID        "82de634e-79b6-4d5e-b04a-116b66fec88c"
#define CHARACTERISTIC_UUID "91c92fda-19ec-4227-a1d1-e6f95c75d672"
#define DEVICE_NAME         "EC-MS-Controller"

// ---------------- Globals used in loop/handlers ----------------
float measured_temp = 0.0;
float measured_hum  = 0.0;

float flow_rate;
float vol;
float volumes[4];
float pwm;
float seconds;
int   motor;
float local_back_flow;

float voltage;
float ph_value;

String action;
String buffer;

bool valveOpen = false;

unsigned long CurrentTime;
unsigned long ElapsedTime;
unsigned long valveLastCall = 0;
unsigned long pumpLastCall = 0;
unsigned long valveResetTime = 1800; // 30mins max CO2 release
unsigned long pumpResetTime = 6 * 3600; // 6hrs

DFRobot_PH ph;

// ------ Prototypes ------
void pwmDrivingSignal(int motor, int power);
long volToSteps(float vol);
void runSteppers();
void driveStepper(int motor, float vol, float flow_rate = default_flow_rate, float local_back_flow = back_flow);
void driveAllSteppers(float volumes[4], float flow_rate = default_flow_rate, float local_back_flow = back_flow);
void closeValve();
void openValve();
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
  ph.begin();

  // GPIO + PWM + steppers
  for (int i = 0; i < 4; i++) {
    pinMode(pwm_pins[i], OUTPUT);
    pinMode(dir_pins[i], OUTPUT);
    pinMode(en_pins[i], OUTPUT);

    digitalWrite(pwm_pins[i], LOW);
    digitalWrite(dir_pins[i], LOW);
    digitalWrite(en_pins[i], HIGH);

    ledcSetup(i, 10000, 10);
    ledcAttachPin(pwm_pins[i], i);

    steppers[i]->setAcceleration(MAX_ACCEL);
    steppers[i]->setMaxSpeed((float)volToSteps(default_flow_rate));
  }

  // I2C + sensors + display
  Wire.begin(SDA, SCL, 400000);
  sht4.begin(&Wire);
  sht4.setPrecision(SHT4X_HIGH_PRECISION);

  // LEDs
  pinMode(LEDPIN, OUTPUT);
  LEDS.begin(LEDPIN, NUMPIXELS, CENTER_IDX, OUTER, 6, BLUE);

  pinMode(BUTTON, INPUT);

  closeValve();

  Serial.println("Available functions:");
  Serial.println("singleStepperPump(int motor, float volume [ml], float flow_rate [ml/s])");
  Serial.println("multiStepperPump(float v1 [ml], float v2 [ml], float v3 [ml], float v4 [ml], float flow_rate [ml/s])");
  Serial.println("transferPump(int motor, float power [+-%], float time [s])");
  Serial.println("getTemperature()");
  Serial.println("getHumidity()");
  Serial.println("statusCheck()");

  // BLE
  ble_begin(DEVICE_NAME, SERVICE_UUID, CHARACTERISTIC_UUID);

  LEDS.spinnerTimer(1, GREEN);
  Serial.println("# SDL Board Ready");
}

// ---------------- Loop ----------------
void loop() {
  delay(500);

  // Read either BLE message or Serial command
  if (ble_message_received() || Serial.available() > 0) {
    LEDS.pulse(PURPLE, 1, 1);
    pumpLastCall = ceil(millis() / 1000.0);

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
    else if (action == "transferPump") {
      motor   = readArg(',').toInt();
      pwm     = readArg(',').toFloat();
      seconds = readArg(')').toFloat();
      pwmDrivingSignal(motor, pwm);
      LEDS.spinnerTimer(seconds, ORANGE);
      pwmDrivingSignal(motor, 0);
      respond("# Transfer complete");
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
    else if (action == "releaseCO2") {
      seconds = readArg(')').toFloat();

      openValve();
      LEDS.spinnerTimer(seconds);
      closeValve();

      respond("# CO2 dosing complete");
    }
    else if (action == "addChemical") {
      vol = readArg(')').toFloat();
      driveStepper(1, vol);

      respond("# Chemical added");
    }
    else if (action == "sendToPh") {
      vol = readArg(')').toFloat();
      driveStepper(3, vol);

      respond("# Mixture sent to pH chamber");
    }
    else if (action == "addWater") {
      vol = readArg(')').toFloat();
      driveStepper(2, vol);

      respond("# Water added");
    }
    else if (action == "transferToECMS") {
      vol = readArg(',').toFloat();
      flow_rate = readArg(')').toFloat();
      
      // safety
      if (flow_rate > max_ecms_flow_rate) {
        flow_rate = max_ecms_flow_rate;
      }
      else if (flow_rate==0) {
        flow_rate = ecms_flow_rate;
      }

      driveStepper(motor=4, vol=vol, flow_rate=flow_rate, local_back_flow=0);

      respond("# Mixture sent to ECMS");
    }
    else if (action == "getPh") {
      (void)readArg(')');
      voltage = (analogRead(ADC3)/4095.0)*3300.0;;
      //ph_value = ph.readPH(voltage, 25)
      ph_value = -0.00579 * voltage + 15.40708;

      respond(String(ph_value, 2));
    }
    else if (action == "sendToWaste") {
      seconds = readArg(')').toFloat();

      pwmDrivingSignal(1, 75);
      LEDS.spinnerTimer(seconds, ORANGE);
      pwmDrivingSignal(1, 0);

      respond("# Sent to Waste");
    }
    else if (action == "refreshPhWater") {
      seconds = readArg(')').toFloat();
        
      pwmDrivingSignal(2, 75);
      LEDS.spinnerTimer(seconds, ORANGE);
      pwmDrivingSignal(2, 0);

      pwmDrivingSignal(2, -100);
      LEDS.spinnerTimer(seconds, ORANGE);
      pwmDrivingSignal(2, 0);

      respond("# Refreshed Water");
    }
    else {
      respond("Unknown command: " + action);
      LEDS.pulse(RED, 3);
    }

    // Clear BLE state (if any) for next cycle
    if (ble_message_received()) ble_clear_message();
  }
  else if (digitalRead(BUTTON)==LOW) {
    if (valveOpen) {
      closeValve();
    }
    else {
      openValve();
    }

    // delay to allow button release
    LEDS.spinnerTimer(2, RED);
  }
  else {
    CurrentTime = ceil(millis() / 1000.0);
    
    ElapsedTime = CurrentTime - valveLastCall;
    if (ElapsedTime > valveResetTime) {
      valveLastCall = CurrentTime;
      if (openValve) {
        closeValve();
      }
    }

    ElapsedTime = CurrentTime - pumpLastCall;
    if (ElapsedTime > pumpResetTime) {
      pumpLastCall = CurrentTime;
      for (int i = 0; i < 4; i++) {
        volumes[i] = 0.15;
      }
      driveAllSteppers(volumes, ecms_flow_rate);
      for (int i = 0; i < 4; i++) {
        volumes[i] = -0.2;
      }
      driveAllSteppers(volumes, ecms_flow_rate);
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

void driveStepper(int motor, float vol, float flow_rate, float local_back_flow) {
  if (motor < 1 || motor > 4) {
    Serial.println("Error: Invalid stepper index (must be 1-4)");
    return;
  }

  digitalWrite(en_pins[motor - 1], LOW);
  steppers[motor - 1]->setMaxSpeed((float)volToSteps(flow_rate));

  steppers[motor - 1]->move(volToSteps(vol + local_back_flow));
  runSteppers();

  steppers[motor - 1]->move(volToSteps(-1 * local_back_flow));
  runSteppers();

  digitalWrite(en_pins[motor - 1], HIGH);

  LEDS.pulse(ORANGE);
}

void driveAllSteppers(float volumes[4], float flow_rate, float local_back_flow) {
  for (int i = 0; i < 4; i++) {
    if (volumes[i] != 0) {
      steppers[i]->setMaxSpeed((float)volToSteps(flow_rate));
      steppers[i]->move(volToSteps(volumes[i] + local_back_flow));
    }
  }
  runSteppers();

  for (int i = 0; i < 4; i++) {
    if (volumes[i] != 0) {
      steppers[i]->move(volToSteps(-1 * local_back_flow));
    }
  }
  runSteppers();

  LEDS.pulse(ORANGE);
}

void pwmDrivingSignal(int motor, int power) {
  power = constrain(power, -100, 100);

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

void updateEnvironmentReadings() {
  sht4.getEvent(&hum, &temp);
  measured_temp = temp.temperature;
  measured_hum  = hum.relative_humidity;
}
 
void closeValve() {
    pwmDrivingSignal(4, 0);
    valveOpen = false;
}

void openValve() {
    pwmDrivingSignal(4, 55);
    valveOpen = true;
    valveLastCall = ceil(millis() / 1000.0);
}