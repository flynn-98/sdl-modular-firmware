#include <Arduino.h>
#include <esp_sleep.h>
#include <Adafruit_NeoPixel.h>
#include <Adafruit_SHT4x.h>

#include <cmath>
#include <cstdint>

// Local modules
#include "ble.h"
#include "led_animations.h"

// ---------------- Pinout ----------------
#define ADC1 GPIO_NUM_1
#define PROBE1 GPIO_NUM_2
#define PROBE2 GPIO_NUM_4

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

static const int pwm_pins[4] = { PWM1, PWM2, PWM3, PWM4 };
static const int dir_pins[4] = { DIR1, DIR2, DIR3, DIR4 };

// ---------------- NTC ----------------
// Calibrated thermistor parameters (example):
// R0 at T0 (Kelvin), and Beta in Kelvin.
struct ThermistorBeta {
    float invT0;   // 1/T0
    float invB;    // 1/B
    float lnR0;    // ln(R0)
};

constexpr float R0   = 10000.0f;
constexpr float T0_K = 298.15f;    // 25°C
constexpr float B    = 3892.0f;

// Build once (constexpr) from your datasheet values.
constexpr ThermistorBeta make_thermistor(float R0, float T0_K, float B) {
    return ThermistorBeta{
        1.0f / T0_K,
        1.0f / B,
        std::log(R0)
    };
}

constexpr auto TH = make_thermistor(R0, T0_K, B);

constexpr uint32_t ADC_MAX = 4095;   // 12-bit ADC
constexpr float Rp = 10000.0f;       // 10k pull-up

// ---------------- Sensors ----------------
#define SHT4x_DEFAULT_ADDR 0x46
Adafruit_SHT4x sht4 = Adafruit_SHT4x();
sensors_event_t hum, temp;

// ---------------- LED Animations ----------------
#define NUMPIXELS 7
static const uint8_t OUTER[6] = {0,1,2,3,4,5};
static const uint8_t CENTER_IDX = 6;
LedAnimations LEDS;

// ---------------- BLE UUIDs/Names ----------------
#define SERVICE_UUID        "b71fbb66-f4f2-4b2e-870f-8eacc12489af"
#define CHARACTERISTIC_UUID "0ea834f6-64ca-4ca0-a2b4-47ba6862378e"
#define DEVICE_NAME         "HeatingModule"

// ---------------- Globals used in loop/handlers ----------------
float env_temp = 0.0;
float env_hum  = 0.0;

float pwm;
float seconds;
int   motor;

String action;
String buffer;

bool control_active = false;

float kp = 1;
float ki = 1;
const float int_limit = 100;

float setpoint1 = 25;
float setpoint2 = 25;

unsigned long CurrentTime;
unsigned long ElapsedTime;
unsigned long LastCall = 0;
unsigned long timeOut = 60;

// ---- PI state ----
static float i1 = 0.0f, i2 = 0.0f;   // integrators for system1/2
static uint32_t last_ms = 0;         // timebase for dt
static uint32_t lastPrint = 0;
static const float out_min = 0.0f;   // heater off
static const float out_max = 100.0f; // heater full power

// Optional: integrate only within sane dt range (avoids huge jumps on pauses)
static const float dt_min = 0.01f;   // 10 ms
static const float dt_max = 1.0f;    // 1 s

// ------ Prototypes ------
void pwmDrivingSignal(int motor, int power);
void updateEnvironmentReadings();

// Route arg reads: prefer BLE message args when present, else Serial.
static inline String readArg(char delim = ',') {
  if (ble_message_received()) return ble_read_arg(delim);
  return Serial.readStringUntil(delim);
}
static inline void respond(const String& s) { ble_respond(s); }

inline float ntc_temp_c_from_resistance(float R_ohm, const ThermistorBeta& th);
inline float ntc_resistance_from_adc(uint32_t adc_raw, uint32_t adc_max, float Rp);
float temperature_c(uint32_t adc_raw);
float getProbe1Temp();
float getProbe2Temp();

// ---------------- Setup ----------------
void setup() {
  Serial.begin(115200);

  // GPIO + PWM + steppers
  for (int i = 0; i < 4; i++) {
    pinMode(pwm_pins[i], OUTPUT);
    pinMode(dir_pins[i], OUTPUT);

    digitalWrite(pwm_pins[i], LOW);
    digitalWrite(dir_pins[i], LOW);

    ledcSetup(i, 10000, 10);
    ledcAttachPin(pwm_pins[i], i);
  }

  // I2C + sensors
  Wire.begin(SDA, SCL, 400000);
  sht4.begin(&Wire);
  sht4.setPrecision(SHT4X_HIGH_PRECISION);

  // LEDs
  pinMode(LEDPIN, OUTPUT);
  LEDS.begin(LEDPIN, NUMPIXELS, CENTER_IDX, OUTER, 6, BLUE);

  pinMode(BUTTON, INPUT_PULLUP);
  pinMode(PROBE1, INPUT);
  pinMode(PROBE2, INPUT);

  Serial.println("Available functions:");
  Serial.println("toggleControl(String state)");
  Serial.println("setGains(float kp, float ki)");
  Serial.println("setSetpoints(float setpoint1, float setpoint2)");
  Serial.println("getBoardTemp()");
  Serial.println("getProbe1Temp()");
  Serial.println("getProbe2Temp()");
  Serial.println("getBoardHum()");

  // BLE
  ble_begin(DEVICE_NAME, SERVICE_UUID, CHARACTERISTIC_UUID);

  LEDS.spinnerTimer(1, GREEN);
}

// ---------------- Loop ----------------
void loop() {
  delay(500);

  if (control_active) {
    const uint32_t now = millis();
    float dt = (last_ms == 0) ? 0.1f : (now - last_ms) / 1000.0f; // seconds
    last_ms = now;
    if (dt < dt_min) dt = dt_min;
    if (dt > dt_max) dt = dt_max;

    // --- System 1 ---
    const float pv1   = getProbe1Temp();          // °C
    const float e1    = setpoint1 - pv1;          // error
    const float p1    = kp * e1;                  // proportional
    const float ui1   = i1 + ki * e1 * dt;        // trial updated integrator
    float u1_unsat    = p1 + ui1;                 // before clamp
    float u1          = u1_unsat;
    if (u1 < out_min) u1 = out_min;
    if (u1 > out_max) u1 = out_max;

    // Anti-windup: only accept integrator step if it would not increase saturation
    const bool allow_i1 =
        (u1 == u1_unsat) ||                      // not saturated
        ((u1 == out_max) && (e1 < 0)) ||         // at top, error reduces
        ((u1 == out_min) && (e1 > 0));           // at bottom, error increases
    if (allow_i1) i1 = ui1;

    pwmDrivingSignal(1, (int)lroundf(u1));       // 0–100%

    // --- System 2 ---
    const float pv2   = getProbe2Temp();
    const float e2    = setpoint2 - pv2;
    const float p2    = kp * e2;
    const float ui2   = i2 + ki * e2 * dt;
    float u2_unsat    = p2 + ui2;
    float u2          = u2_unsat;
    if (u2 < out_min) u2 = out_min;
    if (u2 > out_max) u2 = out_max;

    const bool allow_i2 =
        (u2 == u2_unsat) ||
        ((u2 == out_max) && (e2 < 0)) ||
        ((u2 == out_min) && (e2 > 0));
    if (allow_i2) i2 = ui2;

    pwmDrivingSignal(2, (int)lroundf(u2));

    if (now - lastPrint >= 5000) {
      lastPrint = now;

      Serial.print("[System1] ");
      Serial.print("SP: ");   Serial.print(setpoint1, 1);  Serial.print(" °C, ");
      Serial.print("PV: ");   Serial.print(pv1, 1);        Serial.print(" °C, ");
      Serial.print("Err: ");  Serial.print(e1, 1);         Serial.print(" °C, ");
      Serial.print("Out: ");  Serial.print(u1, 1);         Serial.println(" %");

      Serial.print("[System2] ");
      Serial.print("SP: ");   Serial.print(setpoint2, 1);  Serial.print(" °C, ");
      Serial.print("PV: ");   Serial.print(pv2, 1);        Serial.print(" °C, ");
      Serial.print("Err: ");  Serial.print(e2, 1);         Serial.print(" °C, ");
      Serial.print("Out: ");  Serial.print(u2, 1);         Serial.println(" %");
    }
  }
  else {
    pwmDrivingSignal(1, 0);
    pwmDrivingSignal(2, 0);
  }

  // Read either BLE message or Serial command
  if (ble_message_received() || Serial.available() > 0) {
    LEDS.pulse(PURPLE);

    if (!ble_message_received()) {
      action = Serial.readStringUntil('(');
    } else {
      action = ble_get_action();
    }

    if (action == "toggleControl") {
      buffer = readArg(')');

      if (buffer == "true") {
        control_active = true;
        respond("# Heater turned ON");
      }
      else {
        control_active = false;
        respond("# Heater turned OFF");
      }
      
    }
    else if (action == "setGains") {
      kp = readArg(',').toFloat();
      ki = readArg(')').toFloat();
    
      respond("# New gains received");
    }
    else if (action == "setSetpoints") {
      setpoint1 = readArg(',').toFloat();
      setpoint2 = readArg(')').toFloat();
    
      respond("# New setpoints received");
    }
    else if (action == "getBoardTemp") {
      (void)readArg(')');
      updateEnvironmentReadings();
      respond(String(env_temp, 2));
    }
    else if (action == "getBoardHum") {
      (void)readArg(')');
      updateEnvironmentReadings();
      respond(String(env_hum, 2));
    }
    else if (action == "getProbe1Temp") {
      (void)readArg(')');
      respond(String(getProbe1Temp(), 2));
    }
    else if (action == "getProbe2Temp") {
      (void)readArg(')');
      respond(String(getProbe2Temp(), 2));
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

    if (ElapsedTime > timeOut) {
      LastCall = CurrentTime;
      // Do something?
    }

    // BLE housekeeping (connection status + re-adv)
    ble_loop();
  }
}

// ---------------- Implementation ----------------

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

// Fast path: R in ohms -> Temperature in Celsius.
inline float ntc_temp_c_from_resistance(float R_ohm, const ThermistorBeta& th) {
    // 1/T = 1/T0 + (1/B)*ln(R/R0)  ->  invT = invT0 + invB*(lnR - lnR0)
    const float lnR = std::log(R_ohm);
    const float invT = th.invT0 + th.invB * (lnR - th.lnR0);
    const float T_K  = 1.0f / invT;        // Kelvin
    return T_K - 273.15f;                  // Celsius
}

inline float ntc_resistance_from_adc(uint32_t adc_raw, uint32_t adc_max, float Rp) {
    // Vout/Vref = adc_raw/adc_max
    // Rntc = Rp * (adc_raw) / (adc_max - adc_raw)
    const float num = static_cast<float>(adc_raw);
    const float den = static_cast<float>(adc_max) - num;
    // Clamp to avoid div-by-zero at rails.
    const float safe_den = (den <= 0.5f) ? 0.5f : den;
    return Rp * (num / safe_den);
}

float temperature_c(uint32_t adc_raw) {
    const float R = ntc_resistance_from_adc(adc_raw, ADC_MAX, Rp);
    return ntc_temp_c_from_resistance(R, TH);
}

float getProbe1Temp() {
  return temperature_c(analogRead(PROBE1));
}

float getProbe2Temp() {
  return temperature_c(analogRead(PROBE2));
}

void updateEnvironmentReadings() {
  sht4.getEvent(&hum, &temp);
  env_temp = temp.temperature;
  env_hum  = hum.relative_humidity;
}
