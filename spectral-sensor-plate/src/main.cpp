#include <Arduino.h>

#include "mux_tca9548.h"
#include "as7341_array.h"

// ---------------- Pinout ----------------
#define RESET1 GPIO_NUM_16 // For Mux #1
#define RESET2 GPIO_NUM_15 // For Mux #2

#define HEATER1 GPIO_NUM_40 
#define HEATER2 GPIO_NUM_39 

#define CSO GPIO_NUM_34
#define MOSI GPIO_NUM_35
#define SCLK GPIO_NUM_36
#define MISO GPIO_NUM_37
#define CLR GPIO_NUM_12

#define SDA GPIO_NUM_8
#define SCL GPIO_NUM_9

#define USER_BUTTON GPIO_NUM_48

// Mux Mapping
// Mux #1 0x70
// Channel: 0, 1, 2, 3, 4, 5, 6, 7
// Sensor#: 8, 6, 4, 2, 1, 3, 5, 7

// Mux #2 0x71
// Channel: 0, 1, 2, 3, 4, 5, 6, 7
// Sensor#: 16, 14, 12, 10, 9, 11, 13, 15

// Optional: if you want I2C to run faster (AS7341 + mux usually fine at 400k)
static constexpr uint32_t I2C_FREQ = 400000;

Tca9548MuxManager* muxMgr = nullptr;
AS7341Array* asArray = nullptr;

AS7341Array::Reading r{};
AS7341Array::Reading all[16];

String action;
int req_index;
float req_power;

// ------ Prototypes ------
void setHeaterPower(int heater, int power);
static void printReading(uint8_t idx, const AS7341Array::Reading& r);
static void fatal(const char* msg);
static void fullShutdown();
static void wakeAll();
static void readSensor(int sensor);
static void readAllSensors();

// ---------------- Setup ----------------
void setup() {
  Serial.begin(115200);

  ledcSetup(1, 10000, 10);
  ledcAttachPin(HEATER1, 1);

  ledcSetup(2, 10000, 10);
  ledcAttachPin(HEATER2, 2);

  Wire.begin(SDA, SCL);
  Wire.setClock(I2C_FREQ);

  Tca9548MuxManager::Config muxCfg;
  muxCfg.wire = &Wire;
  muxCfg.mux1_addr = 0x70;
  muxCfg.mux2_addr = 0x71;
  muxCfg.reset1_gpio = (int)RESET1;
  muxCfg.reset2_gpio = (int)RESET2;
  muxCfg.sda_gpio = (int)SDA;
  muxCfg.scl_gpio = (int)SCL;
  muxCfg.i2c_retries = 3;
  muxCfg.retry_delay_ms = 2;
  muxCfg.wire_timeout_ms = 50;
  muxCfg.enable_bus_recovery = true;
  muxCfg.verify_after_select = true;

  static Tca9548MuxManager mux(muxCfg);
  muxMgr = &mux;

  // 1) Init muxes
  auto ms = muxMgr->begin();
  if (ms != Tca9548MuxManager::Status::Ok) {
    fatal("Mux init failed.");
  }

  AS7341Array::Settings s = AS7341Array::defaults();
  s.gain = AS7341_GAIN_1X;
  s.atime = 29;
  s.astep = 599;
  s.enable_sensor_led = false;
  s.keep_measurement_enabled = true;
  s.normalize_by_visible_sum = true;
  s.include_clear_nir_in_norm = true;

  static AS7341Array arr(*muxMgr, &Wire);
  asArray = &arr;

  // 2) Init all sensors once (this is the “heavy” step)
  auto ss = asArray->begin(s);
  if (ss != AS7341Array::Status::Ok) {
    fatal("AS7341 array init failed.");
  }

  Serial.println("Available functions:");
  Serial.println("setHeaterPower(int heater, int power)");
  Serial.println("readSensor(int sensor)");
  Serial.println("readAllSensors()");
  Serial.println("fullShutdown()");
  Serial.println("wakeAll()");

}

// ---------------- Loop ----------------
void loop() {
  delay(100);
  // Wait until data received from PC, via Serial (USB)
  if (Serial.available() > 0) {
    // data structure to receive = action(var1, var2..)

    // Read until open bracket to extract action, continue based on which action was requested
    action = Serial.readStringUntil('(');

    if (action == "setHeaterPower") {
      req_index = Serial.readStringUntil(',').toInt();
      req_power = Serial.readStringUntil(')').toFloat();

      setHeaterPower(req_index, req_power);
      Serial.println("#");
    }
    else if (action == "readSensor") {
      req_index = Serial.readStringUntil(')').toInt();
      
      readSensor(req_index);
      Serial.println("#");
    }
    else if (action == "readAllSensors") {
      (void)Serial.readStringUntil(')');
      
      readAllSensors();
      Serial.println("#");
    }
    else if (action == "fullShutdown") {
      (void)Serial.readStringUntil(')');
      
      fullShutdown();
      Serial.println("#");
    }
    else if (action == "wakeAll") {
      (void)Serial.readStringUntil(')');
      
      wakeAll();
        Serial.println("#");
    }
    else {
      Serial.println("UNKNOWN COMMAND");
    }
  }
}

// ---------------- Implementation ----------------

void setHeaterPower(int heater, int power) {
  power = constrain(power, 0, 100);

  if (heater < 1 || heater > 2) {
    Serial.println("Error: Invalid heater index. Must be 1 or 2.");
    return;
  }

  ledcWrite(heater, map(power, 0, 100, 0, 1023));
}

static void printReading(uint8_t idx, const AS7341Array::Reading& r) {
  Serial.printf("S%02u RAW  F1..F8:", idx);
  for (int i = 0; i < 8; i++) Serial.printf(" %u", r.f[i]);
  Serial.printf("  CLR:%u NIR:%u", r.clear, r.nir);
  if (r.likely_saturated) Serial.print("  [SAT?]");
  Serial.println();

  Serial.printf("S%02u NORM (denom=%.1f) F1..F8:", idx, r.denom);
  for (int i = 0; i < 8; i++) Serial.printf(" %.4f", r.nf[i]);
  Serial.printf("  CLR:%.4f NIR:%.4f\n", r.nclear, r.nnir);
}

static void fatal(const char* msg) {
  Serial.println(msg);
  while (true) delay(1000);
}

static void readSensor(int sensor) {
  auto st = asArray->readSensor(sensor, r);
  if (st == AS7341Array::Status::Ok || st == AS7341Array::Status::SaturatedHint) {
    printReading(1, r);
  } else {
    Serial.printf("Sensor 1 read failed (status=%u)\n", (unsigned)st);
  }
}

static void readAllSensors() {
  auto st = asArray->readAll(all);
  if (st == AS7341Array::Status::Ok) {
    for (uint8_t i = 1; i <= 16; i++) {
      printReading(i, all[i - 1]);
    }
  } else {
    Serial.printf("Read all sensors failed (status=%u)\n", (unsigned)st);
  }
}

static void fullShutdown() {
  auto st = asArray->shutdownAll();
  if (st == AS7341Array::Status::Ok) {
    muxMgr->resetBoth();
    muxMgr->disableAll();
    Serial.printf("Shutdown successful");
  } else {
    Serial.printf("Failed to shutdown (status=%u)\n", (unsigned)st);
  }
}

static void wakeAll() {
  auto st = asArray->wakeAll();
  if (st != AS7341Array::Status::Ok) {
    Serial.printf("Device ready");
  } else {
    Serial.printf("Failed to wake (status=%u)\n", (unsigned)st);
  }
}