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

// Additional headers
#include "SerialLogger.h"

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

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

AccelStepper STEPPER1(AccelStepper::DRIVER, STEP5, DIR5); 
AccelStepper STEPPER2(AccelStepper::DRIVER, STEP6, DIR6);
AccelStepper STEPPER3(AccelStepper::DRIVER, STEP7, DIR7);
AccelStepper STEPPER4(AccelStepper::DRIVER, STEP8, DIR8);

// SHT40-CD1B-R3 settings
#define SHT4x_DEFAULT_ADDR 0x46
Adafruit_SHT4x sht4 = Adafruit_SHT4x();
sensors_event_t hum, temp;

// NEO_KHZ800 800 KHz bitstream (most NeoPixel products w/WS2812 LEDs)
// NEO_GRB Pixels are wired for GRB bitstream (most NeoPixel products)
#define NUMPIXELS 7
Adafruit_NeoPixel pixels(NUMPIXELS, LEDPIN);

String ssid;
String password;

struct RGB {
  uint8_t r;
  uint8_t g;
  uint8_t b;
};

enum LedColor {
  LED_WHITE,
  BLUETOOTH,
  TRANSMIT,
  ERROR,
  HAPPY,
  OFF
};

const RGB colorMap[] = {
  {127, 127, 127},  // WHITE
  {30, 50, 120},   // BLUETOOTH
  {70, 30, 116},   // SPACE PURPLE / TRANSMIT
  {110, 24, 30},    // RED
  {26, 90, 34},    // GREEN
  {0, 0, 0}         // OFF
};

// Arrays to map motor index to DIR/PWM pins
const int pwm_pins[4] = {PWM1, PWM2, PWM3, PWM4};
const int dir_pins[4] = {DIR1, DIR2, DIR3, DIR4};

const int en_pins[4] = {EN5, EN6, EN7, EN8};

const float STEPS_REV = 200.0;
const float MICROSTEPS = 4.0;

const float MAX_ACCEL = 300.0 * MICROSTEPS; //microsteps/s2
const float MAX_SPEED = 1000.0 * MICROSTEPS; //microsteps/s2

// put function declarations here:
static void connectToWiFi();
void pwmDrivingSignal(int motor, int power);
void drawCloudBerry(float scale);
void pulseLEDs(LedColor color, int pulses = 1, int stepDelay = 5);

void setup() {
  // put your setup code here, to run once:
  pinMode(LEDPIN, OUTPUT);

  for (int i = 0; i < 4; i++) {
    pinMode(pwm_pins[i], OUTPUT);
    pinMode(dir_pins[i], OUTPUT);
    pinMode(en_pins[i], OUTPUT);
    
    digitalWrite(pwm_pins[i], LOW);
    digitalWrite(dir_pins[i], LOW);
    digitalWrite(en_pins[i], HIGH);

    ledcSetup(i, 10000, 10); // Channel i, 10kHz, 10-bit resolution
    ledcAttachPin(pwm_pins[i], i);
  }

  STEPPER1.setAcceleration(MAX_ACCEL);
  STEPPER1.setMaxSpeed(MAX_SPEED);

  STEPPER2.setAcceleration(MAX_ACCEL);
  STEPPER2.setMaxSpeed(MAX_SPEED);

  STEPPER3.setAcceleration(MAX_ACCEL);
  STEPPER3.setMaxSpeed(MAX_SPEED);

  STEPPER4.setAcceleration(MAX_ACCEL);
  STEPPER4.setMaxSpeed(MAX_SPEED);

  Wire.begin(SDA, SCL, 400000);
  pixels.begin();

  sht4.begin(&Wire);
  sht4.setPrecision(SHT4X_HIGH_PRECISION);
  sht4.getEvent(&hum, &temp); // populates both temp and humidity

  float measured_temp = temp.temperature;
  float measured_hum = hum.relative_humidity;

  Logger.Info("T Reading: " + String(measured_temp, 2));
  Logger.Info("H Reading: " + String(measured_hum, 2));

  pwmDrivingSignal(1, -50);

  //STEPPER1.move(10000);
  //digitalWrite(en_pins[0], LOW);
  //STEPPER1.runToPosition();
  //digitalWrite(en_pins[0], HIGH);

  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Logger.Info(F("OLED not found!"));
    while (true);
  }

  display.clearDisplay();
  display.display();

  pulseLEDs(HAPPY, 1, 20);
  delay(2000);
}

void loop() {
  //pulseLEDs(TRANSMIT);

  // Pulsing scale using sine wave
  static float angle = 0.0;
  float scale = 1.0 + 0.2 * sin(angle);
  drawCloudBerry(scale);
  angle += 0.1;
  delay(30);
}

static void connectToWiFi()
{
  if (ssid == "" || password == "") {
    Logger.Error("No stored Wi-Fi credentials.");
    pulseLEDs(ERROR, 3);
    ESP.restart();
  }

  WiFi.mode(WIFI_STA);
  Logger.Info("Connecting to WiFi..");

  delay(100);
  WiFi.begin(ssid.c_str(), password.c_str());

  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 20) {
    delay(200);
    Serial.print(".");
    attempts++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    Logger.Info("Connected!");
  } else {
    Logger.Error("Wi-Fi connection failed — restarting.");
    pulseLEDs(ERROR, 3);
    ESP.restart();
  }
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

void drawCloudBerry(float scale) {
  display.clearDisplay();

  // Center of the screen
  int cx = SCREEN_WIDTH / 2;
  int cy = SCREEN_HEIGHT / 2;

  // Offsets for berry cluster pattern
  int dx[] = { 0,  8, -8,  6, -6,  0};
  int dy[] = {-10, -2, -2,  6,  6, 10};

  int num_blobs = sizeof(dx) / sizeof(dx[0]);

  for (int i = 0; i < num_blobs; i++) {
    int r = scale * 6;
    display.fillCircle(cx + scale * dx[i], cy + scale * dy[i], r, SSD1306_WHITE);
  }

  display.display();
}