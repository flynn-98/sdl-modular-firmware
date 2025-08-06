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
const float ML_REV = 0.1; //ml/rev

const float MAX_ACCEL = 300.0 * MICROSTEPS; //microsteps/s2
const float MAX_SPEED = 1000.0 * MICROSTEPS; //microsteps/s2

// put function declarations here:
void pwmDrivingSignal(int motor, int power);
long volToSteps(float vol);
void stepperDrivingSignal(int motor, float vol, float max_speed);
void populateOLED();
void pulseLEDs(LedColor color, int pulses = 1, int stepDelay = 5);

float measured_temp;
float measured_hum;

float speed;
float vol;
float pwm;
float seconds;

int motor;
String action = "No command";

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

    steppers[i]->setAcceleration(MAX_ACCEL);
    steppers[i]->setMaxSpeed(MAX_SPEED);
  }

  Wire.begin(SDA, SCL, 400000);
  pixels.begin();

  sht4.begin(&Wire);
  sht4.setPrecision(SHT4X_HIGH_PRECISION);
  sht4.getEvent(&hum, &temp);
  delay(100);
  measured_temp = temp.temperature;
  measured_hum = hum.relative_humidity;

  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Logger.Error(F("No OLED found!"));
  }

  populateOLED();

  Logger.Info("Available functions:");
  Logger.Info("stepperMotor(int motor, float volume [ml], float speed [ml/s])");
  Logger.Info("dcMotor(int motor, float power [+-%], float time [s])");

  pulseLEDs(HAPPY, 1, 20);
}

void loop() {
  // Main code here, to run repeatedly on a loop 
  delay(1000);

  action = "No command";
  
  sht4.getEvent(&hum, &temp);
  delay(100);
  measured_temp = temp.temperature;
  measured_hum = hum.relative_humidity;

  populateOLED();

  // Wait until data received from PC, via Serial (USB)
  if (Serial.available() > 0) {
      // data structure to receive = action(var1, var2..)
      // Read until open bracket to extract action, continue based on which action was requested
      action = Serial.readStringUntil('(');

      pulseLEDs(TRANSMIT);

      if (action == "stepperMotor") {
        motor = Serial.readStringUntil(',').toInt();
        vol = Serial.readStringUntil(',').toFloat();
        speed = Serial.readStringUntil(')').toFloat();

        // Call action using received variables
        stepperDrivingSignal(motor, vol, speed);

        Logger.Info("Stepper Action Complete");
      }
      else if (action == "dcMotor") {
        motor = Serial.readStringUntil(',').toInt();
        pwm = Serial.readStringUntil(',').toFloat();
        seconds = Serial.readStringUntil(')').toFloat() * 1000;

        // Call action using received variables
        pwmDrivingSignal(motor, pwm);
        delay(floor(seconds));
        pwmDrivingSignal(motor, 0);

        Logger.Info("DC Action Complete");
      }
      else {
          // Report back to PC if confused
          Logger.Error("Unknown command: " + action);
      }

  }

}

long volToSteps(float vol) {
    return floor(MICROSTEPS * STEPS_REV * vol / ML_REV);
};

void stepperDrivingSignal(int motor, float vol, float max_speed) {
  // Volume in ml and speed in ml/s

  // Validate motor index
  if (motor < 1 || motor > 4) {
    Serial.println("Error: Invalid stepper index (must be 1-4)");
    return;
  }

  // Configure stepper
  steppers[motor - 1]->setMaxSpeed(volToSteps(max_speed));
  steppers[motor - 1]->move(volToSteps(vol));

  // Enable driver
  digitalWrite(en_pins[motor - 1], LOW);

  // Run to target position (blocking)
  steppers[motor - 1]->runToPosition();

  // Disable driver to save power/heat
  digitalWrite(en_pins[motor - 1], HIGH);
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

void populateOLED() {
  display.clearDisplay();

  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  display.setCursor(0, 0);
  display.print(action);

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
