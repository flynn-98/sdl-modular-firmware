#include <Servo.h>
#include <AccelStepper.h>

const int TX = 0;
const int RX = 1;

// Pins for pump stepper motors, see https://learn.sparkfun.com/tutorials/big-easy-driver-hookup-guide/all
const int AXIS_1_STEP = 2;
const int AXIS_1_DIR = 3;

const int AXIS_2_STEP = 4;
const int AXIS_2_DIR = 5;

const int AXIS_3_STEP = 6;
const int AXIS_3_DIR = 7;

// Pump 1 = Electrolyte into Cell, Pump 2 = Electrolyte out of Cell, Pump 3 = Cleaning Solution into Cell

// Pins for 4th stepper motor
const int AXIS_4_STEP = 8;
const int AXIS_4_DIR = 9;

// Define Arduino pins for each function
const int SERVO_PIN = 10;

// Define relay pin
const byte RELAY_PIN = A0;

// Define comms pins
const int MOSI_PIN = 11;
const int MISO_PIN = 12;
const int SCK_PIN = 13;
const int SDA_PIN = 18;
const int SCL_PIN = 19;

// Define remaining pins (A6 & A7 are analog only)
const byte LIMIT = A1;
const byte ANALOG_2 = A2;
const byte ANALOG_3 = A3;
const byte ANALOG_6 = A6;
const byte ANALOG_7 = A7;

// Motor speed and acceleration parameters, stepper motors have 200 steps / revolution.
// Microsteps (per step) used for increased positional accuracy and smoother stepping
const float STEPS_REV = 200.0;
const float MICROSTEPS = 4.0;

const float GEAR_RATIO = 14.0;
const float ROD_PITCH = 2.0; //mm

const float MOTOR_SPEED = 1000.0 * MICROSTEPS; //microsteps/s
const float HOME_SPEED = 200.0 * MICROSTEPS; //microsteps/s
const float MAX_ACCEL = 500.0 * MICROSTEPS; //microsteps/s2

// Define steppers with pins (STEP, DIR)
AccelStepper AXIS_1(AccelStepper::DRIVER, AXIS_1_STEP, AXIS_1_DIR); 
AccelStepper AXIS_2(AccelStepper::DRIVER, AXIS_2_STEP, AXIS_2_DIR);
AccelStepper AXIS_3(AccelStepper::DRIVER, AXIS_3_STEP, AXIS_3_DIR); 
AccelStepper AXIS_4(AccelStepper::DRIVER, AXIS_4_STEP, AXIS_4_DIR);

// Ensure motor direction matches desired pump direction
const float heightDir = 1;
const float spinDir = -1;
const float home_degs = 10;

float target = 0;
String action;

void relayOn() {
    digitalWrite(RELAY_PIN, HIGH);
    delay(500);
};

void relayOff() {
    digitalWrite(RELAY_PIN, LOW);
};

long degreesToSteps(float degs) {
    return floor(spinDir * MICROSTEPS * STEPS_REV * degs * GEAR_RATIO / 360);
};

long heightToSteps(float milli) {
    return floor(heightDir * MICROSTEPS * STEPS_REV * milli / ROD_PITCH);
};

void moveDegrees(float angle) {
    relayOn();

    AXIS_2.moveTo(degreesToSteps(angle));

    // Run until complete
    AXIS_2.runToPosition();

    relayOff();
};

void moveHeight(float height) {
    relayOn();

    AXIS_1.moveTo(heightToSteps(height));

    // Run until complete
    AXIS_1.runToPosition();

    relayOff();
};

void setup() {
  // Setup code here, will run just once on start-up

  // Set pins to be used
  pinMode(AXIS_1_STEP, OUTPUT);
  pinMode(AXIS_1_DIR, OUTPUT);

  pinMode(AXIS_2_STEP, OUTPUT);
  pinMode(AXIS_2_DIR, OUTPUT);

  pinMode(AXIS_3_STEP, OUTPUT);
  pinMode(AXIS_3_DIR, OUTPUT);

  pinMode(AXIS_4_STEP, OUTPUT);
  pinMode(AXIS_4_DIR, OUTPUT);

  pinMode(SERVO_PIN, OUTPUT);
  
  pinMode(RELAY_PIN, OUTPUT);
  pinMode(LIMIT, INPUT_PULLUP);

  // Set motor speeds / acceleration
  AXIS_1.setAcceleration(MAX_ACCEL);
  AXIS_1.setMaxSpeed(MOTOR_SPEED);

  AXIS_2.setAcceleration(MAX_ACCEL);
  AXIS_2.setMaxSpeed(MOTOR_SPEED);

  AXIS_3.setAcceleration(MAX_ACCEL);
  AXIS_3.setMaxSpeed(MOTOR_SPEED);

  AXIS_4.setAcceleration(MAX_ACCEL);
  AXIS_4.setMaxSpeed(MOTOR_SPEED);

  Serial.begin(9600);
  
  relayOff();
  Serial.println("# Sampler Kit Ready");
};

void loop() {
    // Main code here, to run repeatedly on a loop 
    delay(500);

    // Wait until data received from PC, via Serial (USB)
    if (Serial.available() > 0) {
        // data structure to receive = action(var1, var2..)

        // Read until open bracket to extract action, continue based on which action was requested
        action = Serial.readStringUntil('(');

        if (action == "moveDegrees") {
            // Extract variables spaced by commas, then last variable up to closed bracket
            target = Serial.readStringUntil(')').toFloat();
            
            // Call action using received variables
            moveDegrees(target);

            Serial.println("# Move complete");
        }
        else if (action == "moveHeight") {
            target = Serial.readStringUntil(')').toFloat();
            
            // Call action using received variables
            moveHeight(target);

            Serial.println("# Move complete");
        }
        else if (action == "home") {
            (void)Serial.readStringUntil(')').toFloat();

            relayOn();
            // Z axis
            AXIS_1.move(heightToSteps(-50));
            // Run until complete
            AXIS_1.runToPosition();
            AXIS_1.setCurrentPosition(0);
            AXIS_2.setMaxSpeed(HOME_SPEED);

            AXIS_2.move(degreesToSteps(-360));
            // Run until complete
            while (digitalRead(LIMIT) == HIGH && AXIS_2.distanceToGo() != 0) {
                AXIS_2.run();
            }

            AXIS_2.setCurrentPosition(0);
            AXIS_2.moveTo(degreesToSteps(home_degs));
            AXIS_2.runToPosition();

            AXIS_2.setCurrentPosition(0);

            relayOff();

            AXIS_2.setMaxSpeed(MOTOR_SPEED);
            Serial.println("# Homing complete");
        }
        else if (action == "getStatus") {
            (void)Serial.readStringUntil(')').toFloat();

            Serial.println("# Sampler Kit Ready");
        }
        else {
            // Report back to PC if confused
            Serial.println("Unknown command");
        }
    }
};