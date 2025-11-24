#include <Servo.h>
#include <AccelStepper.h>

const int TX = 0;
const int RX = 1;

// Pins for pump stepper motors, see https://learn.sparkfun.com/tutorials/big-easy-driver-hookup-guide/all
const int PUMP_1_STEP = 2;
const int PUMP_1_DIR = 3;

const int PUMP_2_STEP = 4;
const int PUMP_2_DIR = 5;

const int PUMP_3_STEP = 6;
const int PUMP_3_DIR = 7;

// Pump 1 = Electrolyte into Cell, Pump 2 = Electrolyte out of Cell, Pump 3 = Cleaning Solution into Cell

// Pins for 4th stepper motor
const int PUMP_4_STEP = 8;
const int PUMP_4_DIR = 9;

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
const byte ANALOG_1 = A1;
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

const float MOTOR_SPEED = 2000.0 * MICROSTEPS; //microsteps/s
const float MAX_ACCEL = 1000.0 * MICROSTEPS; //microsteps/s2

// Define steppers with pins (STEP, DIR)
AccelStepper PUMP_1(AccelStepper::DRIVER, PUMP_1_STEP, PUMP_1_DIR); 
AccelStepper PUMP_2(AccelStepper::DRIVER, PUMP_2_STEP, PUMP_2_DIR);
AccelStepper PUMP_3(AccelStepper::DRIVER, PUMP_3_STEP, PUMP_3_DIR); 
AccelStepper PUMP_4(AccelStepper::DRIVER, PUMP_4_STEP, PUMP_4_DIR);

// Ensure motor direction matches desired pump direction
const float motorDir = 1;

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
    return floor(motorDir * MICROSTEPS * STEPS_REV * degs * GEAR_RATIO / 360);
};

long heightToSteps(float milli) {
    return floor(motorDir * MICROSTEPS * STEPS_REV * milli / ROD_PITCH);
};

void moveDegrees(float angle) {
    relayOn();

    // No limits for Pump
    PUMP_2.move(degreesToSteps(angle));

    // Run until complete
    PUMP_2.runToPosition();

    // Report back to PC
    Serial.println("# Move complete");
    relayOff();
};

void moveHeight(float height) {
    relayOn();

    // No limits for Pump
    PUMP_1.move(heightToSteps(height));

    // Run until complete
    PUMP_1.runToPosition();

    // Report back to PC
    Serial.println("# Move complete");
    relayOff();
};

void setup() {
  // Setup code here, will run just once on start-up

  // Set pins to be used
  pinMode(PUMP_1_STEP, OUTPUT);
  pinMode(PUMP_1_DIR, OUTPUT);

  pinMode(PUMP_2_STEP, OUTPUT);
  pinMode(PUMP_2_DIR, OUTPUT);

  pinMode(PUMP_3_STEP, OUTPUT);
  pinMode(PUMP_3_DIR, OUTPUT);

  pinMode(PUMP_4_STEP, OUTPUT);
  pinMode(PUMP_4_DIR, OUTPUT);

  pinMode(SERVO_PIN, OUTPUT);
  
  pinMode(RELAY_PIN, OUTPUT);

  // Set motor speeds / acceleration
  PUMP_1.setAcceleration(MAX_ACCEL);
  PUMP_1.setMaxSpeed(MOTOR_SPEED);

  PUMP_2.setAcceleration(MAX_ACCEL);
  PUMP_2.setMaxSpeed(MOTOR_SPEED);

  PUMP_3.setAcceleration(MAX_ACCEL);
  PUMP_3.setMaxSpeed(MOTOR_SPEED);

  PUMP_4.setAcceleration(MAX_ACCEL);
  PUMP_4.setMaxSpeed(MOTOR_SPEED);

  Serial.begin(115200);
  
  relayOff();
  Serial.println("Fluid Handling Kit Ready");
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
        }
        else if (action == "moveHeight") {
            target = Serial.readStringUntil(')').toFloat();
            
            // Call action using received variables
            moveHeight(target);
        }
        else if (action == "getStatus") {
            (void)Serial.readStringUntil(')').toFloat();

            Serial.println("Autosampler Kit Ready");
        }
        else {
            // Report back to PC if confused
            Serial.println("Unknown command");
        }
    }
};