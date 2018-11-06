#include "AccelStepper.h"
#include "MultiStepper.h"

// Stepper motor setup
#define MOTOR_STEPS_REV 200
#define STEPPER_MAX_SPEED 1500

#define SLEEP_A 52
#define STEP_A 3
#define DIR_A 50

#define SLEEP_B 48
#define STEP_B 4
#define DIR_B 46

AccelStepper stepperA(AccelStepper::DRIVER, STEP_A, DIR_A);
AccelStepper stepperB(AccelStepper::DRIVER, STEP_B, DIR_B);

MultiStepper steppers;

void setup()
{
    // Configure stepper motors
    pinMode(SLEEP_A, OUTPUT);
    digitalWrite(SLEEP_A, HIGH);

    pinMode(SLEEP_B, OUTPUT);
    digitalWrite(SLEEP_B, HIGH);

    stepperA.setMaxSpeed(STEPPER_MAX_SPEED);
    stepperB.setMaxSpeed(STEPPER_MAX_SPEED);

    // Give them to MultiStepper to manage
    steppers.addStepper(stepperA);
    steppers.addStepper(stepperB);

    stepperA.setSpeed(1000);
    stepperB.setSpeed(1000);
}

void loop()
{
    // MOTOR TESTING

    stepperA.runSpeed();
    stepperB.runSpeed();
    // long positions[2]; // Array of desired stepper positions

    // positions[0] = 200;
    // positions[1] = 200;
    // stepperA.setSpeed(200);
    // stepperB.setSpeed(200);
    // steppers.moveTo(positions);
    // steppers.runSpeedToPosition(); // Blocks until all are in position

    //MOTOR TESTING
}