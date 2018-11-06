#include "AccelStepper.h"
#include "TimerOne.h"

// Stepper motor setup
#define MOTOR_STEPS_REV 200
#define STEPPER_MAX_SPEED 800

#define SLEEP_A 52
#define STEP_A 11
#define DIR_A 50

AccelStepper stepperA(AccelStepper::DRIVER, STEP_A, DIR_A);

void cb(void)
{
    stepperA.runSpeed();
}

void setup()
{
    Timer1.initialize(1500);
    Timer1.attachInterrupt(cb);

    // Configure stepper motors
    pinMode(SLEEP_A, OUTPUT);
    digitalWrite(SLEEP_A, HIGH);

    stepperA.setMaxSpeed(STEPPER_MAX_SPEED);

    stepperA.setSpeed(1000);
}

void loop()
{
}