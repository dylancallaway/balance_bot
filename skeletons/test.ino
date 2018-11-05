#include "BasicStepperDriver.h"
#include "SyncDriver.h"

#define MOTOR_STEPS_REV 200

#define SLEEP_A 52
#define STEP_A 3
#define DIR_A 50
BasicStepperDriver stepperA(MOTOR_STEPS_REV, DIR_A, STEP_A);

#define SLEEP_B 48
#define STEP_B 4
#define DIR_B 46
BasicStepperDriver stepperB(MOTOR_STEPS_REV, DIR_B, STEP_B);

SyncDriver controller(stepperA, stepperB);

// Looks like maximum RPM is about 250.
#define RPM 250

void setup()
{
    pinMode(SLEEP_A, OUTPUT);
    digitalWrite(SLEEP_A, HIGH);
    stepperA.begin(RPM, 1);

    pinMode(SLEEP_B, OUTPUT);
    digitalWrite(SLEEP_B, HIGH);
    stepperB.begin(RPM, 1);
}

void loop()
{
    controller.rotate(360, 360);
    delay(1000);
    controller.rotate(-360, -360);
    delay(1000);
}
