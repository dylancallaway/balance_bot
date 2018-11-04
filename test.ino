#include "BasicStepperDriver.h"

#define SLEEP 52
#define STEP 3
#define DIR 50

#define MOTOR_STEPS_REV 200
// Looks like maximum RPM is about 250.
#define RPM 250

BasicStepperDriver stepper(MOTOR_STEPS_REV, DIR, STEP);

void setup()
{
    pinMode(SLEEP, OUTPUT);
    digitalWrite(SLEEP, HIGH);

    stepper.begin(RPM, 1);
}

void loop()
{
    stepper.rotate(360);
    delay(1000);
    stepper.move(-MOTOR_STEPS_REV);
    delay(1000);
}
