#include "TimerOne.h"
#include "TimerThree.h"

// Stepper motor setup
#define SLEEP_A 52
#define STEP_A 11
#define DIR_A 50
#define stepperA Timer1

#define SLEEP_B 48
#define STEP_B 5
#define DIR_B 46
#define stepperB Timer3

float last_micros = 0, current_micros = 0, LOOP_PERIOD = 0;

void setup()
{
	// Serial.begin(115200);
	// Configure stepper motors
	pinMode(SLEEP_A, OUTPUT);
	digitalWrite(SLEEP_A, HIGH);

	pinMode(DIR_A, OUTPUT);
	digitalWrite(DIR_A, HIGH);

	pinMode(SLEEP_B, OUTPUT);
	digitalWrite(SLEEP_B, HIGH);

	pinMode(DIR_B, OUTPUT);
	digitalWrite(DIR_B, HIGH);

	stepperA.initialize();
	stepperA.pwm(STEP_A, 256);

	// stepperB.initialize();
	// stepperB.pwm(STEP_B, 256, 3000);

	stepperA.stop();
	stepperB.stop();
}

void loop()
{
	// MOTOR TESTING

	// last_micros = current_micros;
	// current_micros = micros();
	// LOOP_PERIOD = (current_micros - last_micros) / 1000;
	// Serial.print("\tLOOP_PERIOD: ");
	// Serial.println(LOOP_PERIOD, 4);

	//MOTOR TESTING
}