#include "TimerOne.h"

// Stepper motor setup
#define MOTOR_STEPS_REV 200
#define STEPPER_MAX_SPEED 1500

#define SLEEP_A 52
#define STEP_A 11
#define DIR_A 50

#define SLEEP_B 48
#define STEP_B 12
#define DIR_B 46

float last_micros = 0, current_micros = 0, LOOP_PERIOD = 0;

void setup()
{
	// Serial.begin(115200);
	// Configure stepper motors
	pinMode(SLEEP_A, OUTPUT);
	digitalWrite(SLEEP_A, HIGH);

	pinMode(DIR_A, OUTPUT);
	digitalWrite(DIR_A, HIGH);

	pinMode(13, OUTPUT);

	Timer1.initialize(800);
	Timer1.pwm(STEP_A, 64);
	Timer1.attachInterrupt(callback); // attaches callback() as a timer overflow interrupt
}

void callback()
{
	digitalWrite(13, digitalRead(13) ^ 1);
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