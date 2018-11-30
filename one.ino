#include "MPU9250.h"

#define ENCODER_OPTIMIZE_INTERRUPTS
#include "Encoder.h"

#define PWM_A_FWD 9
#define PWM_A_REV 6

#define PWM_B_FWD 5
#define PWM_B_REV 10

#define ENC_A_PIN_A 2 // Interrupt pin
#define ENC_A_PIN_B 7

#define ENC_B_PIN_A 3 // Interrupt pin
#define ENC_B_PIN_B 8

#define ENC_STEPS_REV 900

MPU9250 mpu;
Encoder encoder_A(ENC_A_PIN_A, ENC_A_PIN_B);
Encoder encoder_B(ENC_B_PIN_A, ENC_B_PIN_B);

void setup()
{
	Serial.begin(115200);
	Serial.println("***   Power on...running program.   ***");
	delay(500);

	pinMode(LED_BUILTIN, OUTPUT);

	pinMode(PWM_A_FWD, OUTPUT);
	pinMode(PWM_B_FWD, OUTPUT);
	pinMode(PWM_A_REV, OUTPUT);
	pinMode(PWM_B_REV, OUTPUT);

	Wire.begin();
	delay(500);
	mpu.setup();
	delay(2000);
}

bool blinkstate;
int loop_count;
float theta, error, last_error, output;
float cur_us, prev_us, time_delta;
float p_gain, i_gain, d_gain;
float kp = 1200, ki = 0, kd = 0;
float setpoint = 0;

long pos_A, pos_B;

void loop()
{
	cur_us = micros();
	time_delta = (cur_us - prev_us) / 1000000; // s

	if (time_delta >= 0.01)
	{
		mpu.update();
		theta = PI * mpu.getPitch() / 180;
		error = theta - setpoint;

		pos_A = encoder_A.read();
		pos_B = -encoder_B.read();

		if (theta > 0.5 || theta < -0.5)
		{
			analogWrite(PWM_A_FWD, 0);
			analogWrite(PWM_B_FWD, 0);
			analogWrite(PWM_A_REV, 0);
			analogWrite(PWM_B_REV, 0);
			loop_count += 5;
		}
		else
		{
			p_gain = kp * error;
			i_gain += ki * error * time_delta;
			d_gain = kd * (error - last_error) / time_delta;
			output = p_gain + i_gain + d_gain;

			if (output > 0)
			{
				if (output >= 255)
				{
					output = 255;
				}
				analogWrite(PWM_A_FWD, output);
				analogWrite(PWM_B_FWD, output);
				analogWrite(PWM_A_REV, 0);
				analogWrite(PWM_B_REV, 0);
			}
			else
			{
				output = abs(output);
				if (output >= 255)
				{
					output = 255;
				}
				analogWrite(PWM_A_FWD, 0);
				analogWrite(PWM_B_FWD, 0);
				analogWrite(PWM_A_REV, output);
				analogWrite(PWM_B_REV, output);
			}

			loop_count += 1;
		}

		if (loop_count >= 50)
		{
			digitalWrite(LED_BUILTIN, blinkstate);
			blinkstate = !blinkstate;
			loop_count = 0;
		}

		Serial.print("Error: ");
		Serial.print(error, 5);
		Serial.print("    Output: ");
		Serial.print(output, 5);
		Serial.print("    Position A: ");
		Serial.print(pos_A);
		Serial.print("    Position B: ");
		Serial.print(pos_B);
		Serial.print("    Time Delta: ");
		Serial.println(time_delta, 5);

		last_error = error;
		prev_us = cur_us;
	}
}