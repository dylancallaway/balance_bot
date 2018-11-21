#include "MPU9250.h"

#define PWM_A_FWD 12
#define PWM_A_REV 11

#define PWM_B_FWD 10
#define PWM_B_REV 9

MPU9250 mpu;

void setup()
{
	Serial.begin(115200);

	pinMode(LED_BUILTIN, OUTPUT);

	pinMode(PWM_A_FWD, OUTPUT);
	pinMode(PWM_B_FWD, OUTPUT);
	pinMode(PWM_A_REV, OUTPUT);
	pinMode(PWM_B_REV, OUTPUT);

	Wire.begin();
	delay(1000);
	mpu.setup();
	delay(1000);
}

bool blinkstate;
int loop_count;
float theta, error, last_error, output;
float cur_us, prev_us, time_delta;
float p_gain, i_gain, d_gain;
float kp = 1000, ki = 0, kd = 0;
float setpoint = 0;

void loop()
{
	cur_us = micros();
	time_delta = (cur_us - prev_us) / 1000000; // s

	if (time_delta >= 0.01)
	{
		mpu.update();
		theta = mpu.getPitch();
		error = theta - setpoint;

		if (theta > 0.5 || theta < -0.5)
		{
			analogWrite(PWM_A_FWD, 0);
			analogWrite(PWM_B_FWD, 0);
			analogWrite(PWM_A_REV, 0);
			analogWrite(PWM_B_REV, 0);
			loop_count = 0;
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

			if (loop_count >= 50)
			{
				digitalWrite(LED_BUILTIN, blinkstate);
				blinkstate = !blinkstate;
				loop_count = 0;
			}

			loop_count += 1;
		}

		Serial.print("Error: ");
		Serial.print(error, 5);
		Serial.print("    Output: ");
		Serial.print(output, 5);
		Serial.print("    Time Delta: ");
		Serial.println(time_delta, 5);

		last_error = error;
		prev_us = cur_us;
	}
}