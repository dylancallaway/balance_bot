#include "MPU9250.h"

#define PWM_A_FWD 9
#define PWM_A_REV 6
#define PWM_B_FWD 5
#define PWM_B_REV 3

#define ENC_A_PIN_A 8
#define ENC_A_PIN_B 7


MPU9250 mpu;

void setup()
{
	Serial.begin(115200);
	Serial.println("***   Power on successful...running program.   ***");

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

int pos_A;

bool blinkstate;
int loop_count;
float theta, error, last_error, output;
float cur_us, prev_us, time_delta;
float p_gain, i_gain, d_gain;
float kp = 1200, ki = 0, kd = 0;
float setpoint = 0;

void loop()
{
	cur_us = micros();
	time_delta = (cur_us - prev_us) / 1000000; // s

	if (time_delta >= 0.01)
	{
		mpu.update();
		theta = PI * mpu.getPitch() / 180;
		error = theta - setpoint;

		if (theta > 0.5 || theta < -0.5)
		{
			analogWrite(PWM_A_FWD, 0);
			analogWrite(PWM_B_FWD, 0);
			analogWrite(PWM_A_REV, 0);
			analogWrite(PWM_B_REV, 0);
			blinkstate = 0;
			digitalWrite(LED_BUILTIN, blinkstate);
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
		Serial.print("    Position A: ");
		Serial.print(pos_A);
		Serial.print("    Time Delta: ");
		Serial.println(time_delta, 5);

		last_error = error;
		prev_us = cur_us;
	}
}