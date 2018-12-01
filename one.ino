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
	delay(1000);
}

bool blinkstate;
float cur_us, prev_us, time_delta;
int loop_count;

float pitch, pitch_error, last_pitch_error;
float pitch_p_gain, pitch_i_gain, pitch_d_gain;
float pitch_kp = 1250, pitch_ki = 0, pitch_kd = 15;
float pitch_setpoint = 0;
float pitch_output;

long pos_A, pos_B, pos_error, last_pos_error;
float pos_p_gain, pos_i_gain, pos_d_gain;
float pos_kp = 2, pos_ki = 0, pos_kd = 0.1;
float pos_setpoint = 0;
float pos_output, pos_output_A, pos_output_B;

float tot_pos_avg, tot_pos_error, last_tot_pos_error;
float tot_pos_kp = -0, tot_pos_ki = 0, tot_pos_kd = 0;
float tot_pos_p_gain, tot_pos_i_gain, tot_pos_d_gain;
float tot_pos_setpoint = 0;
float tot_pos_output = 0;

float output_A, output_B;
float pwm_A_fwd, pwm_A_rev, pwm_B_fwd, pwm_B_rev;

void loop()
{
	cur_us = micros();
	time_delta = (cur_us - prev_us) / 1000000; // s

	if (time_delta >= 0.01)
	{
		pos_A = encoder_A.read();
		pos_B = -encoder_B.read();

		pos_error = (pos_A - pos_B) - pos_setpoint;
		pos_p_gain = pos_kp * pos_error;
		pos_i_gain += pos_ki * pos_error * time_delta;
		pos_d_gain = pos_kd * (pos_error - last_pos_error) / time_delta;
		pos_output = pos_p_gain + pos_i_gain + pos_d_gain;
		pos_output_A = pos_output / 2;
		pos_output_B = -pos_output / 2;

		tot_pos_avg = pos_A;
		tot_pos_error = tot_pos_avg - tot_pos_setpoint;
		tot_pos_p_gain = tot_pos_kp * tot_pos_error;
		tot_pos_i_gain += tot_pos_ki * tot_pos_error * time_delta;
		tot_pos_d_gain = tot_pos_kd * (tot_pos_error - last_tot_pos_error) / time_delta;
		tot_pos_output = tot_pos_p_gain + tot_pos_i_gain + tot_pos_d_gain;

		mpu.update();
		pitch = PI * mpu.getPitch() / 180;
		pitch_error = pitch - pitch_setpoint;

		pitch_p_gain = pitch_kp * pitch_error;
		pitch_i_gain += pitch_ki * pitch_error * time_delta;
		pitch_d_gain = pitch_kd * (pitch_error - last_pitch_error) / time_delta;
		pitch_output = pitch_p_gain + pitch_i_gain + pitch_d_gain;

		output_A = pitch_output + pos_output_A + tot_pos_output;
		output_B = pitch_output + pos_output_B + tot_pos_output;

		if (pitch > 0.5 || pitch < -0.5)
		{
			analogWrite(PWM_A_FWD, 0);
			analogWrite(PWM_B_FWD, 0);
			analogWrite(PWM_A_REV, 0);
			analogWrite(PWM_B_REV, 0);
			loop_count += 5;
		}
		else
		{
			if (output_A > 0)
			{
				if (output_A >= 255)
				{
					output_A = 255;
				}
				pwm_A_fwd = output_A;
				pwm_A_rev = 0;
			}
			else if (output_A < 0)
			{
				if (output_A <= -255)
				{
					output_A = -255;
				}
				pwm_A_fwd = 0;
				pwm_A_rev = abs(output_A);
			}

			if (output_B > 0)
			{
				if (output_B >= 255)
				{
					output_B = 255;
				}
				pwm_B_fwd = output_B;
				pwm_B_rev = 0;
			}
			else if (output_B < 0)
			{
				if (output_B <= -255)
				{
					output_B = -255;
				}
				pwm_B_fwd = 0;
				pwm_B_rev = abs(output_B);
			}

			analogWrite(PWM_A_FWD, pwm_A_fwd);
			analogWrite(PWM_A_REV, pwm_A_rev);
			analogWrite(PWM_B_FWD, pwm_B_fwd);
			analogWrite(PWM_B_REV, pwm_B_rev);

			loop_count += 1;
		}

		if (loop_count >= 50)
		{
			digitalWrite(LED_BUILTIN, blinkstate);
			blinkstate = !blinkstate;
			loop_count = 0;
		}

		Serial.print(pitch);
		Serial.print("    Time Delta: ");
		Serial.println(time_delta, 5);

		last_pitch_error = pitch_error;
		last_pos_error = pos_error;
		last_tot_pos_error = tot_pos_error;
		prev_us = cur_us;
	}
}