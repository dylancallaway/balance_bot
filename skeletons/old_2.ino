#include "I2Cdev.h"

#define MPU6050_DMP_FIFO_RATE_DIVISOR 0x01
#include "MPU6050_6Axis_MotionApps20.h"
//#include "MPU6050.h" // not necessary if using MotionApps include file

#include "Wire.h"

#include "Encoder.h"

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu(0x68);

bool blinkState = false;

// MPU control/status vars
bool dmpReady = false; // set true if DMP init was successful
bool mpu_status = false;
uint8_t devStatus;		// return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;	// expected DMP packet size (default is 42 bytes)
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;		 // [w, x, y, z]         quaternion container
VectorFloat gravity; // [x, y, z]            gravity vector
float ypr[3];		 // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
#define MPU_CHECK_LEN 200
bool mpu_check[MPU_CHECK_LEN];
int sum;

#define PWM_A_FWD 9
#define PWM_A_REV 6

#define PWM_B_FWD 5
#define PWM_B_REV 10

#define ENC_A_PIN_A 2 // Interrupt pin
#define ENC_A_PIN_B 7

#define ENC_B_PIN_A 3 // Interrupt pin
#define ENC_B_PIN_B 8

#define ENC_STEPS_REV 900

Encoder encoder_A(ENC_A_PIN_A, ENC_A_PIN_B);
Encoder encoder_B(ENC_B_PIN_A, ENC_B_PIN_B);

float time_delta, cur_us, prev_us;
int loop_count;

float pitch, pitch_error, last_pitch_error;

void setup()
{
	Wire.begin();
	Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties

	Serial.begin(115200);

	// initialize device
	Serial.println(F("Initializing I2C devices..."));
	mpu.initialize();

	// verify connection
	Serial.println(F("Testing device connections..."));
	Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

	// wait for ready
	Serial.println(F("\nBegin DMP programming and demo: "));

	// load and configure the DMP
	Serial.println(F("Initializing DMP..."));
	devStatus = mpu.dmpInitialize();

	mpu.setXGyroOffset(220);
	mpu.setYGyroOffset(76);
	mpu.setZGyroOffset(-85);
	mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

	// make sure it worked (returns 0 if so)
	if (devStatus == 0)
	{
		Serial.println(F("Enabling DMP..."));
		mpu.setDMPEnabled(true);

		packetSize = mpu.dmpGetFIFOPacketSize();
	}
	else
	{
		// ERROR!
		// 1 = initial memory load failed
		// 2 = DMP configuration updates failed
		// (if it's going to break, usually the code will be 1)
		Serial.print(F("DMP Initialization failed (code "));
		Serial.print(devStatus);
		Serial.println(F(")"));
	}

	pinMode(PWM_A_FWD, OUTPUT);
	pinMode(PWM_B_FWD, OUTPUT);
	pinMode(PWM_A_REV, OUTPUT);
	pinMode(PWM_B_REV, OUTPUT);

	pinMode(LED_BUILTIN, OUTPUT);

	while (mpu_status == false)
	{
		if (mpu.getFIFOCount() >= packetSize)
		{
			prev_us = cur_us;
			cur_us = micros();
			time_delta = cur_us - prev_us;

			mpu.getFIFOBytes(fifoBuffer, packetSize);
			mpu.dmpGetQuaternion(&q, fifoBuffer);
			mpu.dmpGetGravity(&gravity, &q);
			mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
			pitch = ypr[1];
			// Serial.print(" PITCH: ");
			// Serial.println(pitch);

			if (pitch >= -0.04 && pitch <= 0.04)
			{
				mpu_check[loop_count] = 1;
			}
			else
			{
				mpu_check[loop_count] = 0;
			}

			if (loop_count >= MPU_CHECK_LEN)
			{
				for (int i = 0; i < MPU_CHECK_LEN; i++)
				{
					sum += mpu_check[i];
				}

				Serial.println(sum);

				if (sum >= MPU_CHECK_LEN)
				{
					mpu_status = true;
				}

				sum = 0;
				loop_count = 0;
			}

			loop_count += 1;
		}
	}
	loop_count = 0;
}

float pitch_p_gain, pitch_i_gain, pitch_d_gain;
float pitch_kp = 0, pitch_ki = 0, pitch_kd = 0;
float pitch_setpoint = 0;
float pitch_output;

long pos_A, pos_B, pos_error, last_pos_error;
float pos_p_gain, pos_i_gain, pos_d_gain;
float pos_kp = 0, pos_ki = 0, pos_kd = 0;
float pos_setpoint = 0;
float pos_output, pos_output_A, pos_output_B;

float tot_pos_avg, tot_pos_error, last_tot_pos_error;
float tot_pos_kp = 0, tot_pos_ki = 0, tot_pos_kd = 0;
float tot_pos_p_gain, tot_pos_i_gain, tot_pos_d_gain;
float tot_pos_setpoint = 0;
float tot_pos_output = 0;

float output_A, output_B;
float pwm_A_fwd, pwm_A_rev, pwm_B_fwd, pwm_B_rev;

// THIS THING IS LIKE FREAKING OUT RIGHT NOW...NEED TO DEBUG.

void loop()
{
	if (mpu.getFIFOCount() >= packetSize)
	{
		prev_us = cur_us;
		cur_us = micros();
		time_delta = (cur_us - prev_us) / 1000000;

		mpu.getFIFOBytes(fifoBuffer, packetSize);
		mpu.dmpGetQuaternion(&q, fifoBuffer);
		mpu.dmpGetGravity(&gravity, &q);
		mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

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

		pitch = ypr[1];
		mpu.resetFIFO();

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

			Serial.print("   PITCH ERROR: ");
			Serial.print(pitch_error, 5);
			Serial.print("   FIFOCOUNT: ");
			Serial.print(mpu.getFIFOCount());
			// Serial.print("   PITCH OUTPUT: ");
			// Serial.print(pitch_output);
			Serial.print("   ELAPSED TIME: ");
			Serial.println(time_delta, 5);

			last_pitch_error = pitch_error;
			last_pos_error = pos_error;
			last_tot_pos_error = tot_pos_error;
			prev_us = cur_us;

			loop_count += 1;
		}

		if (loop_count >= 50)
		{
			blinkState = !blinkState;
			digitalWrite(LED_BUILTIN, blinkState);
			loop_count = 0;
		}
	}
}