// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"

// #include "Encoder.h"

#define MPU6050_DMP_FIFO_RATE_DIVISOR 0x01
#include "MPU6050_6Axis_MotionApps20.h"
//#include "MPU6050.h" // not necessary if using MotionApps include file

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#include "Wire.h"

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu(0x68);
//MPU6050 mpu(0x69); // <-- use for AD0 high

#define PWM_A_FWD 9
#define PWM_A_REV 6

#define PWM_B_FWD 5
#define PWM_B_REV 10

int loop_count;
bool blinkState = false;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t devStatus;		// return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;	// expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;		// count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;		 // [w, x, y, z]         quaternion container
VectorFloat gravity; // [x, y, z]            gravity vector
float ypr[3];		 // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

float time_delta, cur_us, prev_us;
float pitch, pitch_error, last_pitch_error;
float pitch_p_gain, pitch_i_gain, pitch_d_gain;
float pitch_kp = 0, pitch_ki = 0, pitch_kd = 0;
float pitch_setpoint = 0, pitch_output;

float output_A, output_B;
float pwm_A_fwd, pwm_A_rev, pwm_B_fwd, pwm_B_rev;

void setup()
{
	// join I2C bus (I2Cdev library doesn't do this automatically)
	Wire.begin();
	Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties

	// initialize serial communication
	Serial.begin(115200);

	// initialize device
	Serial.println(F("Initializing I2C devices..."));
	mpu.initialize();

	// verify connection
	Serial.println(F("Testing device connections..."));
	Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

	// load and configure the DMP
	Serial.println(F("Initializing DMP..."));
	devStatus = mpu.dmpInitialize();

	// supply your own gyro offsets here, scaled for min sensitivity
	mpu.setXGyroOffset(220);
	mpu.setYGyroOffset(76);
	mpu.setZGyroOffset(-85);
	mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

	// make sure it worked (returns 0 if so)
	if (devStatus == 0)
	{
		// turn on the DMP, now that it's ready
		Serial.println(F("Enabling DMP..."));
		mpu.setDMPEnabled(true);

		// set our DMP Ready flag so the main loop() function knows it's okay to use it
		Serial.println(F("DMP ready! Waiting for first interrupt..."));
		dmpReady = true;

		// get expected DMP packet size for later comparison
		packetSize = mpu.dmpGetFIFOPacketSize();
		Serial.print("Packet Size: ");
		Serial.println(packetSize);
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

	// configure LED for output
	pinMode(LED_BUILTIN, OUTPUT);
}

void loop()
{
	// if programming failed, don't try to do anything
	if (!dmpReady)
		return;

	// wait for MPU interrupt or extra packet(s) available
	while (fifoCount < packetSize)
	{
		if (fifoCount < packetSize)
		{
			// try to get out of the infinite loop
			fifoCount = mpu.getFIFOCount();
		}
		// other program behavior stuff here
		// .
		// .
		// .
		// if you are really paranoid you can frequently test in between other
		// stuff to see if mpuInterrupt is true, and if so, "break;" from the
		// while() loop to immediately process the MPU data
		// .
		// .
		// .
	}

	// get current FIFO count
	fifoCount = mpu.getFIFOCount();

	// check for overflow (this should never happen unless our code is too inefficient)
	if (fifoCount >= 1024)
	{
		// reset so we can continue cleanly
		mpu.resetFIFO();
		fifoCount = mpu.getFIFOCount();
		Serial.println(F("FIFO overflow!"));
	}
	else
	{
		// wait for correct available data length, should be a VERY short wait
		while (fifoCount < packetSize)
			fifoCount = mpu.getFIFOCount();

		// read a packet from FIFO
		mpu.getFIFOBytes(fifoBuffer, packetSize);

		prev_us = cur_us;
		cur_us = micros();
		time_delta = (cur_us - prev_us) / 1000000;

		// track FIFO count here in case there is > 1 packet available
		// (this lets us immediately read more without waiting for an interrupt)
		fifoCount -= packetSize;

		// display Euler angles in degrees
		mpu.dmpGetQuaternion(&q, fifoBuffer);
		mpu.dmpGetGravity(&gravity, &q);
		mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
		pitch = ypr[1];

		mpu.resetFIFO();

		pitch_error = pitch - pitch_setpoint;
		pitch_p_gain = pitch_kp * pitch_error;
		pitch_i_gain += pitch_ki * pitch_error * time_delta;
		pitch_d_gain = pitch_kd * (pitch_error - last_pitch_error) / time_delta;
		pitch_output = pitch_p_gain + pitch_i_gain + pitch_d_gain;

		output_A = pitch_output;
		output_B = pitch_output;

		if (pitch_error > 0.5 || pitch_error < -0.5)
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
			Serial.print("   TIME DELTA: ");
			Serial.println(time_delta, 5);

			last_pitch_error = pitch_error;
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