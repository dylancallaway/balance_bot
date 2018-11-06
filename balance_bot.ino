#include "TimerOne.h"
#include "TimerThree.h"

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
//#include "MPU6050.h" // not necessary if using MotionApps include file

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu(0x68);
//MPU6050 mpu(0x69); // <-- use for AD0 high

/* =========================================================================
   NOTE: In addition to connection 3.3v, GND, SDA, and SCL, this sketch
   depends on the MPU-6050's INT pin being connected to the Arduino's
   external interrupt #0 pin. On the Arduino Uno and Mega 2560, this is
   digital I/O pin 2.
 * ========================================================================= */

#define INTERRUPT_PIN 2 // use pin 2 on Arduino Uno & most boards
#define LED_PIN 13		// (Arduino is 13, Teensy is 11, Teensy++ is 6)
volatile bool blink_state = false;
volatile int loop_count = 0;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;		// return status after each device operation (0 = success, !0 = error)
int packetSize;			// expected DMP packet size (default is 42 bytes)
int fifoCount;			// count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;		 // [w, x, y, z]         quaternion container
VectorFloat gravity; // [x, y, z]            gravity vector
float ypr[3];		 // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// PID setup
// #define LOOP_PERIOD 10
float last_micros = 0, current_micros = 0, LOOP_PERIOD = 0;
#define kp 1
#define ki 0
#define kd 0
float p = 0, i = 0, d = 0, output = 0;
// const float kp = 0, ki = 0, kd = 0;
// const float setpoint = 0; // "Perfect" vertical = 0.0470
#define setpoint 0
float error = 0, last_error = 0;

// Stepper motor setup
#define SLEEP_A 52
#define STEP_A 11
#define DIR_A 50
#define stepperA Timer1

#define SLEEP_B 48
#define STEP_B 5
#define DIR_B 46
#define stepperB Timer3

// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false; // indicates whether MPU interrupt pin has gone high
void dmpDataReady()
{
	mpuInterrupt = true;
}

// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup()
{
// join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
	Wire.begin();
	Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
	Fastwire::setup(400, true);
#endif

	// initialize serial communication
	// (115200 chosen because it is required for Teapot Demo output, but it's
	// really up to you depending on your project)
	Serial.begin(115200);

	// NOTE: 8MHz or slower host processors, like the Teensy @ 3.3V or Arduino
	// Pro Mini running at 3.3V, cannot handle this baud rate reliably due to
	// the baud timing being too misaligned with processor ticks. You must use
	// 38400 or slower in these cases, or use some kind of external separate
	// crystal solution for the UART timer.

	// initialize device
	Serial.println(F("Initializing I2C devices..."));
	mpu.initialize();
	pinMode(INTERRUPT_PIN, INPUT);

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

		// enable Arduino interrupt detection
		Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
		Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
		Serial.println(F(")..."));
		attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
		mpuIntStatus = mpu.getIntStatus();

		// set our DMP Ready flag so the main loop() function knows it's okay to use it
		Serial.println(F("DMP ready! Waiting for first interrupt..."));
		dmpReady = true;

		// get expected DMP packet size for later comparison
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

	// configure LED for output
	pinMode(LED_PIN, OUTPUT);
	digitalWrite(LED_PIN, blink_state);

	// Configure stepper motors
	pinMode(SLEEP_A, OUTPUT);
	digitalWrite(SLEEP_A, HIGH);

	pinMode(SLEEP_B, OUTPUT);
	digitalWrite(SLEEP_B, HIGH);

	pinMode(DIR_A, OUTPUT);
	pinMode(DIR_B, OUTPUT);

	stepperA.initialize(0);
	stepperA.pwm(STEP_A, 512);

	stepperB.initialize(0);
	stepperB.pwm(STEP_B, 512);
}

// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

// TODO Figure out why the stepper motors are not moving...
void loop()
{
	if (mpuInterrupt)
	{
		mpu.getFIFOBytes(fifoBuffer, packetSize);
		mpu.dmpGetQuaternion(&q, fifoBuffer);
		mpu.dmpGetGravity(&gravity, &q);
		mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
		error = ypr[1] - setpoint;
		p = kp * error;
		i += ki * LOOP_PERIOD * error;
		d = kd * (error - last_error) / LOOP_PERIOD; // Loop period in us (microseconds)
		last_error = error;
		output = p + i + d;

		if (error > 0.6 || error < -0.6)
		{
			stepperA.setPeriod(0);
			stepperB.setPeriod(0);
		}
		else
		{
			if (output >= 0)
			{
				digitalWrite(DIR_A, HIGH);
				digitalWrite(DIR_B, HIGH);

				// stepperA.setPeriod(1000 / output);
				// stepperB.setPeriod(1000 / output);

				stepperA.setPeriod(1400);
			}
			else if (output < 0)
			{
				digitalWrite(DIR_A, LOW);
				digitalWrite(DIR_B, LOW);

				stepperA.setPeriod(1000 / abs(output));
				stepperB.setPeriod(1000 / abs(output));
			}
		}

		// Check for full FIFO
		if (mpu.getFIFOCount() >= 1024)
		{
			mpu.resetFIFO();
			Serial.println('FIFO overflow!');
		}

		// Blink LED to indicate activity
		if (loop_count >= 50)
		{
			blink_state = !blink_state;
			digitalWrite(LED_PIN, blink_state);
			loop_count = 0;
		}
		loop_count = loop_count + 1;

		// Reset interrupt flag to false
		mpuInterrupt = false;

		//Print statements for debugging
		// Serial.print("PITCH: ");
		// Serial.print(ypr[1], 4);

		Serial.print("\tERROR: ");
		Serial.print(error, 4);

		Serial.print("\tOUTPUT: ");
		Serial.print(output, 4);
	}

	last_micros = current_micros;
	current_micros = micros();
	LOOP_PERIOD = (current_micros - last_micros);

	Serial.print("\tLOOP_PERIOD: ");
	Serial.println(LOOP_PERIOD, 4);
}
