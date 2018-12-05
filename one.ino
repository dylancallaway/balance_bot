#include <PinChangeInterrupt.h>
#include <PinChangeInterruptBoards.h>
#include <PinChangeInterruptPins.h>
#include <PinChangeInterruptSettings.h>

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"

#define MPU6050_DMP_FIFO_RATE_DIVISOR 0x01
#include "MPU6050_6Axis_MotionApps20.h"
//#include "MPU6050.h" // not necessary if using MotionApps include file

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#include "Wire.h"

#include "Encoder.h"

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

#define INTERRUPT_PIN 4 // use pin 4 on Arduino Pro Mini

// Motor output PWM definitions
#define PWM_A_FWD 9
#define PWM_A_REV 6

#define PWM_B_FWD 5
#define PWM_B_REV 10

// Motor encoder definitions
#define ENC_A_PIN_A 2 // Interrupt pin
#define ENC_A_PIN_B 7

#define ENC_B_PIN_A 3 // Interrupt pin
#define ENC_B_PIN_B 8

#define ENC_STEPS_REV 900

// Encoder encoder_A(ENC_A_PIN_A, ENC_A_PIN_B);
// Encoder encoder_B(ENC_B_PIN_A, ENC_B_PIN_B);

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;        // [w, x, y, z]         quaternion container
VectorFloat gravity; // [x, y, z]            gravity vector
float ypr[3];        // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// Blinky blinky vars
bool blinkstate = false;
int loop_count;

// Control loop vars
float cur_us, prev_us, time_delta;
float output_A, output_B;
float pwm_A_fwd, pwm_A_rev, pwm_B_fwd, pwm_B_rev;

// Pitch control loop vars
float pitch, pitch_error, last_pitch_error;
float pitch_p_gain, pitch_i_gain, pitch_d_gain;
float pitch_kp = 0, pitch_ki = 0, pitch_kd = 0;
float pitch_setpoint = 0;
float pitch_output;

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
    Wire.begin();
    Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties

    // initialize serial communication
    // (115200 chosen because it is required for Teapot Demo output, but it's
    // really up to you depending on your project)
    Serial.begin(115200);
    while (!Serial)
    {
        ; // wait for Leonardo enumeration, others continue immediately
    }

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
        Serial.print(F("Enabling interrupt detection (Arduino pin change interrupt "));
        Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
        Serial.println(F(")..."));
        attachPCINT(digitalPinToPCINT(INTERRUPT_PIN), dmpDataReady, RISING);
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
    pinMode(LED_BUILTIN, OUTPUT);
}

// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop()
{
    // if programming failed, don't try to do anything
    if (!dmpReady)
        return;

    // wait for MPU interrupt or extra packet(s) available
    while (!mpuInterrupt && fifoCount < packetSize)
    {
        if (mpuInterrupt && fifoCount < packetSize)
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

    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & _BV(MPU6050_INTERRUPT_FIFO_OFLOW_BIT)) || fifoCount >= 1024)
    {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        fifoCount = mpu.getFIFOCount();
        Serial.println(F("FIFO overflow!"));

        // otherwise, check for DMP data ready interrupt (this should happen frequently)
    }
    else if (mpuIntStatus & _BV(MPU6050_INTERRUPT_DMP_INT_BIT))
    {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize)
            fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);

        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;

        // Calculate quaternions and yaw, pitch, roll
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

        // ================================================================
        // ===                    CONTROL CALCULATION                   ===
        // ================================================================

        // Get pitch
        pitch = ypr[1];

        // Calculate time delta since last control calc
        prev_us = cur_us;
        cur_us = micros();
        time_delta = (cur_us - prev_us) / 1000000;

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
            loop_count += 1;
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

            loop_count += 2;
        }

        // Blink LED to indicate activity
        if (loop_count >= 100)
        {
            digitalWrite(LED_BUILTIN, blinkstate);
            blinkstate = !blinkstate;
            loop_count = 0;
        }

        Serial.print(" Pitch: ");
        Serial.print(pitch, 5);
        Serial.print("    FIFOCOUNT: ");
        Serial.print(fifoCount);
        Serial.print("    Time Delta: ");
        Serial.println(time_delta, 5);

        last_pitch_error = pitch_error;
        prev_us = cur_us;
    }
}