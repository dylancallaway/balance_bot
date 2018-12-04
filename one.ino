#include "I2Cdev.h"

#define MPU6050_DMP_FIFO_RATE_DIVISOR 0x01
#include "MPU6050_6Axis_MotionApps20.h"
//#include "MPU6050.h" // not necessary if using MotionApps include file

#include "Wire.h"

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu(0x68);

#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;        // [w, x, y, z]         quaternion container
VectorFloat gravity; // [x, y, z]            gravity vector
float ypr[3];        // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

int delta_time, cur_micros, last_micros;
int loop_count;

// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

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
    Serial.println(mpu.getDeviceID());
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

    pinMode(LED_PIN, OUTPUT);
}

void loop()
{
    if (mpu.getFIFOCount() >= packetSize)
    {
        last_micros = cur_micros;
        cur_micros = micros();
        delta_time = cur_micros - last_micros;

        mpu.getFIFOBytes(fifoBuffer, packetSize);
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);


        


        Serial.print("   PITCH: ");
        Serial.print(ypr[1], 4);
        Serial.print("   ELAPSED TIME: ");
        Serial.println(delta_time);

        loop_count += 1;
    }

    if (loop_count >= 50)
    {
        blinkState = !blinkState;
        digitalWrite(LED_PIN, blinkState);
        loop_count = 0;
    }
}