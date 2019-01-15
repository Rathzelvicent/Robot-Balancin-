#include <PID_v1.h>
#include <LMotorController.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
 #include "Wire.h"
#endif

#define MIN_ABS_SPEED 30

MPU6050 mpu;

// MPU control/status vars
bool dmpReady = false; 
uint8_t mpuIntStatus;
uint8_t devStatus; 
uint16_t packetSize; 
uint16_t fifoCount; 
uint8_t fifoBuffer[64]; 

// orientation/motion vars
Quaternion q;
VectorFloat gravity;
float ypr[3]; 

//PID
double originalSetpoint = 177.7;
double setpoint = originalSetpoint;
double movingAngleOffset = 0.1;
double input, output;

//adjust these values to fit your own design
double Kp = 65;  //60                    //60
double Kd = 2;//2//no subir mas de 10   //2.5
double Ki = 600;  //280                   //425
PID pid(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

double motorSpeedFactorLeft = 0.4;
double motorSpeedFactorRight = 0.4;

//MOTOR CONTROLLER
#define STYBY  8
int ENA = 4;
int IN1 = 7;
int IN2 = 6;
int IN3 = 13;
int IN4 = 12;
int ENB = 5;
LMotorController motorController(ENA, IN1, IN2, ENB, IN3, IN4, motorSpeedFactorLeft, motorSpeedFactorRight);

volatile bool mpuInterrupt = false; 
void dmpDataReady()
{
 mpuInterrupt = true;
}


void setup()
{
  pinMode(STYBY  ,OUTPUT);
  pinMode(IN4  ,OUTPUT);
  pinMode(IN3  ,OUTPUT);
  pinMode(ENB  ,OUTPUT);
  pinMode(IN1  ,OUTPUT);
  pinMode(IN2  ,OUTPUT);
  pinMode(ENA  ,OUTPUT);
  digitalWrite(STYBY, HIGH);

 #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
 Wire.begin();
 TWBR = 24; 
 #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
 Fastwire::setup(400, true);
 #endif

 mpu.initialize();

 devStatus = mpu.dmpInitialize();


 mpu.setXGyroOffset(220);
 mpu.setYGyroOffset(76);
 mpu.setZGyroOffset(-85);
 mpu.setZAccelOffset(1788); 

 // make sure it worked (returns 0 if so)
 if (devStatus == 0)
 {
 // turn on the DMP, now that it's ready
 mpu.setDMPEnabled(true);

 // enable Arduino interrupt detection
 attachInterrupt(0, dmpDataReady, RISING);
 mpuIntStatus = mpu.getIntStatus();

 dmpReady = true;

 packetSize = mpu.dmpGetFIFOPacketSize();
 
 //setup PID
 pid.SetMode(AUTOMATIC);
 pid.SetSampleTime(10);
 pid.SetOutputLimits(-255, 255); 
 }
 else
 {
 Serial.print(F("DMP Initialization failed (code "));
 Serial.print(devStatus);
 Serial.println(F(")"));
 }
}


void loop()
{

 if (!dmpReady) return;

 while (!mpuInterrupt && fifoCount < packetSize)
 {
 pid.Compute();
 motorController.move(output, MIN_ABS_SPEED);
 
 }

 mpuInterrupt = false;
 mpuIntStatus = mpu.getIntStatus();

 fifoCount = mpu.getFIFOCount();


 if ((mpuIntStatus & 0x10) || fifoCount == 1024)
 {
 // reset so we can continue cleanly
 mpu.resetFIFO();
 Serial.println(F("FIFO overflow!"));

 }
 else if (mpuIntStatus & 0x02)
 {

 while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

 mpu.getFIFOBytes(fifoBuffer, packetSize);
 

 fifoCount -= packetSize;

 mpu.dmpGetQuaternion(&q, fifoBuffer);
 mpu.dmpGetGravity(&gravity, &q);
 mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
 input = ypr[1] * 180/M_PI + 180;
 }
}
