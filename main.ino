//================//
//main.ino
//Diederik van Linden
//0970665
//TI2C
//================//


#include <cantino.h>

#include "I2Cdev.h"

#include "pid.h"
#include "joystick.h"
#include "wheel.h"


#include "MPU6050_6Axis_MotionApps20.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

int Jsx=0;
int Jsy=0;

float Yaw=0;

float Pwm = 0;

volatile bool mpuInterrupt;     // indicates whether MPU interrupt pin has gone high

#define OUTPUT_READABLE_YAWPITCHROLL

using namespace cantino;

class Gyro {
  public:
    Gyro();
    void connectmpu();
    static void dmpDataReady();
    float getYaw();
    
    bool dmpReady = false;  // set true if DMP init was successful
    uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
    uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
    uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
    uint16_t fifoCount;     // count of all bytes currently in FIFO
    uint8_t fifoBuffer[64]; // FIFO storage buffer


    MPU6050 mpu1;

    // orientation/motion vars
    Quaternion q;           // [w, x, y, z]         quaternion container
    VectorInt16 aa;         // [x, y, z]            accel sensor measurements
    VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
    VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
    VectorFloat gravity;    // [x, y, z]            gravity vector
    float euler[3];         // [psi, theta, phi]    Euler angle container
    float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

    long int LastTime = 0 ;

    // packet structure for InvenSense teapot demo
    uint8_t teapotPacket[14] = { '$', 0x02, 0, 0, 0, 0, 0, 0, 0, 0, 0x00, 0x00, '\r', '\n' };
};

Gyro::Gyro() {
  mpuInterrupt = false;
}

float Gyro::getYaw(){
  return ypr[1];   
}

void Gyro::connectmpu() {

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif

  // initialize serial communication
  Serial.begin(57600);
  while (!Serial); //

  // initialize device
  Serial.println(F("Initializing I2C devices..."));
  mpu1.initialize();

  // verify connection
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu1.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));


  // load and configure the DMP
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu1.dmpInitialize();

  mpu1.setXGyroOffset(44);//(220);
  mpu1.setYGyroOffset(-21);//(76);
  mpu1.setZGyroOffset(-30);//(-85);
  mpu1.setXAccelOffset(-1875);//(1788); // 1688 factory default for my test chip
  mpu1.setYAccelOffset(-1426);
  mpu1.setZAccelOffset(2215);


  if (devStatus == 0) {
    // turn on the DMP, now that it's ready
    Serial.println(F("Enabling DMP..."));
    mpu1.setDMPEnabled(true);

    // enable Arduino interrupt detection
    Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
    attachInterrupt(0, dmpDataReady, RISING);
    mpuIntStatus = mpu1.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu1.dmpGetFIFOPacketSize();
  } 
  else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }
}

 static void Gyro::dmpDataReady() {
  mpuInterrupt = true;
}

int main() {

  Wheel leftWheel(7, 8, 10); // Pins on Arduino
  Wheel rightWheel(5, 6, 9);
  Joystick joystick(A0, A1); 
  Gyro gyro;
  gyro.connectmpu();  
  Pid pid;

  while (true) {

    if (!gyro.dmpReady) return;

    // wait for MPU interrupt or extra packet(s) available
    while (!mpuInterrupt && gyro.fifoCount < gyro.packetSize) {
      // other program behavior stuff here
    }

    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    gyro.mpuIntStatus = gyro.mpu1.getIntStatus();

    // get current FIFO count
    gyro.fifoCount = gyro.mpu1.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((gyro.mpuIntStatus & 0x10) || gyro.fifoCount == 1024) {
      // reset so we can continue cleanly
      gyro.mpu1.resetFIFO();
      Serial.println(F("FIFO overflow!"));

      // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (gyro.mpuIntStatus & 0x02) {
      // wait for correct available data length, should be a VERY short wait
      while (gyro.fifoCount < gyro. packetSize) gyro.fifoCount = gyro.mpu1.getFIFOCount();

      // read a packet from FIFO
      gyro.mpu1.getFIFOBytes(gyro.fifoBuffer, gyro.packetSize);

      // track FIFO count here in case there is > 1 packet available
      // (this lets us immediately read more without waiting for an interrupt)
      gyro.fifoCount -= gyro.packetSize;

#ifdef OUTPUT_READABLE_YAWPITCHROLL
      // display Euler angles in degrees
      gyro.mpu1.dmpGetQuaternion(&gyro.q, gyro.fifoBuffer);
      gyro.mpu1.dmpGetGravity(&gyro.gravity, &gyro.q);
      gyro.mpu1.dmpGetYawPitchRoll(gyro.ypr, &gyro.q, &gyro.gravity);

      Jsx= joystick.readX();
      Jsy= joystick.readY();

Yaw = gyro.getYaw()*1000;

Serial.println(Yaw);

Pwm = pid.calculate(Yaw, Jsx);

// If pwm less then 0 vehicle moves backwards
if (Pwm < 0) {         
  leftWheel.setBackwards();
  rightWheel.setBackwards();
}                       
        
// if pwm is higher then 0 vehicle moves forward
if (Pwm > 0) {
  leftWheel.setForward();
  rightWheel.setForward();
}

// makes the integer absolut
Pwm  = abs(Pwm);   

// gives the pwm a maximum
if ( Pwm > 190){   
  Pwm = 190;
}

//updates the wheels
if (true){
  rightWheel.writePWM(Pwm);
  leftWheel.writePWM(Pwm);
} 

//Stops both wheels
else {
  leftWheel.stopWheel();
  rightWheel.stopWheel();
  pid.I = 0;
}

//When Joystick is shifted left.
while (Jsy <= -5 ){
  Serial.println("Left");
  rightWheel.writePWM(60);
  rightWheel.setForward();
  
  leftWheel.writePWM(60);
  leftWheel.setBackwards();
  Jsy = 0;
}

//When Joystick is shifted right
while (Jsy >= 5) {
  Serial.println("Right");
  leftWheel.writePWM(60);
  leftWheel.setForward();

  rightWheel.writePWM(60);
  rightWheel.setBackwards();
  
  Jsy =0;

}

#endif
    }
  } 
  return 0;
}
