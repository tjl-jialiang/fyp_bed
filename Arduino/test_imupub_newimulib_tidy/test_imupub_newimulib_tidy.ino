//=======ROS=========
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32.h>
//==================
  
//===============================IMU==================================================
/*
I2Cdev device library code is placed under the MIT license
Copyright (c) 2012 Jeff Rowberg

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
*/


#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps20.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

MPU6050 mpu;

#define OUTPUT_READABLE_YAWPITCHROLL

#define INTERRUPT_PIN 19  //MPU6050 INT pin

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

//Interruprt routine of MPU6050
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}
//===============================IMU==================================================

#include <math.h>
#include <BasicLinearAlgebra.h>
using namespace BLA;

//MDD10A pins
//mdd_wheel1_right[2] = {pwm1, dir1} or {pwm2, dir2}
int mdd_wheel1_right[2] = {5,24};
int mdd_wheel1_left[2] = {6,25};
int mdd_wheel2_right[2] = {2,26};
int mdd_wheel2_left[2] = {3,27};

//Absolute Encoder Pins
//Pins label: http://www.farnell.com/datasheets/1884437.pdf
//absenc_wheel1[3] = {Csn,CLK,DO}
int absenc_wheel1[3] = {4,7,8};
int absenc_wheel2[3] = {48,49,50};
const float enc_res = 2*PI/1024; // Resolution of encoder (10bits - 0 to 1024)

//Angle from absolute encoder
float alpha1 = 0;
float alpha2 = 0; 

//Velocity data from bed_cmd_vel
float xvel = 0; 
float yvel = 0; 
float rotvel = 0;

//after inverse kinematics
BLA::Matrix<2,1> wheel1_vel; 
BLA::Matrix<2,1> wheel2_vel; 

//Absolute wheel velocities
float wheel1_abs[2]= {0,0};
float wheel2_abs[2]= {0,0};

float max_norm = 0;

//pwm values (intermediate)
float wheel1_pwm[2] = {0,0};
float wheel2_pwm[2] = {0,0};

//To prevent abrupt changes
float wheel1_pwm_signed[2] = {0,0};
float wheel2_pwm_signed[2] = {0,0}; 
float prev_wheel1_pwm_signed[2] = {0,0};
float prev_wheel2_pwm_signed[2] = {0,0};
float out_wheel1_pwm_signed[2] = {0,0};
float out_wheel2_pwm_signed[2] = {0,0};
float Kp= 0.3;

float Lratio = 2.204724409; //L_offset = 3.175cm, L_split = 7cm
float gamma_1 = 1.956271282; //(PI/2) + 0.38547
float gamma_2 = -1.185321371; //(-PI/2) + 0.38547
float D = 0.3550538692; //in metres
float psi = 0; //IMU yaw angle for inverse kin calc
std_msgs::Float32 yaw_angle; //IMU yaw angle for ROS

//declare functions
BLA::Matrix<2,1> invkin(float xvel,float yvel,float rotvel, float alpha, float gamma);
void matrixAbsolute(BLA::Matrix<2,1> wheelx_vel, float wheelx_abs[]);
void signedpwm(float wheelx_pwm[],float wheelx_pwm_signed[], BLA::Matrix<2,1> wheelx_vel);

//======ROS===============
ros::NodeHandle nh;

void velCb(const geometry_msgs::Twist& velmsg){
    xvel = velmsg.linear.x;
    yvel = velmsg.linear.y;
    rotvel = velmsg.angular.z;
}

ros::Subscriber<geometry_msgs::Twist> sub("bed_cmd_vel", &velCb);
ros::Publisher imuyaw("imuyaw", &yaw_angle);
//==========================

void setup() {
    //MDD10A Pins
    pinMode(mdd_wheel1_right[0], OUTPUT);
    pinMode(mdd_wheel1_right[1], OUTPUT);
    
    pinMode(mdd_wheel1_left[0], OUTPUT);
    pinMode(mdd_wheel1_left[1], OUTPUT);

    pinMode(mdd_wheel2_right[0], OUTPUT);
    pinMode(mdd_wheel2_right[1], OUTPUT);
    
    pinMode(mdd_wheel2_left[0], OUTPUT);
    pinMode(mdd_wheel2_left[1], OUTPUT);

    //Abs Encoder
    pinMode(absenc_wheel1[0], OUTPUT);
    pinMode(absenc_wheel1[1], OUTPUT);
    pinMode(absenc_wheel1[2], INPUT);
    digitalWrite(absenc_wheel1[0], HIGH);
    digitalWrite(absenc_wheel1[1], HIGH);

    pinMode(absenc_wheel2[0], OUTPUT);
    pinMode(absenc_wheel2[1], OUTPUT);
    pinMode(absenc_wheel2[2], INPUT);
    digitalWrite(absenc_wheel2[0], HIGH);
    digitalWrite(absenc_wheel2[1], HIGH);

    //Default baud rate of rosserial is 57600
    Serial.begin(57600);

    //========================IMU============================
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);
    devStatus = mpu.dmpInitialize();
    //Values obtained via zero-ing code in library
    mpu.setXGyroOffset(80);
    mpu.setYGyroOffset(30);
    mpu.setZGyroOffset(39);
    mpu.setXAccelOffset(-472); 
    mpu.setYAccelOffset(112); 
    mpu.setZAccelOffset(1018);
    if (devStatus == 0) {
        // Calibration Time: generate offsets and calibrate our MPU6050
        mpu.CalibrateAccel(6);
        mpu.CalibrateGyro(6);
        // turn on the DMP, now that it's ready
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }
    //========================IMU============================

    //==============ROS===============
    nh.initNode();
    nh.subscribe(sub);
    nh.advertise(imuyaw);
}

void loop() {
    //Obtain IMU reading
    getpsi();
    //Publish IMU reading on ROS topic
    imuyaw.publish(&yaw_angle);
    
    //Get absolute encoder readings
    alpha1 = readAbsEncoder(absenc_wheel1) ;
    alpha2 = readAbsEncoder(absenc_wheel2) ;
      
    nh.spinOnce(); //handles ROS communication callback 
    
    //Calculate inverse kinematics to determine wheel speed required
    wheel1_vel = invkin(xvel,yvel,rotvel,alpha1,gamma_1);
    wheel2_vel = invkin(xvel,yvel,rotvel,alpha2,gamma_2);

    //Make positive
    matrixAbsolute(wheel1_vel,wheel1_abs);
    matrixAbsolute(wheel2_vel,wheel2_abs);
    
    //Find maximum velocity to normalise to
    max_norm = maximum(wheel1_abs,wheel2_abs);
    
    //Find PWM values
    mappwm(wheel1_abs,wheel1_pwm);
    mappwm(wheel2_abs,wheel2_pwm);

    //Make PWM signed
    signedpwm(wheel1_pwm,wheel1_pwm_signed,wheel1_vel);
    signedpwm(wheel2_pwm,wheel2_pwm_signed,wheel2_vel);
 
    //proportional control
    prop_pwm(wheel1_pwm_signed,prev_wheel1_pwm_signed,out_wheel1_pwm_signed);
    prop_pwm(wheel2_pwm_signed,prev_wheel2_pwm_signed,out_wheel2_pwm_signed);

    //Drive wheel module using signed PWM
    drivemotor_dirchange(out_wheel1_pwm_signed, mdd_wheel1_right, mdd_wheel1_left);
    drivemotor_dirchange(out_wheel2_pwm_signed, mdd_wheel2_right, mdd_wheel2_left);
    
    delay(50);
}   
