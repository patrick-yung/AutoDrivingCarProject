#include <TimerOne.h>

#include "LMotorController.h"
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
 #include "Wire.h"
#endif

#define MIN_ABS_SPEED 60#define NEWLINE Serial.write(0x0a); Serial.write(0x0d)

MPU6050 mpu;

// MPU control/status vars
bool dmpReady = false; // set true if DMP init was successful
uint8_t mpuIntStatus; // holds actual interrupt status byte from MPU
uint8_t devStatus; // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize; // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount; // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q; // [w, x, y, z] quaternion container
VectorFloat gravity; // [x, y, z] gravity vector
float ypr[3]; // [yaw, pitch, roll] yaw/pitch/roll container and gravity vector

double ang,lang,delta_ang,int_ang,fang;
double error,last_error,delta_error,integrate_error;

#define LA 3
#define RA 4
#define PWML 5
#define PWMR 6
#define LM1 7
#define LM2 8
#define RM1 9
#define RM2 10
#define LED 12
// CHANGE HERE: use your PID params here
#define P 10
#define I 10
#define D 5
// CHANGE END
#define CM 2050
#define Rmotor_offset 0 
#define Lmotor_offset 0
#define ROTATE_SPEED 200

int lcur,lold,rcur,rold;
volatile int lcount,rcount;
volatile long cnt,ms;
volatile int lcountCopy,rcountCopy;
long wheel_cnt;

volatile bool mpuInterrupt = false; // indicates whether MPU interrupt pin has gone high

void dmpDataReady(void);
void wait_imu(void);
void read_angle(void);
void IntHandler(void);
void cal_angle(void);
char atob(unsigned char d);
void stop_motors(void);
void forward(void);
void backward(void);
void rotate_left(void);
void rotate_right(void);
void show_ang(void);
void car_forward(double steps);
void car_rotate_left(double dir_ang);
void car_rotate_right(double dir_ang);

void setup(void) {
  pinMode(LA, INPUT);
  pinMode(RA, INPUT);
  pinMode(PWML, OUTPUT);
  pinMode(PWMR, OUTPUT);
  pinMode(LM1, OUTPUT);
  pinMode(LM2, OUTPUT);
  pinMode(RM1, OUTPUT);
  pinMode(RM2, OUTPUT);
  pinMode(LED, OUTPUT);
  Timer1.initialize(100); //timer period set to 100 microseconds
  Timer1.attachInterrupt(IntHandler);
  Serial.begin(115200);

   // join I2C bus (I2Cdev library doesn't do this automatically)
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
  #endif
  
  mpu.initialize();

  devStatus = mpu.dmpInitialize();

  // supply your own gyro offsets here, scaled for min sensitivity
mpu.setXAccelOffset(-2843);
mpu.setYAccelOffset(-524);
mpu.setZAccelOffset(1473);
mpu.setXGyroOffset(-76);
  mpu.setYGyroOffset(-4);
  mpu.setZGyroOffset(-171);
// 1688 factory default for my test chip

  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // turn on the DMP, now that it's ready
    mpu.setDMPEnabled(true);

    // enable Arduino interrupt detection
    attachInterrupt(0, dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  }
  else  {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }
  delta_ang=2;
  wait_imu();
  int_ang=0;
  lang=ang;
}

// The main program will print the blink count
// to the Arduino Serial Monitor
void loop(void) {
  char value,s0,s1,s2;
  int sp;
  
  Serial.println("SI");

 while(1) {
    while((value!='f')&&(value!='r')&&(value!='l')) value=Serial.read();
    if(value=='r'){
      while((s1<'0')||(s1>'9')) s1 = Serial.read();
      while((s0<'0')||(s0>'9')) s0 = Serial.read();
      sp=atob(s1)*10 + atob(s0);
      car_rotate_right(sp);
    } else if(value=='l'){
      while((s1<'0')||(s1>'9')) s1 = Serial.read();
      while((s0<'0')||(s0>'9')) s0 = Serial.read();
      sp=(atob(s1)*10 + atob(s0))*(-1);
      car_rotate_left(sp);
    } else if(value=='f'){
      while((s2<'0')||(s2>'9')) s2 = Serial.read();
      while((s1<'0')||(s1>'9')) s1 = Serial.read();
      while((s0<'0')||(s0>'9')) s0 = Serial.read();
      sp=atob(s2)*100 + atob(s1)*10 + atob(s0);
      car_forward(sp);
    }
    value=0;   
    s0=0;
    s1=0;
    s2=0;
    Serial.println("SI");
  }
}

void dmpDataReady(void) {
  mpuInterrupt = true;
}

void wait_imu(void) {        
  while(delta_ang>1) {    //wait until the difference between current angle value 
                          //current angle value and last second value is less than 1
    read_angle();
    if(ms>10000) {      //wait for 1 second (ms=0.1 milisecond)
      ms=0;
      digitalWrite(LED,HIGH);
      Serial.print("Please wait. Not ready! ；");
      Serial.print(ang);
      Serial.print(" ; ");
      Serial.print(lang);
      Serial.print(" ; ");
      delta_ang=abs(ang-lang);
      lang=ang;
      Serial.println(delta_ang);
      digitalWrite(LED,LOW);
    }
  }
  Serial.println("Ready!");    
}

void read_angle(void) {
  // if programming failed, don't try to do anything
  if (!dmpReady) return;

  // reset interrupt flag and get INT_STATUS byte
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();

  // get current FIFO count
  fifoCount = mpu.getFIFOCount();

  // check for overflow (this should never happen unless our code is too inefficient)
  if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
    // reset so we can continue cleanly
    mpu.resetFIFO();
    Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
   }
   else if (mpuIntStatus & 0x02) {
    // wait for correct available data length, should be a VERY short wait
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

    //read a packet from FIFO
    mpu.getFIFOBytes(fifoBuffer, packetSize);
 
    // track FIFO count here in case there is > 1 packet available
    // (this lets us immediately read more without waiting for an interrupt)
    fifoCount -= packetSize;

    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    //input = ypr[1] * 180/M_PI + 180;
    ang = ypr[0] * 180/M_PI + 180;   
  }
}

//int ledState = LOW;
//volatile unsigned long blinkCount = 0; // use volatile for shared variables

void IntHandler(void) {
  cnt++;
  ms++;       // 0.1 ms counter

  //get the current wheel sensor values
  lcur=digitalRead(LA);
  rcur=digitalRead(RA);

  //count the number of switching            
  if(lcur!=lold) {
    lcount++;
    lold=lcur;
  }
  if(rcur!=rold) {
    rcount++;
    rold=rcur;
  }

  if(cnt>2000){   // every 200 miliseconds update the 
                  // lcountCopy and rcountCopy for displaying the speeds
    cnt=0;
    lcountCopy = lcount;
    lcount = 0;
    rcountCopy = rcount;
    rcount = 0;
  }
}

void cal_angle(void) {
    if(abs(ang-lang)>180) {
        if(lang>ang) fang=ang+360-lang;
        else if(ang>lang) fang=ang-360-lang;
    }
    if(fang!=0) int_ang+=fang;
    else int_ang+=(ang-lang);

    fang=0;
    lang=ang;
}

char atob(unsigned char d) {
  int a;

  switch(d) {

    case '0':
    a=0;
    break;
    case '1':
    a=1;
    break;
    case '2':
    a=2;
    break;
    case '3':
    a=3;
    break;
    case '4':
    a=4;
    break;
    case '5':
    a=5;
    break;
    case '6':
    a=6;
    break;
    case '7':
    a=7;
    break;
    case '8':
    a=8;
    break;
    case '9':
    a=9;
    }

  return a;
}

void stop_motors(void) {
  digitalWrite(LM1,LOW);
  digitalWrite(LM2,LOW);
  digitalWrite(RM1,LOW);
  digitalWrite(RM2,LOW);
}

void forward(void) {
  digitalWrite(LM1,HIGH);
  digitalWrite(LM2,LOW);
  digitalWrite(RM1,LOW);
  digitalWrite(RM2,HIGH);
}

void backward(void) {
  // CHANGE HERE, drive the motor to go backward
    digitalWrite(LM1,LOW);
  digitalWrite(LM2,HIGH);
  digitalWrite(RM1,HIGH);
  digitalWrite(RM2,LOW);
}

void rotate_left(void) {
  analogWrite(PWML,ROTATE_SPEED);
  analogWrite(PWMR,ROTATE_SPEED);
  //CHANGE HERE, drive the motor to turn left
   digitalWrite(LM1,LOW);
  digitalWrite(LM2,HIGH);
  digitalWrite(RM1,LOW);
  digitalWrite(RM2,HIGH);
}

void rotate_right(void) {
  //CHANGE HERE, drive the motor to turn right
    analogWrite(PWML,ROTATE_SPEED);
  analogWrite(PWMR,ROTATE_SPEED);
  //CHANGE HERE, drive the motor to turn left
    digitalWrite(LM1,HIGH);
  digitalWrite(LM2,LOW);
  digitalWrite(RM1,HIGH);
  digitalWrite(RM2,LOW);
}

void show_ang(void) {
  Serial.print(ang);
  Serial.print("；");
  Serial.print(int_ang);
  Serial.print("；");
  Serial.print("L Count = ");
  Serial.print(lcountCopy);
  Serial.print("; R Count = ");
  Serial.println(rcountCopy);
}

void car_forward(double steps) {  //steps = distance in cm
  double tmp_pwm;
  
  wheel_cnt=0;
  read_angle();
  cal_angle();
  int_ang=0;
  lang=ang;

  while(wheel_cnt<(steps*CM)) {   //convert cm to encoder steps
    read_angle();     // update the current angle value
    cal_angle();      // convert the angle in reference to robot orientation
    show_ang();
  
    wheel_cnt+=(lcountCopy+rcountCopy)/2; // average the left and right wheel count
    error = int_ang;
    tmp_pwm=255-abs(error*P + integrate_error*I + delta_error*D);           // PID control of the wheels
    delta_error = error - last_error;
    last_error = error;
    integrate_error += error;
    
    if(int_ang>0) {
      analogWrite(PWML,tmp_pwm-Lmotor_offset);
      analogWrite(PWMR,255-Rmotor_offset);
    }
    else if(int_ang<0){
      analogWrite(PWML,255-Lmotor_offset);
      analogWrite(PWMR,tmp_pwm-Rmotor_offset);
    }
    else {
      analogWrite(PWML,255-Lmotor_offset); 
      analogWrite(PWMR,255-Rmotor_offset);
    }
    forward();
  }
  stop_motors();
}

void car_rotate_left(double dir_ang) {
 int_ang=0;
 while(int_ang>(dir_ang)){
    // CHANGE HERE, drive the car to turn left.
    // we turn a small angle once until reach the target.
    read_angle();     // update the current angle value
    cal_angle();      // convert the angle in reference to robot orientation
    rotate_left();
  }
  stop_motors();
  int_ang=0;
}

void car_rotate_right(double dir_ang) {
  // CHANGE HERE, drive the car to turn right.
  // similar to car_rotate_left.

   int_ang=0;
 while(int_ang<(dir_ang)){
    // CHANGE HERE, drive the car to turn right.
    // we turn a small angle once until reach the target.
    read_angle();     // update the current angle value
    cal_angle();      // convert the angle in reference to robot orientation
    rotate_right();
  }
  stop_motors();
  int_ang=0;
}
