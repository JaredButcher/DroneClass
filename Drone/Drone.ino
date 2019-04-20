#include"Drone.h"

unsigned long imuClock = 0;
unsigned long motorClock = 0;
unsigned long currentMillis = 0;
unsigned long previousMillis = 0;

int joyThrottle =0 ; // 
int joyYaw  =0 ; // 
int joyRoll  =0 ; // 
int joyPitch  =0 ; // 

Vector3 rawAccel;
Vector3 rawGyro;
Vector3 accel;
Vector3 gyro;

MPU6050 imu;

void setup() {
  // put your setup code here, to run once:
  initialization();
}

uint16_t reqTimer = 0;

void loop() {
  currentMillis = millis();
  updateCommand();
  updateSensors();
  updatePlan();
  updateAction();
  previousMillis = currentMillis;
  poll();
  if(currentMillis > reqTimer + 2000){
    reqTimer = currentMillis;
    #ifdef ARDUINO_AVR_NANO
      reqEcho();
    #endif
    #ifdef ARDUINO_AVR_MEGA2560
    #endif
  }
}

void initialization(){
  Wire.begin();
  imu.initialize();
  currentMillis = millis();
  reqTimer = millis();
  init_value();
  init_communication();
  init_pin();
}

void init_value(){  
}

int16_t filter(int16_t oldValue, int16_t newValue){
  return (oldValue << 2 + oldValue << 4 + oldValue << 8)+ newValue << 8;
}

void poll(){
    imu.getMotion6(&rawAccel.x, &rawAccel.y, &rawAccel.z, &rawGyro.x, &rawGyro.y, &rawGyro.z);
    accel.x = filter(accel.x, rawAccel.x);
    accel.y = filter(accel.y, rawAccel.y);
    accel.z = filter(accel.z, rawAccel.z);
    gyro.x = filter(rawGyro.x, rawGyro.x);
    gyro.y = filter(rawGyro.y, rawGyro.y);
    gyro.z = filter(rawGyro.z, rawGyro.z);
}

void init_communication(){
  Serial.begin(9600);
  blueToothSerial.begin(9600);
  init_buffer();
}

void init_pin(){}

void updatePlan(){}

void updateAction(){}

void updateSensors(){}
