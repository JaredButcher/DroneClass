#ifndef RES_UAV_MAIN_H_
#define RES_UAV_MAIN_H_

#include "Protocol.h"
#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050.h"


#define IMU_INTERVAL  10

#define MOTOR_0_ENABLE   13
#define MOTOR_0_A        12
#define MOTOR_0_B        11

extern unsigned long imuClock;
extern unsigned long motorClock;
extern unsigned long currentMillis;
extern unsigned long previousMillis;

extern int joyThrottle ; // 
extern int joyYaw ; // 
extern int joyRoll ; // 
extern int joyPitch ; //

struct Vector3{
    int16_t x;
    int16_t y;
    int16_t z;
};

void poll();

int16_t filter(int16_t oldValue, int16_t newValue);


extern Vector3 rawAccel;
extern Vector3 rawGyro;
extern Vector3 accel;
extern Vector3 gyro;

extern MPU6050 imu;


#endif /* RES_UAV_MAIN_H_ */
