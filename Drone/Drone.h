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

extern uint8_t joyThrottle ; // 
extern uint8_t joyYaw ; // 
extern uint8_t joyRoll ; // 
extern uint8_t joyPitch ; //

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
extern Vector3 otherAccel;
extern Vector3 otherGyro;

extern MPU6050 imu;

extern String droneStatus;

#endif /* RES_UAV_MAIN_H_ */
