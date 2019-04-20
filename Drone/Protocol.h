#ifndef PROTOCOL_H_
#define PROTOCOL_H_

#include <string.h>
#include <stdio.h>
#include "Arduino.h"
#include "Drone.h"

/************************************** ATR Lab Simple UAV Protocol *******************************************************/
#define UAV_VERSION              1

#define MSG_GET_IDENT                0x41   // 'A' out message         ATR protocol version
#define MSG_GET_STATUS               0x42   // 'B' out message         status data 
#define MSG_GET_RAW_IMU              0x43   // 'C' out message         9 DOF
#define MSG_GET_ECHO                 0x44   // 'D' in message       message echo         

#define MSG_REQ_ECHO              0x45
#define MSG_SEND_RAW_IMU          0x63
#define MSG_SET_AUTO             0x61   // 'a' in message          Auto/Manual 
#define MSG_SET_JOYINPUT            0x62   // 'b' in message       Joystick input

#define MESSAGE_BUFFER_SIZE   64

extern HardwareSerial &blueToothSerial;

extern char msgBuffer[MESSAGE_BUFFER_SIZE]; 
extern int msgBufferPointer;


void evaluateCommand();
void updateCommand();
void init_buffer();

void reqEcho();
void getRawIMU();
void sendRawIMU();
void recRawIMU();
void statusCommand();
void setMotorPower();
void setJoyInput();

#endif /* PROTOCOL_H_ */