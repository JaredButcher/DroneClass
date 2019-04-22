#include "Protocol.h"

char msgBuffer[MESSAGE_BUFFER_SIZE]; 
int msgBufferPointer = 0;
bool bluetoothConnected = false;

void evaluateStringCommand();
void evaluateBinaryCommand();
void echoCommand();

#ifdef ARDUINO_AVR_NANO
    HardwareSerial &blueToothSerial = Serial;
#endif
#ifdef ARDUINO_AVR_MEGA2560
    HardwareSerial &blueToothSerial = Serial3;
#endif

void init_buffer(){
  for (int i = 0; i < MESSAGE_BUFFER_SIZE; i++){
    msgBuffer[i] = 0x00;
  }
  msgBufferPointer = 0;
}


void updateCommand(){
  if (blueToothSerial.available() > 0){
    char tmpChar = blueToothSerial.read();
    if ((msgBufferPointer == 0)&&(tmpChar == '<')){           // '<' 0x3C
      msgBuffer[msgBufferPointer] = tmpChar; msgBufferPointer++;
    }else if (msgBufferPointer == 1){
      if  ((tmpChar == '@')||(tmpChar == '!')){     // '!'  (0x21 ==> Binary Format)or '@' (0x40 ==> String format)
        msgBuffer[msgBufferPointer] = tmpChar; msgBufferPointer++; 
      }else{
        msgBufferPointer = 0;
        bluetoothConnected = false;
      }
    }else if (msgBufferPointer > MESSAGE_BUFFER_SIZE){ 
      msgBufferPointer = 0;
      if ( tmpChar == '<') {
        msgBuffer[0] = tmpChar;    
        msgBufferPointer = 1;          
      }
    }else if (msgBufferPointer > 1) {
      if (( msgBuffer[1] == '@')&& (tmpChar == '>')) {
        evaluateStringCommand();
        msgBufferPointer = 0;        
      }else if  (( msgBuffer[1] == '!')&& (msgBuffer[2] == msgBufferPointer )) {
        evaluateBinaryCommand();
        msgBufferPointer = 0;
      }else{
        msgBuffer[msgBufferPointer] = tmpChar;
        msgBufferPointer++;
      }
    }
  }  
}




void evaluateStringCommand(){
  bluetoothConnected = true;
  if (msgBuffer[2] == MSG_GET_ECHO)   echoCommand();
  else if (msgBuffer[2] == MSG_SEND_LOG) logCommand();
  else if (msgBuffer[2] == MSG_GET_STATUS){
    #ifdef ARDUINO_AVR_MEGA2560
      getStatus();
    #endif
    #ifdef ARDUINO_AVR_NANO
      sendStatus();
    #endif
  }
  else bluetoothConnected = false;
  
}
void evaluateBinaryCommand(){
  bluetoothConnected = true;
  switch(msgBuffer[3]){
    case MSG_GET_RAW_IMU:
      sendRawIMU();
    break;
    case MSG_SEND_RAW_IMU:
      recRawIMU();
    break;
    case MSG_GET_ECHO:
      echoCommand();
    break;
    case MSG_SEND_JOYINPUT:
      getJoyInput();
    break;
    default:
      bluetoothConnected = false;
    break;
  }
}

void recRawIMU(){
  otherAccel.x = (((uint16_t)msgBuffer[4]) << 8) + msgBuffer[5];
  otherAccel.y = (((uint16_t)msgBuffer[6]) << 8) + msgBuffer[7];
  otherAccel.z = (((uint16_t)msgBuffer[8]) << 8) + msgBuffer[9];
  otherGyro.x = (((uint16_t)msgBuffer[10]) << 8) + msgBuffer[11];
  otherGyro.y = (((uint16_t)msgBuffer[12]) << 8) + msgBuffer[13];
  otherGyro.z = (((uint16_t)msgBuffer[14]) << 8) + msgBuffer[15];
}

void getRawIMU(){
  blueToothSerial.write("<!");
  blueToothSerial.write(4);
  blueToothSerial.write(MSG_GET_RAW_IMU);
}

void logCommand(){
  Serial.print("Log: ");
  for(int i = 3; i < msgBufferPointer; i++){
    Serial.print(msgBuffer[i]);
  }
}

void sendLogCommand(char text[]){
  blueToothSerial.write("<@");
  blueToothSerial.write(MSG_SEND_LOG);
  blueToothSerial.write(text);
  blueToothSerial.write(">");
}

void echoCommand(){
  #ifdef ARDUINO_AVR_MEGA2560
    Serial.println("Echo Command");
    Serial.print("[");
  #endif
  for(int i = 0; i < msgBufferPointer; i++){
    Serial.print(msgBuffer[i]);
  }
  #ifdef ARDUINO_AVR_MEGA2560
    Serial.println("]");
  #endif
}

void reqStatus(){
  blueToothSerial.write("<@");
  blueToothSerial.write(MSG_GET_STATUS);
  blueToothSerial.write(">");
}

void sendStatus(){
  blueToothSerial.write("<@");
  blueToothSerial.write(MSG_GET_STATUS);
  blueToothSerial.write("Test Status");
  blueToothSerial.write(">");
}

void getStatus(){
  droneStatus = (String(msgBuffer)).substring(3, msgBufferPointer);
}

void sendRawIMU(){
  blueToothSerial.write("<!");
  blueToothSerial.write(16);
  blueToothSerial.write(MSG_SEND_RAW_IMU);
  blueToothSerial.write((char*)&accel.x, 2);
  blueToothSerial.write((char*)&accel.y, 2);
  blueToothSerial.write((char*)&accel.z, 2);
  blueToothSerial.write((char*)&gyro.x, 2);
  blueToothSerial.write((char*)&gyro.y, 2);
  blueToothSerial.write((char*)&gyro.z, 2);
}

void sendJoyInput(){
  blueToothSerial.write("<!");
  blueToothSerial.write(8);
  blueToothSerial.write(MSG_SEND_JOYINPUT);
  blueToothSerial.write(joyThrottle);
  blueToothSerial.write(joyYaw);
  blueToothSerial.write(joyRoll);
  blueToothSerial.write(joyPitch);
}

void getJoyInput(){
  joyThrottle = msgBuffer[4];
  joyYaw = msgBuffer[5];
  joyRoll = msgBuffer[6];
  joyPitch = msgBuffer[7];
}
