#include "Protocol.h"

char msgBuffer[MESSAGE_BUFFER_SIZE]; 
int msgBufferPointer = 0;


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
  if (msgBuffer[2] == MSG_GET_ECHO)   echoCommand();
  else if (msgBuffer[2] == MSG_REQ_ECHO)   reqEcho();
  else if (msgBuffer[2] == MSG_GET_STATUS)  statusCommand();
  else if (msgBuffer[2] == MSG_SET_JOYINPUT) setJoyInput();
  
}
void evaluateBinaryCommand(){
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
  }
}

void recRawIMU(){
  Serial.print("RAW IMU REC: ");
  uint16_t imuValue = 0;
  imuValue = msgBuffer[6];
  imuValue << 8;
  imuValue += msgBuffer[7];
  Serial.println(imuValue);
}

void getRawIMU(){
  blueToothSerial.write("<!");
  blueToothSerial.write(4);
  blueToothSerial.write(MSG_GET_RAW_IMU);
}

void reqEcho(){
  Serial.println("Echo");
  blueToothSerial.write("<@");
  blueToothSerial.write(MSG_REQ_ECHO);
  blueToothSerial.write('>');
}

void echoCommand(){
  Serial.println("Echo Command");
  Serial.print("[");
  for(int i = 0; i < msgBufferPointer; i++){
    Serial.print(msgBuffer[i]);
  }
  Serial.println("]");
}

void statusCommand(){
  Serial.print( joyThrottle ); Serial.print(" ");  Serial.print( joyYaw ); Serial.print(" ");  Serial.print( joyRoll ); Serial.print(" ");   Serial.print( joyPitch );
}

void sendRawIMU(){
  blueToothSerial.write("<!");
  blueToothSerial.write(16);
  blueToothSerial.write(MSG_SEND_RAW_IMU);
  blueToothSerial.write((char*)&rawAccel.x, 2);
  blueToothSerial.write((char*)&rawAccel.y, 2);
  blueToothSerial.write((char*)&rawAccel.z, 2);
  blueToothSerial.write((char*)&rawGyro.x, 2);
  blueToothSerial.write((char*)&rawGyro.y, 2);
  blueToothSerial.write((char*)&rawGyro.z, 2);
}

void setJoyInput(){    
}

/*
   char str[64] = "<@ b 1111 2222 3333 4444";
   const char s[2] = "-";
   char *token;   
   // get the first token 
   token = strtok(str, s);   
   // walk through other tokens 
   while( token != NULL ) {
      printf( " %s\n", token );    
      token = strtok(NULL, s);
   }
*/

/*
    char *s;
    s = " -9885";
    i = atoi(s);     // i = -9885 
     
*/
