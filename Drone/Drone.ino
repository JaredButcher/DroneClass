#include <Adafruit_SSD1306.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#include"Drone.h"

unsigned long imuClock = 0;
unsigned long motorClock = 0;
unsigned long currentMillis = 0;
unsigned long previousMillis = 0;

char joyThrottle =0 ; // 
char joyYaw  =0 ; // 
char joyRoll  =0 ; // 
char joyPitch  =0 ; // 

String droneStatus = "";

Vector3 rawAccel;
Vector3 rawGyro;
Vector3 accel;
Vector3 gyro;
Vector3 otherAccel;
Vector3 otherGyro;

MPU6050 imu;

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET     4
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

void setup() {
  // put your setup code here, to run once:
  initialization();
}

uint16_t reqTimer = 0;

#define UPDATE_DELAY_MS 100

void loop() {
  currentMillis = millis();
  updateCommand();
  updatePlan();
  updateAction();
  previousMillis = currentMillis;
  pollIMU();
  if(currentMillis > reqTimer + UPDATE_DELAY_MS){
    reqTimer = currentMillis;
    #ifdef ARDUINO_AVR_NANO
    #endif
    #ifdef ARDUINO_AVR_MEGA2560
      reqStatus();
    #endif
  }
  #ifdef ARDUINO_AVR_MEGA2560
    updateScreen();
    updateJoy();
  #endif
}

void initialization(){
  Wire.begin();
  imu.initialize();
  currentMillis = millis();
  reqTimer = millis();
  #ifdef ARDUINO_AVR_MEGA2560
    Serial.begin(115200);
    Serial.flush();
    Serial3.flush();
    display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
    pinMode(A0, INPUT);
    pinMode(A1, INPUT);
    pinMode(A2, INPUT);
    pinMode(A3, INPUT);
    pinMode(A4, INPUT);
    pinMode(A5, INPUT);
  #endif
  blueToothSerial.begin(9600);
  init_buffer();
}

int16_t filter(int16_t oldValue, int16_t newValue){
  return (oldValue << 2 + oldValue << 4 + oldValue << 8)+ newValue << 8;
}

void pollIMU(){
    imu.getMotion6(&rawAccel.x, &rawAccel.y, &rawAccel.z, &rawGyro.x, &rawGyro.y, &rawGyro.z);
    accel.x = filter(accel.x, rawAccel.x);
    accel.y = filter(accel.y, rawAccel.y);
    accel.z = filter(accel.z, rawAccel.z);
    gyro.x = filter(gyro.x, rawGyro.x);
    gyro.y = filter(gyro.y, rawGyro.y);
    gyro.z = filter(gyro.z, rawGyro.z);
}

void updateJoy(){
  joyThrottle = analogRead(A2);
  joyYaw  = analogRead(A1);
  joyRoll  = analogRead(A4);
  joyPitch  = analogRead(A5);
}

void updatePlan(){}

void updateAction(){}

void updateSensors(){}

void updateScreen(){
  display.clearDisplay();

  if(bluetoothConnected){
    display.fillRect(90, 0, 8, 6, WHITE);
  }else{
    display.drawRect(90, 0, 8, 6, WHITE);
  }
  
  display.setTextColor(WHITE);
  display.setCursor(30,32);
  display.setTextSize(1);
  display.print("Status:");
  display.setCursor(30,40);
  display.print(droneStatus);

  display.display();
}
