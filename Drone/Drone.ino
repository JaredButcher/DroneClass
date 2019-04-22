#include <Adafruit_SSD1306.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#include"Drone.h"

unsigned long imuClock = 0;
unsigned long motorClock = 0;
unsigned long currentMillis = 0;
unsigned long previousMillis = 0;

uint8_t joyThrottle =0 ; // 
uint8_t joyYaw  =0 ; // 
uint8_t joyRoll  =0 ; // 
uint8_t joyPitch  =0 ; //

String droneStatus = "Disconnected";

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
  #ifdef ARDUINO_AVR_NANO
    updateMotors();
  #endif
  #ifdef ARDUINO_AVR_MEGA2560
    updateJoy();
    printIMU();
    updateScreen();
  #endif
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
    pinMode(6, OUTPUT);
  #endif
  blueToothSerial.begin(9600);
  init_buffer();
}

int16_t filter(uint8_t power, int16_t oldValue, int16_t newValue){
  return oldValue - (oldValue >> power)+ newValue >> power;
}

void pollIMU(){
    imu.getMotion6(&rawAccel.x, &rawAccel.y, &rawAccel.z, &rawGyro.x, &rawGyro.y, &rawGyro.z);
    accel.x = filter(4, accel.x, rawAccel.x);
    accel.y = filter(4, accel.y, rawAccel.y);
    accel.z = filter(4, accel.z, rawAccel.z);
    gyro.x = filter(4, gyro.x, rawGyro.x);
    gyro.y = filter(4, gyro.y, rawGyro.y);
    gyro.z = filter(4, gyro.z, rawGyro.z);
}

void updateJoy(){
  joyThrottle = analogRead(A1) >> 2;
  joyYaw  = analogRead(A0) >> 2;
  joyRoll  = analogRead(A4) >> 2;
  joyPitch  = analogRead(A3) >> 2;
}

void updatePlan(){}

#define kp 1
#define ki 0
#define kd 0
#define defaultX 0
#define defaultY 0
#define defaultZ 0

Vector3 error;
Vector3 pastError;
Vector3 gyroGoal;
gyroGoal.x = defaultX;
gyroGoal.y = defaultY;
gyroGoal.z = defaultZ;

void updateMotors(){
    error.x = kp * (gyroGoal.x - gyro.x) + ki * pastError.x;;
    pastError.x += error.x;
    pastError.y = filter(6, pastError.y, gyro.y);
    pastError.z = filter(6, pastError.z, gyro.x);
}

void updateSensors(){}

void printIMU(){
  Serial.print("ax: ");
  Serial.print(accel.x);
  Serial.print(" ay: ");
  Serial.print(accel.y);
  Serial.print(" az: ");
  Serial.print(accel.z);
  Serial.print(" gx: ");
  Serial.print(gyro.x);
  Serial.print(" gy: ");
  Serial.print(gyro.y);
  Serial.print(" gz: ");
  Serial.print(gyro.z);
  Serial.println();
}

void updateScreen(){
  display.clearDisplay();

  if(bluetoothConnected){
    display.fillRect(80, 0, 8, 6, WHITE);
  }else{
    display.drawRect(80, 0, 8, 6, WHITE);
  }
  display.drawRect(105, 0, 4, 16, WHITE);
  display.fillRect(105, 0, 4, joyThrottle >> 4, WHITE);
  display.drawRect(110, 0, 4, 16, WHITE);
  display.fillRect(110, 0, 4, joyYaw >> 4, WHITE);
  display.drawRect(115, 0, 4, 16, WHITE);
  display.fillRect(115, 0, 4, joyPitch >> 4, WHITE);
  display.drawRect(120, 0, 4, 16, WHITE);
  display.fillRect(120, 0, 4, joyRoll >> 4, WHITE);
  
  display.setTextColor(WHITE);
  display.setCursor(30,32);
  display.setTextSize(1);
  display.print("Status:");
  display.setCursor(30,40);
  display.print(droneStatus);

  display.display();
}
