#include "I2Cdev.h"
//#include "MPU6050.h"
#include "MPU6050_6Axis_MotionApps20.h"
MPU6050 mpu;

int16_t gyroX,gyroY,gyroZ, gyroRate1,gyroRate2,gyroRate3;
float gyroAngle1=0,gyroAngle2=0,gyroAngle3=0;
unsigned long currTime, prevTime=0, loopTime;

void setup() {  
  mpu.initialize();
  Serial.begin(9600);
}

void loop() {
  currTime = millis();
  loopTime = currTime - prevTime;
  prevTime = currTime;
  
  gyroZ = mpu.getRotationZ();
  gyroY = mpu.getRotationY();
  gyroX = mpu.getRotationX();
  gyroRate1 = map(gyroX, -32768, 32767, -250, 250);
  gyroRate2 = map(gyroY, -32768, 32767, -250, 250);
  gyroRate3 = map(gyroZ, -32768, 32767, -250, 250);
  gyroAngle1 = gyroAngle1 + (float)gyroRate1*loopTime/1000;
  gyroAngle2 = gyroAngle2 + (float)gyroRate2*loopTime/1000;
  gyroAngle3 = gyroAngle3 + (float)gyroRate3*loopTime/1000;
  
  Serial.print(gyroAngle1);
  Serial.print("\t");
  Serial.print(gyroAngle2);
  Serial.print("\t");
  Serial.print(gyroAngle3);
  Serial.println();
}
