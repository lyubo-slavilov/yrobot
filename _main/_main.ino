#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <PID_v1.h>
#include "src/Leg/Leg.h"

 


/*===================================== */

Adafruit_PWMServoDriver pwmDriver = Adafruit_PWMServoDriver();

Leg leftLeg("LEFT", LegConfig{1, 0, 3, 2, 5, 4, 7, 6, A1, A0, A3, A2, 2640, 2023, 2881, 3819}, &pwmDriver);
Leg rightLeg("RIGHT", LegConfig{9, 8, 11, 10, 13, 12, 15, 14, A5, A4, A7, A6, 2657, 1846, 2633, 2354}, &pwmDriver);



void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  analogReadResolution(12);
  
  pwmDriver.begin();
  pwmDriver.setPWMFreq(800); 
  Wire.setClock(200000);
  
  leftLeg.setup();
  rightLeg.setup();

  pinMode(22, INPUT_PULLUP);
  pinMode(23, INPUT_PULLUP);


}

void loop() {
  // put your main code here, to run repeatedly:
  int lBtn = ! digitalRead(22);
  int rBtn = ! digitalRead(23);

  leftLeg.loop();
  rightLeg.loop();
  //Serial.println (String(lBtn) + "," + String(rBtn));
  
  if (!lBtn && !rBtn) {
//    leftLeg.stopAllMotors();
//    rightLeg.stopAllMotors();
      leftLeg.ankleRollTarget = 0;
      rightLeg.ankleRollTarget = 0;
      leftLeg.hipRollTarget = 0;
      rightLeg.hipRollTarget = 0;
      
      leftLeg.anklePitchTarget = 0;
      rightLeg.anklePitchTarget = 0;
      leftLeg.hipPitchTarget = 0;
      rightLeg.hipPitchTarget = 0;
  } else {
    if (lBtn) {
      leftLeg.ankleRollTarget = 0;
      rightLeg.ankleRollTarget = 0;
      leftLeg.hipRollTarget = 0;
      rightLeg.hipRollTarget = 0;

      leftLeg.anklePitchTarget = 0;
      rightLeg.anklePitchTarget = 0;
      leftLeg.hipPitchTarget = 0;
      rightLeg.hipPitchTarget = 0;
      
//      leftLeg.runAllMotorsCW(1024);
//      rightLeg.runAllMotorsCW(1024);
    } else {
      leftLeg.anklePitchTarget = 400;
      rightLeg.anklePitchTarget = 400;
      leftLeg.hipPitchTarget = 400;
      rightLeg.hipPitchTarget = 400;
//      leftLeg.runAllMotorsCCW(1024);
//      rightLeg.runAllMotorsCCW(1024);
    }
  }

  leftLeg.runForTargets();
  rightLeg.runForTargets();
  
  leftLeg.debugSensors(",512,-512,");
  rightLeg.debugSensors();
}
