
/**
 * Author: <Lyubo Slavilov> lyubo.slavilov@gmail.com
 *
 * DO WHAT THE FUCK YOU WANT TO PUBLIC LICENSE
 *              Version 2, December 2004
 *
 * Copyright (C) 2004 Sam Hocevar <sam@hocevar.net>
 *
 * Everyone is permitted to copy and distribute verbatim or modified
 * copies of this license document, and changing it is allowed as long
 * as the name is changed.
 *
 *      DO WHAT THE FUCK YOU WANT TO PUBLIC LICENSE
 * TERMS AND CONDITIONS FOR COPYING, DISTRIBUTION AND MODIFICATION
 *
 * 0. You just DO WHAT THE FUCK YOU WANT TO.
 */

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <PID_v1.h>
#include "src/Leg/Leg.h"
#include "src/Pelvis/Pelvis.h"
#include <EasyTransfer.h>

// Our loop should exec 100 times per second
// in order to leave time  the Arduino to deal with itself...
#define ROBOT_LOOP_RATE 100

/*===================================== */


//Motion data to be transfered via Serial
struct MotionData {
  float pitch;
  float roll;
};

MotionData motionData;
EasyTransfer pelvisET;

unsigned long lastLoopTickAt; //in milliseconds;
unsigned long currentLoopTickAt; //in milliseconds



Adafruit_PWMServoDriver pwmDriver = Adafruit_PWMServoDriver();

Leg leftLeg("LEFT",
  LegConfig{
    1, 0, 3, 2, 5, 4, 7, 6,
    A1, A0, A3, A2,
    2640, 2023, 2881, 3819,
    1000 / ROBOT_LOOP_RATE
  },
  &pwmDriver
);

Leg rightLeg("RIGHT",
  LegConfig{
    9, 8, 11, 10, 13, 12, 15, 14,
    A5, A4, A7, A6,
    2657, 1846, 2633, 2354,
    1000 / ROBOT_LOOP_RATE
  },
  &pwmDriver
);


double pelvisPitch = 0;
Pelvis pelvis(
  PelvisConfig{
    2, 3, A8, 2636,
    1000 / ROBOT_LOOP_RATE
  },
  &pelvisPitch
);

bool tick = false; //tick flag for visual debugging
int counter = 0;

/**
 * -----------------------------------------------------------------------------
 * The setup function.
 * This docblock only purpose is to provide visual clue where the setup() starts
 * -----------------------------------------------------------------------------
 */
void setup() {

  Serial.begin(115200);
  analogReadResolution(12);

  delay(500);
  pwmDriver.begin();
  delay(500);
  pwmDriver.setPWMFreq(1900);
  //Wire.setClock(200000);

  leftLeg.setup();
  rightLeg.setup();
  pelvis.setup();

  pinMode(22, INPUT_PULLUP);
  pinMode(23, INPUT_PULLUP);

  Serial1.begin(115200);
  pelvisET.begin(details(motionData), &Serial1);


  //Prepare for start
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(13, false);
  delay(2000);
  pelvisET.receiveData();
}



/**
 * -----------------------------------------------------------------------------
 * The loop function.
 * This docblock only purpose is to provide visual clue where the loop() starts
 * -----------------------------------------------------------------------------
 */
void loop() {

  if (pelvisET.receiveData()){
    //IMPORTANT: We are using the "roll" field as pitch value since the sensor is mounted 90 deg rotated;
    pelvisPitch = motionData.roll*180/M_PI;
  };


  //Perform tick-tack tracking and decide if the main loop should be executed
  currentLoopTickAt  = millis();
  if (currentLoopTickAt - lastLoopTickAt < 1000 / ROBOT_LOOP_RATE) {
    //It is too early to do anything. Get out of here;
    return;
  }
  lastLoopTickAt = currentLoopTickAt;

  //Visual debugging
  counter++;
  if (counter >= 10) {
    counter = 0;
    tick = !tick;
    digitalWrite(LED_BUILTIN, tick);
  }


  //Read the "remote" control
  int lBtn = ! digitalRead(22);
  int rBtn = ! digitalRead(23);

  //Perform pelvis internal computations
  pelvis.loop();

  //1. Compute  and set targets for the legs
  //IMPORTATN: we need pelvis' outputs to do this properly
  if (!lBtn && !rBtn) {

      leftLeg.clearTargets();
      rightLeg.clearTargets();

  } else {
    if (lBtn) {
      leftLeg.hipPitchTarget = pelvis.pitchTarget*2; // *2 because leg sensors has two times the resolution of the pelvis pot
      rightLeg.hipPitchTarget = pelvis.pitchTarget*2; //
    } else {

    }
  }

  //Perform legs internal computations
  leftLeg.loop();
  rightLeg.loop();

  //We have all the PID outputs so run for targets
  pelvis.runForTargets();
  leftLeg.runForTargets();
  rightLeg.runForTargets();

  return;
//  leftLeg.debugSensors(",512,-512,");
  rightLeg.debugSensors();
}
