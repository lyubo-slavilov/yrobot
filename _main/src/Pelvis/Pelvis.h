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

/**
 * The Pelvis config structure
 * Contains information for different pins and PWM channels used by the Pelvis class to
 * control the leg
 */
struct PelvisConfig {
  byte pwmPinCW;
  byte pwmPinCCW;
  byte pitchPin;
  int pitchOffset;
  int pidSampleTime;
};

/**
 * This class represent the pelvis and
 * its resemblence with the LEG is big.
 * For further details how it works:
 * @see Leg
 */
class Pelvis {
private:
    PelvisConfig cfg;
    double *imuPitch;


    PID *pitchPid;
    PID *stabilityPid;




    double pitchSpeed = 0;
    double stabilytyCorrection = 0;

    void readSensors() {
      int tmp = analogRead(cfg.pitchPin ) - cfg.pitchOffset;
      pitch = (pitch*9  +  tmp) / 10;
    }

    void computeStability() {

      //Tries to follow the IMU readings
      stabilityPid->Compute();

      //Positioning
      pitchTarget = constrain(-stabilytyCorrection, -150, 150);
      pitchPid->Compute();

    }

public:
  double pitch = 0;
  double pitchTarget;
  double stabilityTarget;

  Pelvis(PelvisConfig cfg, double* imuPitch) {
    this->cfg = cfg;
    this->imuPitch = imuPitch;

    this->pitchPid = new PID(&pitch, &pitchSpeed, &pitchTarget, 5, 0, 0.1, P_ON_E, DIRECT);
    this->stabilityPid = new PID(imuPitch, &stabilytyCorrection, &stabilityTarget, 50, 0, 1, P_ON_E, DIRECT);

  }

  void runForTargets() {
    if (abs(pitchSpeed) <= 10) {
      pitchSpeed = 0;
    };

    if (pitchSpeed >= 0) {
      analogWrite(cfg.pwmPinCW, (int) pitchSpeed);
      analogWrite(cfg.pwmPinCCW, 0);
    } else {
      analogWrite(cfg.pwmPinCW, 0);
      analogWrite(cfg.pwmPinCCW,  (int) - pitchSpeed);
    }
  }

  void runMotorCW() {
    analogWrite(cfg.pwmPinCW, 512);
    analogWrite(cfg.pwmPinCCW, 0);
  }

  void runMotorCCW() {
    analogWrite(cfg.pwmPinCW, 0);
    analogWrite(cfg.pwmPinCCW, 512);
  }

  void stopMotor() {
    analogWrite(cfg.pwmPinCW, 0);
    analogWrite(cfg.pwmPinCCW, 0);
  }

  void setup() {
    pinMode(cfg.pitchPin, INPUT);
    pinMode(cfg.pwmPinCW, OUTPUT);
    pinMode(cfg.pwmPinCCW, OUTPUT);

    pitchPid->SetMode(AUTOMATIC);
    pitchPid->SetOutputLimits(-512, 512);
    pitchPid->SetSampleTime(cfg.pidSampleTime);

    stabilityPid->SetMode(AUTOMATIC);
    stabilityPid->SetOutputLimits(-150, 150);
    stabilityPid->SetSampleTime(cfg.pidSampleTime);
  }

  void loop() {
    readSensors();
    computeStability();
  }

  void debugSensors(String suffix = "\n") {
    Serial.println("imu pitch, pot pitch, stability, servo");
    Serial.print( *imuPitch*100);
    Serial.print(",");
    Serial.print(pitch);
    Serial.print(",");
    Serial.print(stabilytyCorrection);
    Serial.print(",");
    Serial.print(pitchSpeed);
    Serial.print(suffix);
  }

};
