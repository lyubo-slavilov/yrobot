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
 * The Leg config structure
 * Contains information for different pins and PWM channels used by the Leg class to
 * control the leg
 */
struct LegConfig {
  byte t1CW;            //pwm channel for the top motor 1 clock wise
  byte t1CCW;           //pwm channel for the top motor 1 counter clock wise
  byte t2CW;            //same for the top motor 2
  byte t2CCW;
  byte b1CW;            //pwm channel for the bottom motor 1 clock wise
  byte b1CCW;           //etc...
  byte b2CW;
  byte b2CCW;
  byte anklePitchPin;   //Pin used to read the anckle pitch potentiometer value
  byte ankleRollPin;    //etc...
  byte hipPitchPin;
  byte hipRollPin;
  int anklePitchOffset; //Offsets for the pots
  int ankleRollOffset;
  int hipPitchOffset;
  int hipRollOffset;
  int pidSampleTime;

};

/**
 * Leg class representing, guess what...
 * It contains most of the logic for PID control the leg pitch & roll
 */
class Leg {
  private:
    byte foo;
    LegConfig cfg;
    Adafruit_PWMServoDriver *pwmDriver;

    PID *anklePitchPid;
    PID *ankleRollPid;
    PID *hipPitchPid;
    PID *hipRollPid;

    /**
     * Reads the potentiometers values, offsets them and perfomr
     * weighted smoothing of the signal
     */
    void readSensors() {
      int tmp;
      tmp = analogRead(cfg.anklePitchPin) - cfg.anklePitchOffset;
      anklePitch = (anklePitch * 4 + tmp) / 5;

      tmp = analogRead(cfg.ankleRollPin) - cfg.ankleRollOffset;
      ankleRoll =(ankleRoll * 4 + tmp)/ 5;

      tmp = analogRead(cfg.hipPitchPin) - cfg.hipPitchOffset;
      hipPitch = (hipPitch * 4 + tmp) / 5;

      tmp = analogRead(cfg.hipRollPin) - cfg.hipRollOffset;
      hipRoll = (hipRoll * 4 + tmp) / 5;
    }

  public:
    String name;

    double anklePitch;
    double ankleRoll;
    double hipPitch;
    double hipRoll;

    double anklePitchTarget;
    double ankleRollTarget;
    double hipPitchTarget;
    double hipRollTarget;

    double anklePitchSpeed = 0;
    double ankleRollSpeed = 0;
    double hipPitchSpeed = 0;
    double hipRollSpeed = 0;

    /**
     *
     * @param name      Useful for debugging
     * @param cfg       the configuration of the leg
     * @param pwmDriver A reference to the PWM driver object
     */
    Leg(String name, LegConfig cfg, Adafruit_PWMServoDriver* pwmDriver) {

      this->name = name;
      this->cfg = cfg;
      this->pwmDriver = pwmDriver;
      this->anklePitchPid = new PID(&anklePitch, &anklePitchSpeed, &anklePitchTarget, 10, 0, 0, P_ON_E, DIRECT);
      this->ankleRollPid = new PID(&ankleRoll, &ankleRollSpeed, &ankleRollTarget, 10, 0, 0, P_ON_E, DIRECT);
      this->hipPitchPid = new PID(&hipPitch, &hipPitchSpeed, &hipPitchTarget, 20, 0, 0, P_ON_E, DIRECT);
      this->hipRollPid = new PID(&hipRoll, &hipRollSpeed, &hipRollTarget, 10, 0, 0, P_ON_E, DIRECT);

    };

    /**
     * Dumps a bunch of data read from the pots
     * @param endChar Useful when debugging both the legs.
     * You can use "," for the first leg and "\n" for the second.
     * This way you will get one line for the two legs and values will be comma separated
     */
    void debugSensors(String endChar = "\n") {
      Serial.print(anklePitch); Serial.print(",");
      Serial.print(ankleRoll); Serial.print(",");
      Serial.print(hipPitch); Serial.print(",");
      Serial.print(hipRoll); ; Serial.print(endChar);
    }


    void stopAllMotors() {
      pwmDriver->setPWM(cfg.t1CW, 0, 4096);
      pwmDriver->setPWM(cfg.t1CCW, 0, 4096);
      pwmDriver->setPWM(cfg.t2CW, 0, 4096);
      pwmDriver->setPWM(cfg.t2CCW, 0, 4096);
      pwmDriver->setPWM(cfg.b1CW, 0, 4096);
      pwmDriver->setPWM(cfg.b1CCW, 0, 4096);
      pwmDriver->setPWM(cfg.b2CW, 0, 4096);
      pwmDriver->setPWM(cfg.b2CCW, 0, 4096);
      //delay(10);
    }

    /**
     * Runs all motors clockwise (hopefully)
     * @param speed 4095 is bassicaly the full motor speed which it is capable of
     */
    void runAllMotorsCW(word speed) {
      if (speed < 0) speed = 0;
      if (speed > 4095) speed = 4095;

      pwmDriver->setPWM(cfg.t1CW, 0, speed);
      pwmDriver->setPWM(cfg.t2CW, 0, speed);
      pwmDriver->setPWM(cfg.b1CW, 0, speed);
      pwmDriver->setPWM(cfg.b2CW, 0, speed);
      //delay(10);

    }
    /**
     * @see runAllMotorsCW()
     * @param speed [description]
     */
    void runAllMotorsCCW(word speed) {
      if (speed < 0) speed = 0;
      if (speed > 4095) speed = 4095;

      pwmDriver->setPWM(cfg.t1CCW, 0, speed);
      pwmDriver->setPWM(cfg.t2CCW, 0, speed);
      pwmDriver->setPWM(cfg.b1CCW, 0, speed);
      pwmDriver->setPWM(cfg.b2CCW, 0, speed);
      //delay(10);

    }
    /**
     * Helper to clear all targets.
     * It basically will tell the leg to go for "stand" posture
     */
    void clearTargets() {
      ankleRollTarget = 0;
      hipRollTarget = 0;
      anklePitchTarget = 0;
      hipPitchTarget = 0;
    }

    /**
     * Uses PID calculated speeds to drive the motors accordingly
     */
    void runForTargets() {

        //Ankle
        double mtr1 =  anklePitchSpeed + ankleRollSpeed;
        double mtr2 =  anklePitchSpeed -  ankleRollSpeed;

        if (mtr1 > 0) {
          if (mtr1 > 4095) mtr1 = 4095;
          pwmDriver->setPWM(cfg.b1CW, 0, 4096);
          pwmDriver->setPWM(cfg.b1CCW, 0, (int) mtr1);
        } else {
          if (mtr1 < -4095) mtr1 = -4095;
          pwmDriver->setPWM(cfg.b1CW, 0, (int) - mtr1);
          pwmDriver->setPWM(cfg.b1CCW, 0, 4096);
        }

        if (mtr2 > 0) {
          if (mtr2 > 4095) mtr2 = 4095;
          pwmDriver->setPWM(cfg.b2CW, 0, 4096);
          pwmDriver->setPWM(cfg.b2CCW, 0,(int) mtr2);
        } else {
          if (mtr2 < -4095) mtr2 = -4095;
          pwmDriver->setPWM(cfg.b2CW, 0, (int) - mtr2);
          pwmDriver->setPWM(cfg.b2CCW, 0, 4096);
        }

        //Hip
        mtr1 =  ( hipPitchSpeed - hipRollSpeed);
        mtr2 =  (hipPitchSpeed +  hipRollSpeed);

        if (mtr1 > 0) {
          if (mtr1 > 4095) mtr1 = 4095;
          pwmDriver->setPWM(cfg.t1CW, 0, 4096);
          pwmDriver->setPWM(cfg.t1CCW, 0, (int) mtr1);
        } else {
          if (mtr1 < -4095) mtr1 = -4095;
          pwmDriver->setPWM(cfg.t1CW, 0, (int) - mtr1);
          pwmDriver->setPWM(cfg.t1CCW, 0, 4096);
        }

        if (mtr2 > 0) {
          if (mtr2 > 4095) mtr2 = 4095;
          pwmDriver->setPWM(cfg.t2CW, 0, 4096);
          pwmDriver->setPWM(cfg.t2CCW, 0,(int) mtr2);
        } else {
          if (mtr2 < -4095) mtr2 = -4095;
          pwmDriver->setPWM(cfg.t2CW, 0, (int) - mtr2);
          pwmDriver->setPWM(cfg.t2CCW, 0, 4096);
        }
    }

    void computeSpeeds() {
      //Do the PID magic
      anklePitchPid->Compute();
      ankleRollPid->Compute();
      hipPitchPid->Compute();
      hipRollPid->Compute();

      //Check for deadspots
      if (abs(anklePitch - anklePitchTarget) <= 10) {
        anklePitchSpeed = 0;
      }

      if (abs(ankleRoll - ankleRollTarget) <= 10) {
        ankleRollSpeed = 0;
      }

      if (abs(hipPitch - hipPitchTarget) <= 10) {
        hipPitchSpeed = 0;
      }

      if (abs(hipRoll - hipRollTarget) <= 10) {
        hipRollSpeed = 0;
      }
    }


    /**
     * Setups the leg.
     * Call this in the main sketch's setup()
     *
     */
    void setup() {
      pinMode(cfg.anklePitchPin, INPUT);
      pinMode(cfg.ankleRollPin, INPUT);
      pinMode(cfg.hipPitchPin, INPUT);
      pinMode(cfg.hipRollPin, INPUT);


      anklePitchPid->SetMode(AUTOMATIC);
      anklePitchPid->SetOutputLimits(-4095, 4095);
      anklePitchPid->SetSampleTime(cfg.pidSampleTime);

      ankleRollPid->SetMode(AUTOMATIC);
      ankleRollPid->SetOutputLimits(-4095, 4095);
      ankleRollPid->SetSampleTime(cfg.pidSampleTime);


      hipPitchPid->SetMode(AUTOMATIC);
      hipPitchPid->SetOutputLimits(-4095, 4095);
      hipPitchPid->SetSampleTime(cfg.pidSampleTime);

      hipRollPid->SetMode(AUTOMATIC);
      hipRollPid->SetOutputLimits(-4095, 4095);
      hipRollPid->SetSampleTime(cfg.pidSampleTime);

    }

    /**
     * Does the repeatable stuff
     * It is important to call this AFTER you set the targets for the leg pitch & roll
     * After the call, all new sensor data and computed speeds will be available and runForTargets() can be executed()
     */
    void loop() {
      readSensors();
      computeSpeeds();
    }


};
