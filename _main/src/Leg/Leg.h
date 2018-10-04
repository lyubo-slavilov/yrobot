struct LegConfig {
  byte t1CW;
  byte t1CCW;
  byte t2CW;
  byte t2CCW;
  byte b1CW;
  byte b1CCW;
  byte b2CW;
  byte b2CCW;
  byte anklePitchPin;
  byte ankleRollPin;
  byte hipPitchPin;
  byte hipRollPin;
  int anklePitchOffset;
  int ankleRollOffset;
  int hipPitchOffset;
  int hipRollOffset;
};

class Leg {
  private:
    byte foo;
    LegConfig cfg;
    Adafruit_PWMServoDriver *pwmDriver;

    PID *anklePitchPid;
    PID *ankleRollPid;
    PID *hipPitchPid;
    PID *hipRollPid;

    void readSensors() {
      int tmp;
      tmp = analogRead(cfg.anklePitchPin) - cfg.anklePitchOffset;
      anklePitch = (anklePitch * 4 + tmp) / 5;

      tmp = analogRead(cfg.ankleRollPin) - cfg.ankleRollOffset;
      ankleRoll = (ankleRoll * 4 + tmp)/ 5;

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

    Leg(String name, LegConfig cfg, Adafruit_PWMServoDriver* pwmDriver) {
      this->name = name;
      this->cfg = cfg;
      this->pwmDriver = pwmDriver;
      this->anklePitchPid = new PID(&anklePitch, &anklePitchSpeed, &anklePitchTarget, 10, 0, 0, P_ON_E, DIRECT);
      this->ankleRollPid = new PID(&ankleRoll, &ankleRollSpeed, &ankleRollTarget, 10, 0, 0, P_ON_E, DIRECT);
      this->hipPitchPid = new PID(&hipPitch, &hipPitchSpeed, &hipPitchTarget, 10, 0, 0, P_ON_E, DIRECT);
      this->hipRollPid = new PID(&hipRoll, &hipRollSpeed, &hipRollTarget, 10, 0, 0, P_ON_E, DIRECT);

    };

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
      delay(10);
    }

    void runAllMotorsCW(word speed) {
      if (speed < 0) speed = 0;
      if (speed > 4095) speed = 4095;

      pwmDriver->setPWM(cfg.t1CW, 0, speed);
      pwmDriver->setPWM(cfg.t2CW, 0, speed);
      pwmDriver->setPWM(cfg.b1CW, 0, speed);
      pwmDriver->setPWM(cfg.b2CW, 0, speed);
      delay(10);

    }

    void runAllMotorsCCW(word speed) {
      if (speed < 0) speed = 0;
      if (speed > 4095) speed = 4095;

      pwmDriver->setPWM(cfg.t1CCW, 0, speed);
      pwmDriver->setPWM(cfg.t2CCW, 0, speed);
      pwmDriver->setPWM(cfg.b1CCW, 0, speed);
      pwmDriver->setPWM(cfg.b2CCW, 0, speed);
      delay(10);

    }

    void runForTargets() {

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

        // String mtrDir = "CW";
        // if (mtrLeft > 0) {
        //   analogWrite(mtrLeftPwmCCWPin, 0);
        //   analogWrite(mtrLeftPwmCWPin, (int) mtrLeft);
        // } else {
        //   analogWrite(mtrLeftPwmCCWPin, (int) (- mtrLeft));
        //   analogWrite(mtrLeftPwmCWPin, 0);
        // }
        //
        //
        // if (mtrRight > 0) {
        //   analogWrite(mtrRightPwmCCWPin, 0);
        //   analogWrite(mtrRightPwmCWPin, (int) mtrRight);
        // } else {
        //   analogWrite(mtrRightPwmCCWPin, (int) (- mtrRight));
        //   analogWrite(mtrRightPwmCWPin, 0);
        // }
    }

    void computeSpeeds() {

      anklePitchPid->Compute();
      ankleRollPid->Compute();
      hipPitchPid->Compute();
      hipRollPid->Compute();


      if (abs(anklePitch - anklePitchTarget) <= 10) {
        anklePitchSpeed = 0;
      }
      //anklePitchSpeed = map(anklePitchSpeed, -255, 255, -4095, 4095);

      if (abs(ankleRoll - ankleRollTarget) <= 10) {
        ankleRollSpeed = 0;
      }
      //ankleRollSpeed = map(ankleRollSpeed, -255, 255, -4095, 4095);

      if (abs(hipPitch - hipPitchTarget) <= 10) {
        hipPitchSpeed = 0;
      }
      //hipPitchSpeed = map(hipPitchSpeed, -255, 255, -4095, 4095);

      if (abs(hipRoll - hipRollTarget) <= 10) {
        hipRollSpeed = 0;
      }
      //hipRollSpeed = map(hipRollSpeed, -255, 255, -4095, 4095);
    }

    void setup() {
      pinMode(cfg.anklePitchPin, INPUT);
      pinMode(cfg.ankleRollPin, INPUT);
      pinMode(cfg.hipPitchPin, INPUT);
      pinMode(cfg.hipRollPin, INPUT);


      anklePitchPid->SetMode(AUTOMATIC);
      anklePitchPid->SetOutputLimits(-4095, 4095);
      ankleRollPid->SetMode(AUTOMATIC);
      ankleRollPid->SetOutputLimits(-4095, 4095);

      hipPitchPid->SetMode(AUTOMATIC);
      hipPitchPid->SetOutputLimits(-4095, 4095);
      hipRollPid->SetMode(AUTOMATIC);
      hipRollPid->SetOutputLimits(-4095, 4095);
    }

    void loop() {
      readSensors();
      computeSpeeds();

      Serial.println(String(anklePitchSpeed) + "," + String(ankleRollSpeed));
    }


};
