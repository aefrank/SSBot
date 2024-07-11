/*

  SSBot.h - Library for controlling a simple differential drive robot.
  
  Created by Andrea E. Frank for Summer Springboard 2024 Fundamentals of Engineering course.

*/
#ifndef SSBot_h
#define SSBot_h

#include <string.h>

#include "Arduino.h"

#include <NewPing.h>

#define DECODE_NEC      
#define USE_IRREMOTE_HPP_AS_PLAIN_INCLUDE
#include <IRremote.hpp>


// ------------------ SONAR ------------------

class SSBotSonar{
    NewPing _sensor;
    const uint8_t _trigPin, _echoPin;
    const unsigned long _sensorPeriodMillis;
    unsigned long _lastReadTime;
    unsigned int _lastDistance;
  public:
    const unsigned int clearanceThreshold;
    SSBotSonar(uint8_t trigPin, uint8_t echoPin, unsigned int clearanceThreshold=10, unsigned long Hz=20);
    bool clearAhead();
    void init();
    int read();
};

// speeds can range from -100 to 100, so "314" will be used to indicate default argument
#define DEFAULT_SPEED_FLAG 314

class SSBotMC {
    const uint8_t _leftEnable, _leftFwd, _leftRev, _rightEnable, _rightFwd, _rightRev;
    const int _leftMaxSpeed, _rightMaxSpeed, _defaultSpeed;

  public:
    SSBotMC(uint8_t leftEnablePin, uint8_t leftFwdPin, uint8_t leftRevPin, 
            uint8_t rightEnablePin, uint8_t rightFwdPin, uint8_t rightRevPin, 
            int leftMaxSpeed = 100, int rightMaxSpeed = 100, int defaultSpeed = 50);
    void init();
    // ------------------ Tandem motor actions ------------------
    void driveFwd(int speed = DEFAULT_SPEED_FLAG);
    void driveRev(int speed = DEFAULT_SPEED_FLAG);
    void drive(int speed = DEFAULT_SPEED_FLAG);
    void turnLeft(int speed = DEFAULT_SPEED_FLAG);
    void turnRight(int speed = DEFAULT_SPEED_FLAG);
    void stopMotors();
    // ---- control direction and PWM separately ---
    void setDirStop();
    void setDirFwd();
    void setDirRev();
    void setDirLeftTurn();
    void setDirRightTurn();
    void setPWM(int duty);
    // ------------------ Control motors individually ------------------
    void driveLeftMotorFwd(int speed = DEFAULT_SPEED_FLAG);
    void driveLeftMotorRev(int speed = DEFAULT_SPEED_FLAG);
    void driveRightMotorFwd(int speed = DEFAULT_SPEED_FLAG);
    void driveRightMotorRev(int speed = DEFAULT_SPEED_FLAG);
    void driveLeftMotor(int speed = DEFAULT_SPEED_FLAG);
    void driveRightMotor(int speed = DEFAULT_SPEED_FLAG);
    void stopLeftMotor();
    void stopRightMotor();
    // ---- control direction and PWM separately ---
    void setLeftMotorFwd();
    void setLeftMotorRev();
    void setRightMotorFwd();
    void setRightMotorRev();
    void setLeftMotorPWM(int duty);
    void setRightMotorPWM(int duty);
};



// ------------------ IR Receiver ------------------

class SSBotIR {
    const uint8_t _IRpin;
    // const IRrecv* _receiver;
    unsigned long _timeOfLastInterrupt;
  public:
    enum IRCommand {
      CMD_CHDOWN,   CMD_CH,     CMD_CHUP, 
      CMD_PREV,     CMD_NEXT,   CMD_PLAY, 
      CMD_VOLDOWN,  CMD_VOLUP,  CMD_EQ, 
      CMD_0,        CMD_100,    CMD_200, 
      CMD_1,        CMD_2,      CMD_3, 
      CMD_4,        CMD_5,      CMD_6, 
      CMD_7,        CMD_8,      CMD_9,
      NONE, ERROR
    };
    SSBotIR(uint8_t IRpin);
    // SSBotIR(uint8_t IRpin, IRrecv* rec);
    void init();
    bool iRCommandReceived();
    IRCommand query();
    static bool isValid(IRCommand command);
    static String str(IRCommand command);
  private:
    IRCommand _currentCommand();
    static String _raw2str(uint32_t command);

};





#endif



