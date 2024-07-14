/*

  SSBotMotor.cpp - Library for controlling a simple differential drive robot, v.2.
  
  Created by Andrea E. Frank for Summer Springboard 2024 Fundamentals of Engineering course.


*/

#include <SSBotMotor.hpp>

#define sgn(x) ((x) < 0 ? -1 : ((x) > 0 ? 1 : 0))

using namespace SummerSpringBot;

//================  MOTOR CONTROL =================



Motor::Motor( uint8_t fwdPin, uint8_t revPin, uint8_t pwmPin, 
                    int maxPWM, int defaultSpeed) :
        _fwdPin(fwdPin), _revPin(revPin), _pwmPin(pwmPin), 
        _maxPWM(maxPWM), _defaultSpeed(defaultSpeed) {
    _pwm = 0;
    _enabled = true;
    _state = STOPPED;
}

void Motor::init() {
    pinMode(_pwmPin, OUTPUT);
    pinMode(_fwdPin, OUTPUT);
    pinMode(_revPin, OUTPUT);
    sendMotorControl();
}

void Motor::_setDir(int8_t dir){
    switch (dir) {
        case 1:
            digitalWrite(_fwdPin, 1);
            digitalWrite(_revPin, 0);
            break;
        case -1:
            digitalWrite(_fwdPin, 0);
            digitalWrite(_revPin, 1);
            break;
        case 0:
            digitalWrite(_fwdPin, 0);
            digitalWrite(_revPin, 0);
            break;
        // default:
        //     throw std::invalid_argument("Motor._setDir() only accepts values 1, 0, or -1.");
    }
}

void Motor::_setPWM(uint8_t pwm){
    analogWrite(_pwmPin, pwm);
}

uint8_t Motor::_speedToPWM(uint8_t speed) {
    if (speed == NO_ARG_FLAG) {
        if (_pwm == 0) 
            return map(_defaultSpeed, 0, 100, 0, _maxPWM);
        else 
            return _pwm;
    } else {
        return map(speed, 0, 100, 0, _maxPWM);
    }
}

void Motor::sendMotorControl(){
    if(_enabled){
        _setDir(_state);
        _setPWM(_pwm);
    } else {
        _setPWM(0);
        _setDir(0);
    }
}

bool Motor::isEnabled() {
    return _enabled;
}

int8_t Motor::getState() {
    return _state;
}

String Motor::getStateString() {
    return stateToString(_state);
}

uint8_t Motor::getSpeed() {
    return map(_pwm, 0, 255, 0, 100);
}

int8_t Motor::getVelocity() {
    int8_t dir = getState();
    uint8_t speed = getSpeed();
    return dir * speed;
}

String Motor::stateToString(bool enabled) {
    if (enabled)
        return "ENABLED";
    else
        return "DISABLED";
}

String Motor::stateToString(MotorState state) {
    switch(state){
        case REV:
            return "REVERSE";
            break;
        case STOPPED:
            return "STOPPED";
            break;
        case FWD:
            return "FORWARD";
            break;
        }
}

// State Machine

void Motor::enable(){
    _enabled = true;
    sendMotorControl();
}

void Motor::disable(){
    _enabled = false;
    sendMotorControl();
}

void Motor::stop() {
    _state = STOPPED;
    _pwm = 0;
    sendMotorControl();
}

void Motor::setSpeed(uint8_t speed) {
    if (_state == STOPPED) 
        _state = FWD;
    _pwm = _speedToPWM(speed);
    sendMotorControl();
}

void Motor::fwd() { 
    _state = FWD; 
    sendMotorControl();
}

void Motor::rev() { 
    _state = REV; 
    sendMotorControl();
}

void Motor::drive(int8_t speed){
    _pwm = _speedToPWM(abs(speed));
    _state = (MotorState)sgn(speed);
    // if (speed == NO_ARG_FLAG){
    //     speed = _defaultSpeed;
    //     _state = FWD;
    // } else {
    //     _state = (MotorState) sgn(speed);
    // }

    // if (speed > 100 || speed < -100)
    //     throw std::invalid_argument("Motor.drive() only accepts values between -100 and 100.");

    _pwm = map(abs(speed), 0, 100, 0, _maxPWM);
    sendMotorControl();
}



//////////////////////////////////////////////////////////////////////



DualMotors::DualMotors(
        uint8_t fwdPin0, uint8_t revPin0, uint8_t pwmPin0,  uint8_t fwdPin1, uint8_t revPin1, uint8_t pwmPin1, 
        uint8_t maxPWM0, uint8_t maxPWM1, uint8_t defaultSpeed
    ) :
        _motor0(fwdPin0, revPin0, pwmPin0, maxPWM0, defaultSpeed),
        _motor1(fwdPin1, revPin1, pwmPin1, maxPWM1, defaultSpeed)
{ }

void DualMotors::init() {
    _motor0.init();
    _motor1.init();
}

void DualMotors::enable(MotorID motorID) {
    switch (motorID) {
        case 0:
            _motor0.enable();
            break;
        case 1:
            _motor1.enable();
            break;
    }
}

void DualMotors::disable(MotorID motorID) {
    switch (motorID) {
        case 0:
            _motor0.disable();
            break;
        case 1:
            _motor1.disable();
            break;
    }
}

void DualMotors::stop(MotorID motorID) {
    switch (motorID) {
        case 0:
            _motor0.stop();
            break;
        case 1:
            _motor1.stop();
            break;
    }
}

void DualMotors::setSpeed(MotorID motorID, uint8_t speed) {
    switch (motorID) {
        case 0:
            _motor0.setSpeed(speed);
            break;
        case 1:
            _motor1.setSpeed(speed);
            break;
    }
}

void DualMotors::drive(MotorID motorID, int8_t speed) {
    switch (motorID) {
        case 0:
            _motor0.drive(speed);
            break;
        case 1:
            _motor1.drive(speed);
            break;
    }
}

bool DualMotors::isEnabled(MotorID id) {
    switch (id) {
        case 0:
            return _motor0.isEnabled();
            break;
        case 1:
            return _motor1.isEnabled();
            break;
    }
}

int8_t DualMotors::getState(MotorID id) {
    switch (id) {
        case 0:
            return _motor0.getState();
            break;
        case 1:
            return _motor1.getState();
            break;
    }
}

String DualMotors::getStateString(MotorID id) {
    switch (id) {
        case 0:
            return _motor0.getStateString();
            break;
        case 1:
            return _motor1.getStateString();
            break;
    }
}

uint8_t DualMotors::getSpeed(MotorID id) {
    switch (id) {
        case 0:
            return _motor0.getSpeed();
            break;
        case 1:
            return _motor1.getSpeed();
            break;
    }
}

int8_t DualMotors::getVelocity(MotorID id) {
    switch (id) {
        case 0:
            return _motor0.getVelocity();
            break;
        case 1:
            return _motor1.getVelocity();
            break;
    }
}

void DualMotors::driveFwd(MotorID motorID, uint8_t speed) {
    drive(motorID, speed);
}

void DualMotors::driveRev(MotorID motorID, uint8_t speed) {
    drive(motorID, -speed);
}



//////////////////////////////////////////////////////////////////////


DifferentialDrive::DifferentialDrive(
    uint8_t leftFwdPin, uint8_t leftRevPin, uint8_t leftPwmPin, 
    uint8_t rightFwdPin, uint8_t rightRevPin, uint8_t rightPwmPin, 
    uint8_t leftMaxPWM, uint8_t rightMaxPWM, uint8_t defaultSpeed) : 
    _leftWheel(leftFwdPin, leftRevPin, leftPwmPin, leftMaxPWM, defaultSpeed),
    _rightWheel(rightFwdPin, rightRevPin, rightPwmPin, rightMaxPWM, defaultSpeed),
    _defaultSpeed(defaultSpeed)
{
    _enabled = true;
    _state = STOPPED;
    _speed = 0;
};

void DifferentialDrive::init(){
    _leftWheel.init();
    _rightWheel.init();
}

void DifferentialDrive::enable() {
    _enabled = true;
    _leftWheel.enable();
    _rightWheel.enable();
}

void DifferentialDrive::disable() {
    _enabled = false;
    _leftWheel.disable();
    _rightWheel.disable();
}

void DifferentialDrive::stop() {
    _state = STOPPED;
    _speed = 0;
    _leftWheel.stop();
    _rightWheel.stop();
}

void DifferentialDrive::setSpeed(uint8_t speed) {
    if (_state == STOPPED) 
        _state = FWD;
    _speed = _speedArgHandler(speed);
    _leftWheel.setSpeed(_speed);
    _rightWheel.setSpeed(_speed);
}


void DifferentialDrive::drive(int8_t speed) {
    _speed = _speedArgHandler(abs(speed));
    _state = (MotorState) sgn(speed);
    speed = _state * _speed;

    _leftWheel.drive(speed);
    _rightWheel.drive(speed);
}

void DifferentialDrive::fwd(uint8_t speed) {
    drive(speed);
}

void DifferentialDrive::rev(uint8_t speed) {
    drive(-speed);
}

void DifferentialDrive::turnLeft(uint8_t speed) {
    _speed = _speedArgHandler(speed);
    _state = TURN_LEFT;

    _leftWheel.drive(-_speed);
    _rightWheel.drive(_speed);
}

void DifferentialDrive::turnRight(uint8_t speed) {
    _speed = _speedArgHandler(speed);
    _state = TURN_RIGHT;

    _leftWheel.drive(_speed);
    _rightWheel.drive(-_speed);
}



int8_t DifferentialDrive::getVelocity()
{
    int8_t velocity;
    switch (_state) {
        case REV:
        case STOPPED:
        case FWD:
            velocity = _state * _speed;
            break;
        case TURN_LEFT: // CCW rotation is positive
            velocity = _speed;
            break;
        case TURN_RIGHT: // CW rotation is negative
            velocity = -_speed;
            break;
    }
    return velocity;
}

DifferentialDrive::MotorState DifferentialDrive::getState() {
    return _state;
}

String DifferentialDrive::getStateString() {
    return stateToString(_state);
}

 bool DifferentialDrive::isEnabled() {
    return _enabled;
}

String DifferentialDrive::stateToString(bool enabled) {
    if (enabled)
        return "ENABLED";
    else
        return "DISABLED";
}

String DifferentialDrive::stateToString(MotorState state) {
    switch(state){
        case REV:
            return "REVERSE";
            break;
        case STOPPED:
            return "STOPPED";
            break;
        case FWD:
            return "FORWARD";
            break;
        case TURN_LEFT:
            return "TURN_LEFT";
            break;
        case TURN_RIGHT:
            return "TURN_RIGHT";
            break;
        }
}

uint8_t DifferentialDrive::_speedArgHandler(uint8_t speed) {
    if (speed == NO_ARG_FLAG) {
        if (_speed == 0) 
            speed = _defaultSpeed;
        else {
            speed = _speed;
        }
    }
    
    // if (speed > 100)
        // throw std::invalid_argument("Speed must be in range [0, 100].");

    return speed;
}
