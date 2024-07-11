/*

  SSBot.cpp - Library for controlling a simple differential drive robot.
  
  Created by Andrea E. Frank for Summer Springboard 2024 Fundamentals of Engineering course.


*/

// ----------LIBRARIES--------------

#include <IRremote.hpp>

#include "SSBot.h"

// #include <NewPing.h>


// --------MACROS (won't change)---------------

#define sgn(x) ((x) < 0 ? -1 : ((x) > 0 ? 1 : 0))



//================  SONAR =================

SSBotSonar::SSBotSonar(uint8_t trigPin, uint8_t echoPin, unsigned int clearance, unsigned long Hz) : 
  _sensor(trigPin, echoPin), _trigPin(trigPin), _echoPin(echoPin), clearanceThreshold(clearance), _sensorPeriodMillis(1000.0 / Hz) {
  // _sensor = NewPing(trigPin, echoPin, 500);
  // _sensorPeriodMillis = 1000.0 / Hz;
  _lastReadTime = 0;
}

bool SSBotSonar::clearAhead() {
  return (read() > clearanceThreshold);
}

void SSBotSonar::init(){
  pinMode(_trigPin, OUTPUT);
  pinMode(_echoPin, INPUT);
  _sensor.ping();
}

int SSBotSonar::read(){
    if (millis() - _lastReadTime > 50){
        _lastReadTime = millis();
        _lastDistance = _sensor.convert_cm(_sensor.ping_median()); // Send ping, get distance in cm (0 = outside set distance range)
    }
    return _lastDistance;
}

//================  MOTOR CONTROL =================



SSBotMC::SSBotMC(uint8_t leftEnablePin, uint8_t leftFwdPin, uint8_t leftRevPin, 
                 uint8_t rightEnablePin, uint8_t rightFwdPin, uint8_t rightRevPin,
                 int leftMaxSpeed, int rightMaxSpeed, int defaultSpeed) :
  _leftEnable(leftEnablePin), _leftFwd(leftFwdPin), _leftRev(leftRevPin), 
  _rightEnable(rightEnablePin), _rightFwd(rightFwdPin), _rightRev(rightRevPin), 
  _leftMaxSpeed(map(leftMaxSpeed,  0, 100, 0, 255)), _rightMaxSpeed(map(rightMaxSpeed,  0, 100, 0, 255)), _defaultSpeed(defaultSpeed) {}

void SSBotMC::init(){
  pinMode(_leftEnable, OUTPUT);
  pinMode(_leftFwd, OUTPUT);
  pinMode(_leftRev, OUTPUT);
  pinMode(_rightEnable, OUTPUT);
  pinMode(_rightFwd, OUTPUT);
  pinMode(_rightRev, OUTPUT);
  stopMotors();
}

void SSBotMC::driveFwd(int speed){
  drive(speed);
}

void SSBotMC::driveRev(int speed){
  if (speed == DEFAULT_SPEED_FLAG){
    speed = _defaultSpeed;
  }
  drive(-speed);
}

// ------------------ Control motors individually ------------------

void SSBotMC::setLeftMotorFwd(){
  digitalWrite(_leftFwd,  1);
  digitalWrite(_leftRev,  0);
}

void SSBotMC::setLeftMotorRev(){
  digitalWrite(_leftFwd,  0);
  digitalWrite(_leftRev,  1);
}

void SSBotMC::setRightMotorFwd(){
  digitalWrite(_rightFwd,  1);
  digitalWrite(_rightRev,  0);
}

void SSBotMC::setRightMotorRev(){
  digitalWrite(_rightFwd,  0);
  digitalWrite(_rightRev,  1);
}

void SSBotMC::setLeftMotorPWM(int duty){
  int pwm = map(abs(duty), 0, 100, 0, _leftMaxSpeed);
  analogWrite(_leftEnable, pwm);
}

void SSBotMC::setRightMotorPWM(int duty){
  int pwm = map(abs(duty), 0, 100, 0, _rightMaxSpeed);
  analogWrite(_rightEnable, pwm);
}

void SSBotMC::stopLeftMotor(){
  analogWrite(_leftEnable, 0);
  digitalWrite(_leftFwd, 0);
  digitalWrite(_leftRev, 0);
}

void SSBotMC::stopRightMotor(){
  analogWrite(_rightEnable, 0);
  digitalWrite(_rightFwd, 0);
  digitalWrite(_rightRev, 0);
}

void SSBotMC::driveLeftMotorFwd(int speed)
{
  driveLeftMotor(speed);
}

void SSBotMC::driveLeftMotorRev(int speed)
{
  if (speed == DEFAULT_SPEED_FLAG){
    speed = _defaultSpeed;
  }
  driveLeftMotor(-speed);
}

void SSBotMC::driveRightMotorFwd(int speed)
{
  driveRightMotor(speed);
}

void SSBotMC::driveRightMotorRev(int speed)
{
  if (speed == DEFAULT_SPEED_FLAG){
    speed = _defaultSpeed;
  }
  driveRightMotor(-speed);
}

void SSBotMC::driveLeftMotor(int speed){
  if (speed == DEFAULT_SPEED_FLAG){
    speed = _defaultSpeed;
  }
  if (speed == 0) {
    stopLeftMotor();
    return;
  }  
  if (speed > 0) {
    setLeftMotorFwd();
  }
  else {
    setLeftMotorRev();
  }
  setLeftMotorPWM(speed);
}

void SSBotMC::driveRightMotor(int speed){
  if (speed == DEFAULT_SPEED_FLAG){
    speed = _defaultSpeed;
  }
  if (speed == 0)
  {
    stopRightMotor();
    return;
  }
  if (speed > 0) {
    setRightMotorFwd();
  }
  else {
    setRightMotorRev();
  }
  setRightMotorPWM(speed);
}



// ------------------ Control tandem motor direction and PWM separately ------------------

void SSBotMC::setDirStop(){
  digitalWrite(_leftFwd,  0);
  digitalWrite(_leftRev,  0);
  digitalWrite(_rightFwd, 0);
  digitalWrite(_rightRev, 0);
}

void SSBotMC::setDirFwd(){
  digitalWrite(_leftFwd,  1);
  digitalWrite(_leftRev,  0);
  digitalWrite(_rightFwd, 1);
  digitalWrite(_rightRev, 0);
}

void SSBotMC::setDirRev(){
  digitalWrite(_leftFwd,  0);
  digitalWrite(_leftRev,  1);
  digitalWrite(_rightFwd, 0);
  digitalWrite(_rightRev, 1);
}

void SSBotMC::setDirLeftTurn(){
  digitalWrite(_leftFwd,  0);
  digitalWrite(_leftRev,  1);
  digitalWrite(_rightFwd, 1);
  digitalWrite(_rightRev, 0);
}

void SSBotMC::setDirRightTurn(){
  digitalWrite(_leftFwd,  1);
  digitalWrite(_leftRev,  0);
  digitalWrite(_rightFwd, 0);
  digitalWrite(_rightRev, 1);
}


void SSBotMC::setPWM(int duty){
  int leftPWM = map(abs(duty), 0, 100, 0, _leftMaxSpeed);
  int rightPWM = map(abs(duty), 0, 100, 0, _rightMaxSpeed);
  analogWrite(_leftEnable, leftPWM);
  analogWrite(_rightEnable, rightPWM);
}



  // ------------------ Tandem motor actions ------------------


void SSBotMC::drive(int speed){
  if (speed == DEFAULT_SPEED_FLAG){
    speed = _defaultSpeed;
  }
  if (speed == 0) {
    stopMotors();
    return;
  }  
  if (speed > 0) {
    setDirFwd();
  }
  else {
    setDirRev();
  }
  setPWM(speed);
}

void SSBotMC::stopMotors(){
  analogWrite(_leftEnable, 0);
  analogWrite(_rightEnable, 0);
  digitalWrite(_leftFwd, 0);
  digitalWrite(_leftRev, 0);
  digitalWrite(_rightFwd, 0);
  digitalWrite(_rightRev, 0);
}

void SSBotMC::turnLeft(int speed){
  if (speed == DEFAULT_SPEED_FLAG){
    speed = _defaultSpeed;
  }
  if (speed == 0) {
    stopLeftMotor();
    return;
  }  
  if (speed > 0) {
    setDirLeftTurn();
  }
  else {
    setDirRightTurn();
  }
  setPWM(speed);

}

void SSBotMC::turnRight(int speed){
  if (speed == DEFAULT_SPEED_FLAG){
    speed = _defaultSpeed;
  }
  if (speed == 0) {
    stopLeftMotor();
    return;
  }  
  if (speed > 0) {
    setDirRightTurn();
  }
  else {
    setDirLeftTurn();
  }
  setPWM(speed);
}




//================  IR REMOTE   =================


SSBotIR::SSBotIR(uint8_t IRpin) : _IRpin(IRpin){}
// SSBotIR::SSBotIR(uint8_t IRpin, IRrecv *rec) : _IRpin(IRpin), _receiver(rec) {}

void SSBotIR::init()
{
    IrReceiver.begin(_IRpin, ENABLE_LED_FEEDBACK);
    // _receiver->begin(_IRpin, ENABLE_LED_FEEDBACK);
}

bool SSBotIR::iRCommandReceived() {
  if ((millis()-_timeOfLastInterrupt) < 50) 
    return false;
  else {
    _timeOfLastInterrupt = millis();
    bool available = IrReceiver.decode();
    IrReceiver.resume(); // re-enable listening for next command
    return ( available && (!(IrReceiver.decodedIRData.flags & IRDATA_FLAGS_IS_REPEAT))
             && (IrReceiver.decodedIRData.command > 0) );
    // bool available = _receiver->decode();
    // _receiver->resume(); // re-enable listening for next command
    // return ( available && (!(_receiver->decodedIRData.flags & IRDATA_FLAGS_IS_REPEAT))
    //          && (_receiver->decodedIRData.command > 0) );
  }
}


SSBotIR::IRCommand SSBotIR::query(){
  if(iRCommandReceived()){
    return _currentCommand();
  }
  else {
    return NONE;
  }
}

bool SSBotIR::isValid(IRCommand command) {
  return (!(command == NONE || command == ERROR));
}

SSBotIR::IRCommand SSBotIR::_currentCommand(){
  switch (IrReceiver.decodedIRData.command) {
  // switch (_receiver->decodedIRData.command) {
    case 69:
      return CMD_CHDOWN;
    case 70: 
      return CMD_CH;
    case 71:
      return CMD_CHUP;
    case 68:
      return CMD_PREV;
    case 64:
      return CMD_NEXT;
    case 67:
      return CMD_PLAY;
    case 7:
      return CMD_VOLDOWN;
    case 21:
      return CMD_VOLUP;
    case 9:
      return CMD_EQ;
    case 22:
      return CMD_0;
    case 25:
      return CMD_100;
    case 13:
      return CMD_200;
    case 12:
      return CMD_1;
    case 24:
      return CMD_2;
    case 94:
      return CMD_3;
    case 8:
      return CMD_4;
    case 28:
      return CMD_5;
    case 90:
      return CMD_6;
    case 66:
      return CMD_7;
    case 82:
      return CMD_8;
    case 74:
      return CMD_9;
    case 0:
      return NONE;
    default:
      return ERROR;
  }
}

String SSBotIR::_raw2str(uint32_t command){
  switch (command) {
    case 69:
      return "CH-";
    case 70: 
      return "CH";
    case 71:
      return "CH+";
    case 68:
      return "<<";
    case 64:
      return ">>";
    case 67:
      return "PLAY";
    case 7:
      return "-";
    case 21:
      return "+";
    case 9:
      return "EQ";
    case 22:
      return "0";
    case 25:
      return "100+";
    case 13:
      return "200+";
    case 12:
      return "1";
    case 24:
      return "2";
    case 94:
      return "3";
    case 8:
      return "4";
    case 28:
      return "5";
    case 90:
      return "6";
    case 66:
      return "7";
    case 82:
      return "8";
    case 74:
      return "9";
  }
}

String SSBotIR::str(SSBotIR::IRCommand command){
  switch (command){
    case CMD_CHDOWN:
      return "CH-";
    case CMD_CH:
      return "CH";
    case CMD_CHUP:
      return "CH+";
    case CMD_PREV:
      return "<<";
    case CMD_NEXT:
      return ">>";
    case CMD_PLAY:
      return "PLAY";
    case CMD_VOLDOWN:
      return "-";
    case CMD_VOLUP:
      return "+";
    case CMD_EQ:
      return "EQ";
    case CMD_0:
      return "0";
    case CMD_100:
      return "100+";
    case CMD_200:
      return "200+";
    case CMD_1:
      return "1";
    case CMD_2:
      return "2";
    case CMD_3:
      return "3";
    case CMD_4:
      return "4";
    case CMD_5:
      return "5";
    case CMD_6:
      return "6";
    case CMD_7:
      return "7";
    case CMD_8:
      return "8";
    case CMD_9:
      return "9";
  }
}
