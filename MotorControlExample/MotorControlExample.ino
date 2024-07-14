#include <SSBotMotor.hpp>

using namespace SummerSpringBot;

////////////////////////////////////////////////////
////////////// MOTOR CONFIGURATION /////////////////
////////////////////////////////////////////////////

// Motor Controller - Arduino Pin Connections

const uint8_t leftMotorPWMPin = 10;
const uint8_t leftMotorFwdPin = 11;
const uint8_t leftMotorRevPin = 12;

const uint8_t rightMotorPWMPin = 9;
const uint8_t rightMotorFwdPin = 7;
const uint8_t rightMotorRevPin = 8;


// Max speed for each wheel, 0-255

const uint8_t leftMotorMaxPWM  = 255;
const uint8_t rightMotorMaxPWM = 255;


// declare global motor controller object so both 
// setup() and loop() can access it

DifferentialDrive motors(
    leftMotorFwdPin, leftMotorRevPin, leftMotorPWMPin, 
    rightMotorFwdPin, rightMotorRevPin, rightMotorPWMPin, 
    leftMotorMaxPWM, rightMotorMaxPWM);


////////////////////////////////////////////////////
///////////////////// MAIN /////////////////////////
////////////////////////////////////////////////////

void setup() {
  motors.init();
}

void loop() {
  motors.drive(); // drive forward
  delay(3000); // wait 3 seconds

  motors.rev(); // drive backward
  delay(3000); // wait 3 seconds

  motors.turnLeft(); // rotate counter clockwise
  delay(3000); // wait 3 seconds

  motors.turnRight(); // rotate clockwise
  delay(3000); // wait 3 seconds

  motors.stop(); // stop
  delay(3000); // wait 3 seconds
}
