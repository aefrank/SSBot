#include <SSBotMotor.hpp>

using namespace SummerSpringBot;

////////////////////////////////////////////////////
////////////// MOTOR CONFIGURATION /////////////////
////////////////////////////////////////////////////

// Motor Controller - Arduino Pin Connections

const uint8_t pwmPin = 10;
const uint8_t fwdPin = 11;
const uint8_t revPin = 12;


// Max speed, 0-255
const uint8_t maxPWM  = 127;


// declare global motor controller object so both 
// setup() and loop() can access it

Motor motor(fwdPin, revPin, pwmPin, maxPWM);


////////////////////////////////////////////////////
///////////////////// MAIN /////////////////////////
////////////////////////////////////////////////////

/**           ----- SETUP -----
* This code runs ONCE at start of program.
*/
void setup() {
  // initialize motor controller and hardware interface
  motor.init();
}

/**            ----- LOOP -----
* This code repeats forever, starting as soon as setup() finishes.
*/
void loop() {
  motor.fwd(); // drive forward
  delay(3000); // wait 3 seconds

  motor.rev(); // drive backward
  delay(3000); // wait 3 seconds

  motor.stop(); // stop
  delay(1000); // wait 1 second

  // accelerate forward from 1% to 100% speed
  for (int speed=1; speed <= 100; speed++) {
    motor.drive(speed);
    delay(10); // spend 10ms at each speed
  }

  // decelerate
  for (int speed=100; speed >= 0; speed--) {
    motor.drive(speed);
    delay(10); // spend 10ms at each speed
  }

  // accelerate backward from 1% to 100% speed
  for (int speed=-1; speed >= -100; speed--) {
    motor.drive(speed);
    delay(10); // spend 10ms at each speed
  }

  // decelerate
  for (int speed=-100; speed <= 0; speed++) {
    motor.drive(speed);
    delay(10); // spend 10ms at each speed
  }

  motor.stop(); // stop
  delay(3000); // wait 3 seconds
}
