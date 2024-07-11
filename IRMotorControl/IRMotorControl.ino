#include <SSBot.h>
#include <SSBotMotor.hpp>

#include "SafeStringReader.h"
#include "BufferedOutput.h"

#define DELIM " ,\r\n"
#define MAX_CMD_LEN 8
#define OUTPUT_BUFFER_LEN 64

createSafeStringReader(serialRx, MAX_CMD_LEN, DELIM);
createBufferedOutput(serialTx, OUTPUT_BUFFER_LEN, DROP_UNTIL_EMPTY); 

// ======================================================================================
// ===================== GLOBALS (accessible by entire program )=========================
// ======================================================================================

// ================ CONSTANTS (values cannot change during execution )===================

// -------- State Control --------

// state space
enum PlayState {PLAY, PAUSE};
enum MotorState {
  STOPPED,
  FWD, 
  REV, 
  TURN_LEFT, 
  TURN_RIGHT, 
  LEFT_FWD, 
  RIGHT_FWD, 
  LEFT_REV, 
  RIGHT_REV
};


// -------- Hardware Interfaces --------

// ir sensor
const uint8_t IRsensorPin = 2;
SSBotIR IR(IRsensorPin);

// sonar

const uint8_t sonarTrigPin = 3;
const uint8_t sonarEchoPin = 4;
SSBotSonar sonar(sonarTrigPin, sonarEchoPin);
const unsigned int clearanceThreshold = 10; // cm

// motor controller

const uint8_t leftMotorPWMPin = 10;
const uint8_t leftMotorFwdPin    = 11;
const uint8_t leftMotorRevPin    = 12;

const uint8_t rightMotorPWMPin = 9;
const uint8_t rightMotorFwdPin    = 7;
const uint8_t rightMotorRevPin    = 8;

const uint8_t leftMotorMaxPWM  = 200;
const uint8_t rightMotorMaxPWM = 255;

SSBotDifferentialDriveMC motors(
    leftMotorFwdPin, leftMotorRevPin, leftMotorPWMPin, 
    rightMotorFwdPin, rightMotorRevPin, rightMotorPWMPin, 
    leftMotorMaxPWM, rightMotorMaxPWM);

// =============== VARIABLES (values CAN change during execution )===========

PlayState playState;
MotorState motorState;

// =========== FORWARD FUNCTION DECLARATIONS ===============
// (enables other code to reference the function earlier in 
// the file than it's implemented)

void remoteControl(); // referenced in loop()


// ======================================================================================
// ================= MAIN CONTROL LOOP (how the rest of the code gets run) ==============
// ======================================================================================

// -------- setup --------

// put your setup code here, to run once

void setup() {
  // set up serial communication
  Serial.begin(115200);
  while (!Serial); // wait for connection
  // SafeString::setOutput(Serial); // enable error messages and SafeString.debug() serialTx to be sent to Serial
  serialRx.connect(Serial);
  //   serialRx.echoOn(); // echo back all input; off by default
  serialTx.connect(Serial);

  // initialize hardware interface
  sonar.init();
  IR.init();
  motors.init();

  // set initial state
  playState = PLAY;
  motorState = STOPPED;
}



// -------- loop --------

// put your main code here, to run repeatedly

void loop() {
  // User Control via IR remote
  SSBotIR::IRCommand command = IR.query();
  if (IR.isValid(command)) {
    remoteControl(command);
  }
  
  // State machine logic
  switch(motorState) {
    case FWD:
    case LEFT_FWD:
    case RIGHT_FWD:
      // Check ultrasonic sensor for nearby obstacles
      int clearance = sonar.read(); 
      if (!sonar.clearAhead())
        serialTx.println("[SONAR] Obstacle detected close ahead! Stopping motors...");
        motors.stop();
        motorState = STOPPED;
      break;
    case REV:
      break;
    case LEFT_REV:
      break;
    case RIGHT_REV:
      break;
    case TURN_LEFT:
      break;
    case TURN_RIGHT:
      break;
    case STOPPED:
      break;
  }

}


// ======================================================================================
// ======== FUNCTIONS (reusable code snippets to use in the main control loop) ==========
// ======================================================================================


// ----------- core functionality -----------

// respond to user controls sent from IR Remote buttons
void remoteControl(SSBotIR::IRCommand command){
  if (command==SSBotIR::CMD_PLAY){
    serialTx.print(F("[REMOTE] playState changed to "));
    if (playState==PLAY){
      motors.disable();
      playState = PAUSE;
      serialTx.println("PAUSE.");
    }
    else {
      motors.enable();
      playState = PLAY;
      serialTx.print("PLAY. ");
    }
  } 
  else {
    switch (command) {
      // CH = STOPPED
      case SSBotIR::CMD_CH:
        serialTx.print(F("[REMOTE] motorState changed to: "));
        motors.stop();
        serialTx.println("STOPPED");
        break;
      // CH+ = FWD
      case SSBotIR::CMD_CHUP:
        serialTx.print(F("[REMOTE] motorState changed to: "));
        motors.fwd();
        serialTx.println("FWD");
        break;
      // CH- = REV
      case SSBotIR::CMD_CHDOWN:
        serialTx.print(F("[REMOTE] motorState changed to: "));
        motors.rev();
        serialTx.println("REV");
        break;
      // << = TURN_LEFT
      case SSBotIR::CMD_PREV:
        serialTx.print(F("[REMOTE] motorState changed to: "));
        motors.turnLeft();
        serialTx.println("TURN_LEFT");
        break;
      // << = TURN_RIGHT
      case SSBotIR::CMD_NEXT:
        serialTx.print(F("[REMOTE] motorState changed to: "));
        motors.turnRight();
        serialTx.println("TURN_RIGHT");
        break;
      // // 1 = LEFT_FWD
      // case SSBotIR::CMD_1:
      //   serialTx.print(F("[REMOTE] motorState changed to: "));
      //   enterMotorState(LEFT_FWD);
      //   serialTx.println("LEFT_FWD");
      //   break;
      // // 4 = LEFT_REV
      // case SSBotIR::CMD_4:
      //   serialTx.print(F("[REMOTE] motorState changed to: "));
      //   enterMotorState(LEFT_REV);
      //   serialTx.println("LEFT_REV");
      //   break;
      // // 3 = RIGHT_FWD
      // case SSBotIR::CMD_3:
      //   serialTx.print(F("[REMOTE] motorState changed to: "));
      //   enterMotorState(RIGHT_FWD);
      //   serialTx.println("RIGHT_FWD");
      //   break;
      // // 6 = RIGHT_REV  -- DISABLED
      // case SSBotIR::CMD_6:
      //   serialTx.print(F("[REMOTE] motorState changed to: "));
      //   enterMotorState(RIGHT_REV);
      //   serialTx.println("RIGHT_REV");
      //   break;
      // // currently unused commands
      // // - = NO OP
      // case SSBotIR::CMD_VOLDOWN:
      //   break;
      // // + = NO OP
      // case SSBotIR::CMD_VOLUP:
      //   break;
      // // EQ = NO OP
      // case SSBotIR::CMD_EQ:
      //   break;
      // // 0 = NO OP
      // case SSBotIR::CMD_0:
      //   break;
      // // 100+ = NO OP
      // case SSBotIR::CMD_100:
      //   break;
      // // 200+ = NO OP
      // case SSBotIR::CMD_200:
      //   break;
      // // 2 = NO OP
      // case SSBotIR::CMD_2:
      //   break;
      // // 5 = NO OP
      // case SSBotIR::CMD_5:
      //   break;
      // // 7 = NO OP
      // case SSBotIR::CMD_7:
      //   break;
      // // 8 = NO OP
      // case SSBotIR::CMD_8:
      //   break;
      // // 9 = NO OP
      // case SSBotIR::CMD_9:
      //   break;
      // // no command = NO OP
      // case SSBotIR::NONE: 
      //   break;
      // // invalid command = NO OP
      // case SSBotIR::ERROR: 
      //   break;
    }
  }
}
