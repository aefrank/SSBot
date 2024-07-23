#include <SSBotSensor.hpp>
#include <SSBotMotor.hpp>

using namespace SummerSpringBot;


// ======================================================================================
// ===================== GLOBALS (accessible by entire program )=========================
// ======================================================================================

// -------- Hardware Interfaces --------

// serial communication
#include "SafeStringReader.h"
#include "BufferedOutput.h"

#define SERIALRX_DELIM " ,\r\n"
#define SERIALRX_MAX_CMD_LEN 8
#define SERIALRX_OUTPUT_BUFFER_LEN 64

createSafeStringReader(serialRx, SERIALRX_MAX_CMD_LEN, SERIALRX_DELIM);
createBufferedOutput(serialTx, SERIALRX_OUTPUT_BUFFER_LEN, DROP_UNTIL_EMPTY);

#define SERIAL_CONNECT_TIMEOUT_MS 2000 
#define SERIAL_BAUD_RATE 115200

// ir sensor
#define IR_SENSOR_PIN 2
IRSensor IR(IR_SENSOR_PIN);


// sonar
#define SONAR_TRIG_PIN 3
#define SONAR_ECHO_PIN 4
#define CLEARANCE_THRESHOLD 10 // cm
Sonar sonar(SONAR_TRIG_PIN, SONAR_ECHO_PIN);


// motor controller
#define LEFT_PWM_PIN 10
#define LEFT_FWD_PIN 11
#define LEFT_REV_PIN 12

#define RIGHT_PWM_PIN 9
#define RIGHT_FWD_PIN 7
#define RIGHT_REV_PIN 8

#define LEFT_MAX_PWM  200
#define RIGHT_MAX_PWM 255

DifferentialDrive motors(
    LEFT_FWD_PIN, LEFT_REV_PIN, LEFT_PWM_PIN, 
    RIGHT_FWD_PIN, RIGHT_REV_PIN, RIGHT_PWM_PIN, 
    LEFT_MAX_PWM, RIGHT_MAX_PWM);



// ======================================================================================
// ================= MAIN CONTROL LOOP (how the rest of the code gets run) ==============
// ======================================================================================

// -------- setup --------

void setup() {
  // set up serial communication
  serialSetup();

  // initialize hardware interface
  sonar.init();
  IR.init();
  motors.init();
}

// -------- loop --------

// put your main code here, to run repeatedly

void loop() {
  // User Control via IR remote
  IRSensor::IRCommand command = IR.query();
  if (IR.isValid(command)) {
    remoteControl(command);
  }
  // Obstacle avoidance via ultrasonic sensor
  if(!sonar.clearAhead()){
    obstacleAvoidance();
  }
}


// ======================================================================================
// ======== FUNCTIONS (reusable code snippets to use in the main control loop) ==========
// ======================================================================================

void serialSetup(){
  Serial.begin(SERIAL_BAUD_RATE);
  unsigned long serialConnectStart = millis();
  while (!Serial && (millis()-serialConnectStart < SERIAL_CONNECT_TIMEOUT_MS)); // wait for connection for a few seconds
  serialRx.connect(Serial);
  serialTx.connect(Serial);
}


// Check ultrasonic sensor for nearby obstacles and STOP if too close and moving forward
void obstacleAvoidance(){
  switch(motors.getState()){
    case DifferentialDrive::FWD:
        serialTx.println(F("[SONAR] Obstacle detected close ahead! Stopping motors..."));
        motors.stop();
  default: // no-op on MotorState REV, STOPPED, TURN_RIGHT, TURN_LEFT
    break;
  } 
} 



// Respond to user controls sent from IR Remote buttons
void remoteControl(IRSensor::IRCommand command){
    createSafeString(serialTxMsg, SERIAL_TX_BUFFER_SIZE);
  if (command == IRSensor::CMD_PLAY) {
      if (motors.isEnabled()) {
        motors.disable();
        serialTxMsg = F("[REMOTE] Play state changed to DISABLED.");
      } 
      else {
          motors.enable();
          serialTxMsg = F("[REMOTE] Play state changed to ENABLED. Resuming execution with motor state ");
          serialTxMsg += motors.getStateString().c_str();
          serialTxMsg += F(".\n");
      } 
  } // end of if(command==IRSensor::CMD_PLAY)
  else {
    switch (command) {
      // CH = STOPPED
      case IRSensor::CMD_CH:
        motors.stop();
        break;
      // CH+ = FWD
      case IRSensor::CMD_CHUP:
        motors.fwd();
        break;
      // CH- = REV
      case IRSensor::CMD_CHDOWN:
        motors.rev();
        break;
      // << = TURN_LEFT
      case IRSensor::CMD_PREV:
        motors.turnLeft();
        break;
      // << = TURN_RIGHT
      case IRSensor::CMD_NEXT:
        motors.turnRight();
        break;
      default:
        ; // no-op
    } // end of switch(command)

    serialTxMsg = F("[REMOTE] Motor state changed to: ");
    serialTxMsg += motors.getStateString().c_str();
    serialTxMsg += F(".\n"); 

  } // end of else 
    serialTx.print(serialTxMsg);

} // end of remoteControl



