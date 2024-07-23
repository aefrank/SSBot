#include <SSBotMotor.hpp>
#include <SSBotSensor.hpp>

using namespace SummerSpringBot;



///////////////////////////////////////////////////////////////////////
// *********************  HARDWARE INTERFACE  ********************* ///
///////////////////////////////////////////////////////////////////////

// motor controller

const uint8_t leftMotorPWMPin = 10;
const uint8_t leftMotorFwdPin = 11;
const uint8_t leftMotorRevPin = 12;

const uint8_t rightMotorPWMPin = 9;
const uint8_t rightMotorFwdPin = 7;
const uint8_t rightMotorRevPin = 8;

const uint8_t leftMotorMaxPWM  = 255;
const uint8_t rightMotorMaxPWM = 255;

DifferentialDrive motors(
    leftMotorFwdPin, leftMotorRevPin, leftMotorPWMPin, 
    rightMotorFwdPin, rightMotorRevPin, rightMotorPWMPin, 
    leftMotorMaxPWM, rightMotorMaxPWM);


/// --------------------- IR SENSOR CONFIGURATION --------------------- ///
// ir sensor
const uint8_t irSensorPin = 2;
IRSensor IR(irSensorPin);


/// --------------------- SERIAL COMMUNICATION  --------------------- ///

#include "SafeStringReader.h"
#include "BufferedOutput.h"

#define SERIALRX_DELIM " ,\r\n"
#define SERIALRX_MAX_CMD_LEN 8
#define SERIALRX_OUTPUT_BUFFER_LEN 64

createSafeStringReader(serialRx, SERIALRX_MAX_CMD_LEN, SERIALRX_DELIM);
createBufferedOutput(serialTx, SERIALRX_OUTPUT_BUFFER_LEN, DROP_UNTIL_EMPTY);



/// --------------------- MAIN --------------------- ///

void setup() {
  motors.init();
}

void loop() {
    // read IR Sensor
    IRSensor::IRCommand command = IR.query();
    // react if a command has been sent
    if (IR.isValid(command)) {
        remoteControl(command);
    }
    // otherwise, continue execution
}

// respond to user controls sent from IR Remote buttons
void remoteControl(IRSensor::IRCommand command){
    createSafeString(serialTxMsg, SERIAL_TX_BUFFER_SIZE);
    
    if (command == IRSensor::CMD_PLAY)
    {
        if (motors.isEnabled())
        {
            motors.disable();
            serialTxMsg = F("[REMOTE] Play state changed to DISABLED.");
        } // end of if(motors.isEnabled)
        else
        {
            motors.enable();
            serialTxMsg = F("[REMOTE] Play state changed to ENABLED. Resuming execution with motor state ");
            serialTxMsg += motors.getStateString().c_str();
            serialTxMsg += F(".\n");
        } // end of else

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
        ;
        // no-op
    } // end of switch(command)

    serialTxMsg = F("[REMOTE] Motor state changed to: ");
    serialTxMsg += motors.getStateString().c_str();
    serialTxMsg += F(".\n"); 

  } // end of else 
  serialTx.print(serialTxMsg);

} // end of remote control

