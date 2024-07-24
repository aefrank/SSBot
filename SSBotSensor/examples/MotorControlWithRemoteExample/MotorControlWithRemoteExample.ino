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
  // set up serialRx communication
  Serial.begin(115200);
  while (!Serial); // wait for connection
  // SafeString::setOutput(Serial); // enable error messages and SafeString.debug() serialTx to be sent to Serial
  serialRx.connect(Serial);
  //   serialRx.echoOn(); // echo back all input; off by default
  serialTx.connect(Serial);

  IR.init();
  motors.init();
}

void loop() {
    // read IR Sensor
    IRSensor::IRCommand command = IR.query();
    // react if a command has been sent
    if (IR.isValid(command)) {
        remoteControl(command);
    }
    // continue looping
}

// respond to user controls sent from IR Remote buttons
void remoteControl(IRSensor::IRCommand command){
    if (command == IRSensor::CMD_PLAY)
    {
        if (motors.isEnabled())
        {
            motors.disable();
            serialTx.print("[REMOTE] Play state changed to DISABLED.");
        } // end of if(motors.isEnabled)
        else
        {
            motors.enable();
            serialTx.print(F("[REMOTE] Play state changed to ENABLED. Resuming execution with motor state "));
            serialTx.print(motors.getStateString().c_str());
            serialTx.print(".");
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

    serialTx.print(F("[REMOTE] Motor state changed to: "));
    serialTx.print(motors.getStateString().c_str());
    serialTx.print(F(".")); 

  } // end of else 
  serialTx.println();

} // end of remote control

