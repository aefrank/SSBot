#include <SSBotMotor.hpp>
#include <SSBotSensor.hpp>
#include <string.h>

using namespace SummerSpringBot;

#define BLUE 0
#define RED 1

#define TEAM RED

#if TEAM==BLUE
#define FWD_BUTTON    CMD_CH
#define REV_BUTTON    CMD_VOLUP
#define LEFT_BUTTON   CMD_PREV
#define RIGHT_BUTTON  CMD_PLAY
#define STOP_BUTTON   CMD_NEXT
#define ENABLE_BUTTON CMD_EQ
#endif
#if TEAM==RED
#define FWD_BUTTON    CMD_2
#define REV_BUTTON    CMD_8
#define LEFT_BUTTON   CMD_4
#define RIGHT_BUTTON  CMD_6
#define STOP_BUTTON   CMD_5
#define ENABLE_BUTTON CMD_9
#endif

///////////////////////////////////////////////////////////////////////
// *********************  HARDWARE INTERFACE  ********************* ///
///////////////////////////////////////////////////////////////////////

// /// --------------------- SERIAL COMMUNICATION  --------------------- ///

#include "SafeStringReader.h"
#include "BufferedOutput.h"

#define BAUD_RATE 115200
#define SSBOTSERIAL_RX_DELIM " ,\r\n"
#define SSBOTSERIAL_RX_BUFFER_LEN 8
#define SSBOTSERIAL_TX_BUFFER_LEN 63

createSafeStringReader(serialRx, SSBOTSERIAL_RX_BUFFER_LEN, SSBOTSERIAL_RX_DELIM);
createBufferedOutput(serialTx, SSBOTSERIAL_TX_BUFFER_LEN, DROP_UNTIL_EMPTY);

void serialInit(){
  Serial.begin(BAUD_RATE);
  delay(2000);
  serialRx.connect(Serial);
  serialTx.connect(Serial);
  serialTx.println(F("Serial communication ready."));
}


/// --------------------- MOTOR CONTROLLER  --------------------- ///

const uint8_t leftMotorPWMPin = 10;
const uint8_t leftMotorFwdPin = 11;
const uint8_t leftMotorRevPin = 12;

const uint8_t rightMotorPWMPin = 9;
const uint8_t rightMotorFwdPin = 7;
const uint8_t rightMotorRevPin = 8;

const uint8_t leftMotorMaxPWM  = 255;
const uint8_t rightMotorMaxPWM = 255;

DifferentialDrive motors(
    leftMotorFwdPin,  leftMotorRevPin,  leftMotorPWMPin, 
    rightMotorFwdPin, rightMotorRevPin, rightMotorPWMPin, 
    leftMotorMaxPWM,  rightMotorMaxPWM);


/// --------------------- IR SENSOR CONFIGURATION --------------------- ///
// ir sensor
const uint8_t irSensorPin = 2;
IRSensor remote(irSensorPin);


///////////////////////////////////////////////////////////////////////
// *************************    MAIN    *************************** ///
///////////////////////////////////////////////////////////////////////

/// --------------------- SETUP --------------------- ///

void setup() {
  serialInit();
  remote.init();
  motors.init();
}


/// --------------------- LOOP --------------------- ///

void loop() {
  serialTx.nextByteOut();
  // read IR Sensor
  IRCommand command = remote.query();
  // react if a command has been sent
  if (IRSensor::isValid(command)) {
    remoteControl(command);
  }
  // otherwise, continue execution
}


///////////////////////////////////////////////////////////////////////////////////
// *************************    HELPER FUNCTIONS    *************************** ///
///////////////////////////////////////////////////////////////////////////////////


void printRemoteCommand(IRCommand command, bool newline=false);
void printEnableStateChange(bool newline=false);
void printMotorStateChange(bool newline=false);

// respond to user controls sent from IR Remote buttons
void remoteControl(IRCommand command){
  printRemoteCommand(command, false);
  if (command == ENABLE_BUTTON)  {
    if (motors.isEnabled()) {
        motors.disable();
    } 
    else {
        motors.enable();
    } 
    printEnableStateChange();
  } 
  else {
    bool motorStateChange = false;
    switch (command) {
      case STOP_BUTTON:
        motors.stop();
        motorStateChange = true;
        break;
      case FWD_BUTTON:
        motors.fwd();
        motorStateChange = true;
        break;
      case REV_BUTTON:
        motors.rev();
        motorStateChange = true;
        break;
      case LEFT_BUTTON:
        motors.turnLeft();
        motorStateChange = true;
        break;
      case RIGHT_BUTTON:
        motors.turnRight();
        motorStateChange = true;
        break;
      // any of the other commands  
      default:
        ;
        // no-op
    } // end of switch(command)

    if (motorStateChange)
        printMotorStateChange();
    else 
      serialTx.print("(no-op) ");

  } // end of else 

  serialTx.println();

} // end of remote control


void printRemoteCommand(IRCommand command, bool newline=false){
  serialTx.print(F("[REMOTE] Button press: "));
  serialTx.print(padRight(IRSensor::str(command).c_str(), 4));
  serialTx.print(F(" |  "));
  if (newline) serialTx.println();
}


void printMotorStateChange(bool newline=false){
  serialTx.print(F("Motor state changed to "));
  serialTx.print(motors.getStateString().c_str());
  if (!motors.isEnabled())
    serialTx.print(" (currently DISABLED)");
  serialTx.print(F("."));
  if (newline) serialTx.println();
}

void printEnableStateChange(bool newline=false){
  if (motors.isEnabled()){
    serialTx.print(F("Play state changed to ENABLED. Resuming execution with motor state "));
    serialTx.print(motors.getStateString().c_str());
    serialTx.print(F(". "));
  } else {
    serialTx.print(F("Play state changed to DISABLED"));
    serialTx.print(F(" (motor state: "));
    serialTx.print(motors.getStateString().c_str());
    serialTx.print(F("). "));
  }
  if (newline) serialTx.println();
}



char* padRight(char* str, size_t fixedWidth) { 
  static char buffer[SSBOTSERIAL_TX_BUFFER_LEN]; 
  size_t L = strlen(str);
  char *b = buffer;
  char *s = str;

  // write chars from str into buffer from left to right
  while ( (b-buffer < L) && (b-buffer < fixedWidth) && (b-buffer < SSBOTSERIAL_TX_BUFFER_LEN))
    *b++ = *s++; // overwite value at b with value at s, then increment both b and s
  
  // then pad any remaining buffer with whitespace
  while ((b < buffer + fixedWidth) && (b < buffer + SSBOTSERIAL_TX_BUFFER_LEN) ) 
    *b++ = ' '; // overwrite at b, then increment

  // b points to end of buffer; add null-termination
  *b = '\0'; 
  
  return buffer;
}
