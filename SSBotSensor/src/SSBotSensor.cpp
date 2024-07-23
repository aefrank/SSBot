
#include <IRremote.hpp>
#include <SSBotSensor.hpp>

using namespace SummerSpringBot;

#define IR_SENSOR_PERIOD 20 // ms

//================  SONAR =================

Sonar::Sonar(uint8_t trigPin, uint8_t echoPin, unsigned int clearance, unsigned long Hz) : 
  _sensor(trigPin, echoPin), _trigPin(trigPin), _echoPin(echoPin), clearanceThreshold(clearance), _sensorPeriodMillis(1000.0 / Hz) {
  // _sensor = NewPing(trigPin, echoPin, 500);
  // _sensorPeriodMillis = 1000.0 / Hz;
  _lastReadTime = 0;
}

bool Sonar::clearAhead() {
  return (read() > clearanceThreshold);
}

void Sonar::init(){
  pinMode(_trigPin, OUTPUT);
  pinMode(_echoPin, INPUT);
  _sensor.ping();
}

int Sonar::read(){
    if (millis() - _lastReadTime > 50){
        _lastReadTime = millis();
        _lastDistance = _sensor.convert_cm(_sensor.ping_median()); // Send ping, get distance in cm (0 = outside set distance range)
    }
    return _lastDistance;
}


//================  IR REMOTE   =================


IRSensor::IRSensor(uint8_t IRpin) : _IRpin(IRpin){}

void IRSensor::init()
{
    IrReceiver.begin(_IRpin, ENABLE_LED_FEEDBACK);
}

bool IRSensor::commandReceived() {
  if ((millis()-_timeOfLastInterrupt) < IR_SENSOR_PERIOD) 
    return false;
  else {
    _timeOfLastInterrupt = millis();
    bool available = IrReceiver.decode();
    IrReceiver.resume(); // re-enable listening for next command
    return ( available && (!(IrReceiver.decodedIRData.flags & IRDATA_FLAGS_IS_REPEAT))
             && (IrReceiver.decodedIRData.command > 0) );
  }
}


IRSensor::IRCommand IRSensor::query(){
  if(commandReceived()){
    return _currentCommand();
  }
  else {
    return NONE;
  }
}

bool IRSensor::isValid(IRCommand command) {
  return (command > 0);
}

IRSensor::IRCommand IRSensor::_currentCommand(){
  switch (IrReceiver.decodedIRData.command) {
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

String IRSensor::_raw2str(uint32_t command){
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

String IRSensor::str(IRSensor::IRCommand command){
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
