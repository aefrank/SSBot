
#include <string.h>
#include "Arduino.h"
#include <NewPing.h>

#define DECODE_NEC      
#define USE_IRREMOTE_HPP_AS_PLAIN_INCLUDE
#include <IRremote.hpp>

namespace SummerSpringBot {


// ------------------ SONAR ------------------

class Sonar{
    NewPing _sensor;
    const uint8_t _trigPin, _echoPin;
    const unsigned long _sensorPeriodMillis;
    unsigned long _lastReadTime;
    unsigned int _lastDistance;
  public:
    const unsigned int clearanceThreshold;
    Sonar(uint8_t trigPin, uint8_t echoPin, unsigned int clearanceThreshold=10, unsigned long Hz=20);
    bool clearAhead();
    void init();
    int read();
};



// ------------------ IR Receiver ------------------

class IRSensor {
    const uint8_t _IRpin;
    unsigned long _timeOfLastInterrupt;
  public:
    enum IRCommand {
      ERROR = -1, 
      NONE,
      CMD_CHDOWN,   CMD_CH,     CMD_CHUP, 
      CMD_PREV,     CMD_NEXT,   CMD_PLAY, 
      CMD_VOLDOWN,  CMD_VOLUP,  CMD_EQ, 
      CMD_0,        CMD_100,    CMD_200, 
      CMD_1,        CMD_2,      CMD_3, 
      CMD_4,        CMD_5,      CMD_6, 
      CMD_7,        CMD_8,      CMD_9,
    };
    IRSensor(uint8_t IRpin);
    void init();
    bool commandReceived();
    IRCommand query();
    static bool isValid(IRCommand command);
    static String str(IRCommand command);
  private:
    IRCommand _currentCommand();
    static String _raw2str(uint32_t command);

};



} // end of namespace SummerSpringBot