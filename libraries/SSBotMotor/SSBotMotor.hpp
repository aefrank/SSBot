#include <Arduino.h>

// speeds can range from -100 to 100, but int8_t can hold values -128 to 127; so "101" will be used to indicate default argument
#define NO_ARG_FLAG 101


// Class for SINGLE motor control
class SSBotMotor {
    
    public:
        enum MotorState {REV=-1, STOPPED, FWD};
        SSBotMotor( uint8_t fwdPin, uint8_t revPin, uint8_t pwmPin,  
                    int maxPWM=255, int defaultSpeed=50);
        void init();        
        void enable();
        void disable();
        void fwd();
        void rev();
        void stop();
        void setSpeed(uint8_t speed=NO_ARG_FLAG);
        void drive(int8_t velocity=NO_ARG_FLAG);
        void sendMotorControl();
        const bool isEnabled();
        const int8_t getState();
        const String getStateString();
        const uint8_t getSpeed();
        const int8_t getVelocity();
        static const String stateToString(bool enabled);
        static const String stateToString(MotorState state);

    private:
        const uint8_t _pwmPin, _fwdPin, _revPin;
        const uint8_t _maxPWM, _defaultSpeed;
        bool _enabled;
        MotorState _state;
        uint8_t _pwm;
        void _setDir(int8_t dir);
        void _setPWM(uint8_t pwm);
        uint8_t _speedToPWM(uint8_t speed);

};

// Class for controlling two motors INDEPENDENTLY
class SSBotDoubleMotorController {

    public:
        typedef bool MotorID;
        SSBotDoubleMotorController(uint8_t fwdPin0, uint8_t revPin0, uint8_t pwmPin0, 
                                   uint8_t fwdPin1, uint8_t revPin1, uint8_t pwmPin1, 
                                   uint8_t maxPWM0 = 255, uint8_t maxPWM1 = 100, uint8_t defaultSpeed = 50);
        void init();
        // control motors individually -- motor=0 for left motor, motor=1 for right motor
        void enable(MotorID id);
        void disable(MotorID id);
        void stop(MotorID id);
        void setSpeed(MotorID id, uint8_t speed=NO_ARG_FLAG);
        void driveFwd(MotorID id, uint8_t speed=NO_ARG_FLAG);
        void driveRev(MotorID id, uint8_t speed=NO_ARG_FLAG);
        void drive(MotorID id, int8_t speed=NO_ARG_FLAG);

        const bool isEnabled(MotorID id);
        const int8_t getState(MotorID id);
        const String getStateString(MotorID id);
        const uint8_t getSpeed(MotorID id);
        const int8_t getVelocity(MotorID id);
    private:
        SSBotMotor _motor0, _motor1;
};


// Class for controlling two motors TOGETHER
class SSBotDifferentialDriveMC {
    public:
        enum MotorState {
            REV = -1,
            STOPPED,   // =  0,
            FWD,       // =  1,
            TURN_LEFT, // =  2,
            TURN_RIGHT // =  3,
        };
        enum MotorID {
            LEFT = 0,
            RIGHT = 1
        };

        /////// INITIALIZE ///////
        SSBotDifferentialDriveMC(uint8_t leftFwdPin, uint8_t leftRevPin, uint8_t leftPwmPin,
                                uint8_t rightFwdPin, uint8_t rightRevPin, uint8_t rightPwmPin,
                                uint8_t leftMaxPWM = 255, uint8_t rightMaxPWM = 255, uint8_t defaultSpeed = 50);
        void init();

        /////// CONTROL ///////
        // suspend movement
        void enable();
        void disable();
        void stop();
        // change movement speed without affecting direction
        void setSpeed(uint8_t speed = NO_ARG_FLAG);
        // control movement speed and direction simulaneously
        void drive(int8_t speed = NO_ARG_FLAG);
        void fwd(uint8_t speed = NO_ARG_FLAG);
        void rev(uint8_t speed = NO_ARG_FLAG);
        void turnLeft(uint8_t speed = NO_ARG_FLAG);
        void turnRight(uint8_t speed = NO_ARG_FLAG);

        /////// MONITOR  ///////
        // get current movement speed and direction as a signed integer
        const int8_t getVelocity();
        const MotorState getState();
        const String getStateString();
        const bool isEnabled();
        static const String stateToString(MotorState state);
        static const String stateToString(bool enabled);

    private:
        const uint8_t _defaultSpeed;
        SSBotMotor _leftWheel, _rightWheel;
        bool _enabled;
        MotorState _state;
        uint8_t _speed;
        uint8_t _speedArgHandler(uint8_t speedArg);
};