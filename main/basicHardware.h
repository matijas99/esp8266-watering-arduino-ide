#ifndef basicHardware_h
#define basicHardware_h

#include "Arduino.h"
#include "AccelStepper.h"
#include "MCP23017-SOLDERED.h"

//////////////////////////////////////////////////////////
// PIN
//////////////////////////////////////////////////////////
class Pin {
  protected:
    uint8_t _pinAddress;

  public:
    virtual void setPinMode(uint8_t mode);
    virtual void doDigitalWrite(uint8_t val);
    virtual int doDigitalRead();
};
//////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////
// PIN NATIVE
//////////////////////////////////////////////////////////
class PinNative : public Pin {
  public:
    PinNative(uint8_t pinAddress);

    void setPinMode(uint8_t mode);
    void doDigitalWrite(uint8_t val);
    int doDigitalRead();
};
//////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////
// PIN EXTENDER
//////////////////////////////////////////////////////////
class PinExtender : public Pin {
  public:
    PinExtender(MCP_23017* mcp, uint8_t pinAddress);
    
    void setPinMode(uint8_t mode);
    void doDigitalWrite(uint8_t val);
    int doDigitalRead();
  
  private:
    MCP_23017* _mcp;
};
//////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////
// SWITCH
//////////////////////////////////////////////////////////
class Switch {
  public:
    Switch(Pin* pin);

    bool isPressed(); 

  private:
    Pin* _pin;
};
//////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////
// GENERIC ACCEL STEPPER
//////////////////////////////////////////////////////////
class GenericAccelStepper : public AccelStepper {
  public:
    GenericAccelStepper(uint8_t interface = AccelStepper::FULL4WIRE, Pin* pin1 = new PinNative(2), Pin* pin2 = new PinNative(3), Pin* pin3 = new PinNative(4), Pin* pin4 = new PinNative(5));
    void setOutputPins(uint8_t mask);
    void disableOutputs();
    void enableOutputs();
    void setEnablePin(uint8_t enablePin);
  
  private:
    Pin* _genericPins[4];
    // AccelStepper _accelStepper;
    void _digitalWrite(uint8_t pin, uint8_t value);
    void _pinMode(uint8_t pin, uint8_t value);
};
//////////////////////////////////////////////////////////


// //////////////////////////////////////////////////////////
// // STEPPER
// //////////////////////////////////////////////////////////
// class Stepper {
//   public:
//     static const int stepperFullRotationSteps = 200;

//     Stepper(Pin* enablePin,  Pin* stepPin, Pin* directionPin);
//     void move(long position);

//   private:
//     Pin* _pins[3];
//     GenericAccelStepper* _accelStepper;
// };
// //////////////////////////////////////////////////////////


// //////////////////////////////////////////////////////////
// // STEPPER
// //////////////////////////////////////////////////////////
// class OldStepper : public Component {
//   public:
//     static const int stepperFullRotationSteps = 200;
//     static const int stepDelayMicroSeconds = 1000;

//     Stepper(String name, Pin* enablePin,  Pin* stepPin, Pin* directionPin);

//     void turnForwardSteps(int steps);
//     void turnBackwardSteps(int steps);
//     void resetPosition();
//     String toString();

//   private:
//     Pin* _enablePin;
//     Pin* _directionPin;
//     Pin* _stepPin;
//     int _currentPositionSteps;
//     AccelStepper* _accelStepper;    

//     void _turnSteps(int steps);
// };
// //////////////////////////////////////////////////////////


#endif