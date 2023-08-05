#include "Arduino.h"
#include "MCP23017-SOLDERED.h"
#include "basicHardware.h"


//////////////////////////////////////////////////////////
// PIN NATIVE
//////////////////////////////////////////////////////////
PinNative :: PinNative(uint8_t pinAddress) {
    _pinAddress = pinAddress;
}
void PinNative :: setPinMode(uint8_t mode) {
  pinMode(_pinAddress, mode);
}
void PinNative :: doDigitalWrite(uint8_t val) {
  digitalWrite(_pinAddress, val);
}
int PinNative :: doDigitalRead() {
  return digitalRead(_pinAddress);
}
//////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////
// PIN EXTENDER
//////////////////////////////////////////////////////////
PinExtender :: PinExtender(MCP_23017* mcp, uint8_t pinAddress) {
    _mcp = mcp;
    _pinAddress = pinAddress;
}

void PinExtender :: setPinMode(uint8_t mode) {
  _mcp->pinMode(_pinAddress, mode);
}
void PinExtender :: doDigitalWrite(uint8_t val) {
  _mcp->digitalWrite(_pinAddress, val);
}
int PinExtender :: doDigitalRead() {
  return _mcp->digitalRead(_pinAddress);
}
//////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////
// SWITCH
//////////////////////////////////////////////////////////
Switch :: Switch(String name, Pin* pin) {
  _name = name;
  _pin = pin;
  _pin->setPinMode(INPUT);
}

bool Switch :: isPressed() {
  return _pin->doDigitalRead() == LOW;
}

String Switch :: toString() {
  return "[" + _name + "] is pressed: " + String(isPressed());
}
//////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////
// STEPPER
//////////////////////////////////////////////////////////
Stepper :: Stepper(String name, Pin* enablePin,  Pin* stepPin, Pin* directionPin) {
    _name = name;
    _enablePin = enablePin;
    _stepPin = stepPin;
    _directionPin = directionPin;
    _currentPositionSteps = 0;

    _enablePin->setPinMode(OUTPUT);
    _directionPin->setPinMode(OUTPUT);
    _stepPin->setPinMode(OUTPUT);
    _enablePin->doDigitalWrite(LOW);
}

void Stepper :: turnForwardSteps(int steps) {
    _directionPin->doDigitalWrite(LOW);
    _turnSteps(steps);
    this->_currentPositionSteps += steps;
}

void Stepper :: turnBackwardSteps(int steps) {
    _directionPin->doDigitalWrite(HIGH);
    _turnSteps(steps);
    this->_currentPositionSteps -= steps;
}

void Stepper :: resetPosition() {
    this->_currentPositionSteps = 0;
}

String Stepper :: toString() {
    return "[STEPPER " + _name + "] position: " + String(_currentPositionSteps);
}

void Stepper :: _turnSteps(int steps) {
  for(int i = 0; i < steps; i++)
  {
    _stepPin->doDigitalWrite(HIGH);
    delayMicroseconds(500);
    _stepPin->doDigitalWrite(LOW);
    delayMicroseconds(500);
  }
}
//////////////////////////////////////////////////////////


