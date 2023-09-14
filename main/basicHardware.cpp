#include "Arduino.h"
#include "MCP23017-SOLDERED.h"
#include "AccelStepper.h"
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
Switch :: Switch(Pin* pin) {

  _pin = pin;
  _pin->setPinMode(INPUT_PULLUP);
}

bool Switch :: isPressed() {
  return _pin->doDigitalRead() == LOW;
}
//////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////
// GENERIC ACCEL STEPPER
//////////////////////////////////////////////////////////
GenericAccelStepper :: GenericAccelStepper(uint8_t interface, Pin* pin1, Pin* pin2, Pin* pin3, Pin* pin4) : AccelStepper(interface, 0, 1, 2, 3, false) {
  _genericPins[0] = pin1;
  _genericPins[1] = pin2;
  _genericPins[2] = pin3;
  _genericPins[3] = pin4;

  enableOutputs();
}

void GenericAccelStepper :: setOutputPins(uint8_t mask) {
    uint8_t numpins = 2;
    if (_interface == FULL4WIRE || _interface == HALF4WIRE)
	numpins = 4;
    else if (_interface == FULL3WIRE || _interface == HALF3WIRE)
	numpins = 3;
    uint8_t i;
    for (i = 0; i < numpins; i++)
	_digitalWrite(_pin[i], (mask & (1 << i)) ? (HIGH ^ _pinInverted[i]) : (LOW ^ _pinInverted[i]));
}

void GenericAccelStepper :: disableOutputs() {
  if (! _interface) return;

  setOutputPins(0); // Handles inversion automatically
  if (_enablePin != 0xff)
  {
      _pinMode(_enablePin, OUTPUT);
      _digitalWrite(_enablePin, LOW ^ _enableInverted);
  }
}

void GenericAccelStepper :: enableOutputs() {
  if (! _interface) return;

  _pinMode(_pin[0], OUTPUT);
  _pinMode(_pin[1], OUTPUT);
  if (_interface == FULL4WIRE || _interface == HALF4WIRE)
  {
      _pinMode(_pin[2], OUTPUT);
      _pinMode(_pin[3], OUTPUT);
  }
  else if (_interface == FULL3WIRE || _interface == HALF3WIRE)
  {
      _pinMode(_pin[2], OUTPUT);
  }

  if (_enablePin != 0xff)
  {
      _pinMode(_enablePin, OUTPUT);
      _digitalWrite(_enablePin, HIGH ^ _enableInverted);
  }
}

void GenericAccelStepper :: setEnablePin(uint8_t enablePin){
  _enablePin = enablePin;

  // This happens after construction, so init pin now.
  if (_enablePin != 0xff)
  {
      _pinMode(_enablePin, OUTPUT);
      _digitalWrite(_enablePin, HIGH ^ _enableInverted);
  }
}

void GenericAccelStepper :: _digitalWrite(uint8_t pin, uint8_t value) {
  _genericPins[pin]->doDigitalWrite(value);
}

void GenericAccelStepper :: _pinMode(uint8_t pin, uint8_t value) {
 _genericPins[pin]->setPinMode(value);
}
//////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////
// STEPPER
//////////////////////////////////////////////////////////
Stepper :: Stepper(Pin* enablePin, Pin* stepPin, Pin* directionPin, int microStepsPerStep) {
  _microStepsPerStep = microStepsPerStep;
  _enablePin = enablePin;
  _enablePin->setPinMode(OUTPUT);
  _enablePin->doDigitalWrite(LOW);

  _accelStepper = new GenericAccelStepper(AccelStepper::DRIVER, stepPin, directionPin);
  _accelStepper->setPinsInverted(true, false, false);
  setMaxSpeed(1000.0);
  setAcceleration(70.0);
  setCurrentPosition(0);
}

void Stepper :: setMaxSpeed(float speed) {
  float microStepsSpeed = (float)(speed * _microStepsPerStep);
  _accelStepper->setMaxSpeed(microStepsSpeed);
}

void Stepper :: setAcceleration(float acceleration) {
  float microStepsAcceleration = (float)(acceleration * _microStepsPerStep);
  _accelStepper->setAcceleration(microStepsAcceleration);
}

void Stepper :: move(long steps) {
  long microSteps = (long)(steps * _microStepsPerStep);
  _accelStepper->move(microSteps);
  _accelStepper->run();
}

void Stepper :: moveToPosition(long steps) {
  long microSteps = (long)(steps * _microStepsPerStep);
  _accelStepper->runToNewPosition(microSteps);
}

void Stepper :: moveRelative(long steps) {
  long microSteps = (long)(steps * _microStepsPerStep);
  _accelStepper->move(microSteps);
  _accelStepper->runToPosition();
}

void Stepper :: setCurrentPosition(long position) {
  long microStepsPosition = (long)(position * _microStepsPerStep);
  _accelStepper->setCurrentPosition(microStepsPosition);
}
//////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////
// RELAY
//////////////////////////////////////////////////////////
Relay :: Relay(Pin* controlPin) {
  _controlPin = controlPin;
  _controlPin->setPinMode(OUTPUT);
  _controlPin->doDigitalWrite(LOW);
}

void Relay :: turnOn() {
  _controlPin->doDigitalWrite(HIGH);
}

void Relay :: turnOff() {
  _controlPin->doDigitalWrite(LOW);
}
//////////////////////////////////////////////////////////


