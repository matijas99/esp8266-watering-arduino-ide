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
  // Serial.println("GenericAccelStepper::GenericAccelStepper");
  // Serial.println("interface");
  // Serial.println(_interface);
  // Serial.println(interface);
  
  _genericPins[0] = pin1;
  _genericPins[1] = pin2;
  _genericPins[2] = pin3;
  _genericPins[3] = pin4;

  enableOutputs();
  Serial.println("5");
  delay(1000);

}

void GenericAccelStepper :: setOutputPins(uint8_t mask) {
  // Serial.print("GenericAccelStepper :: setOutputPins ");
  // Serial.println(mask);

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
  // Serial.println("GenericAccelStepper :: disableOutputs");
  
    if (! _interface) return;

    setOutputPins(0); // Handles inversion automatically
    if (_enablePin != 0xff)
    {
        _pinMode(_enablePin, OUTPUT);
        _digitalWrite(_enablePin, LOW ^ _enableInverted);
    }
}

void GenericAccelStepper :: enableOutputs() {
  // Serial.println("GenericAccelStepper :: enableOutputs");
  
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
  // Serial.print("GenericAccelStepper :: setEnablePin ");
  // Serial.println(enablePin);

  _enablePin = enablePin;

  // This happens after construction, so init pin now.
  if (_enablePin != 0xff)
  {
      _pinMode(_enablePin, OUTPUT);
      _digitalWrite(_enablePin, HIGH ^ _enableInverted);
  }
}

void GenericAccelStepper::_digitalWrite(uint8_t pin, uint8_t value) {
  // Serial.print("GenericAccelStepper :: _digitalWrite ");
  // Serial.print(pin);
  // Serial.print(" ");
  // Serial.println(value);

  _genericPins[pin]->doDigitalWrite(value);
}

void GenericAccelStepper::_pinMode(uint8_t pin, uint8_t value) {
  // Serial.print("GenericAccelStepper :: _pinMode ");
  // Serial.print(pin);
  // Serial.print(" ");
  // Serial.println(value);

  _genericPins[pin]->setPinMode(value);
}
//////////////////////////////////////////////////////////


// //////////////////////////////////////////////////////////
// // STEPPER
// //////////////////////////////////////////////////////////
// Stepper :: Stepper(Pin* enablePin,  Pin* stepPin, Pin* directionPin) {
//     _pins[0] = enablePin;
//     _pins[1] = stepPin;
//     _pins[2] = directionPin;

//     _pins[0]->setPinMode(OUTPUT);
//     _pins[1]->setPinMode(OUTPUT);
//     _pins[2]->setPinMode(OUTPUT);
//     _pins[0]->doDigitalWrite(LOW);

//     _accelStepper = new GenericAccelStepper(AccelStepper::DRIVER, stepPin, directionPin);
//     _accelStepper->setMaxSpeed(1000.0);
//     _accelStepper->setMaxSpeed(50.0);
// }

// void Stepper::move(long steps) {
//   _accelStepper->move(steps)
// }
// //////////////////////////////////////////////////////////






// //////////////////////////////////////////////////////////
// // STEPPER
// //////////////////////////////////////////////////////////
// Stepper :: Stepper(String name, Pin* enablePin,  Pin* stepPin, Pin* directionPin) {
//     _name = name;
//     _enablePin = enablePin;
//     _stepPin = stepPin;
//     _directionPin = directionPin;
//     _currentPositionSteps = 0;

//     _enablePin->setPinMode(OUTPUT);
//     _directionPin->setPinMode(OUTPUT);
//     _stepPin->setPinMode(OUTPUT);
//     _enablePin->doDigitalWrite(LOW);

//     _accelStepper = new AccelStepper(AccelStepper::DRIVER)
// }

// void Stepper :: turnForwardSteps(int steps) {
//     _directionPin->doDigitalWrite(LOW);
//     _turnSteps(steps);
//     this->_currentPositionSteps += steps;
// }

// void Stepper :: turnBackwardSteps(int steps) {
//     _directionPin->doDigitalWrite(HIGH);
//     _turnSteps(steps);
//     this->_currentPositionSteps -= steps;
// }

// void Stepper :: resetPosition() {
//     this->_currentPositionSteps = 0;
// }

// String Stepper :: toString() {
//     return "[STEPPER " + _name + "] position: " + String(_currentPositionSteps);
// }

// void Stepper :: _turnSteps(int steps) {
//   for(int i = 0; i < steps; i++)
//   {
//     _stepPin->doDigitalWrite(HIGH);
//     delayMicroseconds(Stepper :: stepDelayMicroSeconds);
//     _stepPin->doDigitalWrite(LOW);
//     delayMicroseconds(Stepper :: stepDelayMicroSeconds);
//   }
// }

// void MCP3017AccelStepper::_digitalWrite(uint8_t pin, uint8_t value) {
//   // if (_usingMcp == true) {
//   //   _mcp.digitalWrite(pin, value);
//   // } else {
//   //   digitalWrite(pin, value);
//   // }
// }

// void MCP3017AccelStepper::_pinMode(uint8_t pin, uint8_t value) {
//   // if (_usingMcp == true) {
//   //   _mcp.pinMode(pin, value);
//   // } else {
//   //   pinMode(pin, value);
//   // }
// }
// //////////////////////////////////////////////////////////


