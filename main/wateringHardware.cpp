#include "Arduino.h"
#include "wateringHardware.h"
#include "pin.cpp"

//////////////////////////////////////////////////////////
// COMPONENT
//////////////////////////////////////////////////////////
class Component {
  protected:
    String _name;

  public:
    virtual String toString();
};
//////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////
// SWITCH
//////////////////////////////////////////////////////////
class Switch: public Component {

  uint8_t _pin;

  public:
   void init(String name, uint8_t pin) {
     _name = name;
     _pin = pin;
     pinMode(_pin, INPUT);
   }

   bool isPressed() {
    return digitalRead(_pin) == HIGH;
  }

  String toString() {
    return "[" + _name + "] is pressed: " + String(isPressed());
  }
};
//////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////
// STEPPER
//////////////////////////////////////////////////////////
class Stepper : public Component {

  Pin* _enablePin;
  Pin* _directionPin;
  Pin* _stepPin;
  int _currentPositionSteps;

  void _turnSteps(int steps) {
    for(int i = 0; i < steps; i++)
    {
      _stepPin->doDigitalWrite(HIGH);
      delayMicroseconds(500);
      _stepPin->doDigitalWrite(LOW);
      delayMicroseconds(500);
    }
  }

public:
    static const int stepperFullRotationSteps = 200;

    Stepper(String name, Pin* enablePin,  Pin* stepPin, Pin* directionPin) {
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

    void turnForwardSteps(int steps) {
        _directionPin->doDigitalWrite(LOW);
        _turnSteps(steps);
        this->_currentPositionSteps += steps;
    }

    void turnBackwardSteps(int steps) {
        _directionPin->doDigitalWrite(HIGH);
        _turnSteps(steps);
        this->_currentPositionSteps -= steps;
    }

    void resetPosition() {
        this->_currentPositionSteps = 0;
    }

    String toString() {
        return "[STEPPER " + _name + "] position: " + String(_currentPositionSteps);
    }
};
//////////////////////////////////////////////////////////


// //////////////////////////////////////////////////////////
// // STEPPER
// //////////////////////////////////////////////////////////
// class Stepper : public Component {
// public:
//     static const int stepperFullRotationSteps = 200;

//     uint8_t enablePin;
//     uint8_t directionPin;
//     uint8_t stepPin;
//     int currentPositionSteps;

//     Stepper(String name, uint8_t enablePin, uint8_t directionPin, uint8_t stepPin) : Component(name) {
//         this->enablePin = enablePin;
//         this->directionPin = directionPin;
//         this->stepPin = stepPin;
//         this->currentPositionSteps = 0;
//     }

//     void turnForwardSteps(int steps) {
//         // Implementation for Arduino stepper motor library goes here
//         // For example: stepperMotorTurnSteps(stepper_name, "forward", steps);
//         // TODOMS
//         this->currentPositionSteps += steps;
//     }

//     void turnBackwardSteps(int steps) {
//         // Implementation for Arduino stepper motor library goes here
//         // For example: stepperMotorTurnSteps(stepper_name, "reverse", steps);
//         // TODOMS
//         this->currentPositionSteps -= steps;
//     }

//     void resetPosition() {
//         this->currentPositionSteps = 0;
//     }

//     String toString() {
//         return "[STEPPER " + name + "] position: " + String(currentPositionSteps);
//     }
// };
// //////////////////////////////////////////////////////////


// //////////////////////////////////////////////////////////
// // ARM
// //////////////////////////////////////////////////////////
// class Arm : public Component {
// public:
//     static const int lengthMillimeters = 500;
//     static constexpr float beltRatio = 1.0 / 3.0;
//     static const int maxAngleDegrees = 330;

//     Stepper* rotationStepper;
//     Switch* rotationLimit;
//     int stepsPerDegree;
//     int currentAngleDegrees;

//     Arm(String name, Stepper* rotationStepper, Switch* rotationLimit) : Component(name) {
//         this->rotationStepper = rotationStepper;
//         this->rotationLimit = rotationLimit;
//         int fullRotationSteps = Stepper::stepperFullRotationSteps / beltRatio;
//         this->stepsPerDegree = fullRotationSteps / 360;
//         this->currentAngleDegrees = 0;
//     }

//     void moveToAngle(int newAngleDegrees) {
//         if (newAngleDegrees < 0) {
//             newAngleDegrees = 0;
//         } else if (newAngleDegrees > maxAngleDegrees) {
//             newAngleDegrees = maxAngleDegrees;
//         }

//         int angleDiff = newAngleDegrees - currentAngleDegrees;
//         int stepsDiff = angleDiff * stepsPerDegree;

//         if (stepsDiff > 0) {
//           rotationStepper->turnBackwardSteps(stepsDiff);
//         } else {
//           rotationStepper->turnForwardSteps(abs(stepsDiff));
//         }

//         currentAngleDegrees = newAngleDegrees;
//     }

//     void resetPosition() {
//         while (!rotationLimit->isPressed()) {
//             rotationStepper->turnForwardSteps(1);
//         }
//         rotationStepper->resetPosition();
//         currentAngleDegrees = 0;
//     }

//     String toString() {
//         return "[ARM " + name + "] angle degrees: " + String(currentAngleDegrees);
//     }
// };
// //////////////////////////////////////////////////////////


// //////////////////////////////////////////////////////////
// // RAIL
// //////////////////////////////////////////////////////////
// class Rail : public Component {
// public:
//     static const int lengthMillimeters = 1500;
//     static const int pulleyTeeth = 20;

//     Stepper* positionStepper;
//     Switch* positionLimit;
//     int stepsPerMillimeter;
//     int currentPositionMillimeters;

//     Rail(String name, Stepper* positionStepper, Switch* positionLimit) : Component(name) {
//         int pulleyMillimetersPerRotation = 2 * pulleyTeeth;
//         stepsPerMillimeter = Stepper::stepperFullRotationSteps / pulleyMillimetersPerRotation;
//         currentPositionMillimeters = 0;
//         this->positionStepper = positionStepper;
//         this->positionLimit = positionLimit;
//     }

//     void moveToPosition(int newPositionMillimeters) {
//         if (newPositionMillimeters < 0) {
//             newPositionMillimeters = 0;
//         } else if (newPositionMillimeters > lengthMillimeters) {
//             newPositionMillimeters = lengthMillimeters;
//         }

//         int positionMillimetersDiff = newPositionMillimeters - currentPositionMillimeters;
//         int stepsDiff = positionMillimetersDiff * stepsPerMillimeter;

//         if (stepsDiff > 0) {
//           positionStepper->turnBackwardSteps(stepsDiff);
//         } else {
//           positionStepper->turnForwardSteps(abs(stepsDiff));
//         }

//         currentPositionMillimeters = newPositionMillimeters;
//     }

//     void resetPosition() {
//         while (!positionLimit->isPressed()) {
//             positionStepper->turnForwardSteps(1);
//         }
//         positionStepper->resetPosition();
//         currentPositionMillimeters = 0;
//     }

//     String toString() {
//         return "[RAIL " + name + "] position millimeters: " + String(currentPositionMillimeters);
//     }
// };
// //////////////////////////////////////////////////////////

