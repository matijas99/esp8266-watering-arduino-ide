#include "Arduino.h"
#include "wateringHardware.h"
#include "basicHardware.h"


//////////////////////////////////////////////////////////
// ARM
//////////////////////////////////////////////////////////
Arm :: Arm(String name, Stepper* rotationStepper, Switch* rotationLimit) {
  _rotationStepper = rotationStepper;
  _rotationLimit = rotationLimit;
  int fullRotationSteps = Stepper::stepperFullRotationSteps / beltRatio;
  _stepsPerDegree = fullRotationSteps / 360;
  _currentAngleDegrees = 0;
}

void Arm :: moveToAngle(int newAngleDegrees) {
  if (newAngleDegrees < 0) {
    newAngleDegrees = 0;
  } else if (newAngleDegrees > Arm::maxAngleDegrees) {
    newAngleDegrees = Arm::maxAngleDegrees;
  }

  int angleDiff = newAngleDegrees - _currentAngleDegrees;
  int stepsDiff = angleDiff * _stepsPerDegree;

  if (stepsDiff > 0) {
    _rotationStepper->turnBackwardSteps(stepsDiff);
  } else {
    _rotationStepper->turnForwardSteps(abs(stepsDiff));
  }

  _currentAngleDegrees = newAngleDegrees;
}

void Arm :: resetPosition() {
  while (!_rotationLimit->isPressed()) {
    _rotationStepper->turnForwardSteps(1);
  }
  _rotationStepper->resetPosition();
  _currentAngleDegrees = 0;
}

String Arm :: toString() {
  return "[ARM " + _name + "] angle degrees: " + String(_currentAngleDegrees);
}
//////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////
// RAIL
//////////////////////////////////////////////////////////
Rail :: Rail(String name, Stepper* positionStepper, Switch* positionLimit) {
  _name = name;
  int pulleyMillimetersPerRotation = 2 * pulleyTeeth;
  _stepsPerMillimeter = Stepper::stepperFullRotationSteps / pulleyMillimetersPerRotation;
  _currentPositionMillimeters = 0;
  _positionStepper = positionStepper;
  _positionLimit = positionLimit;
}

void Rail :: moveToPosition(int newPositionMillimeters) {
  if (newPositionMillimeters < 0) {
    newPositionMillimeters = 0;
  } else if (newPositionMillimeters > lengthMillimeters) {
    newPositionMillimeters = lengthMillimeters;
  }

  int positionMillimetersDiff = newPositionMillimeters - _currentPositionMillimeters;
  int stepsDiff = positionMillimetersDiff * _stepsPerMillimeter;

  if (stepsDiff > 0) {
    _positionStepper->turnBackwardSteps(stepsDiff);
  } else {
    _positionStepper->turnForwardSteps(abs(stepsDiff));
  }

  _currentPositionMillimeters = newPositionMillimeters;
}

void Rail :: resetPosition() {
  while (!_positionLimit->isPressed()) {
    _positionStepper->turnForwardSteps(1);
  }
  _positionStepper->resetPosition();
  _currentPositionMillimeters = 0;
}

String Rail :: toString() {
  return "[RAIL " + _name + "] position millimeters: " + String(_currentPositionMillimeters);
}

//////////////////////////////////////////////////////////

