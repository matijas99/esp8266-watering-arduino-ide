#include "Arduino.h"
#include "MCP23017-SOLDERED.h"
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
    _rotationStepper->turnForwardSteps(stepsDiff);
  } else {
    _rotationStepper->turnBackwardSteps(abs(stepsDiff));
  }

  _currentAngleDegrees = newAngleDegrees;
}

void Arm :: resetPosition() {
  while (!_rotationLimit->isPressed()) {
    _rotationStepper->turnBackwardSteps(1);
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
    _positionStepper->turnForwardSteps(stepsDiff);
  } else {
    _positionStepper->turnBackwardSteps(abs(stepsDiff));
  }

  _currentPositionMillimeters = newPositionMillimeters;
}

void Rail :: resetPosition() {
  while (!_positionLimit->isPressed()) {
    _positionStepper->turnBackwardSteps(1);
  }
  _positionStepper->resetPosition();
  _currentPositionMillimeters = 0;
}

String Rail :: toString() {
  return "[RAIL " + _name + "] position millimeters: " + String(_currentPositionMillimeters);
}

//////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////
// WATERMAN
//////////////////////////////////////////////////////////
Waterman :: Waterman() {
  
  _mcp = new MCP_23017();
  _mcp->begin();

  Pin* pinEnablePositionStepper = new PinExtender(_mcp, GPB0);
  Pin* pinStepPositionStepper = new PinExtender(_mcp, GPB1);
  Pin* pinDirectionPositionStepper = new PinExtender(_mcp, GPB2);
  Stepper* positionStepper = new Stepper("positionStepper", pinEnablePositionStepper, pinStepPositionStepper, pinDirectionPositionStepper);
  
  Pin* pinPositionLimit = new PinExtender(_mcp, GPB3);
  Switch* positionLimit = new Switch("positionLimit", pinPositionLimit);
  _rail = new Rail("rail", positionStepper, positionLimit);

  Pin* pinEnableRotationStepper = new PinExtender(_mcp, GPA0);
  Pin* pinStepRotationStepper = new PinExtender(_mcp, GPA1);
  Pin* pinDirectionRotationStepper = new PinExtender(_mcp, GPA2);
  Stepper* rotationStepper = new Stepper("positionStepper", pinEnableRotationStepper, pinStepRotationStepper, pinDirectionRotationStepper);

  Pin* pinRotationLimit = new PinExtender(_mcp, GPA3);
  Switch* rotationLimit = new Switch("rotationLimit", pinRotationLimit);
  _arm = new Arm("arm", rotationStepper, rotationLimit);
}

void Waterman :: resetPosition() {
  _arm->resetPosition();
  _rail->resetPosition();
  // delay(3000);
  // _arm->moveToAngle(90);
  // delay(3000);
  // _arm->moveToAngle(45);
  // delay(3000);
  // _arm->moveToAngle(90);
  // delay(3000);
  // _rail->moveToPosition(50);
  // delay(3000);
  // _rail->moveToPosition(100);
  // delay(3000);
  // _rail->moveToPosition(50);
}
//////////////////////////////////////////////////////////

