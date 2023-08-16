#include "Arduino.h"
#include "MCP23017-SOLDERED.h"
#include "wateringHardware.h"
#include "basicHardware.h"


//////////////////////////////////////////////////////////
// ARM
//////////////////////////////////////////////////////////
Arm :: Arm(Stepper* rotationStepper, Switch* rotationLimit) {
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

  _rotationStepper->moveRelative(stepsDiff);
  
  _currentAngleDegrees = newAngleDegrees;
}

void Arm :: resetPosition() {
  while (!_rotationLimit->isPressed()) {
    _rotationStepper->moveRelative(-1);
  }
  _currentAngleDegrees = 0;
  _rotationStepper->setCurrentPosition(0);
}
//////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////
// RAIL
//////////////////////////////////////////////////////////
Rail :: Rail(Stepper* positionStepper, Switch* positionLimit) {
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

  _positionStepper->moveRelative(stepsDiff);
  
  _currentPositionMillimeters = newPositionMillimeters;
}

void Rail :: resetPosition() {
  while (!_positionLimit->isPressed()) {
    _positionStepper->moveRelative(-1);
  }
  _currentPositionMillimeters = 0;
  _positionStepper->setCurrentPosition(0);
}

//////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////
// WATERMAN
//////////////////////////////////////////////////////////
Waterman :: Waterman() {
  
  _mcp = new MCP_23017();
  _mcp->begin();
  Wire.setClock(400000);

  delay(1000);

  Pin* pinEnablePositionStepper = new PinExtender(_mcp, GPB0);
  Pin* pinStepPositionStepper = new PinExtender(_mcp, GPB1);
  Pin* pinDirectionPositionStepper = new PinExtender(_mcp, GPB2);
  Stepper* positionStepper = new Stepper(pinEnablePositionStepper, pinStepPositionStepper, pinDirectionPositionStepper, 2);
  
  Pin* pinPositionLimit = new PinExtender(_mcp, GPB3);
  Switch* positionLimit = new Switch(pinPositionLimit);
  _rail = new Rail(positionStepper, positionLimit);

  Pin* pinEnableRotationStepper = new PinExtender(_mcp, GPA0);
  Pin* pinStepRotationStepper = new PinExtender(_mcp, GPA1);
  Pin* pinDirectionRotationStepper = new PinExtender(_mcp, GPA2);
  Stepper* rotationStepper = new Stepper(pinEnableRotationStepper, pinStepRotationStepper, pinDirectionRotationStepper, 2);

  Pin* pinRotationLimit = new PinExtender(_mcp, GPA3);
  Switch* rotationLimit = new Switch(pinRotationLimit);
  _arm = new Arm(rotationStepper, rotationLimit);
}

void Waterman :: resetPosition() {
  int delayMs = 3000;
  _arm->resetPosition();
  delay(delayMs);
  _rail->resetPosition();
  delay(delayMs);
  _arm->moveToAngle(90);
  delay(delayMs);
  _arm->moveToAngle(0);
  delay(delayMs);
  _arm->moveToAngle(200);
  delay(delayMs);
  _arm->moveToAngle(45);
  delay(delayMs);
  _rail->moveToPosition(50);
  delay(delayMs);
  _rail->moveToPosition(100);
  delay(delayMs);
  _rail->moveToPosition(0);
  delay(delayMs);
  _rail->moveToPosition(350);
}
//////////////////////////////////////////////////////////

