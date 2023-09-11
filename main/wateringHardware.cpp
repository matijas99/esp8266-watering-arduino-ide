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
  float fullRotationSteps = float(Stepper::stepperFullRotationSteps) / beltRatio;
  _stepsPerDegree = fullRotationSteps / 360.0;
  _currentAngleDegrees = 0;
}

void Arm :: moveToAngle(float newAngleDegrees) {
  if (abs(newAngleDegrees) > Arm::maxAngleDegrees) {
    if (newAngleDegrees < 0) {
      newAngleDegrees = -1 * Arm::maxAngleDegrees;
    } else {
      newAngleDegrees = Arm::maxAngleDegrees;
    }
  }

  int angleDiff = newAngleDegrees - _currentAngleDegrees;
  int stepsDiff = angleDiff * _stepsPerDegree;

  _rotationStepper->moveRelative(-1 * int(stepsDiff));
  _currentAngleDegrees = newAngleDegrees;
}

void Arm :: resetPosition() {
  while (!_rotationLimit->isPressed()) {
    _rotationStepper->moveRelative(-1);
  }
  _currentAngleDegrees = Arm::_zeroPositionDegrees;
  moveToAngle(0.0);
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
  _currentPositionMillimeters = Rail::zeroPositionMillimeters;
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

  Pin* pinEnablePositionStepper = new PinExtender(_mcp, GPB0);
  Pin* pinStepPositionStepper = new PinExtender(_mcp, GPB1);
  Pin* pinDirectionPositionStepper = new PinExtender(_mcp, GPB2);
  Stepper* positionStepper = new Stepper(pinEnablePositionStepper, pinStepPositionStepper, pinDirectionPositionStepper, 1);
  
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
  _arm->resetPosition();
  _rail->resetPosition();

  // delay(2000);
  // _arm->moveToAngle(-200.0);
  // _arm->moveToAngle(180.0);
  // _arm->moveToAngle(0.0);
  // delay(2000);
  // _arm->moveToAngle(-45.0);
  // _arm->moveToAngle(45.0);
  // _arm->moveToAngle(-170.0);
  // _arm->moveToAngle(180.0);
  // _arm->moveToAngle(0.0);
  // delay(2000);
  // _arm->moveToAngle(170.0);
  // _rail->moveToPosition(50);
  // _rail->moveToPosition(100);
  // _rail->moveToPosition(0);
  // _rail->moveToPosition(300);
  // _rail->moveToPosition(20);

  // _moveToCoordinates({ 200, 600 });
  // _moveToCoordinates({ 0, 800 });
  // _moveToCoordinates({ -400, 250 });
  // _moveToCoordinates({ 200, 600 });
  // _moveToCoordinates({ -300, 400 });

  // _moveToCoordinates({ 500, 50 });

  // _moveToCoordinates({ 0, 550 });
}

void Waterman :: _moveToCoordinates(Coordinates coordinates) {
  float armAngleRadians = asin(float(abs(coordinates.x)) / float(Arm::lengthMillimeters));
  float armAngleDegrees = (armAngleRadians * 4068) / 71;

  float yArmLengthMillimeters = sqrt(float(Arm::lengthMillimeters * Arm::lengthMillimeters - coordinates.x * coordinates.x));
  long yRailPosition = coordinates.y - long(yArmLengthMillimeters);

  if (coordinates.y < (Rail::zeroPositionMillimeters + Arm::lengthMillimeters)) {
    yRailPosition += 2 * yArmLengthMillimeters;
    if (armAngleDegrees > 0) {
      armAngleDegrees = 180.0 - armAngleDegrees;
    } else {
      armAngleDegrees = -180.0 + abs(armAngleDegrees);
    }
  }

  _arm->moveToAngle(armAngleDegrees);
  _rail->moveToPosition(yRailPosition);

  Serial.println("_moveToCoordinates");
  Serial.println(armAngleDegrees);
  Serial.println(yRailPosition);
}
//////////////////////////////////////////////////////////

