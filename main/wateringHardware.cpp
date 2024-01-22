#include "Arduino.h"
#include "MCP23017-SOLDERED.h"
#include "wateringHardware.h"
#include "basicHardware.h"
#include "LiquidCrystal_I2C.h"


//////////////////////////////////////////////////////////
// ARM
//////////////////////////////////////////////////////////
Arm :: Arm(Stepper* rotationStepper, Switch* rotationLimit) {
  _rotationStepper = rotationStepper;
  _rotationStepper->setMaxSpeed(500.0);
  _rotationStepper->setAcceleration(40.0);
  _rotationLimit = rotationLimit;
  float fullRotationSteps = (float)_rotationStepper->getStepsPerRotation() / beltRatio;
  _stepsPerDegree = fullRotationSteps / 360.0;
  _currentAngleDegrees = 0.0;
}

void Arm :: on() {
  _rotationStepper->on();
}

void Arm :: off() {
  _rotationStepper->off();
}

void Arm :: moveToAngle(float newAngleDegrees) {
  if (abs(newAngleDegrees) > (float)Arm::maxAngleDegrees) {
    if (newAngleDegrees < 0.0) {
      newAngleDegrees = (float)(-1 * Arm::maxAngleDegrees);
    } else {
      newAngleDegrees = (float)Arm::maxAngleDegrees;
    }
  }

  float angleDiff = newAngleDegrees - _currentAngleDegrees;
  long stepsDiff = (long)(angleDiff * _stepsPerDegree);

  // correction for rounding between float degrees and long microsteps
  angleDiff = stepsDiff / _stepsPerDegree;
  float newAngleDegreesCorrected = angleDiff + _currentAngleDegrees;
  Serial.println("Arm correction: " + String(newAngleDegrees - newAngleDegreesCorrected));

  _rotationStepper->moveRelative(-1 * stepsDiff);
  _currentAngleDegrees = newAngleDegreesCorrected;
}

void Arm :: resetPosition() {
  while (!_rotationLimit->isPressed()) {
    _rotationStepper->moveRelative(-4);
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
  _positionStepper = positionStepper;
  _positionStepper->setMaxSpeed(700.0);
  _positionStepper->setAcceleration(70.0);
  _positionLimit = positionLimit;

  int pulleyMillimetersPerRotation = 2 * pulleyTeeth;
  _stepsPerMillimeter = (float)_positionStepper->getStepsPerRotation() / (float)pulleyMillimetersPerRotation;
  _currentPositionMillimeters = 0.0;
}

void Rail :: on() {
  _positionStepper->on();
}

void Rail :: off() {
  _positionStepper->off();
}

void Rail :: moveToPosition(float newPositionMillimeters) {
  if (newPositionMillimeters < 0.0) {
    newPositionMillimeters = 0.0;
  } else if (newPositionMillimeters > (float)lengthMillimeters) {
    newPositionMillimeters = (float)lengthMillimeters;
  }

  float positionMillimetersDiff = newPositionMillimeters - _currentPositionMillimeters;
  long stepsDiff = (long)(positionMillimetersDiff * _stepsPerMillimeter);

  // correction for rounding between float degrees and long microsteps
  positionMillimetersDiff = stepsDiff / _stepsPerMillimeter;
  float newPositionMillimetersCorrected = positionMillimetersDiff + _currentPositionMillimeters;
  Serial.println("Rail correction: " + String(newPositionMillimeters - newPositionMillimetersCorrected));

  _positionStepper->moveRelative(stepsDiff);
  
  _currentPositionMillimeters = newPositionMillimetersCorrected;
}

void Rail :: resetPosition() {
  while (!_positionLimit->isPressed()) {
    _positionStepper->moveRelative(-8);
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

  Pin* pinEnablePositionStepper = new PinExtender(_mcp, GPB0);
  Pin* pinStepPositionStepper = new PinExtender(_mcp, GPB1);
  Pin* pinDirectionPositionStepper = new PinExtender(_mcp, GPB2);
  Stepper* positionStepper = new Stepper(pinEnablePositionStepper, pinStepPositionStepper, pinDirectionPositionStepper, 8);
  
  Pin* pinPositionLimit = new PinExtender(_mcp, GPB3);
  Switch* positionLimit = new Switch(pinPositionLimit);
  _rail = new Rail(positionStepper, positionLimit);

  Pin* pinEnableRotationStepper = new PinExtender(_mcp, GPA0);
  Pin* pinStepRotationStepper = new PinExtender(_mcp, GPA1);
  Pin* pinDirectionRotationStepper = new PinExtender(_mcp, GPA2);
  Stepper* rotationStepper = new Stepper(pinEnableRotationStepper, pinStepRotationStepper, pinDirectionRotationStepper, 8);

  Pin* pinRotationLimit = new PinExtender(_mcp, GPA3);
  Switch* rotationLimit = new Switch(pinRotationLimit);
  _arm = new Arm(rotationStepper, rotationLimit);

  Pin* pinPumpRelay = new PinExtender(_mcp, GPA4);
  _pumpRelay = new Relay(pinPumpRelay);

  _lcd = new LiquidCrystal_I2C(0x26, 16, 2); 
  _lcd->init();
  _lcd->backlight();
}

void Waterman :: resetPosition() {
  _lcd->clear();
  _lcd->setCursor(0, 0);
  _lcd->print("Resetting...");
  _lcd->setCursor(0, 1);
  _lcd->print("Arm ");
  _arm->resetPosition();
  _lcd->print("OK, Rail ");
  _rail->resetPosition();
  _lcd->print("OK");

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

bool Waterman :: _moveToCoordinates(Coordinates coordinates) {
  _lcd->clear();
  _lcd->setCursor(0, 0);
  
  float armAngleRadians = asin(float(coordinates.x) / float(Arm::lengthMillimeters));
  float armAngleDegrees = (armAngleRadians * 4068.0) / 71.0;
  if (coordinates.x == 0) {
    armAngleDegrees = 0.0;
  } else if (coordinates.x > Arm::lengthMillimeters) {
    _lcd->print("Err X: " + String(coordinates.x));
    _lcd->setCursor(0, 1);
    _lcd->print("X: " + String(coordinates.x) + " Y: " + String(coordinates.y));
    return false;
  }

  float yArmLengthMillimeters = sqrt(float(Arm::lengthMillimeters * Arm::lengthMillimeters - coordinates.x * coordinates.x));
  float yRailPosition = (float)coordinates.y - yArmLengthMillimeters;

  if (coordinates.y < (Rail::zeroPositionMillimeters + Arm::lengthMillimeters)) {
    yRailPosition += 2 * yArmLengthMillimeters;
    if (armAngleDegrees > 0) {
      armAngleDegrees = 180.0 - armAngleDegrees;
    } else {
      armAngleDegrees = -180.0 + abs(armAngleDegrees);
    }
  }

  if (yRailPosition > (float)Rail::lengthMillimeters) {
    _lcd->print("Err Y: " + String(yRailPosition));
    _lcd->setCursor(0, 1);
    _lcd->print("X: " + String(coordinates.x) + " Y: " + String(coordinates.y));
    return false;
  } else if (abs(armAngleDegrees) > (float)Arm::maxAngleDegrees) {
    _lcd->print("Err angle: " + String(abs(armAngleDegrees)));
    _lcd->setCursor(0, 1);
    _lcd->print("X: " + String(coordinates.x) + " Y: " + String(coordinates.y));
    return false;
  }

  _lcd->print("Moving...");
  _lcd->setCursor(0, 1);
  _lcd->print("X: " + String(coordinates.x) + " Y: " + String(coordinates.y));

  _arm->moveToAngle(0);
  _rail->moveToPosition(yRailPosition);
  _arm->moveToAngle(armAngleDegrees);

  return true;
}

void Waterman :: _pumpWater(Thirstiness thirstiness) {
  int pumpRuntimeMs = 0;
  switch (thirstiness) {
    case Thirstiness::HIGH_THIRST:
      pumpRuntimeMs = 7000;
      break;
    case Thirstiness::MEDIUM_THIRST:
      pumpRuntimeMs = 5000;
      break;
    case Thirstiness::LOW_THIRST:
      pumpRuntimeMs = 3000;
      break;
  }

  _lcd->clear();
  _lcd->setCursor(0, 0);
  _lcd->print("Pumping water...");
  _lcd->setCursor(0, 1);
  _lcd->print("Duration: " + String(pumpRuntimeMs) + "ms");

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  _arm->off();
  _rail->off();
  _pumpRelay->turnOn();
  delay(pumpRuntimeMs);
  _pumpRelay->turnOff();
  _arm->on();
  _rail->on();

  digitalWrite(LED_BUILTIN, HIGH);

  delay(Waterman::pumpDropDelayMs);
}

void Waterman :: _waterPlant(Plant plant) {
  if (_moveToCoordinates(plant.coordinatesMillimeters)) {
    _pumpWater(plant.thirstiness);
  } else {
    delay(5000);
  }
}

void Waterman :: waterPlants(Plant plants[], int plantsCount) {
  for (int i = 0; i < plantsCount; i++) {
    _waterPlant(plants[i]);
  }
  _arm->moveToAngle(0);
  _rail->moveToPosition(Rail::zeroPositionMillimeters);
  _arm->moveToAngle(Arm::maxAngleDegrees);
}
//////////////////////////////////////////////////////////

