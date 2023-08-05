#ifndef wateringHardware_h
#define wateringHardware_h

#include <Arduino.h>
#include "basicHardware.h"

//////////////////////////////////////////////////////////
// ARM
//////////////////////////////////////////////////////////
class Arm : public Component {
  public:
    static const int lengthMillimeters = 500;
    static constexpr float beltRatio = 1.0 / 3.0;
    static const int maxAngleDegrees = 330;

    Arm(String name, Stepper* rotationStepper, Switch* rotationLimit);

    void moveToAngle(int newAngleDegrees);
    void resetPosition();
    String toString();

  private:
    Stepper* _rotationStepper;
    Switch* _rotationLimit;
    int _stepsPerDegree;
    int _currentAngleDegrees;
};
//////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////
// RAIL
//////////////////////////////////////////////////////////
class Rail : public Component {
public:
    static const int lengthMillimeters = 1500;
    static const int pulleyTeeth = 20;

    Rail(String name, Stepper* positionStepper, Switch* positionLimit);

    void moveToPosition(int newPositionMillimeters);
    void resetPosition();
    String toString();
  
  private:
    Stepper* _positionStepper;
    Switch* _positionLimit;
    int _stepsPerMillimeter;
    int _currentPositionMillimeters;
};
//////////////////////////////////////////////////////////

#endif
