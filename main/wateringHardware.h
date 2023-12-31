#ifndef wateringHardware_h
#define wateringHardware_h

#include <Arduino.h>
#include "MCP23017-SOLDERED.h"
#include "basicHardware.h"

//////////////////////////////////////////////////////////
// ARM
//////////////////////////////////////////////////////////
class Arm {
  public:
    static const int lengthMillimeters = 500;
    static constexpr float beltRatio = 1.0 / 3.0;
    static constexpr float maxAngleDegrees = 173.0;

    Arm(Stepper* rotationStepper, Switch* rotationLimit);

    void moveToAngle(float newAngleDegrees);
    void resetPosition();

  private:
    static constexpr float _zeroPositionDegrees = 176.0;
    Stepper* _rotationStepper;
    Switch* _rotationLimit;
    float _stepsPerDegree;
    float _currentAngleDegrees;
};
//////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////
// RAIL
//////////////////////////////////////////////////////////
class Rail {
public:
    static const int lengthMillimeters = 1500;
    static const int pulleyTeeth = 20;
    static const int zeroPositionMillimeters = 150;

    Rail(Stepper* positionStepper, Switch* positionLimit);

    void moveToPosition(int newPositionMillimeters);
    void resetPosition();
  
  private:
    Stepper* _positionStepper;
    Switch* _positionLimit;
    int _stepsPerMillimeter;
    int _currentPositionMillimeters;
};
//////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////
// COORDINATE
//////////////////////////////////////////////////////////
struct Coordinates {
  long x;
  long y;
};
//////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////
// THIRSTINESS
//////////////////////////////////////////////////////////
enum class Thirstiness {
  HIGH_THIRST,
  MEDIUM_THIRST,
  LOW_THIRST
};
//////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////
// PLANT
//////////////////////////////////////////////////////////
struct Plant {
  Coordinates coordinatesMillimeters;
  Thirstiness thirstiness;
};
//////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////
// WATERMAN
//////////////////////////////////////////////////////////
class Waterman {
  public:
    static const int pumpDropDelayMs = 2000;

    Waterman();
    
    void resetPosition();
    void waterPlants(Plant plants[], int plantsCount);
  
  private:
    MCP_23017* _mcp;
    Rail* _rail;
    Arm* _arm;
    Relay* _pumpRelay;
    bool _moveToCoordinates(Coordinates coordinates);
    void _pumpWater(Thirstiness thirstiness);
    void _waterPlant(Plant plant);

};
//////////////////////////////////////////////////////////

#endif
