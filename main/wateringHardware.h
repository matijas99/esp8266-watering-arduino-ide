#ifndef wateringHardware_h
#define wateringHardware_h

#include <Arduino.h>
#include "MCP23017-SOLDERED.h"
#include "basicHardware.h"
#include "LiquidCrystal_I2C.h"

//////////////////////////////////////////////////////////
// ARM
//////////////////////////////////////////////////////////
class Arm {
  public:
    static const int lengthMillimeters = 515;
    static constexpr float beltRatio = 1.0 / 3.0;
    static constexpr int maxAngleDegrees = 173;

    Arm(Stepper* rotationStepper, Switch* rotationLimit);

    void on();
    void off();
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
    static const int lengthMillimeters = 1000;
    static const int pulleyTeeth = 20;
    static const int zeroPositionMillimeters = 115;

    Rail(Stepper* positionStepper, Switch* positionLimit);

    void on();
    void off();
    void moveToPosition(float newPositionMillimeters);
    void resetPosition();
  
  private:
    Stepper* _positionStepper;
    Switch* _positionLimit;
    float _stepsPerMillimeter;
    float _currentPositionMillimeters;
};
//////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////
// COORDINATE
//////////////////////////////////////////////////////////
struct Coordinates {
  int x;
  int y;
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
    LiquidCrystal_I2C* _lcd;
    bool _moveToCoordinates(Coordinates coordinates);
    void _pumpWater(Thirstiness thirstiness);
    void _waterPlant(Plant plant);

};
//////////////////////////////////////////////////////////

#endif
