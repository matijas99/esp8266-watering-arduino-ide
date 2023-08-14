// #ifndef wateringHardware_h
// #define wateringHardware_h

// #include <Arduino.h>
// #include "MCP23017-SOLDERED.h"
// #include "basicHardware.h"

// //////////////////////////////////////////////////////////
// // ARM
// //////////////////////////////////////////////////////////
// class Arm {
//   public:
//     static const int lengthMillimeters = 500;
//     static constexpr float beltRatio = 1.0 / 3.0;
//     static const int maxAngleDegrees = 330;

//     Arm(Stepper* rotationStepper, Switch* rotationLimit);

//     void moveToAngle(int newAngleDegrees);
//     void resetPosition();

//   private:
//     Stepper* _rotationStepper;
//     Switch* _rotationLimit;
//     int _stepsPerDegree;
//     int _currentAngleDegrees;
// };
// //////////////////////////////////////////////////////////


// //////////////////////////////////////////////////////////
// // RAIL
// //////////////////////////////////////////////////////////
// class Rail {
// public:
//     static const int lengthMillimeters = 1500;
//     static const int pulleyTeeth = 20;

//     Rail(Stepper* positionStepper, Switch* positionLimit);

//     void moveToPosition(int newPositionMillimeters);
//     void resetPosition();
  
//   private:
//     Stepper* _positionStepper;
//     Switch* _positionLimit;
//     int _stepsPerMillimeter;
//     int _currentPositionMillimeters;
// };
// //////////////////////////////////////////////////////////


// //////////////////////////////////////////////////////////
// // WATERMAN
// //////////////////////////////////////////////////////////
// class Waterman {
//   public:
//     Waterman();
    
//     void resetPosition();
  
//   private:
//     MCP_23017* _mcp;
//     Rail* _rail;
//     Arm* _arm;
// };
// //////////////////////////////////////////////////////////

// #endif
