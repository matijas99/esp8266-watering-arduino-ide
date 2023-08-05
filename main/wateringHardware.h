#ifndef wateringHardware_h
#define wateringHardware_h

#include <Arduino.h>

// //////////////////////////////////////////////////////////
// // COMPONENT
// //////////////////////////////////////////////////////////
// class Component {
//   public:
//     virtual void toString();

//   private:
//     String _name; 
// };
// //////////////////////////////////////////////////////////


// //////////////////////////////////////////////////////////
// // SWITCH
// //////////////////////////////////////////////////////////
// class Switch : public Component {
//   public:
//     Switch(String name, uint8_t pin);

//     bool isPressed(); 

//   private:
//     uint8_t _pin;
// };
// //////////////////////////////////////////////////////////


// //////////////////////////////////////////////////////////
// // STEPPER
// //////////////////////////////////////////////////////////
// class Stepper : public Component {
//   public:
//     static const int stepperFullRotationSteps = 200;

//     uint8_t enablePin;
//     uint8_t directionPin;
//     uint8_t stepPin;
//     int currentPositionSteps;

//     Stepper(String name, uint8_t enablePin, uint8_t directionPin, uint8_t stepPin) : Component(name);

//     void turnForwardSteps(int steps);
//     void turnBackwardSteps(int steps);
//     void resetPosition();
//     String toString();

//   private:  
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

//     Arm(String name, Stepper* rotationStepper, Switch* rotationLimit) : Component(name);

//     void moveToAngle(int newAngleDegrees);
//     void resetPosition();
//     String toString();

//   private:
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

//     Rail(String name, Stepper* positionStepper, Switch* positionLimit) : Component(name);

//     void moveToPosition(int newPositionMillimeters);
//     void resetPosition();
//     String toString();
  
//   private:
// };
// //////////////////////////////////////////////////////////

#endif
