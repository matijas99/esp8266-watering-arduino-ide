
// Define the pin numbers for the limit switches
// const int arm_limit_pin = 2; // Change this to the appropriate pin number for the arm limit switch
// const int rail_limit_pin = 3; // Change this to the appropriate pin number for the rail limit switch

// Create the limit switches
// Switch arm_limit("arm", arm_limit_pin);
// Switch rail_limit("rail", rail_limit_pin);

// // Create the Steppers
// Stepper arm_stepper("arm", "Stepper1");
// Stepper rail_stepper("rail", "Stepper2");

// // Create the Arms and Rails
// Arm arm("Arm", &arm_stepper, &arm_limit);
// Rail rail("Rail", &rail_stepper, &rail_limit);

#include "MCP23017-SOLDERED.h"
#include "basicHardware.h"

MCP_23017 mcp;

void setup() {
    // setupMotorExtender();
    // setupMotor();

    Pin* pinEnable = new PinNative(D6);
    Pin* pinStep = new PinNative(D5);
    Pin* pinDirection = new PinNative(D7);

    // mcp.begin();
    // Pin* pinEnable = new PinExtender(mcp, GPA0);
    // Pin* pinStep = new PinExtender(mcp, GPA1);
    // Pin* pinDirection = new PinExtender(mcp, GPA2);
    
    Stepper* stepper = new Stepper("stepper", pinEnable, pinStep, pinDirection);
    stepper->turnForwardSteps(1000);
    stepper->turnBackwardSteps(1000);

    // Pin* pin = new PinNative(D5);
}

// the loop function runs over and over again forever
void loop() {
  // runMotorExtender();
  // runMotor();
}
