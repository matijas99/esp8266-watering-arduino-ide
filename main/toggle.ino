// // Include the board file
// #include "MCP23017-SOLDERED.h"

// // Declare the board objecct
// MCP_23017 mcp;

// void setupLedPin() {
//   pinMode(LED_BUILTIN, OUTPUT);
//   pinMode(D5, OUTPUT);
// }

// void toggleLed() {
//   digitalWrite(LED_BUILTIN, LOW);
//   digitalWrite(D5, HIGH);
//   delay(1000);
//   digitalWrite(LED_BUILTIN, HIGH);
//   digitalWrite(D5, LOW);
//   delay(2000);
// }

// void setupMotorExtender() {
//   // Initialize the board
//     mcp.begin();

//     // Set the pin to output
//     mcp.pinMode(GPA0, OUTPUT); // enable
//     mcp.pinMode(GPA1, OUTPUT); // step
//     mcp.pinMode(GPA2, OUTPUT); // direction
//     mcp.digitalWrite(GPA0, LOW);
// }

// void runMotorExtender() {
//   mcp.digitalWrite(GPA2,HIGH);
//   for(int i = 0; i < 2000; i++)
//   {
//     mcp.digitalWrite(GPA1,HIGH);
//     delayMicroseconds(500);
//     mcp.digitalWrite(GPA1,LOW);
//     delayMicroseconds(500);
//   }

//   delay(1000);
//   mcp.digitalWrite(GPA2,LOW);
//   for(int i = 0; i < 2000; i++)
//   {
//     mcp.digitalWrite(GPA1,HIGH);
//     delayMicroseconds(500);
//     mcp.digitalWrite(GPA1,LOW);
//     delayMicroseconds(500);
//   }
//   delay(1000);
// }

// void setupMotor() {
//   pinMode(D6, OUTPUT); //Enable
//   pinMode(D5, OUTPUT); //Step
//   pinMode(D7, OUTPUT); //Direction
//   digitalWrite(D6,LOW);
// }

// void runMotor() {
//   digitalWrite(D7,HIGH);
//   for(int i = 0; i < 2000; i++)
//   {
//     digitalWrite(D5,HIGH);
//     delayMicroseconds(500);
//     digitalWrite(D5,LOW);
//     delayMicroseconds(500);
//   }

//   delay(1000);
//   digitalWrite(D7,LOW);
//   for(int i = 0; i < 2000; i++)
//   {
//     digitalWrite(D5,HIGH);
//     delayMicroseconds(500);
//     digitalWrite(D5,LOW);
//     delayMicroseconds(500);
//   }
//   delay(1000);
// }