#include "basicHardware.h"
#include "wateringHardware.h"

// MCP_23017* mcp;
// Switch* button;
// Pin* pin;
Waterman* waterman;

void setup() {
  Serial.begin(115200);


  // mcp = new MCP_23017();
  // mcp->begin();

  // pin = new PinExtender(mcp, GPB0);
  // // pin = new PinNative(D6);
  // button = new Switch("button", pin);

  // pinMode(LED_BUILTIN, OUTPUT);


  waterman = new Waterman();
}

// the loop function runs over and over again forever
void loop() {
  // if (button->isPressed()) {
  //   Serial.println("ON");
  // } else {
  //   Serial.println("off");
  // }



  // if (button->isPressed() == true) {
  //   digitalWrite(LED_BUILTIN, LOW);
  // } else {
  //   digitalWrite(LED_BUILTIN, HIGH);
  // }

  waterman->resetPosition();
  delay(10000);
}
