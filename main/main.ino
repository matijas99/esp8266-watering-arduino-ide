#include "basicHardware.h"
// #include "wateringHardware.h"

// MCP_23017* _mcp;
// Switch* button;
// Pin* pin;
// Waterman* waterman;

void setup() {
  Serial.begin(115200);

  MCP_23017* _mcp = new MCP_23017();
  _mcp->begin();
  Wire.setClock(400000);

  delay(3000);

  Pin* pinEnablePositionStepper = new PinExtender(_mcp, GPB0);
  Pin* pinStepPositionStepper = new PinExtender(_mcp, GPB1);
  Pin* pinDirectionPositionStepper = new PinExtender(_mcp, GPB2);

  pinEnablePositionStepper->setPinMode(OUTPUT);
  pinEnablePositionStepper->doDigitalWrite(LOW);
  delay(1000);
  GenericAccelStepper* _stepper = new GenericAccelStepper(AccelStepper::DRIVER, pinStepPositionStepper, pinDirectionPositionStepper);
  _stepper->setMaxSpeed(2000.0);
  _stepper->setAcceleration(150.0);
  _stepper->setPinsInverted(false, true, false);
  _stepper->runToNewPosition(1500);



  // mcp = new MCP_23017();
  // mcp->begin();

  // pin = new PinExtender(mcp, GPA3);
  // // pin = new PinNative(D6);
  // button = new Switch("button", pin);

  // pinMode(LED_BUILTIN, OUTPUT);


  // waterman = new Waterman();
  // waterman->resetPosition();


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
  // waterman->resetPosition();  
  delay(5000);
}
