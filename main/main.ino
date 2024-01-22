#include "basicHardware.h"
#include "wateringHardware.h"

// MCP_23017* _mcp;
// Switch* button;
// Pin* pin;
Waterman* waterman;
bool didRun = false;

Plant plants[] = {
  { { -200, 0 }, Thirstiness::HIGH_THIRST },
  { { -310, 290 }, Thirstiness::HIGH_THIRST },
  { { 230, 840 }, Thirstiness::LOW_THIRST },
  { { -290, 880 }, Thirstiness::MEDIUM_THIRST }
};

void setup() {
  Serial.begin(115200);
  Wire.setClock(400000);

  // pin = new PinExtender(mcp, GPA3);
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

  Serial.println("Ready...");
  delay(1000);

  if (!didRun) {
    waterman->resetPosition();
    delay(1000);
    waterman->waterPlants(plants, 5);
    didRun = true;
  } else {
    Serial.println("Already run, nothing to do");
  }

}
